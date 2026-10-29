/**
 * AirSim Multirotor PID Target Tracker
 * 
 * Moves the drone to track a locked target using error_x / error_y
 * from OpenCV 4 target recognition.
 *
 * Build (Linux):
 *   g++ -std=c++17 airsim_pid_tracker.cpp \
 *       -I/path/to/AirSim/AirLib/include \
 *       -I/path/to/AirSim/MavLinkCom/include \
 *       -I/path/to/eigen3 \
 *       -L/path/to/AirSim/AirLib/lib \
 *       -lAirLib -lMavLinkCom \
 *       $(pkg-config --cflags --libs opencv4) \
 *       -lpthread -o airsim_pid_tracker
 *
 * Coordinate convention (image → drone body):
 *   error_x > 0  → target is RIGHT  → yaw right  / move right (+Y world)
 *   error_x < 0  → target is LEFT   → yaw left   / move left  (-Y world)
 *   error_y > 0  → target is DOWN   → descend     (-Z world, AirSim NED)
 *   error_y < 0  → target is UP     → ascend      (+Z world, AirSim NED)
 *
 * The drone maintains its forward position (X) while yawing + translating
 * laterally and vertically to center the target in the frame.
 */

#include <iostream>
#include <chrono>
#include <thread>
#include <cmath>
#include <atomic>
#include <csignal>

// AirSim
#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/ImageCaptureBase.hpp"
STRICT_MODE_ON

// OpenCV 4
#include <opencv2/opencv.hpp>

// ─────────────────────────────────────────────────────────────
//  PID Controller
// ─────────────────────────────────────────────────────────────
struct PIDController {
    double kp, ki, kd;
    double integral    = 0.0;
    double prev_error  = 0.0;
    double integral_max;          // anti-windup clamp
    double output_max;            // output saturation

    PIDController(double kp, double ki, double kd,
                  double integral_max = 5.0,
                  double output_max   = 3.0)
        : kp(kp), ki(ki), kd(kd),
          integral_max(integral_max),
          output_max(output_max) {}

    double update(double error, double dt) {
        if (dt <= 0.0) return 0.0;

        integral  += error * dt;
        // Anti-windup
        integral   = std::clamp(integral, -integral_max, integral_max);

        double derivative = (error - prev_error) / dt;
        prev_error        = error;

        double output = kp * error + ki * integral + kd * derivative;
        return std::clamp(output, -output_max, output_max);
    }

    void reset() {
        integral   = 0.0;
        prev_error = 0.0;
    }
};

// ─────────────────────────────────────────────────────────────
//  Config — tune these for your setup
// ─────────────────────────────────────────────────────────────
namespace cfg {
    constexpr int   IMG_W        = 640;
    constexpr int   IMG_H        = 480;

    // Normalised "dead-zone" — fraction of half-width / half-height
    // below which we treat error as zero (avoids jitter)
    constexpr float DEAD_ZONE    = 0.02f;

    // Loop rate
    constexpr int   LOOP_HZ      = 30;

    constexpr float CRUISE_ALT   = -1.0f;   // 1 m AGL

    // Velocity limits (m/s)
    constexpr float VEL_MAX_YAW  = 30.0f;   // deg/s
    constexpr float VEL_MAX_VERT = 3.0f;    // m/s  (NED Z)
    constexpr float VEL_MAX_LAT  = 4.0f;    // m/s  (NED Y)
    constexpr float VEL_MAX_FWD  = 2.0f;    // m/s  (NED X) — slow approach

    // Target lock — pixel threshold to consider "centered"
    constexpr float LOCKED_PX    = 15.0f;

    // AirSim connection
    const std::string AIRSIM_IP  = "127.0.0.1";
}

// ─────────────────────────────────────────────────────────────
//  Graceful shutdown
// ─────────────────────────────────────────────────────────────
std::atomic<bool> g_running{true};
void sig_handler(int) { g_running = false; }

// ─────────────────────────────────────────────────────────────
//  Helper: normalise pixel errors to [-1, 1]
// ─────────────────────────────────────────────────────────────
inline float normalise_x(float err_px) {
    return err_px / (cfg::IMG_W * 0.5f);
}
inline float normalise_y(float err_px) {
    return err_px / (cfg::IMG_H * 0.5f);
}

// ─────────────────────────────────────────────────────────────
//  Main
// ─────────────────────────────────────────────────────────────
int main()
{
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    // ── Connect to AirSim ──────────────────────────────────────
    msr::airlib::MultirotorRpcLibClient client(cfg::AIRSIM_IP);
    client.confirmConnection();
    client.enableApiControl(true, "Interceptor");
    client.armDisarm(true, "Interceptor");

    std::cout << "[INFO] Arming & taking off...\n";
    client.takeoffAsync(10.0f)->waitOnLastTask();

    // Move to cruise altitude before tracking starts
    auto pose = client.simGetVehiclePose();
    client.moveToPositionAsync(
        pose.position.x(), pose.position.y(), cfg::CRUISE_ALT,
        2.0f)->waitOnLastTask();

    std::cout << "[INFO] Cruising at " << -cfg::CRUISE_ALT << " m AGL\n";
    std::cout << "[INFO] Starting target tracking loop. Ctrl-C to stop.\n";

    // ── PID controllers ───────────────────────────────────────
    //   Lateral (Y)  : driven by normalised error_x
    //   Vertical (Z) : driven by normalised error_y
    //   Yaw          : driven by normalised error_x (softer gain)
    PIDController pid_lateral (2.5, 0.05, 0.4,  4.0, cfg::VEL_MAX_LAT);
    PIDController pid_vertical(2.0, 0.04, 0.35, 4.0, cfg::VEL_MAX_VERT);
    PIDController pid_yaw     (18.0,0.2,  2.0,  20.0, cfg::VEL_MAX_YAW);

    // ── Timing ────────────────────────────────────────────────
    const double dt_target = 1.0 / cfg::LOOP_HZ;
    auto t_prev = std::chrono::steady_clock::now();



    // ── Tracking loop ─────────────────────────────────────────
    while (g_running) {
        auto t_now = std::chrono::steady_clock::now();
        double dt  = std::chrono::duration<double>(t_now - t_prev).count();
        t_prev     = t_now;
        
        const std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request = {
            msr::airlib::ImageCaptureBase::ImageRequest("front_center", msr::airlib::ImageCaptureBase::ImageType::Scene, false, false)
        };
        const std::vector<msr::airlib::ImageCaptureBase::ImageResponse> response = client.simGetImages(request);

        if (response.empty()) continue;
        cv::Mat frame(response[0].height, response[0].width, CV_8UC3, (void*)response[0].image_data_uint8.data());
        cv::Mat hsv, mask, mask1, mask2;

        cv::cvtColor(frame, hsv, cv::COLOR_BGR2HSV);

        cv::inRange(hsv, cv::Scalar(0, 120, 70), cv::Scalar(10, 255, 255), mask1);
        cv::inRange(hsv, cv::Scalar(170, 120, 70), cv::Scalar(180, 255, 255), mask2);

        mask = mask1 | mask2;

        cv::Moments m = cv::moments(mask, true);

        double error_x = 0.0;
        double error_y = 0.0;
        double cx = 0.0;
        double cy = 0.0;

        if (m.m00 >= 100.0) {
            cx = m.m10 / m.m00;
            cy = m.m01 / m.m00;
                
            cv::drawMarker(frame, cv::Point(cx, cy), cv::Scalar(0, 255, 0), cv::MARKER_SQUARE, 30, 2);
            error_x = cx - 320;
            error_y = cy - 240;
        } else {
            // Lost target — hover in place
            client.hoverAsync();
            pid_lateral .reset();
            pid_vertical.reset();
            pid_yaw     .reset();
            // std::this_thread::sleep_for(
            //     std::chrono::milliseconds(static_cast<int>(dt_target * 1000)));
            // continue;
        }

        // ── ② Apply dead-zone ─────────────────────────────────
        float nx = normalise_x(error_x);
        float ny = normalise_y(error_y);
        if (std::abs(nx) < cfg::DEAD_ZONE) { nx = 0.0f; pid_lateral.reset(); pid_yaw.reset(); }
        if (std::abs(ny) < cfg::DEAD_ZONE) { ny = 0.0f; pid_vertical.reset(); }

        // ── ③ Compute PID outputs ─────────────────────────────
        //  Lateral velocity  (NED Y, m/s)
        float vel_y   = static_cast<float>(pid_lateral .update( nx, dt));
        //  Vertical velocity (NED Z, m/s)  — error_y>0 means descend → +Z in NED
        float vel_z   = static_cast<float>(pid_vertical.update( ny, dt));
        //  Yaw rate          (deg/s)        — error_x>0 means yaw right
        float yaw_rate= static_cast<float>(pid_yaw     .update( nx, dt));

        // Slow forward approach (optional — comment out if hovering in place)
        float vel_x   = cfg::VEL_MAX_FWD * 0.3f;   // constant creep forward

        // ── ④ Clamp & send velocity command ───────────────────
        vel_x = std::clamp(vel_x, -cfg::VEL_MAX_FWD,  cfg::VEL_MAX_FWD);
        vel_y = std::clamp(vel_y, -cfg::VEL_MAX_LAT,  cfg::VEL_MAX_LAT);
        vel_z = std::clamp(vel_z, -cfg::VEL_MAX_VERT, cfg::VEL_MAX_VERT);

        // moveByVelocityAsync(vx, vy, vz, duration, drivetrainType, yawMode)
        // YawMode: is_rate=true means we pass yaw_rate (deg/s)
        client.moveByVelocityAsync(
            vel_x, vel_y, vel_z,
            static_cast<float>(dt_target * 2.0),   // command duration
            msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
            msr::airlib::YawMode(true, yaw_rate)
        );

        // ── ⑤ Debug output ────────────────────────────────────
        bool locked = (std::abs(error_x) < cfg::LOCKED_PX &&
                       std::abs(error_y) < cfg::LOCKED_PX);

        std::printf("\r[%s] err=(%+6.1f, %+6.1f)px  vel=(%+.2f, %+.2f, %+.2f)m/s  yaw=%+.1f°/s   ",
                    locked ? "LOCKED " : "TRACKING",
                    error_x, error_y,
                    vel_x, vel_y, vel_z,
                    yaw_rate);
        
        std::string status = "LOCK: [" + std::to_string(error_x) + ", " + std::to_string(error_y) + "]";
        cv::putText(frame, status,cv::Point(static_cast<int>(cx) + 20, static_cast<int>(cy) - 20),cv::FONT_HERSHEY_SIMPLEX, 1.0, cv::Scalar(255, 0, 0));
        cv::imshow("Interceptor", frame);
        cv::waitKey(1);
        std::fflush(stdout);

        // ── ⑥ Sleep to maintain loop rate ─────────────────────
        auto elapsed = std::chrono::steady_clock::now() - t_now;
        auto sleep_us = std::chrono::microseconds(
            static_cast<long>(dt_target * 1e6)) - elapsed;
        if (sleep_us.count() > 0)
            std::this_thread::sleep_for(sleep_us);
    }

    // ── Graceful shutdown ─────────────────────────────────────
    std::cout << "\n[INFO] Shutting down — hovering in place...\n";
    client.hoverAsync()->waitOnLastTask();
    client.landAsync(5.0f)->waitOnLastTask();
    client.armDisarm(false);
    client.enableApiControl(false);

    return 0;
}