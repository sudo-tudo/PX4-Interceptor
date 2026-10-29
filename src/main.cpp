/**
 * AirSim Multirotor PID Target Tracker
 *
 * Coordinate convention (image → drone body):
 *   error_x > 0  → target is RIGHT  → yaw right  / move right (+Y world)
 *   error_x < 0  → target is LEFT   → yaw left   / move left  (-Y world)
 *   error_y > 0  → target is DOWN   → descend     (-Z world, AirSim NED)
 *   error_y < 0  → target is UP     → ascend      (+Z world, AirSim NED)
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

#include <opencv2/opencv.hpp>

#include "AirSimDrone.hpp"

struct PIDController {
    double kp, ki, kd;
    double integral   = 0.0;
    double prev_error = 0.0;
    double integral_max;
    double output_max;

    PIDController(double kp, double ki, double kd,
                  double integral_max = 5.0,
                  double output_max   = 3.0)
        : kp(kp), ki(ki), kd(kd),
          integral_max(integral_max),
          output_max(output_max) {}

    double update(double error, double dt) {
        if (dt <= 0.0) return 0.0;
        integral = std::clamp(integral + error * dt, -integral_max, integral_max);
        double derivative = (error - prev_error) / dt;
        prev_error = error;
        return std::clamp(kp * error + ki * integral + kd * derivative, -output_max, output_max);
    }

    void reset() { integral = 0.0; prev_error = 0.0; }
};

namespace cfg {
    constexpr int   IMG_W        = 1280;
    constexpr int   IMG_H        = 720;
    constexpr float DEAD_ZONE    = 0.02f;
    constexpr int   LOOP_HZ      = 30;
    constexpr float CRUISE_ALT   = -2.0f;
    constexpr float VEL_MAX_YAW  = 30.0f;
    constexpr float VEL_MAX_VERT = 3.0f;
    constexpr float VEL_MAX_LAT  = 4.0f;
    constexpr float VEL_MAX_FWD  = 2.0f;
    constexpr float LOCKED_PX    = 15.0f;
    const std::string AIRSIM_IP  = "127.0.0.1";
    constexpr int   TARGET_CLASS_ID        = 4;
    constexpr float CONFIDENCE_THRESHOLD   = 0.45f;
    constexpr float NMS_THRESHOLD          = 0.4f;
    constexpr float TIMEOUT_TAKEOFF        = 10.f;
    constexpr float TIMEOUT_LAND           = 10.f;
}


std::atomic<bool> g_running{true};
void sig_handler(int) { g_running = false; }


inline float normalise_x(float err_px) { return err_px / (cfg::IMG_W * 0.5f); }
inline float normalise_y(float err_px) { return err_px / (cfg::IMG_H * 0.5f); }

inline cv::Rect detectByClassID(cv::dnn::Net& net, const cv::Mat& frame, int class_id)
{
    if (frame.empty()) return {};

    cv::Mat blob;
    cv::dnn::blobFromImage(frame, blob, 1/255.0, cv::Size(640, 640), cv::Scalar(0,0,0), true, false);
    net.setInput(blob);

    std::vector<cv::Mat> outputs;
    net.forward(outputs, net.getUnconnectedOutLayersNames());

    cv::Mat output = outputs[0];
    output = output.reshape(1, 84);
    cv::transpose(output, output);

    std::vector<cv::Rect>  boxes;
    std::vector<float>     confidences;

    for (int i = 0; i < output.rows; ++i) {
        cv::Mat row    = output.row(i);
        cv::Mat scores = row.colRange(4, 84);
        cv::Point best;
        double score;
        cv::minMaxLoc(scores, nullptr, &score, nullptr, &best);

        if (score > cfg::CONFIDENCE_THRESHOLD && best.x == class_id) {
            float cx = row.at<float>(0), cy = row.at<float>(1);
            float w  = row.at<float>(2), h  = row.at<float>(3);
            int left   = static_cast<int>((cx - 0.5f * w) * (frame.cols / 640.0));
            int top    = static_cast<int>((cy - 0.5f * h) * (frame.rows / 640.0));
            int width  = static_cast<int>(w * (frame.cols / 640.0));
            int height = static_cast<int>(h * (frame.rows / 640.0));
            boxes.push_back({left, top, width, height});
            confidences.push_back(static_cast<float>(score));
        }
    }

    std::vector<int> indices;
    cv::dnn::NMSBoxes(boxes, confidences, cfg::CONFIDENCE_THRESHOLD, cfg::NMS_THRESHOLD, indices);
    return (!indices.empty() && !boxes.empty()) ? boxes[indices[0]] : cv::Rect();
}


int main()
{
    std::signal(SIGINT,  sig_handler);
    std::signal(SIGTERM, sig_handler);

    msr::airlib::MultirotorRpcLibClient client(cfg::AIRSIM_IP);

    try {
        client.confirmConnection();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] confirmConnection: " << e.what() << "\n";
        return 1;
    }

    AirSimDrone interceptorDrone(client, "Interceptor");

    try {
        interceptorDrone.Setup();
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Interceptor Setup: " << e.what() << "\n";
        return 1;
    }

    try {
        interceptorDrone.Takeoff(cfg::TIMEOUT_TAKEOFF, true);
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Interceptor Takeoff: " << e.what() << "\n";
        return 1;
    }

    //interceptorDrone.MoveToPosition(0, 0, -20.f, 6.f, true);

    std::cout << "[INFO] Loading trained model...\n";
    cv::dnn::Net net = cv::dnn::readNetFromONNX("yolo11n/yolo11n.onnx");
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CPU);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_OPENCV);

    std::cout << "[INFO] Starting target tracking loop. Ctrl-C to stop.\n";

    const double dt_target = 1.0 / cfg::LOOP_HZ;

    auto interceptor_behaviour = [
        &client, &net,
        pid_lateral  = PIDController{2.5,  0.05, 0.4,   4.0, cfg::VEL_MAX_LAT},
        pid_vertical = PIDController{2.0,  0.04, 0.35,  4.0, cfg::VEL_MAX_VERT},
        pid_yaw      = PIDController{18.0, 0.2,  2.0,  20.0, cfg::VEL_MAX_YAW},
        dt_target,
        t_prev          = std::chrono::steady_clock::now(),
        last_seek       = std::chrono::steady_clock::now(),
        seek_right      = true,
        last_cx         = 0.0,
        last_cy         = 0.0,
        target_acquired = false
    ](AirSimDrone& self) mutable
    {
        auto t_now = std::chrono::steady_clock::now();
        double dt  = std::chrono::duration<double>(t_now - t_prev).count();
        t_prev     = t_now;

        const std::vector<msr::airlib::ImageCaptureBase::ImageRequest> request = {
            msr::airlib::ImageCaptureBase::ImageRequest(
                "front_center",
                msr::airlib::ImageCaptureBase::ImageType::Scene,
                false, false)
        };
        std::vector<msr::airlib::ImageCaptureBase::ImageResponse> response;
        try {
            response = client.simGetImages(request);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] simGetImages: " << e.what() << "\n";
            g_running = false;
            return;
        }
        if (response.empty()) return;

        cv::Mat frame(response[0].height, response[0].width,
                      CV_8UC3, (void*)response[0].image_data_uint8.data());

        cv::Rect box = detectByClassID(net, frame, cfg::TARGET_CLASS_ID);

        if (!box.empty()) {
            last_cx         = box.x + box.width  / 2.0;
            last_cy         = box.y + box.height / 2.0;
            target_acquired = true;
        } else if (!target_acquired) {
            cv::imshow("Interceptor", frame);
            cv::waitKey(1);
            return;
        }

        double cx = last_cx;
        double cy = last_cy;
        double error_x = cx - frame.cols / 2.0;
        double error_y = cy - frame.rows / 2.0;

        float norm_x = normalise_x(static_cast<float>(error_x));
        float norm_y = normalise_y(static_cast<float>(error_y));
        if (std::abs(norm_x) < cfg::DEAD_ZONE) { norm_x = 0.0f; pid_lateral.reset(); pid_yaw.reset(); }
        if (std::abs(norm_y) < cfg::DEAD_ZONE) { norm_y = 0.0f; pid_vertical.reset(); }

        float vel_y    = static_cast<float>(pid_lateral .update(norm_x, dt));
        float vel_z    = static_cast<float>(pid_vertical.update(norm_y, dt));
        float yaw_rate = static_cast<float>(pid_yaw     .update(norm_x, dt * 2.0));
        float vel_x    = std::clamp(cfg::VEL_MAX_FWD * 10.f, -cfg::VEL_MAX_FWD, cfg::VEL_MAX_FWD);
        vel_y = std::clamp(vel_y, -cfg::VEL_MAX_LAT,  cfg::VEL_MAX_LAT);
        vel_z = std::clamp(vel_z, -cfg::VEL_MAX_VERT, cfg::VEL_MAX_VERT);

        try {
            self.MoveByVelocityBodyFrame(vel_x, vel_y, vel_z,
                                         static_cast<float>(dt_target * 2.0), yaw_rate);
        } catch (const std::exception& e) {
            std::cerr << "[ERROR] MoveByVelocityBodyFrame: " << e.what() << "\n";
            g_running = false;
            return;
        }

        bool locked = std::abs(error_x) < cfg::LOCKED_PX && std::abs(error_y) < cfg::LOCKED_PX;
        const char* state = !box.empty() ? (locked ? "LOCKED  " : "TRACKING") : "LAST POS";
        std::printf("\n[%s] err=(%+6.1f,%+6.1f)px  vel=(%+.2f,%+.2f,%+.2f)m/s  yaw=%+.1f°/s",
                    state,
                    error_x, error_y, vel_x, vel_y, vel_z, yaw_rate);

        cv::circle(frame, cv::Point(static_cast<int>(cx), static_cast<int>(cy)),
                   10, cv::Scalar(0, 255, 0), 2);
        cv::putText(frame,
                    "LOCK: [" + std::to_string(error_x) + ", " + std::to_string(error_y) + "]",
                    cv::Point(static_cast<int>(cx) + 20, static_cast<int>(cy) - 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5f, cv::Scalar(255, 0, 0));
        cv::imshow("Interceptor", frame);
        cv::waitKey(1);

        auto elapsed  = std::chrono::steady_clock::now() - t_now;
        auto sleep_us = std::chrono::microseconds(static_cast<long>(dt_target * 1e6)) - elapsed;
        if (sleep_us.count() > 0) std::this_thread::sleep_for(sleep_us);
    };

    while (g_running) {
        interceptorDrone.Update(interceptor_behaviour);
    }

    std::cout << "\n[INFO] Shutting down...\n";

    try { interceptorDrone.Land(cfg::TIMEOUT_LAND, true); } catch (const std::exception& e) { std::cerr << "[ERROR] Interceptor Land: " << e.what() << "\n"; }

    try {
        client.armDisarm(false, "Interceptor");
        client.enableApiControl(false, "Interceptor");
    } catch (const std::exception& e) {
        std::cerr << "[ERROR] Disarm/disable: " << e.what() << "\n";
    }

    return 0;
}
