#include "PIDController.hpp"
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "vehicles/multirotor/api/MultirotorCommon.hpp"
#include <opencv2/opencv.hpp>

PIDController::PIDController(float gain_yaw, float gain_pitch, float gain_speed)
    : P_gain_yaw(gain_yaw), P_gain_pitch(gain_pitch), P_forward_speed(gain_speed)
{

}

void PIDController::InitiateControl(float error_x, float error_y, cv::Moments &&m, msr::airlib::MultirotorRpcLibClient &client) const
{
    if (m.m00 > 100) {
        int cx = static_cast<int>(m.m10 / m.m00);
        int cy = static_cast<int>(m.m01 / m.m00);

        float yaw_rate = error_x * P_gain_yaw;
        if (std::abs(yaw_rate) < 1.0f) {
            std::cout << "Yaw rate < 1, not sending \n";    
        }
 
        float vx = P_forward_speed - (error_y * P_gain_pitch); 
        
        client.moveByVelocityAsync(vx, 0, 0, yaw_rate, msr::airlib::DrivetrainType::ForwardOnly, msr::airlib::YawMode(true, 0));
    } else {
        client.moveByVelocityAsync(0, 0, 0, 0);
    }
}