#pragma once

namespace msr::airlib {
    class MultirotorRpcLibClient;
}

namespace cv {
    class Moments;
}

class PIDController final {
    float P_gain_yaw;
    float P_gain_pitch;
    float P_forward_speed;

public:
    PIDController(float gain_yaw, float gain_pitch, float gain_speed);
    void InitiateControl(float error_x, float error_y, cv::Moments&& m, msr::airlib::MultirotorRpcLibClient &client) const;
}; 