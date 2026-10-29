#pragma once
#include <string>
#include <vector>
#include <memory>
#include <limits>
#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"

class AirSimDrone {
protected:
    msr::airlib::MultirotorRpcLibClient& client_;
    std::string vehicle_name_;

public:
    AirSimDrone(msr::airlib::MultirotorRpcLibClient& client, std::string vehicle_name); 

    virtual ~AirSimDrone() = default;

    void Setup();

    void Takeoff(float timeout, bool wait);

    void Land(float timeout, bool wait);

    msr::airlib::MultirotorState GetState() const;

    void MoveToPosition(float x, float y, float z, float velocity, bool wait = true);

    void MoveByVelocityBodyFrame(float vx, float vy, float vz, float duration, float yaw_rate, bool wait = false);

    void MoveOnPath(const std::vector<msr::airlib::Vector3r>& path, float velocity, bool wait);

    void RotateToYaw(float timeout, float yaw, bool wait = true);

    std::string GetName() const;

    void Update(std::function<void(AirSimDrone&)> behavior);
};