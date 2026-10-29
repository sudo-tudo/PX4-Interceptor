#include "AirSimDrone.hpp"

AirSimDrone::AirSimDrone(msr::airlib::MultirotorRpcLibClient& client, std::string vehicle_name) 
    : client_(client), vehicle_name_(std::move(vehicle_name)) {

    }

void AirSimDrone::Setup() {
    client_.enableApiControl(true, vehicle_name_);
    client_.armDisarm(true, vehicle_name_);
}

void AirSimDrone::Takeoff(float timeout, bool wait) {
    auto task = client_.takeoffAsync(timeout, vehicle_name_);
    if (wait) task->waitOnLastTask();
}

void AirSimDrone::Land(float timeout, bool wait) {
    auto task = client_.landAsync(timeout, vehicle_name_);
    if (wait) task->waitOnLastTask();
}

msr::airlib::MultirotorState AirSimDrone::GetState() const {
    return client_.getMultirotorState(vehicle_name_);
}

void AirSimDrone::MoveToPosition(float x, float y, float z, float velocity, bool wait)
{
    auto task = client_.moveToPositionAsync(
        x, y, z, velocity,
        std::numeric_limits<float>::max(),
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
        msr::airlib::YawMode(), -1.0f, 1.0f,
        vehicle_name_);
    if (wait) task->waitOnLastTask();
}

void AirSimDrone::MoveByVelocityBodyFrame(float vx, float vy, float vz, float duration, float yaw_rate, bool wait)
{
    auto task = client_.moveByVelocityBodyFrameAsync(
        vx, vy, vz, duration,
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom,
        msr::airlib::YawMode(true, yaw_rate),
        vehicle_name_);
    if (wait) task->waitOnLastTask();
}

void AirSimDrone::MoveOnPath(const std::vector<msr::airlib::Vector3r>& path, float velocity, bool wait) {
    auto task = client_.moveOnPathAsync(
        path, velocity, 
        std::numeric_limits<float>::max(), 
        msr::airlib::DrivetrainType::MaxDegreeOfFreedom, 
        msr::airlib::YawMode(), -1.0f, 1.0f, 
        vehicle_name_
    );
    if (wait) task->waitOnLastTask();
}

void AirSimDrone::RotateToYaw(float timeout, float yaw, bool wait)
{
    auto task = client_.rotateToYawAsync(yaw, std::numeric_limits<float>::max(), timeout, vehicle_name_);
    if (wait) task->waitOnLastTask();
}

std::string AirSimDrone::GetName() const {
    return vehicle_name_; 
}

void AirSimDrone::Update(std::function<void(AirSimDrone&)> behavior)
{
    if (behavior) {
        behavior(*this);
    }
}
