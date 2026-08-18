#include "main.h"
#include <cmath>

using namespace point;

Odometry::Odometry(OdomConfig config)
    : config_(config), pose_({}), prevHeading_(0.0f),
      task_(nullptr), updateIntervalMs_(10) {}

void Odometry::reset(Pose initialPose) {
    mutex_.take(TIMEOUT_MAX);
    pose_        = initialPose;
    prevHeading_ = initialPose.theta;
    mutex_.give();

    if (config_.trackingWheel && config_.trackingWheel->isConnected()) config_.trackingWheel->reset();
    if (config_.imu        && config_.imu->isConnected())        config_.imu->tare();
}

void Odometry::startTask(int updateIntervalMs) {
    if (task_) return;
    updateIntervalMs_ = updateIntervalMs;
    task_ = new pros::Task(taskFn, this, "odom");
}

void Odometry::stopTask() {
    if (task_) {
        task_->remove();
        delete task_;
        task_ = nullptr;
    }
}

void Odometry::update() {
    float deltaTracking = config_.trackingWheel ? config_.trackingWheel->getDeltaDistance() : 0.0f;

    float encoderDeltaTheta = 0.0f;
    // With one tracking wheel, rely on IMU for heading when available.

    float deltaTheta = resolveHeading(encoderDeltaTheta);
    integrateArc(deltaTracking, deltaTracking, deltaTheta);

    if (config_.distanceSensor) {
        pose_ = config_.distanceSensor->update(pose_);
    }

    if (updateCallback_) (*updateCallback_)(pose_);
}

Pose  Odometry::getPose()    { mutex_.take(TIMEOUT_MAX); Pose p = pose_;        mutex_.give(); return p; }
float Odometry::getHeading() { mutex_.take(TIMEOUT_MAX); float v = pose_.theta; mutex_.give(); return v; }
float Odometry::getX()       { mutex_.take(TIMEOUT_MAX); float v = pose_.x;     mutex_.give(); return v; }
float Odometry::getY()       { mutex_.take(TIMEOUT_MAX); float v = pose_.y;     mutex_.give(); return v; }

void Odometry::setPose(Pose pose)        { mutex_.take(TIMEOUT_MAX); pose_ = pose;          mutex_.give(); }
void Odometry::setX(float x)            { mutex_.take(TIMEOUT_MAX); pose_.x = x;           mutex_.give(); }
void Odometry::setY(float y)            { mutex_.take(TIMEOUT_MAX); pose_.y = y;           mutex_.give(); }
void Odometry::setHeading(float degrees) { mutex_.take(TIMEOUT_MAX); pose_.theta = degrees; mutex_.give(); }

void Odometry::onUpdate(std::function<void(Pose)> cb) {
    updateCallback_ = cb;
}

bool Odometry::sensorsOk() {
    if (config_.trackingWheel  && !config_.trackingWheel->isConnected())  return false;
    if (config_.imu            && !config_.imu->isConnected())            return false;
    if (config_.distanceSensor && !config_.distanceSensor->isConnected()) return false;
    return true;
}

void Odometry::integrateArc(float deltaLeft, float deltaRight, float deltaTheta) {
    float deltaS = (deltaLeft + deltaRight) / 2.0f;

    float localX, localY;
    if (std::abs(deltaTheta) < 1e-6f) {
        localX = 0.0f;
        localY = deltaS;
    } else {
        localX = 0.0f;
        localY = 2.0f * std::sin(deltaTheta / 2.0f) * (deltaS / deltaTheta);
    }

    float avgTheta = toRad(pose_.theta) + deltaTheta / 2.0f;
    pose_.x     += localX * std::cos(avgTheta) - localY * std::sin(avgTheta);
    pose_.y     += localX * std::sin(avgTheta) + localY * std::cos(avgTheta);
    pose_.theta += deltaTheta * (180.0f / 3.14159265f);
}

float Odometry::resolveHeading(float encoderDeltaTheta) {
    if (config_.imu && config_.imu->isConnected()) {
        float imuHeading = (float)(config_.imu->getHeading() * config_.imuScale);

        float delta = imuHeading - prevHeading_;

        if (delta >  180.0f) delta -= 360.0f;
        if (delta < -180.0f) delta += 360.0f;

        prevHeading_ = imuHeading;
        return toRad(delta);
    }
    return encoderDeltaTheta;
}

void Odometry::taskFn(void* param) {
    Odometry* self = static_cast<Odometry*>(param);
    while (true) {
        self->update();
        pros::delay(self->updateIntervalMs_);
    }
}