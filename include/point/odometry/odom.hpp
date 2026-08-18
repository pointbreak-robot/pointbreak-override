#pragma once

#include "pose.hpp"
#include "imu.hpp"
#include "trackingWheel.hpp"
#include "distance.hpp"
#include "exitCondition.hpp"
#include "PID.hpp"

#include "pros/rtos.hpp"

#include <functional>
#include <optional>

namespace point {

struct OdomConfig {
    TrackingWheel*  trackingWheel     = nullptr;
    IMU*            imu              = nullptr;
    DistOdom*       distanceSensor   = nullptr;
    float           imuScale         = 1.0f;
};

class Odometry {
public:
    explicit Odometry(OdomConfig config);

    void reset(Pose initialPose = {});
    void startTask(int updateIntervalMs = 10);
    void stopTask();
    void update();

    Pose  getPose();
    float getHeading();
    float getX();
    float getY();

    void setPose(Pose pose);
    void setX(float x);
    void setY(float y);
    void setHeading(float degrees);

    void onUpdate(std::function<void(Pose)> cb);
    bool sensorsOk();

private:
    OdomConfig  config_;
    Pose        pose_;
    float       prevHeading_;
    pros::Mutex mutex_;
    pros::Task* task_;
    int         updateIntervalMs_;

    std::optional<std::function<void(Pose)>> updateCallback_;

    void  integrateArc(float deltaLeft, float deltaRight, float deltaTheta);
    float resolveHeading(float encoderDeltaTheta);

    static void  taskFn(void* param);
    static float toRad(float deg) { return deg * (3.14159265f / 180.0f); }
};

} // namespace point