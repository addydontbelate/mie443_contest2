#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <chrono>

// timer macros
#define TIME std::chrono::time_point<std::chrono::system_clock>
#define CLOCK std::chrono::system_clock
#define TIME_S std::chrono::duration_cast<std::chrono::seconds>
#define TIME_US std::chrono::duration_cast<std::chrono::microseconds>

// global time keeper (integrated into pose_callback)
extern uint64_t seconds_elapsed;
extern TIME rob_start;

class RobotPose
{
 public:
    float x;
    float y;
    float phi;
    RobotPose() { x = 0.0; y = 0.0; phi = 0.0; };
    RobotPose(float x, float y, float phi);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
    bool operator==(const RobotPose& rhs) const;
};
