#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotPose
{
 public:
    float x;
    float y;
    float phi;
    RobotPose() { x = 0; y = 0; phi = 0; };
    RobotPose(float x, float y, float phi);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};
