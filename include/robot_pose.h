#pragma once

#include <geometry_msgs/PoseWithCovarianceStamped.h>

class RobotPose
{
 public:
    float x;
    float y;
    float phi;

    RobotPose(float x, float y, float phi);
    void pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg);
};
