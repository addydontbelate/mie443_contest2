#pragma once
#include <robot_pose.h>
#include <Robot.h>

class Robot
{
 public:
    
    // Properties
    RobotPose pose = RobotPose(0.0, 0.0, 0.0);

    // Methods 
    static bool move_to_goal(float goal_x, float goal_y, float goal_phi);

    // Init/destructor
    Robot();
};
