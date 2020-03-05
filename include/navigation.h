#pragma once

#include <unordered_map>
#include <array>
#include "boxes.h"
#include "robot_pose.h"

#define OFFSET 2

class Navigation
{
 private:
    RobotPose rob_pose;
    std::unordered_map<std::vector<float>, bool> obj_state; // state of objectives (unvisited: FALSE, visited: TRUE)
    std::vector<std::vector<float>> obj_unvisited;
    std::vector<std::array<float, 2>> obj_opt_seq;          // optimal objective sequence
 public:
    static bool move_to_goal(float goal_x, float goal_y, float goal_phi);
    void compute_opt_seq();
    Navigation(ros::NodeHandle& nh, const Boxes& boxes);
};
