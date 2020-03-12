#pragma once

#include <unordered_map>
#include <map>
#include <vector>
#include "boxes.h"
#include "robot_pose.h"

#define IMG_CAPTURE_OFFSET 0.55
#define VISITED true
#define UNVISITED false

class Navigation
{
 private:
    ros::Subscriber amcl_sub;
    RobotPose rob_pose;
    RobotPose init_rob_pose;
    std::vector<std::vector<float>> obj_unvisited; 
    std::vector<std::vector<float>> obj_opt_seq;      // optimal objective sequence
    std::vector<std::vector<float>> goal_opt_seq;     // optimal goal sequence (objectives with offset)
    std::map<std::vector<float>, bool> obj_state;     // state of objectives (UNVISITED/VISITED)
    std::map<std::vector<float>, int> obj_box_idx;   // corresponding box ids for objectives

 public:
    void localize();
    static bool move_to_goal(float goal_x, float goal_y, float goal_phi);
    void compute_opt_seq();
    std::vector<std::vector<float>> get_goal_seq();
    int get_obj_ID(int obj_idx);
    void set_obj_visited(int obj_opt_seq_idx);
    bool any_unvisited_obj(); 
    Navigation(ros::NodeHandle& nh, const Boxes& boxes);
};
