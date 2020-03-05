#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <opencv2/core/mat.hpp>
#include <algorithm>
#include <math.h>
#include "navigation.h"

Navigation::Navigation(ros::NodeHandle& nh, const Boxes& boxes)
{
    // initialize unvisited objectives and states
    for (const auto& box : boxes)
    {
        obj_state.insert(box.coords, false);
        obj_unvisited.push_back(box.coords);
    }
    
    // robot pose object + subscriber
    ros::Subscriber amcl_sub = nh.subscribe("/amcl_pose", 1, &RobotPose::pose_callback, &rob_pose);
}

bool Navigation::move_to_goal(float goal_x, float goal_y, float goal_phi)
{
	// set up and wait for actionClient
    actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> ac("move_base", true);
    while(!ac.waitForServer(ros::Duration(5.0)))
    {
        ROS_INFO("Waiting for the move_base action server to come up");
    }

	// set goal
    geometry_msgs::Quaternion phi = tf::createQuaternionMsgFromYaw(goal_phi);
    move_base_msgs::MoveBaseGoal goal;
    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();
    goal.target_pose.pose.position.x =  goal_x;
    goal.target_pose.pose.position.y =  goal_y;
    goal.target_pose.pose.position.z =  0.0;
    goal.target_pose.pose.orientation.x = 0.0;
    goal.target_pose.pose.orientation.y = 0.0;
    goal.target_pose.pose.orientation.z = phi.z;
    goal.target_pose.pose.orientation.w = phi.w;
    ROS_INFO("Sending goal location ...");

    // Send goal and wait for response.
    ac.sendGoal(goal);
    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    {
        ROS_INFO("You have reached the destination");
        return true;
    }
    else
    {
        ROS_INFO("The robot failed to reach the destination");
        return false;
    }
}


void Navigation::compute_opt_seq()
{
    // step 1: compute distances between all objectives (including the start position)
    cv::Mat dist_bw_obj(obj_unvisited.size() + 1, obj_unvisited.size() + 1, double);
    std::vector<std::vector<float>> compute_obj {{rob_pose.x, rob_pose.y}}; // starts with current position
    compute_obj.insert(compute_obj.end(), obj_unvisited.begin(), obj_unvisited.end());

    float x_diff = 0.0;
    float y_diff = 0.0;
    for(int i = 0; i < dist_bw_obj.rows; i++) 
    {
        for(int j = 0; j < dist_bw_obj.cols; j++) 
        {
            x_diff = compute_obj[i][0] - compute_obj[j][0];
            y_diff = compute_obj[i][1] - compute_obj[j][1];
            dist_bw_obj[i][j] = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
        }
    }

    // step 2: compute permutations for potentional objective orders
    std::vector<int> obj_unvisited_idx(obj_unvisited.size());
    std::iota(std::begin(obj_unvisited_idx), std::end(obj_unvisited_idx), 0); // fill with increasing indices starting at 0

    

}
