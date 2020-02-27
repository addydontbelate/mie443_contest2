#include "robot.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

global bool obj_visited[boxes.coords.size()] = {false};
global int obj_order[boxes.coords.size()] = {0};

bool Robot::move_to_goal(float goal_x, float goal_y, float goal_phi)
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

void Robot::prioritize()
{
    // Count the number of unvisited objectives
    int obj_remaining = 0;
    for(int ctr = 0; ctr < obj_visited.size; ctr++)
    {
        if(obj_visited[ctr] == false) {obj_remaining++;}
    }

    // Get the x and y positions of the robot and the unvisited objectives
    float obj_positions[1 + obj_remaining][2] = {0};
    for(int ctr = 0; ctr < obj_positions.size(); ctr++)
    {
        if(ctr == 0)
        {
            obj_positions[ctr][0] = robotPose.x;
            obj_positions[ctr][1] = robotPose.y;
        }
        else
        {
            if(obj_visited[ctr - 1] == false)
            {
                obj_positions[ctr][0] = boxes.coords[ctr - 1][0];
                obj_positions[ctr][1] = boxes.coords[ctr - 1][1];
            }
        }
    }

    // Compute the distances between each location
    float obj_distances[1 + obj_remaining][1 + obj_remaining] = {0};
    for(int i = 0; i < )
}
