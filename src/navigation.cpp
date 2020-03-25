#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>
#include <algorithm>
#include <numeric>
#include <cmath>
#include "navigation.h"

Navigation::Navigation(ros::NodeHandle& nh, const Boxes& boxes)
{
    // initialize unvisited objectives and states
    int box_idx = 0;
    for (const auto& box : boxes.coords)
    {
        obj_state.insert(std::make_pair(box, UNVISITED));
        obj_box_idx.insert(std::make_pair(box, box_idx++));
        obj_unvisited.push_back(box);
    }
    
    // initial robot pose + subscriber
    amcl_sub = nh.subscribe("/amcl_pose", 1, &RobotPose::pose_callback, &rob_pose);
}

void Navigation::localize()
{
    ros::Rate loop_rate(10); // run at 10 Hz
    // store the initial position after being updated by amcl
    while (init_rob_pose == rob_pose && init_rob_pose == RobotPose(0.0,0.0,0.0)) 
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    init_rob_pose = rob_pose;
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
    // step 0: clear any previous results
    obj_opt_seq.clear();
    goal_opt_seq.clear();

    // step 1: compute distances between all objectives (including the start position)
    std::vector<std::vector<double>> dist_bw_obj(obj_unvisited.size() + 1, std::vector<double>(obj_unvisited.size() + 1, 0.0));
    std::vector<std::vector<float>> compute_obj {{rob_pose.x, rob_pose.y, rob_pose.phi}};       // starts with current position
    compute_obj.insert(compute_obj.end(), obj_unvisited.begin(), obj_unvisited.end());
    compute_obj.push_back({init_rob_pose.x, init_rob_pose.y, init_rob_pose.phi});               // ends with ending position (init_rob_pose)

    for(int i = 0; i < dist_bw_obj.size(); i++) 
    {
        for(int j = i+1; j < dist_bw_obj.size(); j++) 
        {
            float x_diff = compute_obj[i][0] - compute_obj[j][0];
            float y_diff = compute_obj[i][1] - compute_obj[j][1];
            dist_bw_obj[i][j] = sqrt(pow(x_diff, 2) + pow(y_diff, 2));
            dist_bw_obj[j][i] = dist_bw_obj[i][j];
        }
    }

    // step 2: compute distances for all permutations of objective orders (N!) and get optimal order
    std::vector<int> unvisited_obj_idx(obj_unvisited.size());
    std::iota(unvisited_obj_idx.begin(), unvisited_obj_idx.end(), 0); // fill with increasing indices starting at 0
    std::vector<std::vector<int>> univisited_obj_perm;
    std::vector<float> perm_cycle_dist;

    // compute all permutations
    do 
    {
        univisited_obj_perm.push_back(unvisited_obj_idx);
    } while (std::next_permutation(unvisited_obj_idx.begin(), unvisited_obj_idx.end()));

    // compute complete cycle distances for all permuations
    for(const auto& perm : univisited_obj_perm) 
    {
        float total_dist = 0.0;
        // add dist from current position to first objective
        total_dist += dist_bw_obj[0][perm[0]+1]; // offset idx by 1 (matrix starts w curr pos) 

        // add dist from one objective to another in the permutation order
        for(int i = 0; i < perm.size()-1; i++)
            total_dist += dist_bw_obj[perm[i]+1][perm[i+1]+1];

        // add dist from last objective to initial position (init_rob_pose) to complete the cycle
        total_dist += dist_bw_obj[perm.back()+1][dist_bw_obj.size()-1];
        
        // insert total dist in permulation cycle distances vector
        perm_cycle_dist.push_back(total_dist);
    }

    // get minimum permutation with minimum dist
    auto min_dist_it = std::min_element(perm_cycle_dist.begin(), perm_cycle_dist.end());
    int opt_perm_idx = min_dist_it - perm_cycle_dist.begin(); // optimal objective order
    
    // print optimal order
    std::string opt_order;
    for (int i = 0; i < univisited_obj_perm[opt_perm_idx].size()-1; i++) 
        opt_order += std::to_string(univisited_obj_perm[opt_perm_idx][i]) + ", ";
    opt_order += std::to_string(univisited_obj_perm[opt_perm_idx].back());
    ROS_INFO("[NAV] Optimal ojective (box) order is (%s)", opt_order.c_str());
    
    // step 3: create obj_opt_seq from the optimal objective order
    for (const int& obj_idx : univisited_obj_perm[opt_perm_idx])
        obj_opt_seq.push_back(obj_unvisited[obj_idx]);
    obj_opt_seq.push_back({init_rob_pose.x, init_rob_pose.y, init_rob_pose.phi});

    // step 4: having computed obj_opt_seq, set real move_to coordinates for the robot with offset
    // note that we are ending one idx before since the last index is init_rob_pose 
    for (int i = 0; i < obj_opt_seq.size() - 1; i++)
    {
        std::vector<float> goal_pose(3, 0.0);
        float phi = obj_opt_seq[i][2];
        goal_pose[0] = obj_opt_seq[i][0] + cos(phi)*IMG_CAPTURE_OFFSET;
        goal_pose[1] = obj_opt_seq[i][1] + sin(phi)*IMG_CAPTURE_OFFSET;
        goal_pose[2] = phi - M_PI;
        goal_opt_seq.push_back(goal_pose);
    }
    goal_opt_seq.push_back(obj_opt_seq.back());
}

void Navigation::set_obj_visited(int obj_opt_seq_idx)
{
    // get the corresponding obj coordinate from idx
    std::vector<float> obj;
    if (obj_opt_seq_idx >= 0 && obj_opt_seq_idx < obj_opt_seq.size())
        obj = obj_opt_seq.at(obj_opt_seq_idx);
    else 
    {
        ROS_ERROR("[NAV] Invalid object index passed; unable to update state to VISITED!");
        return;   
    }

    // remove from univisted objectives
    auto erase_it = std::find(obj_unvisited.begin(), obj_unvisited.end(), obj);
    obj_unvisited.erase(erase_it);

    // set VISITED state in objective state
    obj_state[obj] = VISITED;
}

std::vector<std::vector<float>> Navigation::get_goal_seq()
{
    return goal_opt_seq;
}

bool Navigation::any_unvisited_obj()
{
    return !(obj_unvisited.empty());
}

int Navigation::get_obj_ID(const int obj_idx)
{
    if (obj_box_idx.find(obj_opt_seq[obj_idx]) != obj_box_idx.end())
        return obj_box_idx[obj_opt_seq[obj_idx]];
    
    // else
    ROS_ERROR("[NAV] Not able to find the objective ID corresponding to the given coordinates!");
    return -1;
}

RobotPose Navigation::get_init_pose()
{
    return init_rob_pose;
}