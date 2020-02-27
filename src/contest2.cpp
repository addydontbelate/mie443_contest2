#include "boxes.h"
#include "navigation.h"
#include "robot_pose.h"
#include "image_pipeline.h"

int main(int argc, char** argv)
{
    // setup ROS
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // robot pose object + subscriber
    RobotPose robot_pose(0,0,0);
    ros::Subscriber amcl_sub = n.subscribe("/amcl_pose", 1, &RobotPose::pose_callback, &robot_pose);

    // initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates())
    {
        ROS_ERROR("[MAIN] Could not load coords or templates! Exiting...");
        exit(EXIT_FAILURE);
    }
    for(int i = 0; i < boxes.coords.size(); ++i)
    {
        ROS_INFO("[Main] Box %d coordinates (x: %f, y: %f, z: %f)", i,
                boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);
    }

    // initialize image object and subscriber
    ImagePipeline img_pipeline(n, boxes);

    // execute strategy
    while(ros::ok())
    {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robot_pose.x, robot_pose.y, robot_pose.phi
        img_pipeline.get_template_ID(boxes);
        ros::Duration(5).sleep();
    }

    exit(EXIT_SUCCESS);
}
