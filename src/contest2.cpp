#include "boxes.h"
#include "navigation.h"
#include "robot_pose.h"
#include "image_pipeline.h"
#include "logger.h"

int main(int argc, char** argv)
{
    // setup ROS
    ros::init(argc, argv, "contest2");
    ros::NodeHandle nh;

    // initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates())
    {
        ROS_ERROR("[MAIN] Could not load coords or templates! Exiting...");
        exit(EXIT_FAILURE);
    }

    for(int i = 0; i < boxes.coords.size(); i++)
        ROS_INFO("[Main] Box %d coordinates (x: %f, y: %f, z: %f)", i, 
            boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);

    // initialize image pipeline and navigation objects
    Navigation nav(nh, boxes);
    ImagePipeline img_pipeline(nh, boxes);
    
    ros::Rate loop_rate(0.065); // run every ~15s

    while(ros::ok())
    {
        ros::spinOnce();
        img_pipeline.get_template_ID(boxes);
        loop_rate.sleep();
    }

    exit(EXIT_SUCCESS);
}