#include <boxes.h>
#include <Robot.h>
#include <robot_pose.h>
#include <image_pipeline.h>

int main(int argc, char** argv)
{
    // setup ROS.
    ros::init(argc, argv, "contest2");
    ros::NodeHandle n;

    // robot pose object + subscriber.
    RobotPose robotPose(0,0,0);
    ros::Subscriber amclSub = n.subscribe("/amcl_pose", 1, &RobotPose::pose_callback, &robotPose);

    // initialize box coordinates and templates
    Boxes boxes; 
    if(!boxes.load_coords() || !boxes.load_templates())
    {
        std::cout << "ERROR: could not load coords or templates" << std::endl;
        return -1;
    }
    for(int i = 0; i < boxes.coords.size(); ++i)
    {
        std::cout << "Box coordinates: " << std::endl;
        std::cout << i << " x: " << boxes.coords[i][0] << " y: " << boxes.coords[i][1] << " z: " 
                  << boxes.coords[i][2] << std::endl;
    }

    // initialize image object and subscriber
    ImagePipeline imagePipeline(n);

    // execute strategy
    while(ros::ok())
    {
        ros::spinOnce();
        /***YOUR CODE HERE***/
        // Use: boxes.coords
        // Use: robotPose.x, robotPose.y, robotPose.phi
        imagePipeline.get_template_ID(boxes);
        ros::Duration(0.01).sleep();
    }

    return 0;
}
