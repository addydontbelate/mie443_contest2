#include "boxes.h"
#include "navigation.h"
#include "robot_pose.h"
#include "image_pipeline.h"
#include "logger.h"

// strategy definitions
#define RESULT_DIR "/home/turtlebot/Desktop/"
#define RESULT_FILENAME "contest2_log.txt"
#define SUCCESS true
#define FAILURE false
#define NUM_REPLANS 2
#define TIME_LIMIT 300 // 5 minutes

// global timer
uint64_t seconds_elapsed = 0.0;
TIME rob_start;

// helper function prototypes
bool classify_obj(ImagePipeline&, Boxes&, const std::vector<float>&, int, Logger&);

// environment tags tracker
std::unordered_map<int, int> classify_obj_id;

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
    Logger logger;
    logger.open(std::string(RESULT_DIR) + std::string(RESULT_FILENAME));
    
    ros::Rate loop_rate(10); // run at 10 Hz
    rob_start = CLOCK::now();
    
    // step 0: localize
    ros::spinOnce();
    nav.localize();

    // keep on executing strategy till there are no unvisited (and un-processed) objectives
    while(ros::ok() && nav.any_unvisited_obj() && seconds_elapsed < TIME_LIMIT)
    {
        ros::spinOnce();

        // step 1: compute optimal sequence
        nav.compute_opt_seq();
        auto box_seq = nav.get_goal_seq();

        // step 2: visit all unvisited objectives in the optimal sequence
        for (int obj_idx = 0; obj_idx < box_seq.size()-1; obj_idx++)
        {
            ros::spinOnce();
            int box_idx = nav.get_obj_ID(obj_idx);
            ROS_INFO("[MAIN] Moving to Box %d", box_idx);

            if (nav.move_to_goal(box_seq[obj_idx][0], box_seq[obj_idx][1], box_seq[obj_idx][2]) == SUCCESS)
            {
                if (classify_obj(img_pipeline, boxes, boxes.coords[box_idx], box_idx, logger))
                    nav.set_obj_visited(obj_idx);
            }
            else
            {
                int num_tries = 0;

                // try re-planning; if still fail, skip and move to next item in sequence
                while (num_tries < NUM_REPLANS)
                {
                    ROS_ERROR("[MAIN] move_to_goal() failed! Trying to replan!");

                    // iteratively try moving closer to the box by 0.1m in each re-plan
                    float phi = box_seq[obj_idx][3];
                    float x = box_seq[obj_idx][0] - (num_tries+1)*0.1*cos(phi);
                    float y = box_seq[obj_idx][1] - (num_tries+1)*0.1*sin(phi); 

                    if (nav.move_to_goal(x, y, phi) == SUCCESS)
                    {
                        if (classify_obj(img_pipeline, boxes, box_seq[obj_idx], box_idx, logger))
                        {
                            nav.set_obj_visited(obj_idx);
                            break;
                        }
                    }
                    else    
                        num_tries++;
                }
            }

            loop_rate.sleep();
        }

        loop_rate.sleep();
    }

    // step 3: once done, go back to the initial position
    RobotPose init_pose = nav.get_init_pose();
    if (nav.move_to_goal(init_pose.x, init_pose.y, init_pose.phi))
    {
        ROS_INFO("[MAIN] Done!");
        return EXIT_SUCCESS;
    }

    ROS_INFO("[MAIN] Failed to reach the initial position!");
    return EXIT_FAILURE;
}

bool classify_obj(ImagePipeline& img_pipeline, Boxes& boxes, const std::vector<float>& box, int box_idx, Logger& logger)
{
    // take a picture and get template ID; skip process if invalid ID received.
    int ID = img_pipeline.get_template_ID(boxes);
    if (ID == -1)
        return false;
    classify_obj_id[ID]++;

    // print box details to log file
    logger.write("Box: " + std::to_string(box_idx+1));
    logger.write("At location ("+ std::to_string(box[0]) + ", " + std::to_string(box[1]) + 
        ", " + std::to_string(box[2]) + ")");

    // print template classification results to log file
    if (ID == 0)
        logger.write("Nothing was found\n");
    else if (classify_obj_id[ID] == 1)
        logger.write(TEMPLATE_NAME[ID] + " was found\n");
    else
        logger.write(TEMPLATE_NAME[ID] + " was found again (duplicate)\n");

    return true;
}