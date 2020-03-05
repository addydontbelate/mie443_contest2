#include "boxes.h"
#include "navigation.h"
#include "robot_pose.h"
#include "image_pipeline.h"
#include "logger.h"

// strategy definitions
#define RESULT_FILEPATH "/home/thursday/Desktop/contest2_run.txt"
#define SUCCESS true
#define FAILURE false
#define NUM_REPLANS 2

// helper function prototypes
void classify_obj(ImagePipeline&, Boxes&, const std::vector<float>&, int, Logger&);

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

    for(int i = 0; i < boxes.coords.size(); ++i)
        ROS_INFO("[Main] Box %d coordinates (x: %f, y: %f, z: %f)", i, 
            boxes.coords[i][0], boxes.coords[i][1], boxes.coords[i][2]);

    // initialize image pipeline and navigation objects
    Navigation nav(nh, boxes);
    ImagePipeline img_pipeline(nh, boxes);
    Logger logger;
    logger.open(RESULT_FILEPATH);
    
    ros::Rate loop_rate(10); // run at 10 Hz
    srand(time(0));
    
    // keep on executing strategy till there are no unvisited (and un-processed) objectives
    while(ros::ok() && nav.any_unvisited_obj()) // TODO: && <time remaining>
    {
        // TODO: try 360 using contest 1 function and see if localization improves
        ros::spinOnce();

        // step 1: compute optimal sequence
        nav.compute_opt_seq();
        auto box_seq = nav.get_goal_seq();

        // step 2: follow the provided sequence
        int seq_idx = 0;
        for (const auto& box : box_seq)
        {
            int num_tries = 0;
            int box_idx = nav.get_obj_ID(box);
            if (nav.move_to_goal(box[0], box[1], box[2]) == SUCCESS)
            {
                nav.set_obj_visited(seq_idx);    
                classify_obj(img_pipeline, boxes, box, box_idx, logger);
            }
            else
                // try replanning; if still fail, skip and move to next item in sequence
                while (num_tries < NUM_REPLANS)
                {
                    ROS_ERROR("[MAIN] move_to_goal() failed! Trying to replan!");

                    // nudge a little in a random walk style
                    int delta_x = 0, delta_y = 0;
                    while (delta_x == 0 && delta_y == 0) // avoid delta_x = delta_y = 0
                    {
                        delta_x  = (int(rand()%3) - 1);
                        delta_y  = (int(rand()%3) - 1);
                    }
                    nav.move_to_goal(box[0] + delta_x*0.3, box[1] + delta_y*0.3, box[2]);

                    if (nav.move_to_goal(box[0], box[1], box[2]) == SUCCESS)
                    {
                        nav.set_obj_visited(seq_idx);
                        classify_obj(img_pipeline, boxes, box, box_idx, logger);
                        break;
                    }
                    else    
                        num_tries++;
                }
            
            seq_idx++;
        }

        loop_rate.sleep();
    }

    exit(EXIT_SUCCESS);
}

// TODO: right now, the FSM disregards any failure in image pipeline. make it not mark as visisted if
//  classification fails!
void classify_obj(ImagePipeline& img_pipeline, Boxes& boxes, const std::vector<float>& box, int box_idx, Logger& logger)
{
    // take a picture and get template ID; skip process if invalid ID received.
    int ID = img_pipeline.get_template_ID(boxes);
    if (ID == -1)
        return;

    // print to log file
    logger.write("Box: " + std::to_string(box_idx) + "\n");
    logger.write("At location ("+ std::to_string(box[0]) + ", " + std::to_string(box[1]) + 
        ", " + std::to_string(box[2]) + ")\n");
    
    if (ID != 0)
        logger.write(TEMPLATE_NAME[ID] + " was found\n");
    else
        logger.write("Nothing was found\n");
}