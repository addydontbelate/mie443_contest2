#include "robot.h"
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <tf/transform_datatypes.h>

std::vector<bool> obj_visited(boxes.coords.size, false);
std::vector<int> obj_priority(boxes.coords.size, 0);

void Robot::prioritize()
{
    
}