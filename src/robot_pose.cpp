#include <robot_pose.h>
#include <tf/transform_datatypes.h>

RobotPose::RobotPose(float x, float y, float phi)
{
	this->x = x;
	this->y = y;
	this->phi = phi;
}

void RobotPose::pose_callback(const geometry_msgs::PoseWithCovarianceStamped& msg)
{
	phi = static_cast<float>(tf::getYaw(msg.pose.pose.orientation));
	x = msg.pose.pose.position.x;
	y = msg.pose.pose.position.y;
    seconds_elapsed = TIME_S(CLOCK::now()-rob_start).count();
}

bool RobotPose::operator==(const RobotPose& rhs) const 
{
	if (this->x == rhs.x && this->y == rhs.y && this->phi == rhs.phi)
		return true;
	return false;
}
