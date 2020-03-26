# mie443_contest2
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/19d57de434c640e79cf2addbd389d31b)](https://app.codacy.com/gh/addydontbelate/mie443_contest2?utm_source=github.com&utm_medium=referral&utm_content=addydontbelate/mie443_contest2&utm_campaign=Badge_Grade_Settings)

## Simulation Launch Instructions

- Set log file paths for `contest2_log.txt`, `vision_log.txt`, and scene images:
	- In `src/contest2.cpp` update `RESULT_DIR` macro with the desired directory path `/path/to/dir/` for the contest log. 
	- In `include/image_pipeline.h` update `VISLOG_DIR` macro with the desired directory path `/path/to/dir/` for the vision log and scene images.
- `roslaunch mie443_contest2 turtlebot_world.launch world:=1`
- `roslaunch turtlebot_gazebo amcl_demo.launch map_file:=/home/<username>/catkin_ws/src/mie443_contest2/maps/map_1.yaml` (**note:** an absolute path to the map file is required)
- `roslaunch turtlebot_rviz_launchers view_navigation.launch`
- Set an initial pose estimate in rviz:
	- To localize, move around with teleop until point cloud converges.
	- To run the keyboard teleop: `roslaunch turtlebot_teleop keyboard_teleop.launch`.
	- To send the robot to a pose in rviz, use "2D Nav Goal".
	- To measure a position in rviz use "Publish Point".   
- `rosrun mie443_contest2 contest2`
