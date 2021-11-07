# For ROS

    Python 2.8
    ROS Melodic
    Turtlebot 3 For normal planning
    Python 3.7


1) Please make sure to include the file "turtlebot31_empty_world.launch" in the launch folder of the turtlebot_simulations folder.
2) Also include the "map" file in the worlds folder in the turtlebot_simulations folder.
3) Please launch gazebo using the command "roslaunch turtlebot3_gazebo turtlebot31_empty_world.launch"

4) Create a ROS package and place the codes in the /package_name/src folder
5) Then type the command rosrun packge_name Project3Phase3_main.py

The default values in the launch files have been set to x = 3.0 and y = 0. Please use this values as user input to run the simulations.
The range of coordinates is from (-5,-5) to (5,5).


## General steps to set up a ROS project
1) Create the workspace and source folder with command mkdir -p ws_name/src
2) Go in src and type command catkin_init_workspace
3) In folder ws_name type command catkin_make
4) Go in src and type catkin_create_pkg pakage_name rospy std_msgs (other dependencies)
5) Go back in ws_name again and type catkin_make
6) Navigate to src/package_name/src/ and create the talker and subscriber
7) In package_name create the launch folder and inside that create the launch file




8)To run a ros project in ws_name type catkin_make first then source devel/setup.bash and then roslaunch <name_of_package> <name_of_launch_file>
