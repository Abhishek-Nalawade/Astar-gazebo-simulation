For ROS

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
