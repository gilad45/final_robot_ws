# final_robot_ws
The final iteration i made of my robot in the end of the school year.

Authors: Gilad Berkove, Evyatar Muchnik, Reuven Lamberg.

this robot was built for a school competition where a robot is placed in a maze,
and needs to find a candle inside and extinguish it.
This robot uses SLAM Toolkit on ROS. To get ODOM we used scan matching with an IMU.
more documentation should be add with time.

specs: 
raspberry pi 5 8gb. 
ld19 lidar. 
arduino uno.


Put all these files into src

Also you need to add a few more packages into src

1. csm
2. ros2 lidar scan matcher https://github.com/AlexKaravaev/ros2_laser_scan_matcher
3. mstrp explore https://github.com/mertgulerx/mrtsp_exploration_ros2
4. SLAM Toolkit
5. NAV2
6. A few more packages i will add later.

The rest of the stuff you can find the names in the launch files as i didn't put all the packages we used.

after adding them all, place in the ws and build using colcon.
