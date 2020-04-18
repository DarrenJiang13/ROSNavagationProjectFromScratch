# A ROS Navagation Project from Scratch

This is a C++ based ROS navigation project which starts from writing a 2D differencial robot kinematic library and ends 
with an ekf slam navigation part. This is actually a course project in Northwestern University, 2020 winter.

**Ubuntu Version**: 18.04  
**ROS Version**: melodic


## Package Introduction:

- **nuslam**: package run ekf-slam
- **nuturtle_description**: for robot description files like urdf and some kinematic parameters
- **nuturtle_gazebo**: creating a virtual robot model in gazebo
- **nuturtle_robot**: connecting ros code with real robot called turtlebot3-burger
- **nuturtlebot**: some essential files for Northwestern turtlebot
- **righd2d**: fundamental files for this project,including some kinematic libraries, and fake_encoder,odometer nodes. 
- **tsim**: get familiar with ROS by control turtle in turtlesim package.
