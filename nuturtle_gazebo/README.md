# ROS Package : nuturtle_gazebo

**Package Description**: this package aims to generate a virtual differential robot model in gazebo.

**Author**: Yujie Jiang

# To visualize the robot in rviz:
    cd (paste your path here)/src/nuturtle_gazebo/launch/
    roslaunch diff_drive_gazebo.launch
    
  or
    
    gazebo_waypoints.launch

# List of every file:
/nuturtle_gazebo:

  - CMakeLists.txt:essential file for catkin make compiling
	
  - package.xml:all the dependencies for the package
	
  - README.md:tutorial for this package
	
  - /config:for some configuration file like .yaml
	
	- diff_params.yaml:parameters of the diff_robot
		
  - /include:header files for this package

  - /launch:save .launch files and other related files.
	
	- diff_drive_gazebo.launch: visualize the robot model in gazebo
	
	- gazebo_waypoints.launch: run the waypoints before in gazebo world

  - /src: source files.
    
    - turtle_drive_plugin.cpp: set configuration for robot in gazebo world.
  
  - /xarco:save the .xarco file which is used for robot configuration.
	
	- diff_drive.xacro: set some dynamic parameters for robot model.
