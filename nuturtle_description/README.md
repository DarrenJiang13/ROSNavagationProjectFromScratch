# ROS Package : nuturtle_description

**Package Description**: this package aims to descript the urdf of the mobile robot.

**Author**: Yujie Jiang

# To visualize the robot in rviz:
    cd (paste your path here)/src/nuturtle_description/launch/
    roslaunch view_diff_drive.launch

# List of every file:
/nuturtle_description:

  - CMakeLists.txt:essential file for catkin make compiling
	
  - package.xml:all the dependencies for the package
	
  - README.md:tutorial for this package
	
  - /config:for some configuration file like .yaml
	
	- diff_params.yaml:parameters of the diff_robot
		
  - /include:nothing yet. for future include files.

  - /launch:save .launch files and other related files.
	
	- config.rviz:save the configuration of the rviz so you do not need to set when you open the rviz.
	
	- view_diff_drive.launch:launch the rviz to visualize the robot model.

  - /src:nothing yet. for future source files.
  
  - /xarco:save the .xarco file which is used for robot configuration.
	
	- diff_drive.urdf.xacro: set the links and joints of the mobile robot.
	
	
