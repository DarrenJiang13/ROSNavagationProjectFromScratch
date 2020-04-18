# ROS Package : nuslam

**Package Description**: this package aims to implement the ekf-slam of the robot in gazebo world

**Author**: Yujie Jiang

# To run the TASK L.002 in rviz:
    source devel/setup.bash
    roslaunch nuslam slam.launch debug:=true
    
# To run the TASK L.003 in rviz:
    source devel/setup.bash
    roslaunch nuslam slam.launch debug:=false
    
# List of every file:
/nuslam:

  - CMakeLists.txt:essential file for catkin make compiling
	
  - package.xml:all the dependencies for the package
	
  - README.md:tutorial for this package
	
  - /config:for some configuration file like .yaml
	
	- config.rviz:configuration file for rviz
		
  - /include:for header files.
    - nuslam/ekf_slam.hpp:Library for ekf_slam
    
  - /launch:save .launch files and other related files.
	- slam.launch:launch the slam node in debug or other mode
	- landmarks.launch:visualize the landmarks. just for testing
	- diff_drive_gazebo.launch:visualize the robot model in gazebo
	
  - /msg:defined message files
    - TurtleMap.msg:nuslam::TurtleMap file  
    
  - /src:source files for ros nodes and library files  
    - ekf_slam/ekf_slam.cpp:implementation of ekf_slam.hpp
    - analysis.cpp: analysis node. publishing ground truth landmarks info.
    - draw_map.cpp: draw_map.node. draw the landmarks in rviz.
    - landmarks.cpp: landmarks.node. get laser scan clusters and fit them as circles
    - slam.cpp: slam.node.  run the ekf slam .
    - slam_debug.cpp: slam_debug.node. run the ekf_slam in debug node.

  
  - /test: for ros test
    - nuslam_test.cpp: main test cpp file
    - test_launchfile.test maybe launch file for test.
	
	
