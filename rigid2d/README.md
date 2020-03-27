# ROS Package: rigid2d
Package Name: rigid2d

Package Description: this package aims to provide essential function for 2D robot transformation.

Author: Yujie Jiang

Mail: yujiejiang2020@u.northwestern.edu

# List of every file:
/rigid2d:

  - CMakeLists.txt:essential file for catkin make compiling
	
  - package.xml:all the dependencies for the package
	
  - README.md:tutorial for this package
		
  - /include:for header files.
    - rigid2d/rigid2d.hpp:Library for two-dimensional rigid body transformations.
    - rigid2d/diff_drive.hpp:Library for a differential drive robot.
    - rigid2d/waypoints.hpp:Library for a differential drive robot moving in a sequence of waypoints

  - /src: for source files.
    - rigid2d/rigid2d.cpp:   cpp file for the header file
    - diff_drive/diff_drive.cpp:   cpp file for the header file
    - waypoints/waypoints.cpp:   cpp file for the header file
    - fake_diff_encoders.cpp:   create a ros node called fake_diff_encoders, publish the encoder for the robot in rviz
    - odometer.cpp: create a ros node called odometer, publish the transformation between body_link and world_frame for the robot in rviz
    - rigid2d_node.cpp: create a ros node called rigid2d_node
  
  - /TASKC.005:homework00 essential files.
  
  - /test: for ros test
    - rigid2d_node_test.cpp: main test cpp file
    - the_test_launchfile.test maybe launch file for test.
    
