# ROS Package: nuturtle_robot
Package Name: nuturtle_robot

Package Description: this package aims to control real turtlebot

Author: Yujie Jiang

Mail: yujiejiang2020@u.northwestern.edu

# How to run rostest:
In this homework, we only need to test turtle_interface  

        cd (your workspace, in your computer)
        source devel/setup.bash 
        rostest nuturtle_robot turtle_interface.test

# How to roslaunch:
1. run following codes first to copy executable files to the turtlebot

        ./arm catkin_arm
        ./arm catkin_arm install
        rsync -av --delete arm_install/ student@turtlebot5:/home/student/install
    
        (X depends on the number of the turtlebot)
        cd (your workspace, in your computer)
        source devel/setup.bash 
        export ROS_MASTER_URI=http://your username:11311
        cd src/nuturtle_robot/launch/
    
2. Then run
    
        roslaunch nuturtle_robot follow_waypoints.launch robot:=X
    or  
    
        roslaunch nuturtle_robot test_movement.launch robot:=X
    or  
    
        roslaunch nuturtle_robot test_translation.launch robot:=X	

3. Wait for the turtlebot to appear in the rviz, then call Start command

        cd (your workspace, in your computer)
        source devel/setup.bash 
        rosservice call /start 1
        
    in test_movement.launch or test_translation.launch, /start 1 means move forward or anti-clockwise, while 0 means backward or clockwise
    
    in follow_waypoints.launch you can call 
    
        rosservice call /start 1
    
    to stop the turtlebot and reset the odometry.
# List of every file:
/nuturtle_robot:

  - CMakeLists.txt:essential file for catkin make compiling
	
  - package.xml:all the dependencies for the package
	
  - README.md:tutorial for this package
  
  - /config:for some configuration file like .yaml
	- config.rviz:config file for rviz
	
  - /images:for some images for homework
	- accbat.svg: image for battery plot and acceleration of z
	- F003.svg: node picture
	
  - /launch:save .launch files and other related files.
	- basic_remote.launch: for remote communication with turtlebot
    - follow_waypoints.launch: for turtlebot to move through 5 waypoints in real world		
    - test_movement.launch:	for turtlebot to rotate	
    - test_translation.launch:	for turtlebot to translation	
    
  - /src: for source files.
    - real_waypoint.cpp:   control turtlebot to move through 5 waypoints in the real world
    - rotation.cpp:   control turtlebot to rotate for 20 cycles, pause 1/20 of a full rotation time after each cycle
    - translation.cpp:   control turtlebot to translate for 20 steps, pause 1/20 of a full translation time
    - turtle_interface.cpp:   get sensor_data from turtlebot and transform it to joint_states, get cmd_vel data from other node and transform it into turtlebot wheel velocities.

  - /srv: for service files.
     - Start.srv:   start service

  - /test: for ros test
    - turtle_interface.test: test file for turtle_interface
    - turtle_interface_test.cpp: test node for turtle_interface
    
