# ROS Package: tsim
Package Name: tsim

Package Description: this package aims to descript the move of the mobile robot.

Author: Yujie Jiang

Mail: yujiejiang2020@u.northwestern.edu

# How to roslaunch:
    cd (your workspace)
    source devel/setup.bash 
    cd src/tsim/launch/
    roslaunch trect.launch
    or  roslaunch turtle_odom.launch
    or	roslaunch turtle_pent.launch	

# List of every file:
/tsim:

  - CMakeLists.txt:essential file for catkin make compiling
	
  - package.xml:all the dependencies for the package
	
  - README.md:tutorial for this package
	
  - /config:for some configuration file like .yaml
	
	- tsim_rec_params.yaml:parameters of the diff_robot
		
  - /include:nothing yet. for future include files.

  - /launch:save .launch files and other related files.
	
	- trect.launch:launch the robot model.

  - /src: for source files.
  
    - turtle_rec.cpp:cpp file for the node

    - turtle_way.cpp:cpp file for the node
    
  - /msg:save the .msg file which is used for ros message.
	
	-PoseError.msg: define a new type of message called PoseError. 
    
    -others: not use yet.
    
    
# Screenshot of the turtle:
<figure>
<center>
<img src='https://raw.githubusercontent.com/DarrenJiang13/Kalman-Filter/master/photo_turtle.png' />
</figure>

# Screenshot of rqt_plot:
<figure>
<center>
<img src='https://raw.githubusercontent.com/DarrenJiang13/Kalman-Filter/master/image.png' />
</figure>

# Turtle Video:
https://www.youtube.com/watch?v=teoYJjwd2vc&feature=youtu.be
