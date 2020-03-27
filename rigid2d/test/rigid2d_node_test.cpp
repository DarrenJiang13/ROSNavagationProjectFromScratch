/// \file: main.cpp
/// \brief: this is the .cpp file for the ME495 Homework00: TaskC

#include <iostream>
#include "ros/ros.h"
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>
#include "rigid2d/rigid2d.hpp"
#include "rigid2d/waypoints.hpp"

using namespace std;
using namespace rigid2d;

//http://wiki.ros.org/gtest
//http://wiki.ros.org/action/show/Quality/Tutorials/UnitTesting?action=show&redirect=UnitTesting
//http://wiki.ros.org/rostest/Writing

//https://github.com/google/googletest/blob/master/googletest/docs/primer.md
//https://github.com/google/googletest/blob/master/googletest/docs/advanced.md

/*
Bool Check:
    Fatal assertion 	            Nonfatal assertion 	        Verifies
ASSERT_TRUE(condition); 	    EXPECT_TRUE(condition); 	condition is true
ASSERT_FALSE(condition); 	    EXPECT_FALSE(condition); 	condition is false

Value Check:
    Fatal assertion 	            Nonfatal assertion 	        Verifies
ASSERT_EQ(expected, actual); 	EXPECT_EQ(expected, actual); 	expected == actual
ASSERT_NE(val1, val2); 	        EXPECT_NE(val1, val2); 	        val1 != val2
ASSERT_LT(val1, val2); 	        EXPECT_LT(val1, val2); 	        val1 < val2
ASSERT_LE(val1, val2); 	        EXPECT_LE(val1, val2); 	        val1 <= val2
ASSERT_GT(val1, val2); 	        EXPECT_GT(val1, val2); 	        val1 > val2
ASSERT_GE(val1, val2); 	        EXPECT_GE(val1, val2); 	        val1 >= val2

String Check:
    Fatal assertion 	                            Nonfatal assertion 	                        Verifies
ASSERT_STREQ(expected_str, actual_str); 	    EXPECT_STREQ(expected_str, actual_str); 	    the two C strings have the same content
ASSERT_STRNE(str1, str2); 	                    EXPECT_STRNE(str1, str2); 	                    the two C strings have different content
ASSERT_STRCASEEQ(expected_str, actual_str); 	EXPECT_STRCASEEQ(expected_str, actual_str); 	the two C strings have the same content, ignoring case
ASSERT_STRCASENE(str1, str2); 	                EXPECT_STRCASENE(str1, str2); 	                the two C strings have different content, ignoring case
*/

/// \brief test Transform2D::Transform2D()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testTransform2D)
{
    cout<<"Transform2D::Transform2D() Testing."<<endl;
    Vector2D vec;
    vec.x=5;
    vec.y=6;
    Transform2D transform2D(vec);
    Twist2D twist=transform2D.displacement();

    Transform2D transform2D1(twist);
    Twist2D twist2=transform2D1.displacement();

    EXPECT_EQ(vec.x, twist.vx);
    EXPECT_EQ(vec.x, twist2.vx);
}

/// \brief test Transform2D::operator()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testOperatorBracket)
{
    cout<<"Transform2D::operator() Testing."<<endl;
    Vector2D vec;
    vec.x=0;
    vec.y=0;
    Transform2D transform2D_1(vec);
    Transform2D transform2D_2;

    Twist2D twist1=transform2D_1.displacement();
    Twist2D twist2=transform2D_2.displacement();
    EXPECT_EQ(twist1.vx, twist2.vx);
}

/// \brief test Transform2D::inv()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testInv)
{
    cout<<"Transform2D::inv() Testing."<<endl;
    Vector2D vec;
    vec.x=0;
    vec.y=0;
    Transform2D transform2D_1(vec);
    Transform2D transform2D_2;

    Twist2D twist1=transform2D_1.inv().displacement();
    Twist2D twist2=transform2D_2.inv().displacement();
    EXPECT_EQ(twist1.vx, twist2.vx);
}

/// \brief test Transform2D::displacement()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testDisplacement)
{
    cout<<"Transform2D::displacement() Testing."<<endl;
    Vector2D vec1,vec2;
    vec1.x=5;
    vec1.y=6;
    Transform2D transform2D_1(vec1,1);
    vec2.x=5;
    vec2.y=6;
    Transform2D transform2D_2(vec2,5);

    Twist2D twist1=transform2D_1.displacement();
    Twist2D twist2=transform2D_2.displacement();
    EXPECT_EQ(twist1.vx, twist2.vx);
}

/// \brief test Transform2D::operator*=
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testOperatorME)
{
    cout<<"Transform2D::operator*= Testing."<<endl;
    Vector2D vec1,vec2;
    vec1.x=5;
    vec1.y=6;
    Transform2D transform2D_1(vec1,0);
    vec2.x=5;
    vec2.y=6;
    Transform2D transform2D_2(vec2,0);

    transform2D_2*=(transform2D_1);
    Twist2D twist=transform2D_2.displacement();

    EXPECT_EQ(twist.vx, 10);
}

/// \brief test rigid2d::operator<<
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testOperatorOS)
{
    cout<<"rigid2d::operator<< Testing."<<endl;

    //overload << for vector
    std::stringstream ss_vec("5 6");
    Vector2D vec;
    ss_vec>>vec;
    EXPECT_EQ(vec.x, 5);

    //overload << for twist
    std::stringstream ss_twist("90 5 6");
    Twist2D twist;
    ss_twist>>twist;
    EXPECT_EQ(twist.vx, 5);

    //overload << for transform
    std::stringstream ss_trans("90 5 6");
    Transform2D trans;
    ss_trans>>trans;
    EXPECT_EQ(trans.displacement().vx, 5);

    cout<<endl;
}

/// \brief test rigid2d::operator>>
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testOperatorIS)
{
    cout<<"rigid2d::operator>> Testing."<<endl;

    // ASSERT_STREQ should be used when comparing raw c strings (char*).To compare std::strings you should use ASSERT_EQ.
    //overload << for vector
    Vector2D vec;
    vec.x=5;
    vec.y=6;
    std::stringstream ss_vec, ss_vec_ans("[ 5 6 ]\n");
    ss_vec<<vec;
    ASSERT_EQ(ss_vec.str(), ss_vec_ans.str());


    //overload << for twist
    Twist2D twist;
    twist.w=0;
    twist.vx=5;
    twist.vy=6;
    std::stringstream ss_twist, ss_twist_ans("[ 0 5 6 ]\n");
    ss_twist<<twist;
    ASSERT_EQ(ss_twist.str(), ss_twist_ans.str());

    //overload << for transform
    std::stringstream ss_trans, ss_trans_ans("theta(in radians): 0 x: 5 y: 6\n");
    Transform2D trans(vec, 0);
    ss_trans<<trans;
    ASSERT_EQ(ss_trans.str(), ss_trans_ans.str());

}

/// \brief test rigid2d::operator*
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testMulti)
{
    cout<<"rigid2d::operator* Testing."<<endl;
    Vector2D vec1,vec2;
    vec1.x=5;
    vec1.y=6;
    Transform2D transform2D_1(vec1,0);
    vec2.x=5;
    vec2.y=6;
    Transform2D transform2D_2(vec2,0);

    Transform2D transform2D_3=transform2D_1*transform2D_2;
    Twist2D twist=transform2D_3.displacement();

    EXPECT_EQ(twist.vx, 10);
}

/// \brief test Vector addition +=
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testAdditionEq)
{
    cout<<"rigid2d::operator+= for Vector2D Testing."<<endl;

    Vector2D vec1(5,6);
    Vector2D vec2(5,6);
    vec1+=vec2;
    EXPECT_EQ(vec1.x, 10);
}


/// \brief test Vector addition +
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testAddition)
{
    cout<<"rigid2d::operator+ for Vector2D Testing."<<endl;

    Vector2D vec1(5,6);
    Vector2D vec2(5,6);
    Vector2D vec3 = vec1+vec2;
    EXPECT_EQ(vec3.x, 10);
}

/// \brief test Vector addition -=
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testSubtractionEq)
{
    cout<<"rigid2d::operator-= for Vector2D Testing."<<endl;

    Vector2D vec1(5,6);
    Vector2D vec2(5,6);
    vec1-=vec2;
    EXPECT_EQ(vec1.x, 0);
}


/// \brief test Vector addition -
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testSubtraction)
{
    cout<<"rigid2d::operator- for Vector2D Testing."<<endl;

    Vector2D vec1(5,6);
    Vector2D vec2(5,6);
    Vector2D vec3 = vec1-vec2;
    EXPECT_EQ(vec3.x, 0);
}

/// \brief test Vector Production *=
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testProductionEq)
{
    cout<<"rigid2d::operator*= for Vector2D Testing."<<endl;

    double num=5.0;
    Vector2D vec1(5,6);
    Vector2D vec2(5,6);

    vec1*=num;
    num*=vec2;

    EXPECT_EQ(vec1.x, 25);
    EXPECT_EQ(vec2.x, 25);
}

/// \brief test Vector Production *
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testProduction)
{
    cout<<"rigid2d::operator* for Vector2D Testing."<<endl;

    double num=5.0;
    Vector2D vec1(5,6);
    Vector2D vec2,vec3;

    vec2=vec1*num;
    vec3=num*vec1;

    EXPECT_EQ(vec2.x, 25);
    EXPECT_EQ(vec3.x, 25);
}

/// \brief test integrateTwist()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testIntegrateTwist)
{
    cout<<"integrateTwist() Testing."<<endl;

    Twist2D twist;
    twist.w=0;
    twist.vx=5;
    twist.vy=6;

    Transform2D transform2D=integrateTwist(twist);
    EXPECT_EQ(transform2D.displacement().vx, twist.vx);
}

/// \brief test length()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testLength)
{
    cout<<"length() Testing."<<endl;


    Vector2D vec;
    vec.x=3;
    vec.y=4;

    EXPECT_EQ(length(vec), 5);
}

/// \brief test distance()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testDistance)
{
    cout<<"distance() Testing."<<endl;


    Vector2D vec1;
    vec1.x=4;
    vec1.y=5;

    Vector2D vec2;
    vec2.x=1;
    vec2.y=1;

    EXPECT_EQ(distance(vec1,vec2), 5);
}

/// \brief test angle()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testAngle)
{
    cout<<"angle() Testing."<<endl;

    Vector2D vec;
    vec.x=-1;
    vec.y=1;

    EXPECT_EQ(rad2deg(angle(vec)), 135);
}

/// \brief test twistToWheels()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testTwistToWheels)
{
    cout<<"twistToWheels() Testing."<<endl;
    DiffDrive diffRobot;
    Twist2D twist={2,0,0};

    WheelVelocities wheel_vel=diffRobot.twistToWheels(twist);

    EXPECT_FLOAT_EQ(wheel_vel.left, -4.848485);
    EXPECT_FLOAT_EQ(wheel_vel.right, 4.848485);
}

/// \brief test wheelsToTwist()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testWheelsToTwist)
{
    cout<<"wheelsToTwist() Testing."<<endl;

    DiffDrive diffRobot;
    WheelVelocities wheel_vel={2,-2};

    Twist2D twist=diffRobot.wheelsToTwist(wheel_vel);

    EXPECT_FLOAT_EQ(twist.w, -0.82499999);
    EXPECT_FLOAT_EQ(twist.vx, 0);
    EXPECT_FLOAT_EQ(twist.vy, 0);

}

/// \brief test feedforward() and updateOdometry()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testFeedforwardUpdateOdometry)
{
    cout<<"feedforward() and updateOdometry() Testing."<<endl;
    Twist2D mypose={PI/2,2,1};

    cout<<"Moving straight."<<endl;
    Twist2D twistStraight(0,5,0);
    DiffDrive diffRobot(mypose,0.16,0.033);
    diffRobot.feedforward(twistStraight);

    EXPECT_FLOAT_EQ(diffRobot.pose().w, mypose.w);
    EXPECT_FLOAT_EQ(diffRobot.pose().vx, mypose.vx+5*cos(mypose.w));
    EXPECT_FLOAT_EQ(diffRobot.pose().vy, mypose.vy+5*sin(mypose.w));

    cout<<"Turning."<<endl;
    Twist2D twistTurn(1,0,0);
    DiffDrive diffRobot2(mypose,0.16,0.035);
    diffRobot2.feedforward(twistTurn);

    EXPECT_FLOAT_EQ(diffRobot2.pose().w, mypose.w+twistTurn.w);
    EXPECT_EQ(diffRobot2.pose().vx, mypose.vx);
    EXPECT_EQ(diffRobot2.pose().vy, mypose.vy);


    cout<<"Translating and turning."<<endl;
    Twist2D twistTransTurn(2,5,0);
    DiffDrive diffRobot3(mypose,0.16,0.035);
    diffRobot3.feedforward(twistTransTurn);
    //Twist2D pose_est=integrateTwist(twistTransTurn).displacement();

    EXPECT_FLOAT_EQ(diffRobot3.pose().w, 2+PI/2);
    EXPECT_FLOAT_EQ(diffRobot3.pose().vx, 5*cos(2)-3);
    EXPECT_FLOAT_EQ(diffRobot3.pose().vy, 5*sin(2)+1);
}


/// \brief test feedforward() and updateOdometry()
/// \param TestSuite, testCase
/// \returns Assertions
TEST(Rigid2DTest, testWaypoints)
{
//    vector<Vector2D> vecTraj;
//    Vector2D vec_push1={0,0};
//    Vector2D vec_push2={0,1};
//    Vector2D vec_push3={2,2};
//    vecTraj.push_back(vec_push1);
//    vecTraj.push_back(vec_push2);
//    vecTraj.push_back(vec_push3);
//
//    Waypoints waypoints(vecTraj);
//
//    for (int i=1;i<=20;i++){
//        waypoints.nextWaypoint();
//    }

    int freq_cmd=60;
    double waypoint_x[]={6,6.618034,5,4.618034,4};
    double waypoint_y[]={0,1.902113,3.0776835,1.902113,0};


    vector<Vector2D> vecTraj;
    Vector2D vec_tmp;

    for(int i=0;i<5;i++){
        vec_tmp.x=waypoint_x[i];
        vec_tmp.y=waypoint_y[i];
        vecTraj.push_back(vec_tmp);
    }

    Waypoints myWaypoints(vecTraj,freq_cmd);

    for (int i=1;i<=7000;i++){
        myWaypoints.nextWaypoint();
        //cout<<myWaypoints.diffRobot.pose()<<endl;
    }


}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);

  	ros::init(argc, argv, "rigid2d_node_test");

    return RUN_ALL_TESTS();
}
