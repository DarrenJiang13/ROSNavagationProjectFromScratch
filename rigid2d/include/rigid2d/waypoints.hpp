/// \file waypoints.hpp
/// \brief Library for a differential drive robot moving in a sequence of waypoints
#include <iosfwd> // contains forward definitions for iostream objects
#include <math.h>
#include <iostream>
#include <vector>
#include "rigid2d/diff_drive.hpp" // include the header file

using namespace std;

namespace rigid2d
{

    class Waypoints
    {
    public:

        /// \brief the default constructor creates waypoints with the input of a sequence of waypoints
        /// \param vecTraj - a sequence of points
        /// \param freq - frequency
        Waypoints(vector<Vector2D> vecTraj,double freq);

        /// \brief the default constructor creates waypoints with the input of a sequence of waypoints
        /// \param vecTraj - a sequence of points
        /// \param freq - frequency
        /// \param max_trans_vel - max translation velocity
        /// \param max_rot_vel - max rotation velocity
        Waypoints(vector<Vector2D> vecTraj,double freq, double max_trans, double max_rot);

        /// \brief the default destructor.
        ~Waypoints();

        /// \brief the default constructor creates waypoints with the input of a sequence of waypoints
        /// \return the wheel velocity to next point
        Twist2D nextWaypoint();

        /// \brief model of the diff_drive robot
        DiffDrive diffRobot;

        /// \brief speed limit
        /// \return limited speed
        double speed_trans_limit(double speed);

        /// \brief speed limit
        /// \return limited speed
        double speed_rot_limit(double speed);

        /// \brief reset target index
        void reset(bool sign);

        /// \brief get pose from actual robot
        void getActualPose(rigid2d::Twist2D ps);

    private:
        int target_index=0;
        bool stop_sign=0;
        bool reset_sign=0;
        double delta_time;
        vector<Vector2D> trajectory;
        Twist2D pose;
        Twist2D initial_robot_pose;
        double max_trans_vel;
        double max_rot_vel;
    };
}
