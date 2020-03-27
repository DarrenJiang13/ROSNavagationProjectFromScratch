/// \file: ekf_slam.hpp
/// \brief Library for ekf slam

#include <iostream>
#include <vector>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include "rigid2d/diff_drive.hpp"
#include "nuslam/TurtleMap.h"
#include <Eigen/Dense>


class EKF_slam{
public:

    /// \brief Constructor.
    EKF_slam();

    /// \brief Constructor with some noise initialization.
    /// \param process_noise_init - process noise matrix Q coefficient
    /// \param measure_noise_init - measurement noise matrix R coefficient
    EKF_slam(float process_noise_init,float measure_noise_init);

    /// \brief get landmarks input and evaluate to the inner class parameters
    /// \param landmarks_x - landmarks detected (x)
    /// \param landmarks_y - landmarks detected (y)
    /// \param landmarks_index - landmarks index
    void setLandmarks(std::vector<double>landmarks_x,std::vector<double>landmarks_y,std::vector<int>landmarks_index);

    /// \brief predict process of ekf when data association is known
    /// \param twist - input twist of the robot
    void predictKnownAssociation(rigid2d::Twist2D twist);

    /// \brief correction process of ekf when data association is known
    /// \param real_robot_pose - the current position of the robot
    /// \return current robot pose
    rigid2d::Twist2D correctionKnownAssociation();

    /// \brief Check for Every landmark you detect. If it is a new one, expand the state vector. else continue.
    /// \param landmarks_x,landmarks_y - landmarks detected
    void landmarkAssociation(std::vector<double>landmarks_x,std::vector<double>landmarks_y);

    /// \brief predict process of ekf
    /// \param twist - input twist of the robot
    void predict(rigid2d::Twist2D twist);

    /// \brief correction process of ekf when data association is known
    /// \param real_robot_pose - the current position of the robot
    /// \return  current robot pose
    rigid2d::Twist2D correction();

    /// \brief return current pose
    /// \return current robot pose
    rigid2d::Twist2D getPose();

private:
    Eigen::VectorXf state_vec;//At the beginning, theta,x,y=0, and no landmarks detected
    Eigen::VectorXf state_vec_prior;//prior state vector

    Eigen::MatrixXf  mat_covariance;
    Eigen::MatrixXf  mat_covariance_prior;

    Eigen::MatrixXf mat_G;

    float process_noise,measure_noise;
    float pose_covalue,measure_covalue;

    Eigen::MatrixXf  process_noise_Q;
    Eigen::MatrixXf  measure_noise_R;

    ///set landmarks
    std::vector<double>center_x;
    std::vector<double>center_y;
    std::vector<int>index;

    std::vector<double> landmark_x_buf; //for those vector which only appears one time, save it to the buffer.
    std::vector<double> landmark_y_buf; //for those vector which only appears one time, save it to the buffer.
    std::vector<int> count_buf; //count how many times the landmark appears
    int buf_count_limit;//for those new landmarks which appears more than n times, trust it as a new landmark
    double ass_distance_limit;//associate radius limit
};

