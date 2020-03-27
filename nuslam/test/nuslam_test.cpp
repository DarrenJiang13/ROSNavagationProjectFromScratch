/// \file: main.cpp
/// \brief: this is the .cpp file for the ME495 Homework00: TaskS

#include <iostream>
#include "ros/ros.h"
#include <gtest/gtest.h>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include <ctime>
#include "nuslam/ekf_slam.hpp"

//clock_t start,end;

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


const double PI=3.1415926535;

/// \brief test least squares method for circle fit
/// \param TestSuite, testCase
/// \returns none
TEST(NuslamTest, LSE_CircleFit)
{

    std::vector<double> input_x = {1, 2, 5, 7, 9, 3};
    std::vector<double> input_y = {7, 6, 8, 7, 5, 7};

    //https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    /// 1. Configure the matrix
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_x.data(), input_x.size());
    Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_y.data(), input_y.size());
    Eigen::VectorXd z = x.array().square()+y.array().square();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(input_x.size());

    Eigen::MatrixXd A(input_x.size(),3);
    A<<x,y,ones;

    /// 2. Get the A2, A3, A4, while A1 is set to be 1
    Eigen::VectorXd cap_A = -A.colPivHouseholderQr().solve(z);
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
//    Eigen::VectorXd cap_A=svd.solve(-z);
//    start=clock();
//    end=clock();
//    double endtime=(double)(end-start)/CLOCKS_PER_SEC;
//    std::cout<<"Total time:"<<endtime<<std::endl;		//s
//    std::cout<<"Total time:"<<endtime*1000<<"ms"<<std::endl;	//ms

    /// 3. Get the center and radius
    double center_a=-cap_A[0]*0.5;
    double center_b=-cap_A[1]*0.5;
    double R_2=(cap_A[0]*cap_A[0]+cap_A[1]*cap_A[1]-4*cap_A[2])*0.25;

    std::cout<<"a: "<<center_a<<" b: "<<center_b<<" R: "<<sqrt(R_2)<<std::endl;
}

/// \brief test hyper method for circle fit
/// \param TestSuite, testCase
/// \returns none
TEST(NuslamTest, Hyper_CircleFit)
{

//    std::vector<double> input_x = {1, 2, 5, 7, 9, 3};
//    std::vector<double> input_y = {7, 6, 8, 7, 5, 7};
    std::vector<double> input_x = {-1, -0.3, 0.3, 1};
    std::vector<double> input_y = {0, -0.06, 0.1, 0};

    //https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    /// 1. Configure the matrix
    Eigen::VectorXd x_ini = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_x.data(), input_x.size());
    Eigen::VectorXd x = x_ini.array()-x_ini.sum()/x_ini.size();
    Eigen::VectorXd y_ini = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_y.data(), input_y.size());
    Eigen::VectorXd y = y_ini.array()-y_ini.sum()/y_ini.size();

    Eigen::VectorXd z = x.array().square()+y.array().square();
    Eigen::VectorXd ones = Eigen::VectorXd::Ones(input_x.size());
    Eigen::MatrixXd mat_Z(input_x.size(),4);
    mat_Z<<z,x,y,ones;

    /// 2. get matrix M
    Eigen::MatrixXd mat_M=mat_Z.transpose()*mat_Z/input_x.size();

    /// 3. get H inverse
    Eigen::Matrix4d mat_H_inv;
    mat_H_inv<<0,0,0,0.5,0,1,0,0,0,0,1,0,0.5,0,0,-2*z.sum()/z.size();

    /// 4. SVD for Z
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(mat_Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
    Eigen::VectorXd mat_Z_values=svd.singularValues();

    /// 5.
    Eigen::VectorXd vec_A;
    if (mat_Z_values[mat_Z_values.size()-1]<1e-12){
        vec_A=svd.matrixV().col(3);
    }
    else{
        Eigen::MatrixXd mat_Y=svd.matrixV()*svd.matrixU().transpose()*mat_Z;
        Eigen::MatrixXd mat_Q=mat_Y*mat_H_inv*mat_Y;

        Eigen::SelfAdjointEigenSolver< Eigen::MatrixXd > eigen_solver(mat_Q);
        Eigen::VectorXd mat_Q_eigenvalues=eigen_solver.eigenvalues();
        Eigen::VectorXd vec_A_star;
        if(mat_Q_eigenvalues[0]>0){vec_A_star=eigen_solver.eigenvectors().col(0);}
        else{
            for(int i=1;i<(int)mat_Q_eigenvalues.size();i++){
                if(mat_Q_eigenvalues[i-1]<0&&mat_Q_eigenvalues[i]>0){
                    vec_A_star=eigen_solver.eigenvectors().col(i);
                    break;}
            }
        }

        vec_A = mat_Y.colPivHouseholderQr().solve(vec_A_star);
    }

    double center_a=-vec_A[1]*0.5/vec_A[0]+x_ini.sum()/x_ini.size();
    double center_b=-vec_A[2]*0.5/vec_A[0]+y_ini.sum()/y_ini.size();
    double R_2=(vec_A[1]*vec_A[1]+vec_A[2]*vec_A[2]-4*vec_A[0]*vec_A[3])*0.25/(vec_A[0]*vec_A[0]);
    std::cout<<"a: "<<center_a<<" b: "<<center_b<<" R: "<<sqrt(R_2)<<std::endl;
}

/// \brief classification test
/// \param TestSuite, testCase
/// \returns none
TEST(NuslamTest, Classification)
{
    std::vector<double> input_x = {-1, -0.3, 0.3, 1};
    std::vector<double> input_y = {0, -0.06, 0.1, 0};

    //https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
    /// 1. Configure the matrix
    Eigen::VectorXd x = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_x.data(), input_x.size());
    Eigen::VectorXd y = Eigen::Map<Eigen::VectorXd, Eigen::Unaligned>(input_y.data(), input_y.size());

    Eigen::VectorXd vec_begin_multi_end=(x.array()-x[0])*(x.array()-x[x.size()-1])+(y.array()-y[0])*(y.array()-y[y.size()-1]);
    Eigen::VectorXd len_vec_begin=sqrt((x.array()-x[0])*(x.array()-x[0])+(y.array()-y[0])*(y.array()-y[0]));
    Eigen::VectorXd len_vec_end=sqrt((x.array()-x[x.size()-1]).abs2()+(y.array()-y[y.size()-1]).abs2());

    Eigen::VectorXd theta_vec=vec_begin_multi_end.array().array()/(len_vec_begin.array()*len_vec_end.array());

    Eigen::VectorXd theta_vec_segment=theta_vec.segment(1,x.size()-2);
    std::cout<<theta_vec_segment<<std::endl;

    for(int i=0;i<theta_vec_segment.size();i++){
        theta_vec_segment[i]=acos(theta_vec_segment[i])*180/PI;
    }

    double mean_theta=theta_vec_segment.mean();
    double std_theta=sqrt((theta_vec_segment.array()-theta_vec_segment.mean()).abs2().mean());
    std::cout<<theta_vec_segment<<std::endl;
    std::cout<<mean_theta<<std::endl;
    std::cout<<std_theta<<std::endl;
}

/// \brief efk slam class test
/// \param TestSuite, testCase
/// \returns none
TEST(NuslamTest, EKFPredictionTest)
{
//    std::cout<<"twist"<<std::endl;
//    EKF_slam ekfSlam;
//
//    rigid2d::Twist2D twist1(2,0,0);
//    std::cout<<twist1<<std::endl;
//    ekfSlam.predict(twist1);

//    std::vector<float> state_vec={2,3,4,5,6};
//    Eigen::Vector3f input_vec={1,2,3};
//    Eigen::VectorXf state_vec_prior=Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(state_vec.data(), state_vec.size());
//    state_vec_prior.segment(0,3).array()+=input_vec.array();
//    //std::cout<<state_vec_prior<<std::endl;
//
//    Eigen::Matrix3f  process_noise_Q;
//    process_noise_Q.setIdentity();
//
//
//    Eigen::MatrixXf mat_G;
//    mat_G.resize(5,5);
//    mat_G.setIdentity();
//
//    Eigen::MatrixXf mat_G1;
//    mat_G1.resize(6,6);
//    mat_G1.setIdentity();
//    mat_G=mat_G1;
//
//    Eigen::MatrixXf  mat_covariance;
//    mat_covariance=Eigen::Matrix4f::Zero();
//
//    Eigen::MatrixXf  value;
//    value=Eigen::MatrixXf::Zero(3,3);
//    //mat_covariance.block<3,3>(0,0).array()=value.array();
//
//    Eigen::MatrixXf  value2;
//    value2=Eigen::MatrixXf::Zero(3,3);
//
//    Eigen::MatrixXf process_noise_Q1;
//    process_noise_Q1=Eigen::Matrix3f::Zero();
//    process_noise_Q1.setIdentity();
//    process_noise_Q1*=0.01;
//
//    Eigen::VectorXf state_vec1;
//    state_vec1=Eigen::Vector3f::Zero();
//    //std::cout<<state_vec1<<std::endl;
//    state_vec1.conservativeResize(state_vec1.size()+1);
//    state_vec1[state_vec1.size()-1]=3;
//    //std::cout<<state_vec1<<std::endl;

//    Eigen::VectorXf state_vec;
//    state_vec=Eigen::VectorXf::Zero(4);
//
//    Eigen::MatrixXf mat_covariance_prior=Eigen::MatrixXf::Zero(state_vec.size(),state_vec.size());
//    mat_covariance_prior.setIdentity();
//
//    state_vec.conservativeResize(state_vec.size()+2);
//    state_vec[state_vec.size()-2]=2;
//    state_vec[state_vec.size()-1]=2;
//
//    //mat_covariance update
//    Eigen::MatrixXf mat_covariance_prior_temp;
//    mat_covariance_prior_temp=Eigen::MatrixXf::Zero(state_vec.size(),state_vec.size());
//    mat_covariance_prior_temp.topLeftCorner(state_vec.size()-2,state_vec.size()-2)=mat_covariance_prior;
//    mat_covariance_prior_temp(state_vec.size()-2,state_vec.size()-2)=100000;
//    mat_covariance_prior_temp(state_vec.size()-1,state_vec.size()-1)=100000;
//    mat_covariance_prior=mat_covariance_prior_temp;
//    Eigen::Matrix2f measure_noise_Q=Eigen::Matrix2f::Identity();
//    measure_noise_Q*=0.01;
//
//    Eigen::VectorXf state_vec=Eigen::Vector3f::Zero();
//    std::vector<double> center_x={ 0.719942, 0.742731, 0.03497, -0.526104};
//    std::vector<double> center_y={-0.02845, -0.792074, 0.407057, -0.516092};

//    std::vector<double> center_x={-0.81738, -0.825084, -0.844742, -0.215862, -0.22333, -0.215669, 0.43455, 0.399071, 0.719942, 0.742731, 0.03497, -0.526104};
//    std::vector<double> center_y={-0.829914, 0.010533, 0.664209, -0.848873, -0.016393, 0.688805, 0.449779, -0.446748, -0.02845, -0.792074, 0.407057, -0.516092};
//    state_vec.conservativeResize(state_vec.size()+center_x.size()*2);
//    for(int i=0;i<(int)center_x.size();i++){
//        state_vec[2*i+3]=center_x[i];
//        state_vec[2*i+4]=center_y[i];
//    }
//    Eigen::MatrixXf mat_covariance_prior=Eigen::MatrixXf::Identity(state_vec.size(),state_vec.size());
//
//    Eigen::VectorXf state_vec_prior=state_vec;
//
//    Eigen::VectorXf state_vec_post=state_vec_prior;
//
//    for(int i=0;i<(int)center_x.size();i++){
//
//        // 1. compute z and z_estimate
//        //z:as data association is known, z is actual distance between robot pose(ground truth from gazebo) and landmarks(ground truth from gazebo)
//        //z_estimate: distance between my slam robot pose and read landmarks
//        //z
//        float gt_dx=center_x[i]-0;
//        float gt_dy=center_y[i]-0;
//        float gt_distance=gt_dx*gt_dx+gt_dy*gt_dy;
//        Eigen::Vector2f vec_Zi;
//        vec_Zi[0]=sqrt(gt_distance);
//        vec_Zi[1]=atan2(gt_dy,gt_dx)-0;//normalize_angle
//        //z_estimate
//        float dx=center_x[i]-state_vec_prior[1];
//        float dy=center_y[i]-state_vec_prior[2];
//        float distance=dx*dx+dy*dy;
//        Eigen::Vector2f vec_Zi_estimate;
//        vec_Zi_estimate[0]=sqrt(distance);
//        vec_Zi_estimate[1]=atan2(dy,dx)-state_vec_prior[0];//normalize_angle
//
//        /// 2. Matrix Hi
//        Eigen::MatrixXf mat_Hi=Eigen::MatrixXf::Zero(2,3+2*center_x.size());
//        Eigen::MatrixXf mat_Hi_left(2,3);
//        mat_Hi_left<<0,-dx/sqrt(distance),-dy/sqrt(distance),-1,dy/distance, -dx/distance;
//        Eigen::MatrixXf mat_Hi_right(2,2);
//        mat_Hi_right<<dx/sqrt(distance),dy/sqrt(distance),-dy/distance, dx/distance;
//        mat_Hi.block<2,3>(0,0)=mat_Hi_left;
//        mat_Hi.block<2,2>(0,2*i+3)=mat_Hi_right;
//
//        /// 3.Kalman Gain
//        Eigen::MatrixXf mat_kal_gain=mat_covariance_prior*mat_Hi.transpose()*((mat_Hi*mat_covariance_prior*mat_Hi.transpose()+measure_noise_Q).inverse());
//
//        /// 4.state_vec_prior update
//        state_vec_post=state_vec_post+mat_kal_gain*(vec_Zi-vec_Zi_estimate);
//        std::cout<<state_vec_post<<std::endl;
//    }
//    state_vec=state_vec_post;

//    std::vector<double> center_x={ 0.719942, 0.742731, 0.03497, -0.526104};
//    std::vector<double> center_y={-0.02845, -0.792074, 0.407057, -0.516092};
//    std::vector<int> index={8,9,10,11};
//    rigid2d::Twist2D gt_pose={0,0,0};
//    EKF_slam ekfSlam(0.1,0.001);
//    rigid2d::Twist2D twist={0.001,0,0};
//
//
//    for(int i=1;i<7000;i++){
//        ekfSlam.setLandmarks(center_x,center_y,index);
//        ekfSlam.predictKnownAssociation(twist);
//        rigid2d::Twist2D ekf_pose=ekfSlam.correctionKnownAssociation(gt_pose);
//        //rigid2d::Twist2D ekf_pose=ekfSlam.getPose();
//        std::cout<<ekf_pose<<std::endl;
//        gt_pose.w=gt_pose.w+twist.w;
//    }


//    rigid2d::Twist2D gt_pose={0,0,0};
//    EKF_slam ekfSlam(0.1,0.001);
//    rigid2d::Twist2D twist={0.001,0,0};
//    std::vector<double> landmarks_x1={ 0.719942, 0.742731, 0.03497, -0.526104};
//    std::vector<double> landmarks_y1={-0.02845, -0.792074, 0.407057, -0.516092};
//    std::vector<double> landmarks_x2={-0.81738, -0.825084, -0.844742, -0.215862};
//    std::vector<double> landmarks_y2={-0.829914, 0.010533, 0.664209, -0.848873};
//    std::vector<double> landmarks_x3={ -0.81738, -0.825084, 0.03497, -0.526104};
//    std::vector<double> landmarks_y3={-0.829914, 0.010533, 0.407057, -0.516092};
//    std::vector<double> landmarks_x4={ 0.719942, 0.742731, 0.03497, -0.526104};
//    std::vector<double> landmarks_y4={-0.02845, -0.792074, 0.407057, -0.516092};

//    ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x3,landmarks_y3);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x3,landmarks_y3);
//    ekfSlam.predict(twist);
//    ekfSlam.landmarkAssociation(landmarks_x3,landmarks_y3);
//    ekfSlam.predict(twist);

//    for(int i=1;i<60;i++){
//        ekfSlam.landmarkAssociation(landmarks_x1,landmarks_y1);
//        ekfSlam.predict(twist);
//        std::cout<<ekfSlam.correction(gt_pose)<<std::endl;
//        gt_pose.w=gt_pose.w+twist.w;
//    }


}


int main(int argc, char **argv){

    testing::InitGoogleTest(&argc, argv);

  	//ros::init(argc, argv, "nuslam_test");

    return RUN_ALL_TESTS();
}
