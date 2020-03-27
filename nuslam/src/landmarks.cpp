/// \file: landmarks.cpp
/// \brief: create a ros node caller landmarks
///
/// PUBLISHES:
///     /landmarks (nuslam::TurtleMap): publish the ground truth landmarks data in gazebo(added with some noise)
/// SUBSCRIBES:
///     /scan (sensor_msgs::LaserScan): subscribe lase scan message


#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <math.h>
#include <sensor_msgs/LaserScan.h>
#include "nuslam/TurtleMap.h"
#include <Eigen/Dense>

const double PI=3.1415926535;

/// \brief class for the ros functions
class Landmarks{
public:
    Landmarks();
    ///call back functions
    void landmarkSubCallback(const sensor_msgs::LaserScan::ConstPtr& ls);

private:
    ros::NodeHandle n;
    ros::Subscriber landmark_sub;
    ros::Publisher landmark_pub;

    const double cluster_threshold=0.1;
    const int min_cluster_size=4;
};

/// \brief class for saving the cluster data
class ClusterData{
public:

    ClusterData(std::vector<float>x,std::vector<float>y){
        point_x=x;
        point_y=y;
    }
    ~ClusterData(){};

    std::vector<float> returnX(){
        return point_x;
    };

    std::vector<float> returnY(){
        return point_y;
    };

    void addVector(std::vector<float>add_x,std::vector<float>add_y){
        point_x.insert(point_x.end(),add_x.begin(),add_x.end());
        point_y.insert(point_y.end(),add_y.begin(),add_y.end());
    };

    int length(){
        return (int)point_x.size();
    }

private:
    std::vector<float>point_x;
    std::vector<float>point_y;
};

/// \brief function for fitting circle
///
/// \param clusters cluster data
/// \return std::vector<Eigen::Vector3f> a vector of the fitted circle data.
std::vector<Eigen::Vector3f> CircleFit(std::vector<ClusterData>clusters);

int main(int argc, char **argv) {
    ros::init(argc, argv, "landmarks");
    ros::NodeHandle n;

    ///wait for message
    ros::topic::waitForMessage<sensor_msgs::LaserScan>("/scan");

    Landmarks landmarks;

    ros::spin();
    return 0;
};


/// \brief constructor function
/// initialize the parameters
Landmarks::Landmarks() {
    landmark_sub = n.subscribe("scan", 100, &Landmarks::landmarkSubCallback,this);
    landmark_pub = n.advertise<nuslam::TurtleMap>("landmarks", 100,true);
};

/// \brief callback function
/// \param  ls - sensor_msgs::LaseScan
void Landmarks::landmarkSubCallback(const sensor_msgs::LaserScan::ConstPtr& ls)
{
    std::vector<float>ranges=ls->ranges;
    std::vector<ClusterData>clusters;
    std::vector<float>x_vec;
    std::vector<float>y_vec;

    /// 1. get all points, attribute to different clusters
    for (int i=0;i<(int)ranges.size();i++){
        if (!isinf(ranges[i])) {
            // push data
            x_vec.push_back(ranges[i] * cos(i * PI/ 180.0 ));
            y_vec.push_back(ranges[i] * sin(i * PI/ 180.0 ));

            // differ two different cluster
            if (abs(ranges[(i+1)%360] - ranges[(i)%360]) > cluster_threshold) {
                ClusterData clusterData(x_vec, y_vec);
                clusters.push_back(clusterData);
                x_vec.clear();
                y_vec.clear();
            }

            // when the last point and the first point belong to same cluster
            if((i==(int)ranges.size()-1)&&(abs(ranges[ranges.size()-1] - ranges[0]) < cluster_threshold)){
                clusters[0].addVector(x_vec, y_vec);
                x_vec.clear();
                y_vec.clear();
            }
        }
    }

    /// 2. delete clusters whose size is smaller than the threshold
    for(int i=0;i<(int)clusters.size();i++){
        if(clusters[i].length()<=min_cluster_size){
            clusters.erase(clusters.begin()+i);
        }
    }

    /// 3. Circle Fitting
    std::vector<Eigen::Vector3f> circle_param = CircleFit(clusters);
    //std::cout<<circle_param.size()<<std::endl;

    /// 4.Publish landmarks
    nuslam::TurtleMap turtle_map;
    for(int i=0;i<(int)circle_param.size();i++){
        //std::cout<<"i: "<<i<<" x: "<<circle_param[i][0]<<" y: "<<circle_param[i][1]<<std::endl;
        turtle_map.center_x.push_back(circle_param[i][0]);
        turtle_map.center_y.push_back(circle_param[i][1]);
        turtle_map.radius.push_back(circle_param[i][2]);
    }

    landmark_pub.publish(turtle_map);
}

std::vector<Eigen::Vector3f> CircleFit(std::vector<ClusterData>clusters){

    std::vector<Eigen::Vector3f> circle_params;

    /// circle fitting
    for(int i=0;i<(int)clusters.size();i++){

        std::vector<float> point_x=clusters[i].returnX();
        std::vector<float> point_y=clusters[i].returnY();

        //https://eigen.tuxfamily.org/dox/group__TutorialArrayClass.html
        /// 0. Configure the matrix
        Eigen::VectorXf x_ini = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(point_x.data(), point_x.size());
        Eigen::VectorXf x = x_ini.array()-x_ini.sum()/x_ini.size();
        Eigen::VectorXf y_ini = Eigen::Map<Eigen::VectorXf, Eigen::Unaligned>(point_y.data(), point_y.size());
        Eigen::VectorXf y = y_ini.array()-y_ini.sum()/y_ini.size();

        /// 0.5. classification for circle and walls
        /// In this part I want to classify circles and walls according to the notes, but no good effects.
        /// Instead I regard those whose radius is too large as walls.
        ///
        //        Eigen::VectorXf vec_begin_multi_end=(x_ini.array()-x_ini[0])*(x_ini.array()-x_ini[x_ini.size()-1])+(y_ini.array()-y_ini[0])*(y_ini.array()-y_ini[y_ini.size()-1]);
        //        Eigen::VectorXf len_vec_begin=sqrt((x_ini.array()-x_ini[0])*(x_ini.array()-x_ini[0])+(y_ini.array()-y_ini[0])*(y_ini.array()-y_ini[0]));
        //        Eigen::VectorXf len_vec_end=sqrt((x_ini.array()-x_ini[x_ini.size()-1]).abs2()+(y_ini.array()-y_ini[y_ini.size()-1]).abs2());
        //
        //        Eigen::VectorXf theta_vec=vec_begin_multi_end.array().array()/(len_vec_begin.array()*len_vec_end.array());
        //
        //        Eigen::VectorXf theta_vec_segment=theta_vec.segment(1,x_ini.size()-2);
        //
        //        for(int i=0;i<theta_vec_segment.size();i++){
        //            theta_vec_segment[i]=acos(theta_vec_segment[i])*180/PI;
        //        }
        //
        //        double mean_theta=theta_vec_segment.mean();
        //        double std_theta=sqrt((theta_vec_segment.array()-theta_vec_segment.mean()).abs2().mean());

        /// 1. get matrix Z
        Eigen::VectorXf z = x.array().square()+y.array().square();
        Eigen::VectorXf ones = Eigen::VectorXf::Ones(z.size());
        Eigen::MatrixXf mat_Z(z.size(),4);
        mat_Z<<z,x,y,ones;

        /// 2. get matrix M
        Eigen::MatrixXf mat_M=mat_Z.transpose()*mat_Z/z.size();

        /// 3. get H inverse
        Eigen::Matrix4f mat_H_inv;
        mat_H_inv<<0,0,0,0.5,0,1,0,0,0,0,1,0,0.5,0,0,-2*z.sum()/z.size();

        /// 4. SVD for Z
        Eigen::JacobiSVD<Eigen::MatrixXf> svd(mat_Z, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::VectorXf mat_Z_values=svd.singularValues();

        /// 5.Get the center for the circle
        Eigen::VectorXf vec_A;
        if (mat_Z_values[mat_Z_values.size()-1]<1e-12){
            vec_A=svd.matrixV().col(3);
        }
        else{
            Eigen::MatrixXf mat_Y=svd.matrixV()*svd.matrixU().transpose()*mat_Z;
            Eigen::MatrixXf mat_Q=mat_Y*mat_H_inv*mat_Y;

            Eigen::SelfAdjointEigenSolver< Eigen::MatrixXf> eigen_solver(mat_Q);
            Eigen::VectorXf mat_Q_eigenvalues=eigen_solver.eigenvalues();
            Eigen::VectorXf vec_A_star;
            if(mat_Q_eigenvalues[0]>0){vec_A_star=eigen_solver.eigenvectors().col(0);}
            else if(mat_Q_eigenvalues[1]>0){vec_A_star=eigen_solver.eigenvectors().col(1);}
            else if(mat_Q_eigenvalues[2]>0){vec_A_star=eigen_solver.eigenvectors().col(2);}
            else{vec_A_star=eigen_solver.eigenvectors().col(3);}

            vec_A = mat_Y.colPivHouseholderQr().solve(vec_A_star);
        }

        float center_a=-vec_A[1]*0.5/vec_A[0]+x_ini.sum()/x_ini.size();
        float center_b=-vec_A[2]*0.5/vec_A[0]+y_ini.sum()/y_ini.size();
        float R_2=(vec_A[1]*vec_A[1]+vec_A[2]*vec_A[2]-4*vec_A[0]*vec_A[3])*0.25/(vec_A[0]*vec_A[0]);

        /// 6.package the circle data.
        Eigen::Vector3f circle_vec(center_a,center_b,sqrt(R_2));

        if (sqrt(R_2)<0.08){//if the obstacle is a wall, the radius would be large. use this radius threshold to do do classification
            circle_params.push_back(circle_vec);
        }
    }
    return circle_params;
}

