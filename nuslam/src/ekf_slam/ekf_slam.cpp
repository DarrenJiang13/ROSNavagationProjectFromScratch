/// \file ekf_slam.cpp
/// \brief implementation of ekf_slam.hpp

#include "nuslam/ekf_slam.hpp"

EKF_slam::EKF_slam(){

    process_noise=0.01;
    measure_noise=0.001;

    pose_covalue=0.0001;
    measure_covalue=1000;

    state_vec=Eigen::VectorXf::Zero(3);

    process_noise_Q=Eigen::Matrix3f::Identity();
    process_noise_Q*=process_noise;

    measure_noise_R=Eigen::Matrix2f::Identity();
    measure_noise_R*=measure_noise;

    mat_covariance=Eigen::Matrix3f::Identity();
    mat_covariance*=pose_covalue;

    buf_count_limit=3;
    ass_distance_limit=0.1;
}

EKF_slam::EKF_slam(float process_noise_init,float measure_noise_init){

    process_noise=process_noise_init;
    measure_noise=measure_noise_init;

    pose_covalue=0.0001;
    measure_covalue=1000;

    state_vec=Eigen::VectorXf::Zero(3);

    process_noise_Q=Eigen::Matrix3f::Identity();
    process_noise_Q*=process_noise;

    measure_noise_R=Eigen::Matrix2f::Identity();
    measure_noise_R*=measure_noise;

    mat_covariance=Eigen::Matrix3f::Identity();
    mat_covariance*=pose_covalue;

    buf_count_limit=3;
    ass_distance_limit=0.1;
}
void EKF_slam::setLandmarks(std::vector<double>landmarks_x,std::vector<double>landmarks_y,std::vector<int>landmarks_index){
    center_x=landmarks_x;
    center_y=landmarks_y;
    index=landmarks_index;
}

void EKF_slam::predictKnownAssociation(rigid2d::Twist2D twist){

    /// 0. for the first iteration, set the state vector size to 3+12*3
    if (state_vec.size()==3){
        std::vector<double> landmarks_x={-0.81738, -0.825084, -0.844742, -0.215862, -0.22333, -0.215669, 0.43455, 0.399071, 0.719942, 0.742731, 0.03497, -0.526104};
        std::vector<double> landmarks_y={-0.829914, 0.010533, 0.664209, -0.848873, -0.016393, 0.688805, 0.449779, -0.446748, -0.02845, -0.792074, 0.407057, -0.516092};
        std::vector<int> landmarks_index={0,1,2,3,4,5,6,7,8,9,10,11};
        EKF_slam::setLandmarks(landmarks_x,landmarks_y,landmarks_index);

        state_vec.conservativeResize(state_vec.size()+center_x.size()*2);
        for(int i=0;i<(int)center_x.size();i++){
            state_vec[2*index[i]+3]=center_x[i]*cos(state_vec[0])-center_y[i]*sin(state_vec[0])+state_vec[1];
            state_vec[2*index[i]+4]=center_x[i]*sin(state_vec[0])+center_y[i]*cos(state_vec[0])+state_vec[2];
        }

        //mat_covariance update
        Eigen::MatrixXf mat_covariance_temp;
        mat_covariance_temp=Eigen::MatrixXf::Identity(state_vec.size(),state_vec.size());
        mat_covariance_temp*=1000;//measure_covalue
        mat_covariance_temp.topLeftCorner(3,3)=mat_covariance;
        mat_covariance=mat_covariance_temp;

        //process_noise_Q update
        Eigen::MatrixXf process_noise_Q_temp;
        process_noise_Q_temp=Eigen::MatrixXf::Zero(state_vec.size(),state_vec.size());
        process_noise_Q_temp.block<3,3>(0,0)=process_noise_Q;
        process_noise_Q=process_noise_Q_temp;
    }


    //initialize the state_vec prior
    state_vec_prior=state_vec;

    /// 1. add input vector
    float input_theta,input_x,input_y;
    if(abs(twist.w)<0.0001){
        input_theta=0;
        input_x=twist.vx*cos(state_vec[0]);
        input_y=twist.vx*sin(state_vec[0]);
    }else{
        input_theta=twist.w;
        input_x=twist.vx/twist.w*(-sin(state_vec[0])+sin(state_vec[0]+twist.w));
        input_y=twist.vx/twist.w*(cos(state_vec[0])-cos(state_vec[0]+twist.w));
    }

    Eigen::Vector3f input_vec={input_theta,input_x,input_y};
    state_vec_prior.segment(0,3).array()+=input_vec.array();

    /// 2. Covariance Matrix
    mat_G=Eigen::MatrixXf::Identity(state_vec.size(),state_vec.size());

    if(abs(twist.w)<0.0001){
        mat_G(1,0)=twist.vx*(-sin(state_vec[0]));
        mat_G(2,0)=twist.vx*(cos(state_vec[0]));
    }else{
        mat_G(1,0)=twist.vx/twist.w*(-cos(state_vec[0])+cos(state_vec[0]+twist.w));
        mat_G(2,0)=twist.vx/twist.w*(-sin(state_vec[0])+sin(state_vec[0]+twist.w));
    }

    /// 3. Process noise
    process_noise_Q(0,0)=process_noise*input_theta*input_theta;
    process_noise_Q(0,1)=process_noise*input_theta*input_x;
    process_noise_Q(0,2)=process_noise*input_theta*input_y;
    process_noise_Q(1,0)=process_noise*input_x*input_theta;
    process_noise_Q(1,1)=process_noise*input_x*input_x;
    process_noise_Q(1,2)=process_noise*input_x*input_y;
    process_noise_Q(2,0)=process_noise*input_y*input_theta;
    process_noise_Q(2,1)=process_noise*input_y*input_x;
    process_noise_Q(2,2)=process_noise*input_y*input_y;

    mat_covariance_prior=mat_G*mat_covariance*mat_G.transpose()+process_noise_Q;
//    state_vec=state_vec_prior;
}

rigid2d::Twist2D EKF_slam::correctionKnownAssociation(){

    Eigen::VectorXf state_vec_post=state_vec_prior;
    Eigen::MatrixXf mat_covariance_post=mat_covariance_prior;

    for(int i=0;i<(int)center_x.size();i++){

        // 1. compute z and z_estimate
        //z:as data association is known, z is actual distance between robot pose(ground truth from gazebo) and landmarks(ground truth from gazebo)
        //z_estimate: distance between my slam robot pose and read landmarks
        //z
        float gt_dx=center_x[i];
        float gt_dy=center_y[i];
        float gt_distance=gt_dx*gt_dx+gt_dy*gt_dy;
        Eigen::Vector2f vec_Zi;
        vec_Zi[0]=sqrt(gt_distance);
        vec_Zi[1]=atan2(gt_dy,gt_dx);//normalize_angle
        //z_estimate
        float dx=state_vec_prior[2*index[i]+3]-state_vec_prior[1];
        float dy=state_vec_prior[2*index[i]+4]-state_vec_prior[2];
        float distance=dx*dx+dy*dy;
        Eigen::Vector2f vec_Zi_estimate;
        vec_Zi_estimate[0]=sqrt(distance);
        vec_Zi_estimate[1]=atan2(dy,dx)-state_vec_prior[0];//normalize_angle

        /// 2. Matrix Hi
        Eigen::MatrixXf mat_Hi=Eigen::MatrixXf::Zero(2,state_vec.size());
        Eigen::MatrixXf mat_Hi_left(2,3);
        mat_Hi_left<<0,-dx/sqrt(distance),-dy/sqrt(distance),-1,dy/distance, -dx/distance;
        Eigen::MatrixXf mat_Hi_right(2,2);
        mat_Hi_right<<dx/sqrt(distance),dy/sqrt(distance),-dy/distance, dx/distance;
        mat_Hi.block<2,3>(0,0)=mat_Hi_left;
        mat_Hi.block<2,2>(0,2*index[i]+3)=mat_Hi_right;

        /// 3.Kalman Gain
        Eigen::MatrixXf mat_kal_gain=mat_covariance_post*mat_Hi.transpose()*((mat_Hi*mat_covariance_post*mat_Hi.transpose()+measure_noise_R).inverse());

        /// 4.state_vec_post update
        Eigen::Vector2f delta_Zi;
        delta_Zi[0]=(vec_Zi[0]-vec_Zi_estimate[0]);
        delta_Zi[1]=(rigid2d::normalize_angle(vec_Zi[1]-vec_Zi_estimate[1]));
        state_vec_post=state_vec_post+mat_kal_gain*(delta_Zi);

        /// 5.mat_covariance_post update
        mat_covariance_post=mat_covariance_post-mat_kal_gain*mat_Hi*mat_covariance_post;
    }
    mat_covariance=mat_covariance_post;
    state_vec=state_vec_post;
    rigid2d::Twist2D slam_pose={state_vec_post[0],state_vec_post[1],state_vec_post[2]};
    return slam_pose;
}

void EKF_slam::landmarkAssociation(std::vector<double>landmarks_x,std::vector<double>landmarks_y){
    /// 0.Define a landmark buffer temporary
    std::vector<double>landmark_x_buf_temp;
    std::vector<double>landmark_y_buf_temp;
    std::vector<int>count_buf_temp;
    if(landmark_x_buf.size()!=0){
        landmark_x_buf_temp=landmark_x_buf;
        landmark_y_buf_temp=landmark_y_buf;
        count_buf_temp=count_buf;
    }

    /// 1. Compare all input with buffer, if same, buffer count+1, if diverse, add a new buffer landmark
    for(int i=0;i<(int)landmarks_x.size();i++) {
        ///0. Compare the input landmark with buffer
        //if no buffer, input as a new buffer
        if (landmark_x_buf.size() == 0) {
            landmark_x_buf_temp.push_back(landmarks_x[i]);
            landmark_y_buf_temp.push_back(landmarks_y[i]);
            count_buf_temp.push_back(1);
        } else {
            //if buffer, find a existing buffer landmark closest to it.
            double smallest_distance = 4.00;
            int smallest_index = -1;
            for (int j = 0; j < (int) landmark_x_buf.size(); j++) {
                double buf_distance =
                        pow(landmarks_x[i] - landmark_x_buf[j], 2) + pow(landmarks_y[i] - landmark_y_buf[j], 2);
                if (buf_distance < smallest_distance) {
                    smallest_distance = buf_distance;
                    smallest_index = j;
                }
            }
            //if we can find the smallest distance, which always happen
            if (smallest_index > -1) {
                if (smallest_distance < ass_distance_limit) {
                    // minimum distance is smaller than the limit, this landmark appears before
                    landmark_x_buf_temp[smallest_index]=0.5*(landmark_x_buf_temp[smallest_index]+landmarks_x[smallest_index]);
                    landmark_y_buf_temp[smallest_index]=0.5*(landmark_y_buf_temp[smallest_index]+landmarks_y[smallest_index]);
                    count_buf_temp[smallest_index] = count_buf_temp[i] + 1;
                } else {
                    // minimum distance is largerer than the limit, new landmark appears
                    landmark_x_buf_temp.push_back(landmarks_x[i]);
                    landmark_y_buf_temp.push_back(landmarks_y[i]);
                    count_buf_temp.push_back(1);
                }
            } else {
                //if smallest_index is still -1,
                // the new landmark is more than 2 meters far from the previous landmarks,which is
                // definitely wrong data.
                //Just ignore it.
            }
        }
    }

    landmark_x_buf=landmark_x_buf_temp;
    landmark_y_buf=landmark_y_buf_temp;
    count_buf=count_buf_temp;
    //std::cout<<"buffer done. "<<std::endl;

    /// 2. Compare all buffer landmarks with state vector landmarks, if close, set a previous index, if far add a new index
    std::vector<double>center_x_temp;
    std::vector<double>center_y_temp;
    std::vector<int>index_temp;
    int landmarks_count_temp=((int)state_vec.size()-3)/2;//how many landmarks do we have now


    int buffer_size=(int)landmark_x_buf.size();

    /// 1. deal with all buffer data
    for (int k=0;k<buffer_size;k++){

        if(count_buf[k]>=buf_count_limit){
                int min_index = -1;
                double min_distance = 4.0;
                for (int j = 0; j < (int)(state_vec.size()-3)/2; j++) {
                    double buf_distance =
                            pow(landmark_x_buf[k] - state_vec[2*j+3], 2) + pow(landmark_y_buf[k] - state_vec[2*j+4], 2);
                    if (buf_distance < min_distance) {
                        min_distance = buf_distance;
                        min_index = j;
                    }
                }
                //if we can find the minimum distance, which always happen
                if (min_index > -1) {
                    if (min_distance < ass_distance_limit) {
                        // minimum distance is smaller than the limit, this landmark appears before
                        center_x_temp.push_back(landmark_x_buf[k]);
                        center_y_temp.push_back(landmark_y_buf[k]);
                        index_temp.push_back(min_index);
                    } else {
                        // minimum distance is largerer than the limit, new landmark appears
                        center_x_temp.push_back(landmark_x_buf[k]);
                        center_y_temp.push_back(landmark_y_buf[k]);
                        index_temp.push_back(landmarks_count_temp);
                        landmarks_count_temp=landmarks_count_temp+1;
                    }
                } else {
                    //if smallest_index is still -1,
                    // the new landmark is more than 2 meters far from the previous landmarks,which is
                    // definitely wrong data.
                    //Just ignore it.

                    //one exception: the state vector size is 3, so no comparision processed.
                    if(state_vec.size()==3){
                        center_x_temp.push_back(landmark_x_buf[k]);
                        center_y_temp.push_back(landmark_y_buf[k]);
                        index_temp.push_back(landmarks_count_temp);
                        landmarks_count_temp=landmarks_count_temp+1;
                    }
                }

        }
    }

    /// 3. get all landmarks to input with its index
    center_x=center_x_temp;
    center_y=center_y_temp;
    index=index_temp;

    //erase all the landmarks which already been merged to the state vector.
    for(int l=0;l<(int)landmark_x_buf.size();l++){
        if(count_buf[l]>=buf_count_limit){
            landmark_x_buf.erase(landmark_x_buf.begin()+l);
            landmark_y_buf.erase(landmark_y_buf.begin()+l);
            count_buf.erase(count_buf.begin()+l);
            l=l-1;
        }
    }

    //get rid of some unessential landmarks which appear only once or twice.
    while(landmark_x_buf.size()>30){
        landmark_x_buf.erase(landmark_x_buf.begin());
        landmark_y_buf.erase(landmark_y_buf.begin());
        count_buf.erase(count_buf.begin());
    }

//    for(int i=0;i<(int)index.size();i++){
//        std::cout<<"index: "<<index[i]<<std::endl;
//    }

}

void EKF_slam::predict(rigid2d::Twist2D twist){

    int landmark_number_in_state_vec=(state_vec.size()-3)/2;
    for (int i=0;i<(int)center_x.size();i++){
        if(index[i]>(landmark_number_in_state_vec-1)){
            state_vec.conservativeResize(state_vec.size()+2);
            state_vec[2*index[i]+3]=center_x[i]*cos(state_vec[0])-center_y[i]*sin(state_vec[0])+state_vec[1];
            state_vec[2*index[i]+4]=center_x[i]*sin(state_vec[0])+center_y[i]*cos(state_vec[0])+state_vec[2];
        }
    }

    //mat_covariance update
    Eigen::MatrixXf mat_covariance_temp;
    mat_covariance_temp=Eigen::MatrixXf::Identity(state_vec.size(),state_vec.size());
    mat_covariance_temp*=1000;//measure_covalue
    mat_covariance_temp.topLeftCorner(landmark_number_in_state_vec*2+3,landmark_number_in_state_vec*2+3)=mat_covariance;
    mat_covariance=mat_covariance_temp;

    //process_noise_Q update
    Eigen::MatrixXf process_noise_Q_temp;
    process_noise_Q_temp=Eigen::MatrixXf::Zero(state_vec.size(),state_vec.size());
    process_noise_Q_temp.topLeftCorner(landmark_number_in_state_vec*2+3,landmark_number_in_state_vec*2+3)=process_noise_Q;
    process_noise_Q=process_noise_Q_temp;

    //initialize the state_vec prior
    state_vec_prior=state_vec;

    /// 1. add input vector
    float input_theta,input_x,input_y;
    if(abs(twist.w)<0.0001){
        input_theta=0;
        input_x=twist.vx*cos(state_vec[0]);
        input_y=twist.vx*sin(state_vec[0]);
    }else{
        input_theta=twist.w;
        input_x=twist.vx/twist.w*(-sin(state_vec[0])+sin(state_vec[0]+twist.w));
        input_y=twist.vx/twist.w*(cos(state_vec[0])-cos(state_vec[0]+twist.w));
    }

    Eigen::Vector3f input_vec={input_theta,input_x,input_y};
    state_vec_prior.segment(0,3).array()+=input_vec.array();

    /// 2. Covariance Matrix
    mat_G=Eigen::MatrixXf::Identity(state_vec.size(),state_vec.size());

    if(abs(twist.w)<0.0001){
        mat_G(1,0)=twist.vx*(-sin(state_vec[0]));
        mat_G(2,0)=twist.vx*(cos(state_vec[0]));
    }else{
        mat_G(1,0)=twist.vx/twist.w*(-cos(state_vec[0])+cos(state_vec[0]+twist.w));
        mat_G(2,0)=twist.vx/twist.w*(-sin(state_vec[0])+sin(state_vec[0]+twist.w));
    }

    /// 3. Process noise
    process_noise_Q(0,0)=process_noise*input_theta*input_theta;
    process_noise_Q(0,1)=process_noise*input_theta*input_x;
    process_noise_Q(0,2)=process_noise*input_theta*input_y;
    process_noise_Q(1,0)=process_noise*input_x*input_theta;
    process_noise_Q(1,1)=process_noise*input_x*input_x;
    process_noise_Q(1,2)=process_noise*input_x*input_y;
    process_noise_Q(2,0)=process_noise*input_y*input_theta;
    process_noise_Q(2,1)=process_noise*input_y*input_x;
    process_noise_Q(2,2)=process_noise*input_y*input_y;

    mat_covariance_prior=mat_G*mat_covariance*mat_G.transpose()+process_noise_Q;
    //state_vec=state_vec_prior;
}

rigid2d::Twist2D EKF_slam::correction(){

    Eigen::VectorXf state_vec_post=state_vec_prior;
    Eigen::MatrixXf mat_covariance_post=mat_covariance_prior;

    for(int i=0;i<(int)center_x.size();i++){

        // 1. compute z and z_estimate
        //z:as data association is known, z is actual distance between robot pose(ground truth from gazebo) and landmarks(ground truth from gazebo)
        //z_estimate: distance between my slam robot pose and read landmarks
        //z
        float gt_dx=center_x[i];
        float gt_dy=center_y[i];
        float gt_distance=gt_dx*gt_dx+gt_dy*gt_dy;
        Eigen::Vector2f vec_Zi;
        vec_Zi[0]=sqrt(gt_distance);
        vec_Zi[1]=atan2(gt_dy,gt_dx);//normalize_angle
        //z_estimate
        float dx=state_vec_prior[2*index[i]+3]-state_vec_prior[1];
        float dy=state_vec_prior[2*index[i]+4]-state_vec_prior[2];
        float distance=dx*dx+dy*dy;
        Eigen::Vector2f vec_Zi_estimate;
        vec_Zi_estimate[0]=sqrt(distance);
        vec_Zi_estimate[1]=atan2(dy,dx)-state_vec_prior[0];//normalize_angle

        /// 2. Matrix Hi
        Eigen::MatrixXf mat_Hi=Eigen::MatrixXf::Zero(2,state_vec.size());
        Eigen::MatrixXf mat_Hi_left(2,3);
        mat_Hi_left<<0,-dx/sqrt(distance),-dy/sqrt(distance),-1,dy/distance, -dx/distance;
        Eigen::MatrixXf mat_Hi_right(2,2);
        mat_Hi_right<<dx/sqrt(distance),dy/sqrt(distance),-dy/distance, dx/distance;
        mat_Hi.block<2,3>(0,0)=mat_Hi_left;
        mat_Hi.block<2,2>(0,2*index[i]+3)=mat_Hi_right;

        /// 3.Kalman Gain
        Eigen::MatrixXf mat_kal_gain=mat_covariance_post*mat_Hi.transpose()*((mat_Hi*mat_covariance_post*mat_Hi.transpose()+measure_noise_R).inverse());

        /// 4.state_vec_post update
        Eigen::Vector2f delta_Zi;
        delta_Zi[0]=(vec_Zi[0]-vec_Zi_estimate[0]);
        delta_Zi[1]=(rigid2d::normalize_angle(vec_Zi[1]-vec_Zi_estimate[1]));
        state_vec_post=state_vec_post+mat_kal_gain*(delta_Zi);

        /// 5.mat_covariance_post update
        mat_covariance_post=mat_covariance_post-mat_kal_gain*mat_Hi*mat_covariance_post;
    }
    mat_covariance=mat_covariance_post;
    state_vec=state_vec_post;
    rigid2d::Twist2D slam_pose={state_vec_post[0],state_vec_post[1],state_vec_post[2]};
    return slam_pose;
}

rigid2d::Twist2D EKF_slam::getPose(){
    rigid2d::Twist2D pose={state_vec[0],state_vec[1],state_vec[2]};
    return pose;
}