//
// Created by larr-laptop on 24. 1. 6.
//
#include <analysis_mp_chaser/Analyzer.h>

using namespace std;

mp_chaser::Analyzer::Analyzer() : nh_("~") {
    nh_.param<bool>("is_dual", is_dual_, false);
    nh_.param<bool>("write_total_trajectory", write_total_trajectory_, false);
    nh_.param<std::string>("total_trajectory_filename_write", total_trajectory_filename_write_, "");
    nh_.param<std::string>("total_trajectory_filename_read", total_trajectory_filename_read_, "");

    // Subscriber
    sub_drone_pose_ = nh_.subscribe("/airsim_node/Chaser/odom_local_ned", 1, &Analyzer::CbDronePose, this);
    if (not is_dual_) { // SINGLE TARGET
        sub_target1_pose_ = nh_.subscribe("/airsim_node/RunningActor_2_pose", 1, &Analyzer::CbTarget1Pose, this);
    } else {   // DUAL TARGET
        sub_target1_pose_ = nh_.subscribe("/airsim_node/RunningActorDraw_2_pose", 1, &Analyzer::CbTarget1Pose,
                                          this);
        sub_target2_pose_ = nh_.subscribe("/airsim_node/RunningActorDynamicDraw_2_pose", 1, &Analyzer::CbTarget2Pose,
                                          this);
    }
    // Publisher
    pub_drone_total_trajectory_ = nh_.advertise<nav_msgs::Path>("drone_total_trajectory",1);
    pub_target1_total_trajectory_ = nh_.advertise<nav_msgs::Path>("target1_total_trajectory",1);
    pub_target2_total_trajectory_ = nh_.advertise<nav_msgs::Path>("target2_total_trajectory",1);
    pub_total_bearing_vector_history_ = nh_.advertise<visualization_msgs::Marker>("bearing_vector_history",1);

    drone_total_trajectory_vis_.header.frame_id = "world_enu";
    target1_total_trajectory_vis_.header.frame_id = "world_enu";
    target2_total_trajectory_vis_.header.frame_id = "world_enu";
    bearing_vector_vis_.header.frame_id ="world_enu";
    bearing_vector_vis_.type = visualization_msgs::Marker::LINE_LIST;
    bearing_vector_vis_.color.a = 0.3;
    bearing_vector_vis_.color.r = 0.54;
    bearing_vector_vis_.color.g = 0.17;
    bearing_vector_vis_.color.b = 0.88;
    bearing_vector_vis_.scale.x = 0.01;


    t0_ = ros::Time::now().toSec();
}

void mp_chaser::Analyzer::run() {
    ros::Rate loop_rate(20.0);
    ReadActorTrajectories();
    while (ros::ok()) {
        if (write_total_trajectory_)
            WriteCurrentPositions();
        PublishVisualization();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void mp_chaser::Analyzer::CbDronePose(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &pose) {
    drone_position_.x = pose->pose.pose.position.y;
    drone_position_.y = pose->pose.pose.position.x;
    drone_position_.z = -pose->pose.pose.position.z;
    drone_position_flag_ = true;
}

void mp_chaser::Analyzer::CbTarget1Pose(const geometry_msgs::PoseStamped_<std::allocator<void>>::ConstPtr &pose) {
    target1_position_.x = pose->pose.position.x;
    target1_position_.y = pose->pose.position.y;
    target1_position_.z = pose->pose.position.z;
    target1_position_flag_ = true;
}

void mp_chaser::Analyzer::CbTarget2Pose(const geometry_msgs::PoseStamped_<std::allocator<void>>::ConstPtr &pose) {
    target2_position_.x = pose->pose.position.x;
    target2_position_.y = pose->pose.position.y;
    target2_position_.z = pose->pose.position.z;
    target2_position_flag_ = true;
}

void mp_chaser::Analyzer::WriteCurrentPositions() {
    double current_time = ros::Time::now().toSec() - t0_;
    ofstream outfile;
    outfile.open(total_trajectory_filename_write_, std::ios_base::app);
    if (not is_dual_) {
        if (drone_position_flag_ and target1_position_flag_)
            outfile << current_time << " " << drone_position_.x << " " << drone_position_.y << " " << drone_position_.z
                    << " " <<
                    target1_position_.x << " " << target1_position_.y << " " << target1_position_.z << endl;
    } else {
        if (drone_position_flag_ and target1_position_flag_ and target2_position_flag_)
            outfile << current_time << " " << drone_position_.x << " " << drone_position_.y << " " << drone_position_.z
                    << " " <<
                    target1_position_.x << " " << target1_position_.y << " " << target1_position_.z << " " <<
                    target2_position_.x << " " << target2_position_.y << " " << target2_position_.z << endl;
    }
}

void mp_chaser::Analyzer::ReadActorTrajectories() {
    ifstream actor_trajectory_file;
    actor_trajectory_file.open(total_trajectory_filename_read_.c_str());
    if (actor_trajectory_file.is_open()) {
        if (not is_dual_) { // SINGLE TARGET
            Point drone_position_temp{0.0, 0.0, 0.0};
            Point target1_position_temp{0.0, 0.0, 0.0};
            double time_temp{0.0};
            while (actor_trajectory_file >> time_temp >> drone_position_temp.x >> drone_position_temp.y
                                         >> drone_position_temp.z >>
                                         target1_position_temp.x >> target1_position_temp.y
                                         >> target1_position_temp.z) {
                time_history_.push_back(time_temp);
                drone_total_trajectory_.push_back(drone_position_temp);
                target1_total_trajectory_.push_back(target1_position_temp);
            }
        } else {   // DUAL TARGET
            Point drone_position_temp{0.0, 0.0, 0.0};
            Point target1_position_temp{0.0, 0.0, 0.0};
            Point target2_position_temp{0.0, 0.0, 0.0};
            double time_temp{0.0};
            while (actor_trajectory_file >> time_temp >> drone_position_temp.x >> drone_position_temp.y
                                         >> drone_position_temp.z >>
                                         target1_position_temp.x >> target1_position_temp.y >> target1_position_temp.z
                                         >>
                                         target2_position_temp.x >> target2_position_temp.y
                                         >> target2_position_temp.z) {
                time_history_.push_back(time_temp);
                drone_total_trajectory_.push_back(drone_position_temp);
                target1_total_trajectory_.push_back(target1_position_temp);
                target2_total_trajectory_.push_back(target2_position_temp);
            }
        }
    }

    if (not drone_total_trajectory_.empty()) {
        geometry_msgs::PoseStamped drone_pose_temp;
        drone_pose_temp.pose.orientation.w = 1.0;
        drone_pose_temp.pose.orientation.x = 0.0;
        drone_pose_temp.pose.orientation.y = 0.0;
        drone_pose_temp.pose.orientation.z = 0.0;
        for (const auto & position_temp : drone_total_trajectory_) {
            drone_pose_temp.pose.position.x = position_temp.x;
            drone_pose_temp.pose.position.y = position_temp.y;
            drone_pose_temp.pose.position.z = position_temp.z;
            drone_total_trajectory_vis_.poses.push_back(drone_pose_temp);
        }
    }
    if (not target1_total_trajectory_.empty()) {
        geometry_msgs::PoseStamped target1_pose_temp;
        target1_pose_temp.pose.orientation.w = 1.0;
        target1_pose_temp.pose.orientation.x = 0.0;
        target1_pose_temp.pose.orientation.y = 0.0;
        target1_pose_temp.pose.orientation.z = 0.0;
        for (const auto & position_temp : target1_total_trajectory_) {
            target1_pose_temp.pose.position.x = position_temp.x;
            target1_pose_temp.pose.position.y = position_temp.y;
            target1_pose_temp.pose.position.z = position_temp.z;
            target1_total_trajectory_vis_.poses.push_back(target1_pose_temp);
        }
    }
    if (not target2_total_trajectory_.empty()) {
        geometry_msgs::PoseStamped target2_pose_temp;
        target2_pose_temp.pose.orientation.w = 1.0;
        target2_pose_temp.pose.orientation.x = 0.0;
        target2_pose_temp.pose.orientation.y = 0.0;
        target2_pose_temp.pose.orientation.z = 0.0;
        for (const auto & position_temp : target2_total_trajectory_) {
            target2_pose_temp.pose.position.x = position_temp.x;
            target2_pose_temp.pose.position.y = position_temp.y;
            target2_pose_temp.pose.position.z = position_temp.z;
            target2_total_trajectory_vis_.poses.push_back(target2_pose_temp);
        }
    }
    if(not is_dual_){
        if(not(drone_total_trajectory_.empty() or target1_total_trajectory_.empty())){
            geometry_msgs::Point drone_position_temp;
            geometry_msgs::Point target1_position_temp;
            for(int i =0;i<drone_total_trajectory_.size();i++){
                drone_position_temp.x = drone_total_trajectory_[i].x;
                drone_position_temp.y = drone_total_trajectory_[i].y;
                drone_position_temp.z = drone_total_trajectory_[i].z;
                bearing_vector_vis_.points.push_back(drone_position_temp);
                target1_position_temp.x = target1_total_trajectory_[i].x;
                target1_position_temp.y = target1_total_trajectory_[i].y;
                target1_position_temp.z = target1_total_trajectory_[i].z;
                bearing_vector_vis_.points.push_back(target1_position_temp);
            }
        }
    }
    else{
        if(not(drone_total_trajectory_.empty() or target1_total_trajectory_.empty() or target2_total_trajectory_.empty())){
            geometry_msgs::Point drone_position_temp;
            geometry_msgs::Point target1_position_temp;
            geometry_msgs::Point target2_position_temp;
            for(int i =0;i<drone_total_trajectory_.size();i++){
                //target1
                drone_position_temp.x = drone_total_trajectory_[i].x;
                drone_position_temp.y = drone_total_trajectory_[i].y;
                drone_position_temp.z = drone_total_trajectory_[i].z;
                bearing_vector_vis_.points.push_back(drone_position_temp);
                target1_position_temp.x = target1_total_trajectory_[i].x;
                target1_position_temp.y = target1_total_trajectory_[i].y;
                target1_position_temp.z = target1_total_trajectory_[i].z;
                bearing_vector_vis_.points.push_back(target1_position_temp);
                //target2
                bearing_vector_vis_.points.push_back(drone_position_temp);
                target2_position_temp.x = target2_total_trajectory_[i].x;
                target2_position_temp.y = target2_total_trajectory_[i].y;
                target2_position_temp.z = target2_total_trajectory_[i].z;
                bearing_vector_vis_.points.push_back(target2_position_temp);
            }
        }
    }
}

void mp_chaser::Analyzer::PublishVisualization() {
    if(not drone_total_trajectory_vis_.poses.empty())
        pub_drone_total_trajectory_.publish(drone_total_trajectory_vis_);
    if(not target1_total_trajectory_vis_.poses.empty())
        pub_target1_total_trajectory_.publish(target1_total_trajectory_vis_);
    if(not target2_total_trajectory_vis_.poses.empty())
        pub_target2_total_trajectory_.publish(target2_total_trajectory_vis_);
    if(not bearing_vector_vis_.points.empty())
        pub_total_bearing_vector_history_.publish(bearing_vector_vis_);
}

