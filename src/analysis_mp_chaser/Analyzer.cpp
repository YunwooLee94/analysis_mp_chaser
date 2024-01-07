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
        sub_target2_pose_ = nh_.subscribe("/airsim_node/RunningDynamicDraw_2_pose", 1, &Analyzer::CbTarget2Pose,
                                          this);
    }
    // Publisher


    t0_ = ros::Time::now().toSec();
}

void mp_chaser::Analyzer::run() {
    ros::Rate loop_rate(20.0);
    while (ros::ok()) {
        if (write_total_trajectory_)
            WriteCurrentPositions();

        ros::spinOnce();
        loop_rate.sleep();
    }
}

void mp_chaser::Analyzer::CbDronePose(const nav_msgs::Odometry_<std::allocator<void>>::ConstPtr &pose) {
    drone_position_.y = pose->pose.pose.position.y;
    drone_position_.x = pose->pose.pose.position.x;
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
                    target1_position_.x << " " << target1_position_.y << " " << target1_position_.z <<
                    target2_position_.x << " " << target2_position_.y << " " << target2_position_.z << endl;
    }
}

