//
// Created by larr-laptop on 24. 1. 11.
//
#include <analysis_los_keeper/Analyzer.h>

los_keeper::Analyzer::Analyzer(): nh_("~"){
    keeper_state_subscriber_ = nh_.subscribe("/los_simulator/drone_state",1,&Analyzer::CallbackKeeperState,this);
    target_state_subscriber_ = nh_.subscribe("/los_simulator/target_state",1,&Analyzer::CallbackTargetState,this);
    obstacle_list_state_subscriber_ = nh_.subscribe("/los_simulator/obstacle_list_state",1,&Analyzer::CallbackObstacleListState,this);

    keeper_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("current_keeper_vis",1);
    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("current_target_vis",1);
    obstacle_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("current_obstacle_list_vis",1);

    keeper_state_vis_.header.frame_id = "map";
    target_state_vis_.header.frame_id = "map";
    obstacle_state_vis_.header.frame_id = "map";
    //type
    keeper_state_vis_.type = visualization_msgs::Marker::CYLINDER;
    target_state_vis_.type = visualization_msgs::Marker::CYLINDER;
    obstacle_state_vis_.type = visualization_msgs::Marker::CYLINDER;
    // scale
    keeper_state_vis_.scale.x = 0.1;    // TODO: size
    keeper_state_vis_.scale.y = 0.1;
    keeper_state_vis_.scale.z = 0.1;
    target_state_vis_.scale.x = 0.1;
    target_state_vis_.scale.y = 0.1;
    target_state_vis_.scale.z = 0.1;
    obstacle_state_vis_.scale.x = 0.1;
    obstacle_state_vis_.scale.y = 0.1;
    obstacle_state_vis_.scale.z = 0.1;
    // color
    keeper_state_vis_.color.a = 1.0;
    keeper_state_vis_.color.r = 0.0;
    keeper_state_vis_.color.g = 0.0;
    keeper_state_vis_.color.b = 1.0;
    target_state_vis_.color.a = 1.0;
    target_state_vis_.color.r = 1.0;
    target_state_vis_.color.g = 0.0;
    target_state_vis_.color.b = 0.0;
    obstacle_state_vis_.color.a = 1.0;
    obstacle_state_vis_.color.r = 0.5;
    obstacle_state_vis_.color.g = 0.5;
    obstacle_state_vis_.color.b = 0.5;
    // Orientation
    target_state_vis_.pose.orientation.w = 1.0;
    target_state_vis_.pose.orientation.x = 0.0;
    target_state_vis_.pose.orientation.y = 0.0;
    target_state_vis_.pose.orientation.z = 0.0;
    keeper_state_vis_.pose.orientation.w = 1.0;
    keeper_state_vis_.pose.orientation.x = 0.0;
    keeper_state_vis_.pose.orientation.y = 0.0;
    keeper_state_vis_.pose.orientation.z = 0.0;
    obstacle_state_vis_.pose.orientation.w = 1.0;
    obstacle_state_vis_.pose.orientation.x = 0.0;
    obstacle_state_vis_.pose.orientation.y = 0.0;
    obstacle_state_vis_.pose.orientation.z = 0.0;
}

void los_keeper::Analyzer::run() {
    ros::Rate loop_rate(20.0);
    while(ros::ok()){
        VisualizeData();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void los_keeper::Analyzer::VisualizeData() {

}

void los_keeper::Analyzer::CallbackTargetState(const los_keeper::ObjectStatus_<std::allocator<void>>::ConstPtr &state) {
    current_target_state_.px = state->px;
    current_target_state_.py = state->py;
    current_target_state_.pz = state->pz;
    target_state_vis_.pose.position.x = state->px;
    target_state_vis_.pose.position.y = state->py;
    target_state_vis_.pose.position.z = state->pz;
    got_target_info = true;
}

void los_keeper::Analyzer::CallbackKeeperState(const los_keeper::ObjectStatus_<std::allocator<void>>::ConstPtr &state) {
    current_keeper_state_.px = state->px;
    current_keeper_state_.py = state->py;
    current_keeper_state_.pz = state->pz;
    keeper_state_vis_.pose.position.x = state->px;
    keeper_state_vis_.pose.position.y = state->py;
    keeper_state_vis_.pose.position.z = state->pz;
    got_keeper_info = true;
}

void los_keeper::Analyzer::CallbackObstacleListState(
        const los_keeper::ObjectStatusArray_<std::allocator<void>>::ConstPtr &state_list) {
    got_obstacle_info = true;
    current_obstacle_list_state_.clear();
    obstacle_state_list_vis_.markers.clear();
    State obstacle_info{0.0,0.0,0.0,0.0,0.0,0.0};
    for(int i =0;i<state_list->object_status_array.size();i++){
        obstacle_info.px = state_list->object_status_array[i].px;
        obstacle_info.py = state_list->object_status_array[i].py;
        obstacle_info.pz = state_list->object_status_array[i].pz;
        obstacle_state_vis_.pose.position.x = obstacle_info.px;
        obstacle_state_vis_.pose.position.y = obstacle_info.py;
        obstacle_state_vis_.pose.position.z = obstacle_info.pz;
        current_obstacle_list_state_.push_back(obstacle_info);
        obstacle_state_list_vis_.markers.push_back(obstacle_state_vis_);
    }
}
