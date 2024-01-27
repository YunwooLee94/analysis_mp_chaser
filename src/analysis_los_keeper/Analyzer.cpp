//
// Created by larr-laptop on 24. 1. 11.
//
#include <analysis_los_keeper/Analyzer.h>

using namespace std;

los_keeper::Analyzer::Analyzer() : nh_("~") {
    nh_.param<bool>("is_2d", is_2d_, true);
    nh_.param<bool>("is_exp", is_exp_, false);

    nh_.param<bool>("calculate_min_distance", calculate_min_distance_, false);
    nh_.param<string>("min_distance_file_name", min_distance_filename_, "");

    nh_.param<bool>("calculate_visibility_score", calculate_visibility_score_, false);
    nh_.param<string>("visibility_score_file_name", visibility_score_filename_, "");

    nh_.param<bool>("write_total_trajectory", write_total_trajectory_, false);
    nh_.param<string>("total_trajectory_filename_write", total_trajectory_filename_write_, "");
    nh_.param<string>("total_trajectory_filename_read", total_trajectory_filename_read_, "");

    // Boundary
    nh_.param<double>("x_min", boundary_.x_min, 0.0);
    nh_.param<double>("x_max", boundary_.x_max, 0.0);
    nh_.param<double>("y_min", boundary_.y_min, 0.0);
    nh_.param<double>("y_max", boundary_.y_max, 0.0);
    nh_.param<double>("z_min", boundary_.z_min, 0.0);
    nh_.param<double>("z_max", boundary_.z_max, 0.0);

    keeper_state_subscriber_ = nh_.subscribe("/los_simulator/drone_state", 1, &Analyzer::CallbackKeeperState, this);
    target_state_subscriber_ = nh_.subscribe("/los_simulator/target_state", 1, &Analyzer::CallbackTargetState, this);
    obstacle_list_state_subscriber_ = nh_.subscribe("/los_simulator/obstacle_list_state", 1,
                                                    &Analyzer::CallbackObstacleListState, this);

    keeper_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("current_keeper_vis", 1);
    target_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("current_target_vis", 1);
    obstacle_list_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>("current_obstacle_list_vis", 1);
    obstacle_prediction_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("obstacle_prediction_vis", 1);

    keeper_total_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("keeper_total_trajectory_vis", 1);
    target_total_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("target_total_trajectory_vis", 1);
    obstacle_list_total_vis_publisher_ = nh_.advertise<visualization_msgs::MarkerArray>(
            "obstacle_list_total_trajectory_vis", 1);
    bearing_vector_history_publisher_ = nh_.advertise<visualization_msgs::Marker>("bearing_vector_history", 1);
    boundary_vis_publisher_ = nh_.advertise<visualization_msgs::Marker>("boundary_vis", 1);


    keeper_state_vis_.header.frame_id = "map";
    target_state_vis_.header.frame_id = "map";
    obstacle_state_vis_.header.frame_id = "map";
    //type
    if (is_2d_) {
        keeper_state_vis_.type = visualization_msgs::Marker::CYLINDER;
        target_state_vis_.type = visualization_msgs::Marker::CYLINDER;
        obstacle_state_vis_.type = visualization_msgs::Marker::CYLINDER;
    } else {
        keeper_state_vis_.type = visualization_msgs::Marker::SPHERE;
        target_state_vis_.type = visualization_msgs::Marker::SPHERE;
        obstacle_state_vis_.type = visualization_msgs::Marker::SPHERE;
    }
    // scale
    if (is_2d_) {
        keeper_state_vis_.scale.x = 0.14;    // TODO: size
        keeper_state_vis_.scale.y = 0.14;
        if (is_exp_) {
            keeper_state_vis_.scale.x = 0.15;
            keeper_state_vis_.scale.y = 0.15;
        }
        keeper_state_vis_.scale.z = 10.0;
        target_state_vis_.scale.x = 0.14;
        target_state_vis_.scale.y = 0.14;
        if (is_exp_) {
            target_state_vis_.scale.x = 0.15;
            target_state_vis_.scale.y = 0.15;
        }
        target_state_vis_.scale.z = 10.0;
        obstacle_state_vis_.scale.x = 0.14;
        obstacle_state_vis_.scale.y = 0.14;
        if (is_exp_) {
            obstacle_state_vis_.scale.x = 0.15;
            obstacle_state_vis_.scale.y = 0.15;
        }
        obstacle_state_vis_.scale.z = 10.0;
    } else {
        keeper_state_vis_.scale.x = 0.3;    // TODO: size
        keeper_state_vis_.scale.y = 0.3;
        keeper_state_vis_.scale.z = 0.3;
        if (is_exp_) {
            keeper_state_vis_.scale.x = 0.15;
            keeper_state_vis_.scale.y = 0.15;
            keeper_state_vis_.scale.z = 0.15;
        }
        target_state_vis_.scale.x = 0.3;
        target_state_vis_.scale.y = 0.3;
        target_state_vis_.scale.z = 0.3;
        if (is_exp_) {
            target_state_vis_.scale.x = 0.15;
            target_state_vis_.scale.y = 0.15;
            target_state_vis_.scale.z = 0.15;
        }
        obstacle_state_vis_.scale.x = 0.3;
        obstacle_state_vis_.scale.y = 0.3;
        obstacle_state_vis_.scale.z = 0.3;
        if (is_exp_) {
            obstacle_state_vis_.scale.x = 0.15;
            obstacle_state_vis_.scale.y = 0.15;
            obstacle_state_vis_.scale.z = 0.15;
        }
    }

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
    obstacle_state_vis_.color.r = 0.0;
    obstacle_state_vis_.color.g = 0.0;
    obstacle_state_vis_.color.b = 0.0;
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

    obstacle_state_vis_.ns = "Obstacle";

    obstacle_prediction_vis_.header.frame_id = "map";
    obstacle_prediction_vis_.type = visualization_msgs::Marker::LINE_LIST;
    obstacle_prediction_vis_.color.a = 0.5;
    obstacle_prediction_vis_.color.r = 0.0;
    obstacle_prediction_vis_.color.g = 0.0;
    obstacle_prediction_vis_.color.b = 0.0;
    obstacle_prediction_vis_.pose.orientation.w = 1.0;
    obstacle_prediction_vis_.pose.orientation.x = 0.0;
    obstacle_prediction_vis_.pose.orientation.y = 0.0;
    obstacle_prediction_vis_.pose.orientation.z = 0.0;
    obstacle_prediction_vis_.scale.x = 0.02;

    num_obstacle_ = 69; // TODO: Automatically count the number of obstacles.
    if (is_exp_)
        num_obstacle_ = 8;
    obstacle_total_trajectory_.resize(num_obstacle_);

    keeper_total_trajectory_vis_.header.frame_id = "map";
    target_total_trajectory_vis_.header.frame_id = "map";
    obstacle_separate_trajectory_vis_.header.frame_id = "map";
    // Keeper
    keeper_total_trajectory_vis_.type = visualization_msgs::Marker::LINE_STRIP;
    keeper_total_trajectory_vis_.scale.x = 0.05;
    keeper_total_trajectory_vis_.color.a = 0.8;
    keeper_total_trajectory_vis_.color.r = 0.0;
    keeper_total_trajectory_vis_.color.g = 0.0;
    keeper_total_trajectory_vis_.color.b = 1.0;
    keeper_total_trajectory_vis_.action = visualization_msgs::Marker::ADD;
    // Target
    target_total_trajectory_vis_.type = visualization_msgs::Marker::LINE_STRIP;
    target_total_trajectory_vis_.scale.x = 0.05;
    target_total_trajectory_vis_.color.a = 0.8;
    target_total_trajectory_vis_.color.r = 1.0;
    target_total_trajectory_vis_.color.g = 0.0;
    target_total_trajectory_vis_.color.b = 0.0;
    target_total_trajectory_vis_.action = visualization_msgs::Marker::ADD;

    // Obstacle
    obstacle_separate_trajectory_vis_.type = visualization_msgs::Marker::LINE_STRIP;
    obstacle_separate_trajectory_vis_.scale.x = 0.02;
    obstacle_separate_trajectory_vis_.color.a = 0.2;
    obstacle_separate_trajectory_vis_.color.r = 0.2;
    obstacle_separate_trajectory_vis_.color.g = 0.2;
    obstacle_separate_trajectory_vis_.color.b = 0.2;
    obstacle_separate_trajectory_vis_.action = visualization_msgs::Marker::ADD;
    obstacle_separate_trajectory_vis_.ns = "ObstacleTotal";
    // Bearing Vector
    bearing_vector_history_vis_.header.frame_id = "map";
    bearing_vector_history_vis_.type = visualization_msgs::Marker::LINE_LIST;
    bearing_vector_history_vis_.color.a = 0.3;
    bearing_vector_history_vis_.color.r = 0.54;
    bearing_vector_history_vis_.color.g = 0.17;
    bearing_vector_history_vis_.color.b = 0.88;
    bearing_vector_history_vis_.scale.x = 0.01;
    bearing_vector_history_vis_.pose.orientation.w = 1.0;
    bearing_vector_history_vis_.pose.orientation.x = 0.0;
    bearing_vector_history_vis_.pose.orientation.y = 0.0;
    bearing_vector_history_vis_.pose.orientation.z = 0.0;

    boundary_vis_.header.frame_id = "map";
    boundary_vis_.scale.x = 0.1;
    boundary_vis_.color.a = 1.0;
    boundary_vis_.color.r = 1.0;
    boundary_vis_.color.g = 1.0;
    boundary_vis_.color.b = 0.0;
    boundary_vis_.type = visualization_msgs::Marker::LINE_LIST;
    boundary_vis_.ns = "Boundary";
    {
        geometry_msgs::Point vertex;
        // seg1
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        // seg2
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        // seg3
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg4
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg5
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        // seg6
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        // seg7
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg8
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg9
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg10
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_min;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg11
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_min;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
        // seg12
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_min;
        boundary_vis_.points.push_back(vertex);
        vertex.x = boundary_.x_max;
        vertex.y = boundary_.y_max;
        vertex.z = boundary_.z_max;
        boundary_vis_.points.push_back(vertex);
    }
    boundary_vis_.pose.orientation.w = 1.0;
    boundary_vis_.pose.orientation.x = 0.0;
    boundary_vis_.pose.orientation.y = 0.0;
    boundary_vis_.pose.orientation.z = 0.0;

    t0_ = ros::Time::now().toSec();
}

void los_keeper::Analyzer::run() {
    ReadActorTrajectories();
    ros::Rate loop_rate(30.0);
    while (ros::ok()) {
        if (write_total_trajectory_)
            WriteCurrentPositions();
        if (calculate_min_distance_)
            WriteMinDistance();
        if(calculate_visibility_score_)
            WriteVisibilityScore();
        VisualizeData();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

void los_keeper::Analyzer::VisualizeData() {
    if (got_keeper_info and got_target_info and got_obstacle_info) {
        keeper_vis_publisher_.publish(keeper_state_vis_);
        target_vis_publisher_.publish(target_state_vis_);
        obstacle_list_vis_publisher_.publish(obstacle_state_list_vis_);
        obstacle_prediction_vis_publisher_.publish(obstacle_prediction_vis_);
    }
    if (not(keeper_total_trajectory_vis_.points.empty() or target_total_trajectory_vis_.points.empty() or
            obstacle_total_trajectory_vis_.markers.empty())) {
        keeper_total_vis_publisher_.publish(keeper_total_trajectory_vis_);
        target_total_vis_publisher_.publish(target_total_trajectory_vis_);
        obstacle_list_total_vis_publisher_.publish(obstacle_total_trajectory_vis_);
        bearing_vector_history_publisher_.publish(bearing_vector_history_vis_);
    }
    boundary_vis_publisher_.publish(boundary_vis_);
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
    current_keeper_state_.vx = state->vx;
    current_keeper_state_.vy = state->vy;
    current_keeper_state_.vz = state->vz;
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
    obstacle_prediction_vis_.points.clear();
    State obstacle_info{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    geometry_msgs::Point temp_point;
    for (int i = 0; i < state_list->object_status_array.size(); i++) {
        obstacle_info.px = state_list->object_status_array[i].px;
        obstacle_info.py = state_list->object_status_array[i].py;
        obstacle_info.pz = state_list->object_status_array[i].pz;
        obstacle_info.vx = state_list->object_status_array[i].vx;
        obstacle_info.vy = state_list->object_status_array[i].vy;
        obstacle_info.vz = state_list->object_status_array[i].vz;
        obstacle_state_vis_.pose.position.x = obstacle_info.px;
        obstacle_state_vis_.pose.position.y = obstacle_info.py;
        obstacle_state_vis_.pose.position.z = obstacle_info.pz;
        obstacle_state_vis_.id = i;
        current_obstacle_list_state_.push_back(obstacle_info);
        obstacle_state_list_vis_.markers.push_back(obstacle_state_vis_);
        // prediction
        temp_point.x = obstacle_info.px;
        temp_point.y = obstacle_info.py;
        temp_point.z = obstacle_info.pz;
        obstacle_prediction_vis_.points.push_back(temp_point);
        temp_point.x = obstacle_info.px + obstacle_info.vx * 1.0; // TODO: Horizon
        temp_point.y = obstacle_info.py + obstacle_info.vy * 1.0;
        temp_point.z = obstacle_info.pz + obstacle_info.vz * 1.0;
        obstacle_prediction_vis_.points.push_back(temp_point);
    }
}

void los_keeper::Analyzer::WriteMinDistance() {
    double current_time = ros::Time::now().toSec() - t0_;
    ofstream outfile;
    outfile.open(min_distance_filename_, ios_base::app);
    if (got_keeper_info and got_target_info and got_obstacle_info) {
        double min_distance = sqrt(pow(current_keeper_state_.px - current_target_state_.px, 2) +
                                   pow(current_keeper_state_.py - current_target_state_.py, 2) +
                                   pow(current_keeper_state_.pz - current_target_state_.pz, 2));
        double distance;
        for (int i = 0; i < current_obstacle_list_state_.size(); i++) {
            distance = sqrt(pow(current_keeper_state_.px - current_obstacle_list_state_[i].px, 2) +
                            pow(current_keeper_state_.py - current_obstacle_list_state_[i].py, 2) +
                            pow(current_keeper_state_.pz - current_obstacle_list_state_[i].pz, 2));
            if (min_distance > distance)
                min_distance = distance;
        }
        outfile<<current_time<<" "<<min_distance<<endl;
    }
    outfile.close();
}

void los_keeper::Analyzer::WriteCurrentPositions() {
    double current_time = ros::Time::now().toSec() - t0_;
    ofstream outfile;
    outfile.open(total_trajectory_filename_write_, ios_base::app);
    if (got_keeper_info and got_target_info and got_obstacle_info) {
        outfile << current_time << " " << current_keeper_state_.px << " " << current_keeper_state_.py << " "
                << current_keeper_state_.pz << " "
                << current_target_state_.px << " " << current_target_state_.py << " " << current_target_state_.pz
                << " ";
        for (int i = 0; i < current_obstacle_list_state_.size(); i++)
            outfile << current_obstacle_list_state_[i].px << " " << current_obstacle_list_state_[i].py << " "
                    << current_obstacle_list_state_[i].pz << " ";
        outfile << endl;
    }
    outfile.close();
}

void los_keeper::Analyzer::ReadActorTrajectories() {
    ifstream actor_trajectory_file;
    actor_trajectory_file.open(total_trajectory_filename_read_.c_str());
    if (actor_trajectory_file.is_open()) {
        Point keeper_position_temp{0.0, 0.0, 0.0};
        Point target_position_temp{0.0, 0.0, 0.0};
        Point obstacle_position_temp{0.0, 0.0, 0.0};
        double time_temp{0.0};
        while (actor_trajectory_file >> time_temp >> keeper_position_temp.px >> keeper_position_temp.py
                                     >> keeper_position_temp.pz
                                     >> target_position_temp.px >> target_position_temp.py >> target_position_temp.pz) {
            for (int i = 0; i < num_obstacle_; i++) {
                actor_trajectory_file >> obstacle_position_temp.px >> obstacle_position_temp.py
                                      >> obstacle_position_temp.pz;
                obstacle_total_trajectory_[i].push_back(obstacle_position_temp);
            }
            keeper_total_trajectory_.push_back(keeper_position_temp);
            target_total_trajectory_.push_back(target_position_temp);
        }
    }

    geometry_msgs::Point temp_position;

    // Keeper Visualization
    for (int i = 0; i < keeper_total_trajectory_.size(); i++) {
        temp_position.x = keeper_total_trajectory_[i].px;
        temp_position.y = keeper_total_trajectory_[i].py;
        temp_position.z = keeper_total_trajectory_[i].pz;
        keeper_total_trajectory_vis_.points.push_back(temp_position);
    }
    // Target Visualization
    for (int i = 0; i < target_total_trajectory_.size(); i++) {
        temp_position.x = target_total_trajectory_[i].px;
        temp_position.y = target_total_trajectory_[i].py;
        temp_position.z = target_total_trajectory_[i].pz;
        target_total_trajectory_vis_.points.push_back(temp_position);
    }
    // Obstacle Visualization
    for (int i = 0; i < num_obstacle_; i++) {
        obstacle_separate_trajectory_vis_.points.clear();
        for (int j = 0; j < obstacle_total_trajectory_[i].size(); j++) {
            temp_position.x = obstacle_total_trajectory_[i][j].px;
            temp_position.y = obstacle_total_trajectory_[i][j].py;
            temp_position.z = obstacle_total_trajectory_[i][j].pz;
            obstacle_separate_trajectory_vis_.points.push_back(temp_position);
        }
        obstacle_separate_trajectory_vis_.id = i;
        obstacle_total_trajectory_vis_.markers.push_back(obstacle_separate_trajectory_vis_);
    }
    // Bearing Vector Visualization
    for (int i = 0; i < keeper_total_trajectory_vis_.points.size(); i++) {
        bearing_vector_history_vis_.points.push_back(keeper_total_trajectory_vis_.points[i]);
        bearing_vector_history_vis_.points.push_back(target_total_trajectory_vis_.points[i]);
    }
}

void los_keeper::Analyzer::WriteVisibilityScore() {
    double current_time = ros::Time::now().toSec() - t0_;
    ofstream outfile;
    outfile.open(visibility_score_filename_, ios_base::app);
    if (got_keeper_info and got_target_info and got_obstacle_info) {
        double min_visibility = 10000000.0;
        double visibility_score;
        Point keeper_position{current_keeper_state_.px,current_keeper_state_.py,current_keeper_state_.pz};
        Point target_position{current_target_state_.px,current_target_state_.py,current_target_state_.pz};
        for(int i = 0;i<current_obstacle_list_state_.size();i++){
            Point obstacle_position{current_obstacle_list_state_[i].px,current_obstacle_list_state_[i].py,current_obstacle_list_state_[i].pz};
            if(is_2d_ and is_exp_)
                visibility_score = GetVisibilityScore(keeper_position,target_position,obstacle_position,0.075);
            if(is_2d_ and not is_exp_)
                visibility_score = GetVisibilityScore(keeper_position,target_position,obstacle_position,0.07);
            if(not is_2d_ and is_exp_)
                visibility_score = GetVisibilityScore(keeper_position,target_position,obstacle_position,0.075);
            if(not is_2d_ and not is_exp_)
                visibility_score = GetVisibilityScore(keeper_position,target_position,obstacle_position,0.15);
            if(min_visibility>visibility_score)
                min_visibility = visibility_score;
        }
        outfile<<current_time<<" "<<min_visibility<<endl;
    }
    outfile.close();
}

double los_keeper::Analyzer::GetVisibilityScore(const los_keeper::Point &keeper, const los_keeper::Point &target,
                                              const los_keeper::Point &obstacle, const double &radius) {
    double segment_squared =  pow(keeper.px-target.px,2)+ pow(keeper.py-target.py,2)+ pow(keeper.pz-target.pz,2);
    if(segment_squared<0.00001)
        return sqrt(segment_squared);
    double dot_result = (obstacle.px-target.px)*(keeper.px-target.px)+(obstacle.py-target.py)*(keeper.py-target.py)+(obstacle.pz-target.pz)*(keeper.pz-target.pz);
    double t = max(0.0,min(1.0,dot_result/segment_squared));
    Point projected_points;
    projected_points.px = target.px + t* (keeper.px-target.px);
    projected_points.py = target.py + t* (keeper.py-target.py);
    projected_points.pz = target.pz + t* (keeper.pz-target.pz);
    double score_squared = sqrt(pow(obstacle.px-projected_points.px,2)+pow(obstacle.py-projected_points.py,2)+pow(obstacle.pz-projected_points.pz,2))-radius;
    return score_squared;
}

