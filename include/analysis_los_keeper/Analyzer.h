//
// Created by larr-laptop on 24. 1. 11.
//

#ifndef ANALYSIS_LOS_KEEPER_ANALYZER_H
#define ANALYSIS_LOS_KEEPER_ANALYZER_H
#include <ros/ros.h>
#include <los_keeper/ObjectStatus.h>
#include <los_keeper/ObjectStatusArray.h>
#include <vector>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <fstream>
#include <nav_msgs/Path.h>

namespace los_keeper{
    using namespace std;

    struct State{
        double px;
        double py;
        double pz;
        double vx;
        double vy;
        double vz;
    };
    struct Point{
        double px;
        double py;
        double pz;
    };

    struct Boundary{
        double x_min;
        double x_max;
        double y_min;
        double y_max;
        double z_min;
        double z_max;
    };
    class Analyzer{
    public:
        Analyzer();
        void run();
    private:
        ros::NodeHandle nh_;

        ros::Subscriber target_state_subscriber_;
        ros::Subscriber keeper_state_subscriber_;
        ros::Subscriber obstacle_list_state_subscriber_;

        ros::Publisher target_vis_publisher_;
        ros::Publisher keeper_vis_publisher_;
        ros::Publisher obstacle_list_vis_publisher_;
        ros::Publisher obstacle_prediction_vis_publisher_;

        ros::Publisher target_total_vis_publisher_;
        ros::Publisher keeper_total_vis_publisher_;
        ros::Publisher obstacle_list_total_vis_publisher_;
        ros::Publisher bearing_vector_history_publisher_;
        ros::Publisher boundary_vis_publisher_;

        void CallbackTargetState(const los_keeper::ObjectStatus::ConstPtr &state);
        void CallbackKeeperState(const los_keeper::ObjectStatus::ConstPtr &state);
        void CallbackObstacleListState(const los_keeper::ObjectStatusArray::ConstPtr  &state_list);

        bool is_2d_;
        bool is_exp_;
        bool write_total_trajectory_;
        string total_trajectory_filename_write_;
        string total_trajectory_filename_read_;

        State current_keeper_state_;
        State current_target_state_;
        vector<State> current_obstacle_list_state_;

        visualization_msgs::Marker keeper_state_vis_;
        visualization_msgs::Marker target_state_vis_;
        visualization_msgs::Marker obstacle_state_vis_;
        visualization_msgs::MarkerArray obstacle_state_list_vis_;
        visualization_msgs::Marker obstacle_prediction_vis_;

        bool got_obstacle_info{false};
        bool got_target_info{false};
        bool got_keeper_info{false};

        double t0_;
        int num_obstacle_;

        vector<double> time_history_;
        vector<Point> keeper_total_trajectory_;
        vector<Point> target_total_trajectory_;
        vector<vector<Point>> obstacle_total_trajectory_;
        visualization_msgs::Marker keeper_total_trajectory_vis_;
        visualization_msgs::Marker target_total_trajectory_vis_;
        visualization_msgs::Marker obstacle_separate_trajectory_vis_;
        visualization_msgs::MarkerArray obstacle_total_trajectory_vis_;
        visualization_msgs::Marker bearing_vector_history_vis_;
        visualization_msgs::Marker boundary_vis_;

        Boundary boundary_;

        void VisualizeData();
        void WriteCurrentPositions();
        void ReadActorTrajectories();


    };
}

#endif //ANALYSIS_LOS_KEEPER_ANALYZER_H
