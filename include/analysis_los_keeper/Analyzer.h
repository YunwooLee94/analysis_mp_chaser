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

        void CallbackTargetState(const los_keeper::ObjectStatus::ConstPtr &state);
        void CallbackKeeperState(const los_keeper::ObjectStatus::ConstPtr &state);
        void CallbackObstacleListState(const los_keeper::ObjectStatusArray::ConstPtr  &state_list);

        State current_keeper_state_;
        State current_target_state_;
        vector<State> current_obstacle_list_state_;

        visualization_msgs::Marker keeper_state_vis_;
        visualization_msgs::Marker target_state_vis_;
        visualization_msgs::Marker obstacle_state_vis_;
        visualization_msgs::MarkerArray obstacle_state_list_vis_;
        bool got_obstacle_info{false};
        bool got_target_info{false};
        bool got_keeper_info{false};


        void VisualizeData();
    };
}

#endif //ANALYSIS_LOS_KEEPER_ANALYZER_H
