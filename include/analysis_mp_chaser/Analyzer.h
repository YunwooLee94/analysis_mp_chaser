//
// Created by larr-laptop on 24. 1. 6.
//
#ifndef ANALYSIS_MP_CHASER_ANALYZER_H
#define ANALYSIS_MP_CHASER_ANALYZER_H
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>
#include <fstream>
#include <vector>
#include <visualization_msgs/MarkerArray.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>

namespace mp_chaser{
    using namespace std;
    struct Point{
        double x;
        double y;
        double z;
    };

    class Analyzer{
    public:
        Analyzer();
        void run();
    private:
        ros::NodeHandle nh_;
        // Parameters
        bool is_dual_;
        bool is_exp_;
        bool write_total_trajectory_;
        string total_trajectory_filename_write_;
        string total_trajectory_filename_read_;

        bool calculate_min_distance_;
        string min_distance_file_name_;

        bool calculate_visibility_score_;
        string visibility_score_file_name_;

        ros::Subscriber sub_drone_pose_;
        ros::Subscriber sub_target1_pose_;
        ros::Subscriber sub_target2_pose_;
        void CbDronePose(const nav_msgs::Odometry::ConstPtr &pose);
        void CbTarget1Pose(const geometry_msgs::PoseStamped::ConstPtr &pose);
        void CbTarget2Pose(const geometry_msgs::PoseStamped::ConstPtr &pose);

        ros::Subscriber sub_pcl_;
        void CbPcl(const sensor_msgs::PointCloud2::ConstPtr &pcl_msgs);

        ros::Publisher pub_drone_total_trajectory_;
        ros::Publisher pub_target1_total_trajectory_;
        ros::Publisher pub_target2_total_trajectory_;
        ros::Publisher pub_total_bearing_vector_history_;

        ros::Publisher pub_current_drone_vis_;
        ros::Publisher pub_current_target1_vis_;
        ros::Publisher pub_current_target2_vis_;


        void WriteCurrentPositions();
        void WriteMinDistance();
        void ReadActorTrajectories();
        void PublishVisualization();
        void WriteVisibilityScore();

        double t0_;
        Point drone_position_;
        Point target1_position_;
        Point target2_position_;
        bool drone_position_flag_{false};
        bool target1_position_flag_{false};
        bool target2_position_flag_{false};

        vector<Point> drone_total_trajectory_;
        vector<Point> target1_total_trajectory_;
        vector<Point> target2_total_trajectory_;
        vector<double> time_history_;

        // Visualize Total Trajectory
        nav_msgs::Path drone_total_trajectory_vis_;
        nav_msgs::Path target1_total_trajectory_vis_;
        nav_msgs::Path target2_total_trajectory_vis_;

        visualization_msgs::Marker current_drone_vis_;
        visualization_msgs::Marker current_target1_vis_;
        visualization_msgs::Marker current_target2_vis_;

        visualization_msgs::Marker bearing_vector_vis_;
        pcl::PointCloud<pcl::PointXYZ> pcl_world_;
        double GetVisibilityScore(const Point &keeper, const Point &target, const Point &obstacle, const double &radius);

    };
}


#endif //ANALYSIS_MP_CHASER_ANALYZER_H
