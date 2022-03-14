#ifndef AMCL_LOCALIZATION_H
#define AMCL_LOCALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#inculde <random>
#inculde <vector>

class AMCL
{
    public:
        AMCL();
        void process();

    private:
        class Particle
        {
            public:
                Particle(int N);
                geometry_msgs p_pose;
                double weight;
                void p_set;
                void p_move;

        };

        int hz;
        int N;
        double INIT_X;    //初期位置
        double INIT_Y;
        double INIT_Z;
        double INIT_ROLL; //パーティクルのばらつき
        double INIT_PITCH;
        double INIT_YAW;
        double MOVE_DIST; //動作更新(距離)
        double MOVE_YAW;  //動作更新(回転)
        double MAX_RANGE;
        double ALPHA_SLOW;
        double ALPHA_FAST;
        double ANGLE_STEP;
        double weight;
        bool flag_scan;
        bool flag_odom;
        bool flag_map;

        geometry_msgs::PoseWithCovarianceStamped amcl_pose;
        geometry_msgs::PoseArray pose_array;
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);

        void calc_weight();
        void motion_update();
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_scan;
        ros::Subscriber sub_odom;
        ros::Subscriber sub_map;

        ros::Publisher pub_amcl_pose;
        ros::Publisher pub_pose_arrray;
};
#endif
