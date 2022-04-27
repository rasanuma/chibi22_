#ifndef AMCL_LOCALIZATION_H
#define AMCL_LOCALIZATION_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <tf/tf.h>
#include <tf/transform_broadcaster.h>
#include <random>
#include <vector>

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
                geometry_msgs::PoseWithCovarianceStamped p_pose;
                double p_yaw;
                double weight;
                // void p_init;
                // void p_move;

        };

        int hz;
        int N;
        double init_x;    //初期位置
        double init_y;
        double init_yaw;
        double init_x_sigma; //パーティクルのばらつき
        double init_y_sigma;
        double init_yaw_sigma;
        double range_sigma;
        double move_dist_sigma; //動作更新(距離)
        double move_yaw_sigma;  //動作更新(回転)
        double MAX_RANGE;
        double ANGLE_STEP;
        double ALPHA_SLOW;
        double ALPHA_FAST;
        // double weight;
        bool flag_scan;
        bool flag_odom;
        bool flag_map;
        bool flag_first;

        std::vector<Particle> p_array;
        std::vector<Particle> p_move_array;
        sensor_msgs::LaserScan scan;
        nav_msgs::Odometry odom;
        nav_msgs::OccupancyGrid map;
        geometry_msgs::PoseWithCovarianceStamped amcl_pose;
        geometry_msgs::PoseArray pose_array;
        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &);
        void map_callback(const nav_msgs::OccupancyGrid::ConstPtr &);

        // void create_particles();
        void create_particles(double, double, double, double, double, double);
        void p_motion_update();
        double calc_weight(AMCL::Particle &);
        double gaussian(double, double, double);
        void resampring_selection();
        void reset();
        void resampring();
        void output_pose();
        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_scan;
        ros::Subscriber sub_odom;
        ros::Subscriber sub_map;

        ros::Publisher pub_amcl_pose;
        ros::Publisher pub_pose_array;
};
#endif
