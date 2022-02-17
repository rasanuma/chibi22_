#ifndef LOCAL_PATH_PLANNER_H
#define LOCAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
// #include <velodyne_msgs/VelodyneScan.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>
// #include <tf2_msgs/TFMessage.h>
#include <vector>
#include <nav_msgs/Path.h>

class DWA
{
    public:
        DWA();
        void process();

        struct Window
        {
            double vel_max;
            double vel_min;
            double omega_max;
            double omega_min;
        };

        struct State
        {
            double x;
            double y;
            double yaw;
            double velocity;
            double omega;
        };

        struct Map
        {
            int x;
            int y;
        };

    private:
        Window Vs;
        Window Vd;
        Window dw;
        int hz;
        double dt;
        double predict_time;
        double robot_radius;
        double visible_dist;
        double reso_velocity;
        double reso_omega;
        double reso_range;
        double reso_mass;

        double SPEED_MAX;
        double SPEED_MIN;
        double ANGULAR_SPEED_MAX;
        double ACCEL_MAX;
        double ANGULAR_ACCEL_MAX;

        double COST_GAIN_TO_GOAL;
        double COST_GAIN_OBSTACLE;
        double COST_GAIN_SPEED;

        bool flag_odom;
        bool flag_scan;
        bool flag_pose;
        bool flag_local_goal;

        State goal;
        State current_state;
        State predict_state;
        State best_state;
        State object;

        std::vector<State> obstacle;
        std::vector<State> traj;
        sensor_msgs::LaserScan scan;
        // sensor_msgs::PointCloud2 scan;
        // geometry_msgs::PoseStamped mcl_pose;
        geometry_msgs::PoseWithCovarianceStamped pose;
        nav_msgs::Odometry odom;
        geometry_msgs::PoseStamped local_goal;
        // geometry_msgs::PoseStamped best_traj;
        nav_msgs::Path list_object;
        nav_msgs::Path best_traj;
        nav_msgs::Path list_traj;
        geometry_msgs::Twist cmd_vel;


        void scan_callback(const sensor_msgs::LaserScan::ConstPtr &);
        // void mcl_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &);
        void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
        void odom_callback(const nav_msgs::Odometry::ConstPtr &);
        void local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &);
        void calc_dynamic_window();
        void scan_obstacle();
        void scan_to_grid_map();
        // void 3D_to_2D_object();
        void calc_trajectory();
        double calc_heading(State&);
        // double calc_dist(std::vector<std::vector<double>>&, State&);
        double calc_dist(std::vector<State>&);
        double calc_velocity(State&);
        void visual_list_object(std::vector<State>&);
        void visual_best_traj();
        void visual_list_traj(State&);
        void robot_control();
        void init();

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;

        ros::Subscriber sub_scan;
        // ros::Subscriber sub_mcl_pose;
        ros::Subscriber sub_amcl_pose;
        ros::Subscriber sub_odom;
        ros::Subscriber sub_local_goal;

        ros::Publisher pub_robot_ctrl;
        ros::Publisher pub_best_traj;
        ros::Publisher pub_list_traj;
        ros::Publisher pub_list_object;

};

#endif
