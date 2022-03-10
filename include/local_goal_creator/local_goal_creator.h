#ifndef LOCAL_GOAL_CREATOR_H
#define LOCAL_GOAL_CREATOR_H

#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <amsl_navigation_msgs/NodeEdgeMap.h>
#include <amsl_navigation_msgs/Node.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>

class LocalGoal
{
    public:
        LocalGoal();
        void process();

        struct Point
        {
            double x;
            double y;
        };
        struct Node
        {
            int id;
            double x;
            double y;
        };

    private:
        int hz;
        double goal_radius;
        bool flag_pose;
        bool flag_node;

        Point goal_point;
        Point current_point;
        Node n;

        std::vector<Node> Nodes;
        std::vector<int> list_cp;
        geometry_msgs::PoseWithCovarianceStamped pose;
        amsl_navigation_msgs::NodeEdgeMap node;
        std_msgs::Int32MultiArray cp;

        void amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &);
        void node_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr &);
        // void node_callback(const amsl_navigation_msgs::Node::ConstPtr &);
        void check_point_callback(const std_msgs::Int32MultiArray::ConstPtr &);
        void calc_goal_dist();
        void update_check_point(int);

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber sub_amcl_pose;
        ros::Subscriber sub_node;
        ros::Subscriber sub_check_point;
        ros::Publisher pub_local_goal;
};
#endif
