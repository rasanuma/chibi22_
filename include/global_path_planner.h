#ifndef GLOBAL_PATH_PLANNER_H
#define GLOBAL_PATH_PLANNER_H

#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <vector>

class GlobalPath
{
    public:
        GlocalPath();
        void process();

        struct Point
        {
            double x;
            double y;
        };

    private:
        Point goal;
        Point current;
        nav_msgs::Path
        int hz;
        double goal_dist;
        int id;
        bool flag_pose;
        bool flag_path;

        ros::NodeHandle nh;
        ros::NodeHandle private_nh;
        ros::Subscriber
        ros::Publisher pub_global_path;
};

#endif
