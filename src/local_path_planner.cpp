#include <local_path_planner/local_path_planner.h>

DWA::DWA():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("dt",dt,{0.1});
    private_nh.param("predict_time",predict_time,{3});
    private_nh.param("robot_radius",robot_radius,{0.4});
    private_nh.param("safe_radius",safe_radius,{0.2});
    private_nh.param("robot_wide",robot_wide,{0.35});
    private_nh.param("robot_depth",robot_depth,{0.8});
    private_nh.param("visible_dist",visible_dist,{5});
    private_nh.param("reso_velocity",reso_velocity,{0.1});
    private_nh.param("reso_omega",reso_omega,{0.1});
    private_nh.param("reso_range",reso_range,{0.1});
    private_nh.param("SPEED_MAX",SPEED_MAX,{0.5});
    private_nh.param("SPEED_MIN",SPEED_MIN,{0});
    private_nh.param("ANGULAR_SPEED_MAX",ANGULAR_SPEED_MAX,{0.5});
    private_nh.param("ACCEL_MAX",ACCEL_MAX,{10});
    private_nh.param("ANGULAR_ACCEL_MAX",ANGULAR_ACCEL_MAX,{1000});
    private_nh.param("COST_GAIN_TO_GOAL",COST_GAIN_TO_GOAL,{0.5});
    private_nh.param("COST_GAIN_OBSTACLE",COST_GAIN_OBSTACLE,{1});
    private_nh.param("COST_GAIN_SPEED",COST_GAIN_SPEED,{1});
    private_nh.param("exclusion",exclusion,{100});
    private_nh.param("flag_odom",flag_odom,{false});
    private_nh.param("flag_scan",flag_scan,{false});
    private_nh.param("flag_pose",flag_pose,{true});
    private_nh.param("flag_local_goal",flag_local_goal,{false});

    sub_scan = nh.subscribe("scan",10,&DWA::scan_callback,this);
    sub_amcl_pose = nh.subscribe("amcl_pose",10,&DWA::amcl_pose_callback,this);
    sub_odom = nh.subscribe("odom",10,&DWA::odom_callback,this);
    sub_local_goal = nh.subscribe("goal",10,&DWA::local_goal_callback,this);

    pub_robot_ctrl = nh.advertise<geometry_msgs::Twist>("/local_path/cmd_vel",10);
    pub_best_traj = nh.advertise<nav_msgs::Path>("best_traj",10);
    pub_list_traj = nh.advertise<nav_msgs::Path>("list_traj",10);
    pub_list_object = nh.advertise<nav_msgs::Path>("list_object",10);
}

void DWA::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = *msg;
    scan_obstacle();
    flag_scan = true;
}

void DWA::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    pose = *msg;
    current_state.x = pose.pose.pose.position.x;
    current_state.y = pose.pose.pose.position.y;
    current_state.yaw = tf::getYaw(pose.pose.pose.orientation);
    flag_pose = true;
}

void DWA::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
    current_state.velocity = odom.twist.twist.linear.x;
    current_state.omega = odom.twist.twist.angular.z;
    flag_odom = true;
}

void DWA::local_goal_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
{
    local_goal = *msg;
    goal.x=local_goal.pose.position.x;
    goal.y=local_goal.pose.position.y;
    local_goal.header.frame_id="map";
    flag_local_goal = true;
}

void DWA::wheel_pose()
{
    // for(int i=-1;i<=1;i++) {
        for(int j=-1;j<=1;j++) {
            // wheel.x = robot_wide * i;
            wheel.x = robot_depth;
            wheel.y = robot_wide * j;
            list_wheel.push_back(wheel);
        }
    // }
}

void DWA::calc_dynamic_window()
{
    Vs.vel_max = SPEED_MAX;
    Vs.vel_min = SPEED_MIN;
    Vs.omega_max = ANGULAR_SPEED_MAX;
    Vs.omega_min = ANGULAR_SPEED_MAX * -1;

    Vd.vel_max = current_state.velocity + ACCEL_MAX * dt;
    Vd.vel_min = current_state.velocity - ACCEL_MAX * dt;
    Vd.omega_max = current_state.omega + ANGULAR_ACCEL_MAX * dt;
    Vd.omega_min = current_state.omega - ANGULAR_ACCEL_MAX * dt;

    dw.vel_max = std::min(Vs.vel_max, Vd.vel_max);
    dw.vel_min = std::max(Vs.vel_min, Vd.vel_min);
    dw.omega_max = std::min(Vs.omega_max, Vd.omega_max);
    dw.omega_min = std::max(Vs.omega_min, Vd.omega_min);
}

void DWA::scan_obstacle()
{
    double angle = scan.angle_min;
    obstacle.clear();
    for(auto& r : scan.ranges) {
        if(scan.range_min<r&&r<visible_dist) {
            object.x=r*std::cos(angle);
            object.y=r*std::sin(angle);
            obstacle.push_back(object);
        }
        angle += scan.angle_increment;
    }
}

void DWA::calc_trajectory()
{
    double cost_min = 1e5;
    for(predict_state.velocity=dw.vel_min; predict_state.velocity<=dw.vel_max; predict_state.velocity+=reso_velocity) {
        for(predict_state.omega=dw.omega_min; predict_state.omega<=dw.omega_max; predict_state.omega+=reso_omega) {
            traj.clear();
            for(double time=0; time<=predict_time; time+=dt) {
                // predict_state.x=current_state.x+predict_state.velocity*time*std::cos(predict_state.omega*time);
                // predict_state.y=current_state.y+predict_state.velocity*time*std::sin(predict_state.omega*time);
                predict_state.yaw = predict_state.omega*time;
                predict_state.x=predict_state.velocity*time*std::cos(predict_state.yaw);
                predict_state.y=predict_state.velocity*time*std::sin(predict_state.yaw);
                traj.push_back(predict_state);
                    // printf("%f,%f\n",predict_state.x,predict_state.y);
            }
            visual_list_traj(traj);

            double cost_heading = calc_heading(traj.back());
            double cost_dist = calc_dist(traj,wheel_traj);
            if(cost_dist == exclusion) break;
            double cost_velocity = calc_velocity(traj.back());
            double cost_evaluation = COST_GAIN_TO_GOAL*cost_heading + COST_GAIN_OBSTACLE*cost_dist + COST_GAIN_SPEED*cost_velocity;

            if(cost_min>cost_evaluation) {
                cost_min = cost_evaluation;
                best_state.velocity = predict_state.velocity;
                best_state.omega = predict_state.omega;
                // printf("%f,%f,%f,%f\n",cost_heading,cost_dist,cost_velocity,cost_min);
            }
            // printf("%f,%f,%f,%f\n",cost_heading,cost_dist,cost_velocity,cost_min);
        }
    }
    visual_best_traj();
}

double DWA::calc_heading(State& traj_)
{
    double x=(goal.x-current_state.x)*std::cos(-current_state.yaw)-(goal.y-current_state.y)*std::sin(-current_state.yaw);
    double y=(goal.x-current_state.x)*std::sin(-current_state.yaw)+(goal.y-current_state.y)*std::cos(-current_state.yaw);
    // printf("goal.x,goal.y,cur.x,cur.y:%lf,%lf,%f,%f\n",x,y,current_state.x,current_state.y);
    double heading_yaw = std::atan2(y,x) - std::atan2(traj_.y,traj_.x);
    double cost=heading_yaw/(M_PI);
    if(cost<0) cost*=-1;
    return cost;
}

double DWA::calc_dist(std::vector<State>& traj_, std::vector<State>& wheel_traj_)
{
    double dist_min=1e5;
    for(auto& traj__ : traj_) {
        for(auto& obs : obstacle) {
            double dist=std::sqrt(pow(traj__.x+robot_radius-obs.x,2) + pow(traj__.y-obs.y,2));
            if(dist_min>dist) dist_min=dist;
            if(dist_min<robot_radius) return exclusion;
                // for(auto& list_wheel_ : list_wheel) {
                //     double x=(traj__.x+list_wheel_.x)*std::cos(traj__.yaw)-(traj__.y+list_wheel_.y)*std::sin(traj__.yaw);
                //     double y=(traj__.x+list_wheel_.x)*std::sin(traj__.yaw)+(traj__.y+list_wheel_.y)*std::cos(traj__.yaw);
                //     double dist_=std::sqrt(pow(x-obs.x,2) + pow(y-obs.y,2));
                // // printf("x,y:%lf,%lf,%lf\n",x,y,dist_);
                    // if(dist_<safe_radius) return 10;
                // }
            // }
        }
    }
    return robot_radius/(dist_min+1e-10);
}

double DWA::calc_velocity(State& traj_)
{
    return (SPEED_MAX-traj_.velocity)/SPEED_MAX;
}

// void DWA::visual_list_object(std::vector<State>& obj)
// {
//     for(auto& obs : obj) {
//         geometry_msgs::PoseStamped object_point;
//         object_point.pose.position.x=obs.x;
//         object_point.pose.position.y=obs.y;
//         list_object.poses.push_back(object_point);
//     }
//     list_object.header.frame_id = "base_link";
//     pub_list_object.publish(list_object);
// }

void DWA::visual_best_traj()
{
    best_traj.poses.clear();
    for(double time=0;time<=predict_time;time+=dt) {
        geometry_msgs::PoseStamped best_traj_point;
        best_traj_point.pose.position.x=best_state.velocity*time*std::cos(best_state.omega*time);
        best_traj_point.pose.position.y=best_state.velocity*time*std::sin(best_state.omega*time);
        best_traj.poses.push_back(best_traj_point);
    }
    best_traj.header.frame_id = "base_link";
    pub_best_traj.publish(best_traj);
}

void DWA::visual_list_traj(std::vector<State>& traj_)
{
    list_traj.poses.clear();
    for(auto& traj__ : traj_){
        geometry_msgs::PoseStamped list_traj_point;
        list_traj_point.pose.position.x=traj__.x;
        list_traj_point.pose.position.y=traj__.y;
        list_traj.poses.push_back(list_traj_point);
        list_traj.header.frame_id = "base_link";
        pub_list_traj.publish(list_traj);
    }
}

void DWA::init()
{
    obstacle.clear();
    traj.clear();
}

void DWA::robot_control()
{
    wheel_pose();
    calc_dynamic_window();
    calc_trajectory();
    cmd_vel.linear.x=best_state.velocity;
    cmd_vel.angular.z=best_state.omega;
    pub_robot_ctrl.publish(cmd_vel);
}

void DWA::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(flag_odom&&flag_scan&&flag_pose&&flag_local_goal) {
            robot_control();
            init();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_path_planner");
    DWA DWA;
    DWA.process();
    return 0;
}
