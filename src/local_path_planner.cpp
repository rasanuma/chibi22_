#include <local_path_planner/local_path_planner.h>

DWA::DWA():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("dt",dt,{0.5});
    private_nh.param("predict_time",predict_time,{3});
    private_nh.param("robot_radius",robot_radius,{1});
    private_nh.param("visible_dist",visible_dist,{5});
    private_nh.param("reso_velocity",reso_velocity,{0.1});
    private_nh.param("reso_omega",reso_omega,{0.1});
    private_nh.param("reso_range",reso_range,{0.1});
    private_nh.param("reso_mass",reso_mass,{0.05});
    private_nh.param("SPEED_MAX",SPEED_MAX,{1});
    private_nh.param("SPEED_MIN",SPEED_MIN,{-0.5});
    private_nh.param("ANGULAR_SPEED_MAX",ANGULAR_SPEED_MAX,{1.57});
    private_nh.param("ACCEL_MAX",ACCEL_MAX,{0.5});
    private_nh.param("ANGULAR_ACCEL_MAX",ANGULAR_SPEED_MAX,{1.57});
    private_nh.param("COST_GAIN_TO_GOAL",COST_GAIN_TO_GOAL,{1});
    private_nh.param("COST_GAIN_OBSTACLE",COST_GAIN_OBSTACLE,{1});
    private_nh.param("COST_GAIN_SPEED",COST_GAIN_SPEED,{1});
    private_nh.param("flag_odom",flag_odom,{true});
    private_nh.param("flag_scan",flag_scan,{true});
    private_nh.param("flag_pose",flag_pose,{true});
    private_nh.param("flag_local_goal",flag_local_goal,{true});

    sub_scan = nh.subscribe("scan",10,&DWA::scan_callback,this);
    // sub_mcl_pose = nh.subscribe("mcl_pose",10,&DWA::mcl_pose_callback,this);
    sub_amcl_pose = nh.subscribe("amcl_pose",10,&DWA::amcl_pose_callback,this);
    sub_odom = nh.subscribe("odom",10,&DWA::odom_callback,this);
    sub_local_goal = nh.subscribe("local_goal",10,&DWA::local_goal_callback,this);

    pub_robot_ctrl = nh.advertise<geometry_msgs::Twist>("controller/mcd_vel",10);
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

// void DWA::mcl_pose_callback(const geometry_msgs::PoseStamped::ConstPtr &msg)
// {
//     pose = *msg;
//     current_state.x = pose.pose.pose.position.x;
//     current_state.y = pose.pose.pose.position.y;
//     current_state.yaw = tf2::getYaw(pose.pose.pose.orientation);
//     flag_pose = true;
// }
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
    // goal.x=5;
    // goal.y=2;
    flag_local_goal = true;
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
    for(auto& r : scan.ranges) {
        if(r<visible_dist) {
            object.x=r*std::sin(angle);
            object.y=r*std::cos(angle);
            obstacle.push_back(object);
            // scan_to_grid_map(double r);
        }
        angle += scan.angle_increment;
    }
    visual_list_object(obstacle);
}

void DWA::calc_trajectory()
{
    // best_traj.clear();
    // list_traj.clear();

    double cost_min = 1e5;
    for(predict_state.velocity=dw.vel_min; predict_state.velocity<=dw.vel_max; predict_state.velocity+=reso_velocity) {
        for(predict_state.omega=dw.omega_min; predict_state.omega<=dw.omega_max; predict_state.omega+=reso_omega) {
            traj.clear();
            for(double time=0; time<=predict_time; time+=dt) {
                // predict_state.x=current_state.x+predict_state.velocity*time*std::cos(predict_state.omega*time);
                // predict_state.y=current_state.y+predict_state.velocity*time*std::sin(predict_state.omega*time);
                predict_state.x=predict_state.velocity*time*std::cos(predict_state.omega*time);
                predict_state.y=predict_state.velocity*time*std::sin(predict_state.omega*time);
                traj.push_back(predict_state);

                visual_list_traj(traj.back());
            }
            double cost_heading = calc_heading(traj.back());
            double cost_dist = calc_dist(traj);
            double cost_velocity = calc_velocity(traj.back());
            double cost_evaluation = COST_GAIN_TO_GOAL*cost_heading + COST_GAIN_OBSTACLE*cost_dist + COST_GAIN_SPEED*cost_velocity;

            if(cost_min>cost_evaluation) {
                cost_min = cost_evaluation;
                best_state.velocity = predict_state.velocity;
                best_state.omega = predict_state.omega;
            }
        }
    }
    visual_best_traj();
}

double DWA::calc_heading(State& traj_)
{
    // double heading_yaw = std::atan2((goal.y-traj_.y), (goal.x-traj_.x));
    double heading_yaw = std::atan2(goal.y-current_state.y,goal.x-current_state.x) - std::atan2(traj_.y-current_state.y,traj_.x-current_state.x);
    double cost=std::sin(heading_yaw);
    if(cost<0) cost*=-1;
    return cost;
}

double DWA::calc_dist(std::vector<State>& traj_)
{
    double dist_min=1e3;
    for(auto& traj__ : traj_) {
        for(auto& obs : obstacle) {
            double dist=std::sqrt(pow(traj__.x-obs.x,2) + pow(traj__.y-obs.y,2));
            if(dist_min>dist) dist_min=dist;
        }
    }
    // double dist=std::sqrt(pow(traj_.x-obstacle.x,2) + pow(traj_.y-obstacle.y,2));
    // if(dist_min>dist) dist_min=dist;
    return robot_radius/dist_min;
}

double DWA::calc_velocity(State& traj_)
{
    return (SPEED_MAX-traj_.velocity)/SPEED_MAX;
}

void DWA::visual_list_object(std::vector<State>& obj)
{
    for(auto& obs : obj) {
        geometry_msgs::PoseStamped object_point;
        object_point.pose.position.x=obs.x;
        object_point.pose.position.y=obs.y;
        list_object.poses.push_back(object_point);
    }
    list_object.header.frame_id = "base_link";
    pub_list_object.publish(list_object);
}

void DWA::visual_best_traj()
{
    for(double time=0;time<=predict_time;time+=dt) {
        geometry_msgs::PoseStamped best_traj_point;
        best_traj_point.pose.position.x=best_state.velocity*time*std::cos(best_state.omega*time);
        best_traj_point.pose.position.y=best_state.velocity*time*std::sin(best_state.omega*time);
        best_traj.poses.push_back(best_traj_point);
    }
    best_traj.header.frame_id = "base_link";
    pub_best_traj.publish(best_traj);
    // best_traj_path.header.frame_id = "base_link";
    // pub_best_traj.publish(best_traj_path);
}

void DWA::visual_list_traj(State& traj_)
{
    geometry_msgs::PoseStamped list_traj_point;
    list_traj_point.pose.position.x=traj_.x;
    list_traj_point.pose.position.y=traj_.y;
    list_traj.poses.push_back(list_traj_point);
    list_traj.header.frame_id = "base_link";
    pub_list_traj.publish(list_traj);
}

void DWA::init()
{
    obstacle.clear();
    traj.clear();
}

void DWA::robot_control()
{
    calc_dynamic_window();
    calc_trajectory();
    cmd_vel.linear.x=best_state.velocity;
    cmd_vel.angular.z=best_state.omega;
    printf("%f, %f\n",cmd_vel.linear.x,cmd_vel.angular.z);
    pub_robot_ctrl.publish(cmd_vel);
}

void DWA::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(flag_odom&&flag_scan&&flag_pose&&flag_local_goal) {
            // printf("1\n");
            robot_control();
            // printf("2\n");
            init();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_path_planner");
    DWA DWA;
    DWA.process();
    return 0;
}
