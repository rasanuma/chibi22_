#include <amcl_localization/amcl_localization.h>

std::random_device seed;
std::mt19937 engine(seed());

AMCL::AMCL():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("N",N,{1000});
    private_nh.param("init_x",init_x,{0});
    private_nh.param("init_y",init_y,{0});
    private_nh.param("init_yaw",init_yaw,{0});
    private_nh.param("init_x_sigma",init_x_sigma,{0.5});
    private_nh.param("init_y_sigma",init_y_sigma,{0.5});
    private_nh.param("init_yaw_sigma",init_yaw_sigma,{0.5});
    private_nh.param("range_sigma",range_sigma,{0.5});
    private_nh.param("move_dist_sigma",move_dist_sigma,{1});
    private_nh.param("move_yaw_sigma",move_yaw_sigma,{0.5});
    private_nh.param("MAX_RANGE",MAX_RANGE,{5});
    private_nh.param("ANGLE_STEP",ANGLE_STEP,{1});
    private_nh.param("ALPHA_SLOW",ALPHA_SLOW,{1});
    private_nh.param("ALPHA_FAST",ALPHA_FAST,{5});

    sub_scan = nh.subscribe("scan",10,&AMCL::scan_callback,this);
    sub_odom = nh.subscribe("odom",10,&AMCL::odom_callback,this);
    sub_map = nh.subscribe("map",10,&AMCL::map_callback,this);

    // pub_amcl_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("amcl_pose",10);
    pub_amcl_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("mcl_pose",10);
    pub_pose_array = nh.advertise<geometry_msgs::PoseArray>("pose_array",10);

    amcl_pose.header.frame_id = "map";
    pose_array.header.frame_id = "map";

    flag_scan = false;
    flag_odom = false;
    flag_map  = true;
    flag_first= true;
}

void AMCL::scan_callback(const sensor_msgs::LaserScan::ConstPtr &msg)
{
    scan = *msg;
    flag_scan = true;
}

void AMCL::odom_callback(const nav_msgs::Odometry::ConstPtr &msg)
{
    odom = *msg;
    flag_odom = true;
}

void AMCL::map_callback(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    flag_map = true;
    // Particle p;
}

void AMCL::create_particles(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
            printf("b2\n");
    std::vector<Particle> p_init;
    for(int i=0;i<N;i++) {
        Particle p(N);
        std::normal_distribution<> dist_x(x, x_sigma);
        p.p_pose.pose.pose.position.x=dist_x(engine);//****************
        std::normal_distribution<> dist_y(y, y_sigma);
        p.p_pose.pose.pose.position.y=dist_y(engine);
        // yaw = tf::getYaw(p.pose.pose.pose.orientation);//*******************
        std::normal_distribution<> dist_yaw(yaw, yaw_sigma);
        p.p_yaw=dist_yaw(engine);//*******************
        p_init.push_back(p);//***********
    }
    p_array=p_init;
}

AMCL::Particle::Particle(int N) {
    p_pose.pose.pose.position.x = 0;
    p_pose.pose.pose.position.y = 0;
    p_yaw = 0;
    weight = 1.0/double(N);
}

// void AMCL::p_init(AMCL::Particle &p, double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
// {
//     std::normal_distribution<> dist(x, x_sigma);
//     p.pose.pose.pose.position.x=dist(engine)
//     std::normal_distribution<> dist(y, y_sigma);
//     p.pose.pose.pose.position.y=dist(engine)
//     std::normal_distribution<> dist(yaw, yaw_sigma);
//     tf::getYaw(p.pose.pose.pose.orientation)=dist(engine)//*******************
// }

void AMCL::p_motion_update()
{
    nav_msgs::Odometry current_odom;
    nav_msgs::Odometry previous_odom;
    current_odom = odom;
    if(flag_first) previous_odom = current_odom;
    double dx=current_odom.pose.pose.position.x-previous_odom.pose.pose.position.x;
    double dy=current_odom.pose.pose.position.y-previous_odom.pose.pose.position.y;
    double current_yaw=tf::getYaw(current_odom.pose.pose.orientation);
            // printf("1111\n");
    double previous_yaw=tf::getYaw(previous_odom.pose.pose.orientation);
            // printf("111\n");
    double dyaw=current_yaw-previous_yaw;
    double weight_max = 0;
    p_move_array.clear();
    for(auto& p_ : p_array) {
        p_.p_pose.pose.pose.position.x += dx;
        p_.p_pose.pose.pose.position.y += dy;
        p_.p_yaw += dyaw;
        p_.weight = calc_weight(p_);
        p_move_array.push_back(p_);
            printf("b4\n");

        if(weight_max<p_.weight) {
            printf("b3\n");
            weight_max = p_.weight;
            amcl_pose.pose.pose.position.x = p_.p_pose.pose.pose.position.x;
            amcl_pose.pose.pose.position.y = p_.p_pose.pose.pose.position.y;
            // tf::Quaternion quat= tf::createQuaternionFromRPY(0,0,p_.p_yaw);
            geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromYaw(p_.p_yaw);
            amcl_pose.pose.pose.orientation = quat;
            // amcl_pose.pose.pose.orientation = p_.p_pose.pose.pose.orientation;
        }
    }
    flag_first = false;
    previous_odom = current_odom;
            // printf("b4\n");
}

double AMCL::calc_weight(AMCL::Particle &pose)
{
    double weight = 0;
    // double yaw = tf::getYaw(pose.pose.orientation);
    for(int i=0; i<scan.ranges.size(); i+=scan.ranges.size()/ANGLE_STEP) {
        // double x = p_pose.pose.pose.position.x+scan.ranges[i]*std::sin(scan.angle_min+i*scan.angle_increment);
        double x = pose.p_pose.pose.pose.position.x+scan.ranges[i]*std::cos(scan.angle_min+i*scan.angle_increment+pose.p_yaw);
        double y = pose.p_pose.pose.pose.position.y+scan.ranges[i]*std::sin(scan.angle_min+i*scan.angle_increment+pose.p_yaw);//sinとcos
        double dist_ = std::sqrt(pow(x,2)+pow(y,2));
        weight += gaussian(scan.ranges[i],range_sigma,dist_);
        // if(scan.ranges[i]>scan.range_min) {
            // double dist_to_wall = dist_p_to_obs();
    }
            // printf("weight:%lf\n",weight/ANGLE_STEP);
    return weight/ANGLE_STEP;
}

double AMCL::gaussian(double mu, double sigma, double x)
{
    double ans = 1/std::sqrt(2*M_PI*pow(sigma,2))*exp(-pow(x-mu,2)/2*pow(sigma,2));
    // printf("ans,%lf\n",ans);
    return ans;
}

// double AMCL::dist_p_to_obs(double x, double y, double yaw, double range)
// {
//     double search_step = map.info.resolution;
//     // double search_max = std::min(range*, MAX_RANGE);
//     for(double dist=search_step; dist<=MAX_RANGE; dist+=search_step) {
//         x = x + dist * std::cos(yaw);
//         y = y + dist * std::sin(yaw);
//         int mass = map_to_mass(x, y);
//
void AMCL::resampring_selection()
{
    resampring();
    // reset();
}

void AMCL::reset()
{
    std::vector<Particle> p_new_array;//************
    double new_x = amcl_pose.pose.pose.position.x;
    double new_y = amcl_pose.pose.pose.position.y;
    double new_yaw = tf::getYaw(amcl_pose.pose.pose.orientation);
    create_particles(new_x, new_y, new_yaw, init_x_sigma,init_y_sigma, init_yaw_sigma);
    // create_particles(p_new_array, new_x, new_y, new_yaw, init_x_sigma,init_y_sigma, init_yaw_sigma);
    p_array = p_new_array;
}

void AMCL::resampring()
{
    std::uniform_real_distribution<> dist(0.0, 1.0);
    double r = dist(engine);
    double sum_weight = 0;
    std::vector<Particle> p_new_array;
    for(int i=0; i<N; i++) {
        sum_weight += p_move_array[i].weight;
        while(sum_weight>r) {
            // printf("c3\n");
            Particle p = p_move_array[i];
            p.weight = 1/double(N);
            p_new_array.push_back(p);
            r += 1/double(N);
        }
    }
    p_array = p_new_array;
            printf("c1\n");
}

// void AMCL::output_pose()
// {
//     double weight_max = 0;
//     for(auto& p : p_move_array) {
//         if(weight_max<p.weight) {
//             amcl_pose.pose.pose.position.x = p.x;
//             amcl_pose.pose.pose.position.y = p.y;
//             pub_amcl_pose.publish(amcl_pose);
//
// }
void AMCL::output_pose()
{
    for(auto& p : p_array) {
        pose_array.poses.push_back(p.p_pose.pose.pose);
    }
    pub_amcl_pose.publish(amcl_pose);
    pub_pose_array.publish(pose_array);
}

void AMCL::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
            printf("bool:%d\n",flag_first);
        if(flag_first) create_particles(init_x,init_y,init_yaw,init_x_sigma,init_y_sigma,init_yaw_sigma);
        if(flag_scan&&flag_odom&&flag_map) {
            // printf("a1\n");
            // if(std::isnan(p_array) printf("aaaaa\n");
            p_motion_update();
            printf("a3\n");
            resampring_selection();
            // printf("a4\n");
        }
        output_pose();
        ros::spinOnce();
        loop_rate.sleep();
            printf("d1\n");
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"amcl_localization");
    AMCL AMCL;
    AMCL.process();
    return 0;
}
