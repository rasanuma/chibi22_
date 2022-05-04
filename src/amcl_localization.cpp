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
    private_nh.param("dxy_sigma",dxy_sigma,{0.3});
    private_nh.param("dyaw_sigma",dyaw_sigma,{0.3});
    private_nh.param("max_range",max_range,{5});
    private_nh.param("range_reso",range_reso,{0.5});
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
}

void AMCL::create_particles(double x, double y, double yaw, double x_sigma, double y_sigma, double yaw_sigma)
{
            //printf("b2\n");
    std::vector<Particle> p_init;
    for(int i=0;i<N;i++) {
        Particle p(N);
        std::normal_distribution<> dist_x(x, x_sigma);
        std::normal_distribution<> dist_y(y, y_sigma);
        std::normal_distribution<> dist_yaw(yaw, yaw_sigma);
        p.p_pose.pose.pose.position.x=dist_x(engine);
        p.p_pose.pose.pose.position.y=dist_y(engine);
        p.p_yaw=dist_yaw(engine);//*******************
        p_init.push_back(p);//***********
        // printf("yaw:%lf\n",p.p_yaw);
        // ROS_WARN_STREAM("yaw:" << p.p_yaw);
    }
    p_array=p_init;
}

AMCL::Particle::Particle(int N) {
    p_pose.pose.pose.position.x = 0;
    p_pose.pose.pose.position.y = 0;
    p_yaw = 0;
    weight = 1.0/double(N);
}

void AMCL::p_motion_update()
{
    current_odom = odom;
    if(flag_first) previous_odom = current_odom;
    double dx=current_odom.pose.pose.position.x-previous_odom.pose.pose.position.x;
    double dy=current_odom.pose.pose.position.y-previous_odom.pose.pose.position.y;
    double current_yaw=tf::getYaw(current_odom.pose.pose.orientation);
    double previous_yaw=tf::getYaw(previous_odom.pose.pose.orientation);
    double dyaw=current_yaw-previous_yaw;
    // double dyaw=tf::getYaw(current_odom.pose.pose.orientation) - tf::getYaw(previous_odom.pose.pose.orientation);
    double weight_max = 0;
    sum_weight = 0;
    p_move_array.clear();
            // printf("current_yaw:%lf\n",current_yaw);
    for(auto& p_ : p_array) {
        std::normal_distribution<> dist_dx(dx, dxy_sigma);
        std::normal_distribution<> dist_dy(dy, dxy_sigma);
        std::normal_distribution<> dist_dyaw(dyaw, dyaw_sigma);

        p_.p_pose.pose.pose.position.x += dist_dx(engine);
        p_.p_pose.pose.pose.position.y += dist_dy(engine);
        p_.p_yaw += dist_dyaw(engine);
        geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromYaw(p_.p_yaw);
        p_.p_pose.pose.pose.orientation = quat;
        p_.weight = calc_weight(p_);
        sum_weight += p_.weight;
        // ROS_WARN_STREAM("yaw:" << p_.p_yaw);
        p_move_array.push_back(p_);

        if(weight_max<p_.weight) {
            weight_max = p_.weight;
        // ROS_WARN_STREAM("weight:" << p_.weight);
            amcl_pose.pose.pose.position.x = p_.p_pose.pose.pose.position.x;
            amcl_pose.pose.pose.position.y = p_.p_pose.pose.pose.position.y;
            // tf::Quaternion quat= tf::createQuaternionFromRPY(0,0,p_.p_yaw);
            // geometry_msgs::Quaternion quat=tf::createQuaternionMsgFromYaw(p_.p_yaw);
            // amcl_pose.pose.pose.orientation = quat;
            amcl_pose.pose.pose.orientation = p_.p_pose.pose.pose.orientation;
        }
    }
    flag_first = false;
    previous_odom = current_odom;
}

double AMCL::calc_weight(AMCL::Particle &pose)
{
    double weight = 0;
    for(int i=0; i<scan.ranges.size(); i+=scan.ranges.size()/ANGLE_STEP) {
        if(1 == std::isinf(scan.ranges[i])) continue;
        double angle = scan.angle_min + i * scan.angle_increment + pose.p_yaw;
        double range = range_from_map(angle, pose);

        // double x = pose.p_pose.pose.pose.position.x - map.info.origin.position.x + scan.ranges[i]*std::cos(scan.angle_min+i*scan.angle_increment+pose.p_yaw);
        // double y = pose.p_pose.pose.pose.position.y + map.info.origin.position.y + scan.ranges[i]*std::sin(scan.angle_min+i*scan.angle_increment+pose.p_yaw);//sinとcos
        // double dist_ = std::sqrt(pow(x,2)+pow(y,2));
        weight += gaussian(scan.ranges[i],range_sigma,range);
    }
    return weight/ANGLE_STEP;
}

double AMCL::range_from_map(double angle, AMCL::Particle pose)
{
    double p_x = pose.p_pose.pose.pose.position.x - map.info.origin.position.x;
    double p_y = pose.p_pose.pose.pose.position.y - map.info.origin.position.y;
    for(double range=0; range<=max_range;range += range_reso) {
        double x = p_x + range*std::cos(angle);
        double y = p_y + range*std::sin(angle);
        int map_x = (int)(x / map.info.resolution);
        int map_y = (int)(y / map.info.resolution);
        // if(map_x>map.info.width || map_x <0) return range;
        // if(map_y>map.info.height || map_y <0) return range;
        if(map.data[map_x + map_y * map.info.width] != 0) return range;
    }
    return max_range;
}

double AMCL::gaussian(double mu, double sigma, double x)
{
    double ans = 1e-10+1/std::sqrt(2*M_PI*pow(sigma,2))*exp(-pow(x-mu,2)/2/pow(sigma,2));
    // printf("ans,%lf,%lf,%lf\n",mu,sigma,x);
    return ans;
}

void AMCL::resampring_selection()
{
    resampring();
    // if(sum_weight/N>0.5) reset();
    // double res = resampring();
    // if(res<100) reset();
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
    std::uniform_real_distribution<> dist(0.0, sum_weight/N);
        ROS_WARN_STREAM("weight:" << sum_weight/N);
    double r = dist(engine);
    double sum_w=0;
    std::vector<Particle> p_new_array;
    for(int i=0; i<N; i++) {
        sum_w += p_move_array[i].weight;
        while(sum_w>r && N != p_new_array.size()) {
            Particle p = p_move_array[i];
            p.weight = 1/double(N);
            p_new_array.push_back(p);
            r += sum_weight/double(N);
        }
    }
    p_array = p_new_array;
}

void AMCL::output_pose()
{
    for(auto& p : p_array) {
        pose_array.poses.push_back(p.p_pose.pose.pose);
    }
    pub_amcl_pose.publish(amcl_pose);
    pub_pose_array.publish(pose_array);
    pose_array.poses.clear();
}

void AMCL::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(flag_first) create_particles(init_x,init_y,init_yaw,init_x_sigma,init_y_sigma,init_yaw_sigma);
        if(flag_scan&&flag_odom&&flag_map) {
            p_motion_update();
            resampring_selection();
        }
        output_pose();
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc,argv,"amcl_localization");
    AMCL AMCL;
    AMCL.process();
    return 0;
}
