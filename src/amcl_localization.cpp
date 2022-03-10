#include <amcl_localization/amcl_localization.h>

std::random_device seed;
std::mt19937 engin(seed());

AMCL::AMCL():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("N",N,{1000});
    private_nh.param("INIT_X",INIT_X,{0});
    private_nh.param("INIT_Y",INIT_Y,{0});
    private_nh.param("INIT_Z",INIT_Z,{0});
    private_nh.param("INIT_ROLl",INIT_ROLL,{0});
    private_nh.param("INIT_PITCH",INIT_PITCH,{0});
    private_nh.param("INIT_YAW",INIT_YAW,{0});
    private_nh.param("MOVE_DIST",MOVE_DIST,{1});
    private_nh.param("MOVE_YAW",MOVE_YAW,{0.5});
    private_nh.param("MAX_RANGE",MAX_RANGE,{5});
    private_nh.param("ALPHA_SLOW",ALPHA_SLOW,{1});
    private_nh.param("ALPHA_SLOW",ALPHA_FAST,{5});
    private_nh.param("hz",hz,{10});

    sub_scan = nh.subscribe("scan",10,&DWA::scan_callback,this);
    sub_odom = nh.subscribe("odom",10,&DWA::odom_callback,this);
    sub_map = nh.subscribe("map",10,&DWA::map_callback,this);

    pub_amcl_pose = nh.advertise<geometry_msgs::PoseWithCovarianceStamped.h>("amcl_pose",10);
    pub_pose_array = nh.advertise<geometry_msgs::PoseArray.h>("amcl_pose",10);

    amcl_pose.header.frame_id = "map";
    pose_array.header.frame_id = "map";

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

void AMCL::map_callback(const geometry_msgs::OccupancyGrid::ConstPtr &msg)
{
    map = *msg;
    flag_map = true;
    Particle p;
}

void p_pose.pose.position.x = gaussian(x, x_sigma);


double AMCL::calc_weight(geometry_msgs::PoseStamped &pose)
{
    double weight = 0;
    double angle_increment = scan.angle_increment;
    double angle_min = scan.angle_min;
    double x = pose.pose.position.x;
    double y = pose.pose.position.y;
    double yaw = tf::getYaw(pose.pose.orientation);
    for(int i=0; i<scan.ranges.size(); i+=scan.ranges.size()/angle_step) {

    }
}

double AMCL::from_dist_p_to_obs(double x, double y, double yaw, double range)
{
    double search_step = map.info.resolution;
    // double search_max = std::min(range*, MAX_RANGE);
    for(double dist=search_step; dist<=MAX_RANGE; dist+=search_step) {
        x = x + dist * std::cos(yaw);
        y = y + dist * std::sin(yaw);
        int mass = map_to_mass(x, y);



int AMCL::map_to_mass



void AMCL::output_pose()
{
    .pose.pose.position.x = .x;
    .pose.pose.position.x = .x;

}

void AMCL::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(flag_scan&&flag_odom&&flag_map) {
            output_pose();
        }
        ros::sinOnce();
        loop_rate.sleep();
    }
}
