#include <local_goal_creator/local_goal_creator.h>
#include <iostream>

LocalGoal::LocalGoal():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("goal_radius",goal_radius,{0.5});
    private_nh.param("reso_localgoal",reso_local_goal,{10});
    private_nh.param("flag_pose",flag_pose,{true});
    private_nh.param("flag_node",flag_node,{false});

    sub_amcl_pose = nh.subscribe("amcl_pose",10,&LocalGoal::amcl_pose_callback,this);
    sub_node = nh.subscribe("map",1,&LocalGoal::node_callback,this);
    sub_check_point = nh.subscribe("checkpoint",1,&LocalGoal::check_point_callback,this);

    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("goal",10);
}

void LocalGoal::amcl_pose_callback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr &msg)
{
    pose = *msg;
    current_point.x = pose.pose.pose.position.x;
    current_point.y = pose.pose.pose.position.y;
    flag_pose = true;
}

void LocalGoal::node_callback(const amsl_navigation_msgs::NodeEdgeMap::ConstPtr &msg)
{
    node = *msg;
    Nodes.clear();
    for(auto& node_ : node.nodes) {
        n.id = node_.id;
        n.x = node_.point.x;
        n.y = node_.point.y;
        // n.label = node_.label;
        Nodes.push_back(n);
    }
    flag_node = true;
}

void LocalGoal::check_point_callback(const std_msgs::Int32MultiArray::ConstPtr &msg)
{
    cp = *msg;
    list_cp.clear();
    for(auto& cp_ : cp.data) {
        list_cp.push_back(cp_);
    }
}

void LocalGoal::calc_goal_dist()
{
    static int id = 0;
    if(id==0) {
        id = 1;
        update_check_point(id);
    }
    double dist = std::sqrt(pow(current_point.x-goal_point.x,2)+pow(current_point.y-goal_point.y,2));
    // printf("dist,id:%lf,%d\n",dist,list_cp[id]);
    if(dist<goal_radius) {
        id++;
        // local_id=0;
        update_check_point(id);
    }

    double dist_ = std::sqrt(pow(current_point.x-local_goal_list[local_id].x,2)+pow(current_point.y-local_goal_list[local_id].y,2));
    if(dist_<goal_radius) local_id++;
    printf("dist,id,list:%lf,%d\n",dist_,local_id);
    // std::cout << local_goal_list.size() << std::endl;

    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = local_goal_list[local_id].x;
    goal.pose.position.y = local_goal_list[local_id].y;
    goal.header.frame_id="map";
    pub_local_goal.publish(goal);
}

void LocalGoal::update_check_point(int i)
{
    int node_id = list_cp[i];
    int node_id_old = list_cp[i-1];
    goal_point.x = Nodes[node_id].x;
    goal_point.y = Nodes[node_id].y;
    goal_point_old.x = Nodes[node_id_old].x;
    goal_point_old.y = Nodes[node_id_old].y;

    local_goal_list.clear();
    local_id = 0;
    int a = (goal_point.y-goal_point_old.y)/(goal_point.x-goal_point_old.x);
    int b = -goal_point_old.x*a+goal_point_old.y;
    int j=1;
    if(goal_point.x-goal_point_old.x<0) j=-1;
    for(int x=j*goal_point_old.x; x<=j*goal_point.x; x+=(j*goal_point.x-j*goal_point_old.x)/reso_local_goal) {
        local_goal_point.x = x*j;
        local_goal_point.y = a*x*j+b;
        local_goal_list.push_back(local_goal_point);
    }
}

void LocalGoal::process()
{
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        if(flag_pose&&flag_node) {
            calc_goal_dist();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "local_goal_creator");
    LocalGoal LocalGoal;
    LocalGoal.process();
    return 0;
}
