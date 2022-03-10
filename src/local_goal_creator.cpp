#include <local_goal_creator/local_goal_creator.h>

LocalGoal::LocalGoal():private_nh("~")
{
    private_nh.param("hz",hz,{10});
    private_nh.param("goal_radius",goal_radius,{2});
    private_nh.param("flag_pose",flag_pose,{true});
    private_nh.param("flag_node",flag_node,{false});

    sub_amcl_pose = nh.subscribe("amcl_pose",10,&LocalGoal::amcl_pose_callback,this);
    sub_node = nh.subscribe("map",1,&LocalGoal::node_callback,this);
    sub_check_point = nh.subscribe("checkpoint",1,&LocalGoal::check_point_callback,this);

    pub_local_goal = nh.advertise<geometry_msgs::PoseStamped>("local_goal",10);
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
    // printf("e");
    static int id = 1;
    // current_point.x = 0;
    // current_point.y = 0;
    if(id==1) update_check_point(id);
    double dist = std::sqrt(pow(current_point.x-goal_point.x,2)+pow(current_point.y-goal_point.y,2));
    printf("dist,id:%lf,%d\n",dist,list_cp[id]);
    // printf("id:%d\n",id);
    if(dist<goal_radius) {
        id++;
        update_check_point(id);
        // printf("%d\n",id);
    }
    geometry_msgs::PoseStamped goal;
    goal.pose.position.x = goal_point.x;
    goal.pose.position.y = goal_point.y;
    pub_local_goal.publish(goal);
}

void LocalGoal::update_check_point(int i)
{
    int node_id = list_cp[i];
    goal_point.x = Nodes[node_id].x;
    goal_point.y = Nodes[node_id].y;

}

void LocalGoal::process()
{
    // printf("a");
    ros::Rate loop_rate(hz);
    while(ros::ok())
    {
        // printf("b");
        if(flag_pose&&flag_node) {
            // printf("c");
            calc_goal_dist();
        }
        ros::spinOnce();
        loop_rate.sleep();
    }
}

int main(int argc, char** argv)
{
    // printf("d");
    ros::init(argc, argv, "local_goal_creator");
    LocalGoal LocalGoal;
    LocalGoal.process();
    return 0;
}
