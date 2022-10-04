#include<ros/ros.h>
#include "roborts_msgs/RobotStatus.h"

int robort_id_;
std::string config_dir;
float initial_pose_x1;
float initial_pose_x2;
float initial_pose_x3;
float initial_pose_x4;
float initial_pose_y1;
float initial_pose_y2;
float initial_pose_y3;
float initial_pose_y4;
float initial_pose_a1;
float initial_pose_a2;
float initial_pose_a3;
float initial_pose_a4;
void set_initialposeCallback(const roborts_msgs::RobotStatus::ConstPtr & robot_status){
    robort_id_=robot_status->id;
    if (robort_id_ == 1) {
        config_dir = "red1";}
    else if (robort_id_ == 2) {
        config_dir = "red2";}
    else if (robort_id_ == 101) {
        config_dir = "blue1";}
    else if (robort_id_ == 102) {
        config_dir = "blue2";}
    ros::param::set("config_dir", config_dir);
    ros::param::set("robort_id", robort_id_);
    switch(robort_id_){
        case 1:
        ros::param::set("initial_pose_x", initial_pose_x1);
        ros::param::set("initial_pose_y", initial_pose_y1);
        ros::param::set("initial_pose_a", initial_pose_a1);
        break;
        case 2:
        ros::param::set("initial_pose_x", initial_pose_x2);
        ros::param::set("initial_pose_y", initial_pose_y2);
        ros::param::set("initial_pose_a", initial_pose_a2);
        break;
        case 101:
        ros::param::set("initial_pose_x", initial_pose_x3);
        ros::param::set("initial_pose_y", initial_pose_y3);
        ros::param::set("initial_pose_a", initial_pose_a3);
        break;
        case 102:
        ros::param::set("initial_pose_x", initial_pose_x4);
        ros::param::set("initial_pose_y", initial_pose_y4);
        ros::param::set("initial_pose_a", initial_pose_a4);
        break;
    }
}

int main(int argc, char **argv){
    initial_pose_x1 = 7.62;
    initial_pose_x2 = 7.55;
    initial_pose_x3 = 0.83;
    initial_pose_x4 = 0.68;
    initial_pose_y1 = 0.53;
    initial_pose_y2 = 3.93;
    initial_pose_y3 = 4.10;
    initial_pose_y4 = 0.72;
    initial_pose_a1 = 3.14;
    initial_pose_a2 = -1.57;
    initial_pose_a3 = 0;
    initial_pose_a4 = 1.57;
    ros::init(argc, argv, "set_initialpose");	
    ros::NodeHandle n;
    ros::Subscriber sub_robort_id = n.subscribe("robot_status",1,set_initialposeCallback);
    ros::spin();
    return 0;

}