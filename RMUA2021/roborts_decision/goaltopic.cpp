#include "ros/ros.h"
#include <string.h>
#include "roborts_msgs/GoGoal.h"
#include <stdlib.h>


int main(int argc, char **argv)
{
  ros::init(argc, argv, "goaltopic_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roborts_msgs::GoGoal>("gogoal_pose_pub", 30);
  ros::Rate loop_rate(30);
    double  goal_x,goal_y;
    std::cout<<"set goal_x:"<< std::endl;
    std::cin>>goal_x ;
    std::cout<<"set goal_y:"<< std::endl;
    std::cin>>goal_y ;      
  while (ros::ok())
  {
    roborts_msgs::GoGoal msg;
    msg.go_goal_x = goal_x;
    msg.go_goal_y = goal_y;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}