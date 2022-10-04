#include "ros/ros.h"
#include <string.h>
#include "roborts_msgs/GoGoal.h"
#include <stdlib.h>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "zcx_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roborts_msgs::GoGoal>("gogoal_pose_pub", 30);
  ros::Rate loop_rate(30);
    double  goal_x,goal_y;
    /*
    cout<<"set goal_x:"<< endl;
    cin>>goal_x ;
    cout<<"set goal_y:"<< endl;
    cin>>goal_y ;
    */
   goal_x = 0.5;
   goal_y = 0.8;      
  while (ros::ok())
  {
    roborts_msgs::GoGoal msg;
    msg.go_goal_x = goal_x;
    msg.go_goal_y = goal_x;
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}