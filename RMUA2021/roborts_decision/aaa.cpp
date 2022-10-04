#include <ros/ros.h>

#include <string.h>
#include "roborts_msgs/sg.h"
#include <stdlib.h>



int main(int argc, char **argv)
{
  ros::init(argc, argv, "aaa_node");
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roborts_msgs::sg>("sg", 30);
  ros::Rate loop_rate(30);
  bool boolcar3,boolcar4;
    double  car3_x,car3_y,car4_x,car4_y;
    std::cout<<"set bool car3:"<< std::endl;
    std::cin>>boolcar3 ;
    if(boolcar3){
     std::cout<<"set car3_x:"<< std::endl;
    std::cin>>car3_x ;
      std::cout<<"set car3_y:"<< std::endl;
    std::cin>>car3_y ;   
    }
     std::cout<<"set bool car4:"<< std::endl;
     std::cin>>boolcar4 ;
   if(boolcar4){
     std::cout<<"set car4_x:"<< std::endl;
    std::cin>>car4_x ;
      std::cout<<"set car4_y:"<< std::endl;
    std::cin>>car4_y ;   
    } 
  while (ros::ok())
  {
    roborts_msgs::sg msg;
    if(boolcar3){
    msg.car1 = boolcar3;
    msg.car1_x = car3_x;
     msg.car1_y = car3_y;
    }
     if(boolcar4){
    msg.car1 = boolcar4;
    msg.car1_x = car4_x;
     msg.car1_y = car4_y;
    }
    pub.publish(msg);
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}