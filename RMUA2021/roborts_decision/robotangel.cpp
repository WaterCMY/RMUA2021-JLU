#include "ros/ros.h"
#include <string.h>
#include "roborts_msgs/RobotAngleTest.h"
#include <stdlib.h>
#include "executor/chassis_executor.h"
#include "blackboard/blackboard.h"


int main(int argc, char **argv)
{
  ros::init(argc, argv, "robot_angle_node");
  ros::Time::init();
  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<roborts_msgs::RobotAngleTest>("robot_angle_pub", 5);
  ros::Rate loop_rate(30);  
  std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
  auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
  while (ros::ok())
  {
    
    roborts_msgs::RobotAngleTest msg;
    msg.self_threa = blackboard_ptr_->GetRobotThrea();
    msg.distance = blackboard_ptr_->GetEnemyDistance();
    msg.enemy_threa = blackboard_ptr_->GetEnemyThrea();
    bool lost1,lost2;
    lost1 =  blackboard_ptr_->GetEnemyOneLost();
    lost2 =  blackboard_ptr_->GetEnemyTwoLost();   
    bool lost = lost1 && lost2;
    if(!lost){
    pub.publish(msg);
    }
    else{
    msg.self_threa = 0;
    msg.enemy_threa = 0;
    pub.publish(msg);
    }
    ros::spinOnce();
    loop_rate.sleep();
  }
  return 0;
}