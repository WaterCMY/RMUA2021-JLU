#ifndef ROBORT_GOAL_FACTORY_H
#define ROBORT_GOAL_FACTORY_H

#include<ros/ros.h>
#include<random>
#include<tf/tf.h>
#include<geometry_msgs/PoseStamped.h>
#include<tf/transform_listener.h>

#include<actionlib/client/simple_action_client.h>
#include "../blackboard/blackboard.h"
#include "../executor/chassis_executor.h"
#include "roborts_msgs/ChassisCtrAction.h"

#include "../behavior_tree/behavior_node.h"
#include "../behavior_tree/behavior_state.h"
#include<ctime>

#include "io/io.h"
#include "../proto/decision.pb.h"
namespace roborts_decision{
  class GoalFactory
  {
protected : 
  Blackboard::Ptr blackboard_ptr_;
    std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
    std::shared_ptr<GimbalExecutor> gimbal_executor_ptr_;





  public:
    typedef std::shared_ptr<GoalFactory> GoalFactoryPtr;
    typedef actionlib::SimpleActionClient<roborts_msgs::ChassisCtrAction> Client;
  GoalFactory(const Blackboard::Ptr &blackboard_ptr):
  blackboard_ptr_(blackboard_ptr){
      chassis_executor_ptr_ = blackboard_ptr_->GetChassisExecutor();
      gimbal_executor_ptr_ = blackboard_ptr_ ->GetGimbalExecutor();
  }
    ~GoalFactory() = default;

void TurnToDetectedDirection() {
  double d_yaw=0;
  //double q=1;  
  //switch (blackboard_ptr_->GetDamageSource()) {
    switch (2) {
          case 0:  //前方
              break;  
          case 1:  //左方
              d_yaw = M_PI / 2;
              break;
          case 2:  //后方
              d_yaw = 3.14;
              //q=0.5;
              break;
          case 3:  //右方
              d_yaw = -M_PI / 2;
              break;
          default:
              return;    
        } 
        try{ 
          //ros::Rate rate(q);
          /*ROS_INFO("d_yaw=%d",d_yaw);
          geometry_msgs::PoseStamped hurt_pose;
          ROS_INFO("65");
          hurt_pose = blackboard_ptr_->GetRobotMapPose();
          tf::Quaternion cur_q;
          tf::quaternionMsgToTF(hurt_pose.pose.orientation, cur_q);
          double r, p, y;
          tf::Matrix3x3(cur_q).getRPY(r, p, y); 
          d_yaw = y + d_yaw;
          ROS_INFO("d_yaw=%d",d_yaw);
          tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,d_yaw);
         hurt_pose.header.frame_id = "map";
          hurt_pose.header.stamp = ros::Time::now();
          hurt_pose.pose.orientation.x = quaternion.x();
          hurt_pose.pose.orientation.y = quaternion.y();
          hurt_pose.pose.orientation.z = quaternion.z();
          hurt_pose.pose.orientation.w = quaternion.w();
          chassis_executor_ptr_->Execute(hurt_pose);*/
	  geometry_msgs::Twist speed; 
	      speed.linear.x = 0;
	          speed.linear.y = 0;
		      speed.linear.z = 0;
		      speed.angular.x = 0;
		    speed.angular.y = 0;
	       speed.angular.z = 4.5;
			chassis_executor_ptr_->Execute(speed);
          //rate.sleep(); 
        }
        catch(std::exception& e){
          ROS_ERROR("ERROR: %S",e.what());
        }
        blackboard_ptr_->SetDamageSourece();
      } 

void GoStopPose() {
      geometry_msgs::PoseStamped pose;
      pose = blackboard_ptr_->GetStopPose();
      chassis_executor_ptr_->Execute(pose);
    }


void GainBlood() {
      geometry_msgs::PoseStamped buff_goal;
      buff_goal.header.frame_id = "map";
      buff_goal.pose.position.x =0;
      buff_goal.pose.position.y =0;
      buff_goal.pose.position.z =0;     
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
      buff_goal.pose.orientation.x = quaternion.x();
      buff_goal.pose.orientation.y = quaternion.y();
      buff_goal.pose.orientation.z = quaternion.z();
      buff_goal.pose.orientation.w = quaternion.w();
      buff_goal = blackboard_ptr_->GetBloodPose();
      chassis_executor_ptr_->Execute(buff_goal);
    }

void GainBullet() {
      geometry_msgs::PoseStamped buff_goal;
      buff_goal.header.frame_id = "map";
      buff_goal.pose.position.x =0;
      buff_goal.pose.position.y =0;
      buff_goal.pose.position.z =0;     
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
      buff_goal.pose.orientation.x = quaternion.x();
      buff_goal.pose.orientation.y = quaternion.y();
      buff_goal.pose.orientation.z = quaternion.z();
      buff_goal.pose.orientation.w = quaternion.w();
      buff_goal = blackboard_ptr_->GetBulletPose();
      chassis_executor_ptr_->Execute(buff_goal);
    }


void GoStaticPose() {
      geometry_msgs::PoseStamped static_pose;
      static_pose.header.frame_id = "map";
      static_pose.pose.position.x =0;
      static_pose.pose.position.y =0;
      static_pose.pose.position.z =0;     
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
      static_pose.pose.orientation.x = quaternion.x();
      static_pose.pose.orientation.y = quaternion.y();
      static_pose.pose.orientation.z = quaternion.z();
      static_pose.pose.orientation.w = quaternion.w();
      static_pose = blackboard_ptr_->GetLeastStaticPose();
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto dx = static_pose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = static_pose.pose.position.y - robot_map_pose.pose.position.y;

      auto static_yaw = tf::getYaw(static_pose.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(static_pose.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_ptr_->Execute(static_pose);
      }
    }

void  GoGoal(){
  geometry_msgs::PoseStamped go_goal;
   go_goal.header.frame_id = "map";
      go_goal.pose.position.x = blackboard_ptr_->GetGoGoalX();
      go_goal.pose.position.y = blackboard_ptr_->GetGoGoalY();
      go_goal.pose.position.z =0;          
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
      go_goal.pose.orientation.x = quaternion.x();
      go_goal.pose.orientation.y = quaternion.y();
      go_goal.pose.orientation.z = quaternion.z();
      go_goal.pose.orientation.w = quaternion.w();
      geometry_msgs::PoseStamped robot_map_pose = blackboard_ptr_->GetRobotMapPose();
       //blackboard_ptr_->CostDetect(go_goal,robot_map_pose);
      chassis_executor_ptr_->Execute(go_goal);
}



void  Follow(){
      geometry_msgs::PoseStamped follow_goal;
      follow_goal.header.frame_id = "map";
      follow_goal.pose.position.x =0;
      follow_goal.pose.position.y =0;
      follow_goal.pose.position.z =0;     
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,0);  
      follow_goal.pose.orientation.x = quaternion.x();
      follow_goal.pose.orientation.y = quaternion.y();
      follow_goal.pose.orientation.z = quaternion.z();
      follow_goal.pose.orientation.w = quaternion.w();
      follow_goal = blackboard_ptr_->GetFollowPose();
      chassis_executor_ptr_->Execute(follow_goal);
}



void BackToBootArea(){
      geometry_msgs::PoseStamped boot_pose ; 
      boot_pose.header.frame_id = "map";
      boot_pose.pose.orientation.x = 0;
      boot_pose.pose.orientation.y = 0;
      boot_pose.pose.orientation.z = 0;
      boot_pose.pose.orientation.w = 1;

      boot_pose.pose.position.x = 0;
      boot_pose.pose.position.y = 0;
      boot_pose.pose.position.z = 0;
      
      boot_pose = blackboard_ptr_->LoadBootPosition();
      auto robot_map_pose = blackboard_ptr_->GetRobotMapPose();
      auto dx = boot_pose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = boot_pose.pose.position.y - robot_map_pose.pose.position.y;

      auto boot_yaw = tf::getYaw(boot_pose.pose.orientation);
      auto robot_yaw = tf::getYaw(robot_map_pose.pose.orientation);

      tf::Quaternion rot1, rot2;
      tf::quaternionMsgToTF(boot_pose.pose.orientation, rot1);
      tf::quaternionMsgToTF(robot_map_pose.pose.orientation, rot2);
      auto d_yaw =  rot1.angleShortestPath(rot2);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) > 0.2 || d_yaw > 0.5) {
        chassis_executor_ptr_->Execute(boot_pose);
      }
    }


    /*void SearchGoal(){
      geometry_msgs::PoseStamped path;
      path.header.frame_id = "map";
      path.pose.orientation.x = 0;
      path.pose.orientation.y = 0;
      path.pose.orientation.z = 0;
      path.pose.orientation.w = 1;
      path.pose.position.x = 0;
      path.pose.position.y = 0;
      path.pose.position.z = 0;
      int count = blackboard_ptr_->GetSearchCount();
      ROS_INFO("search count:%d",count);
      int size =  blackboard_ptr_->GetSearchPointSize();
      ROS_INFO("search size:%d",size);
        path = blackboard_ptr_->GetSearchPose(count);
        geometry_msgs::PoseStamped current_pose =blackboard_ptr_->GetRobotMapPose();
        auto dx = current_pose.pose.position.x - path.pose.position.x;        
        auto dy = current_pose.pose.position.y - path.pose.position.y;        
        double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(s_distance<0.4){
           blackboard_ptr_->SetSearchCount((count+1)%size);
        }
      count = blackboard_ptr_->GetSearchCount();
      path = blackboard_ptr_->GetSearchPose(count);
      try{
        chassis_executor_ptr_->Execute(path);
      }
      catch(std::exception& e){
        ROS_WARN("search execute error %s: ", e.what());
      }
    }*/


  void SearchGoal(){
      geometry_msgs::PoseStamped path;
      path.header.frame_id = "map";
      path.pose.orientation.x = 0;
      path.pose.orientation.y = 0;
      path.pose.orientation.z = 0;
      path.pose.orientation.w = 1;
      path.pose.position.x = 0;
      path.pose.position.y = 0;
      path.pose.position.z = 0;
      int count = blackboard_ptr_->GetSearchCount();
      /*if(count<3){
            srand(time(0));
           count = rand()%3;
      }*/
      ROS_INFO("search count:%d",count);
      int size =  blackboard_ptr_->GetSearchPointSize();
      ROS_INFO("search size:%d",size);
        path = blackboard_ptr_->GetSearchPose(count);
        geometry_msgs::PoseStamped current_pose =blackboard_ptr_->GetRobotMapPose();
        auto dx = current_pose.pose.position.x - path.pose.position.x;        
        auto dy = current_pose.pose.position.y - path.pose.position.y;        
        double s_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(s_distance<0.8){
           blackboard_ptr_->SetSearchCount((count+1)%size);
        }
      count = blackboard_ptr_->GetSearchCount();
      path = blackboard_ptr_->GetSearchPose(count);
      path.pose.orientation.x = current_pose.pose.orientation.x;
      path.pose.orientation.y = current_pose.pose.orientation.y;
      path.pose.orientation.z = current_pose.pose.orientation.z;
      path.pose.orientation.w = current_pose.pose.orientation.w;
      try{
        chassis_executor_ptr_->Execute(path);
      }
      catch(std::exception& e){
        ROS_WARN("search execute error %s: ", e.what());
      }
    }


    void Escape(){
      geometry_msgs::PoseStamped escape_goal;
      escape_goal.header.frame_id = "map";
      escape_goal.pose.orientation.x = 0;
      escape_goal.pose.orientation.y = 0;
      escape_goal.pose.orientation.z = 0;
      escape_goal.pose.orientation.w = 1;
      escape_goal.pose.position.x = 0;
      escape_goal.pose.position.y = 0;
      escape_goal.pose.position.z = 0;
      blackboard_ptr_->UpdateEscapeReg();
      int region = blackboard_ptr_->GetEscapeRegion();
      escape_goal = blackboard_ptr_ -> GetEscapePoints(region);
      try{
        chassis_executor_ptr_->Execute(escape_goal);
      }
      catch(std::exception& e){
        ROS_WARN("escape error occured : %s", e.what());
      }
    }



    /*void SwingDefend()
    {
      float move_dst = 0.20; // 在摆动防御是车身左右移动的距离, 暂定20cm
      geometry_msgs::PoseStamped swing_goal;
      swing_goal.header.frame_id = "map";
      swing_goal = blackboard_ptr_->GetRobotMapPose();
      ROS_INFO("Got pose.");
      tf::Quaternion cur_q;
      tf::quaternionMsgToTF(swing_goal.pose.orientation, cur_q);
      double r, p, y;
      tf::Matrix3x3(cur_q).getRPY(r, p, y); // 从位置信息中获取车身的角度
      float move_x = asin(y) * move_dst;    // x轴方向上移动距离
      float move_y = -acos(y) * move_dst;   // y轴方向上移动距离
      swing_goal.pose.position.z = 0;
      //ros::Rate rate(req.rate);
      ros::Rate rate(20);
      //res.is_swing = true;
      while (ros::ok() && blackboard_ptr_->IsSwing())
      {
        ROS_INFO("Is swing");
        // 更新 x, y 轴坐标
        swing_goal.pose.position.x += move_x;
        swing_goal.pose.position.y += move_y;
        chassis_executor_ptr_->Execute(swing_goal);
        rate.sleep();
        swing_goal.pose.position.x -= move_x;
        swing_goal.pose.position.y -= move_y;
        chassis_executor_ptr_->Execute(swing_goal);
        rate.sleep();
      }
      //return true;
    } //SwingDefend*/

/*void  SwingDefend(){
       geometry_msgs::PoseStamped swing_goal;
       swing_goal = blackboard_ptr_->GetRobotMapPose();
       swing_goal.pose.position.x = swing_goal.pose.position.x+0.2;
       swing_goal.pose.position.y = swing_goal.pose.position.y+0.1;
       chassis_executor_ptr_->Execute(swing_goal);
       swing_goal.pose.position.x = swing_goal.pose.position.x-0.2;
       swing_goal.pose.position.y = swing_goal.pose.position.y-0.1;
        chassis_executor_ptr_->Execute(swing_goal);
}*/

void  SwingDefend(){
    Client client("chassisdefend", true);
    ROS_INFO("WAITING FOR ACTION SERVER TO START !");
    client.waitForServer();
    ROS_INFO("ACTION SERVER START !");
   roborts_msgs::ChassisCtrGoal flag;
    flag.model = 1;   
    client.sendGoal(flag);
    ros::spin();
}




void  RollingDefend(){
geometry_msgs::Twist speed; 
 std::random_device rd;
std::mt19937_64 eng(rd());
std::uniform_int_distribution<unsigned long long> distr(400,600);
double angle = (double)distr(eng)/100;
    speed.linear.x = 0;
    speed.linear.y = 0;
    speed.linear.z = 0;
   speed.angular.x = 0;
   speed.angular.y = 0;
   speed.angular.z = angle;
    chassis_executor_ptr_->Execute(speed); 
    //loopRate.sleep();
}

void  Forward(){
geometry_msgs::Twist speed; 

    speed.linear.x = 1;
    speed.linear.y = 0;
    speed.linear.z = 0;
   speed.angular.x = 0;
   speed.angular.y = 0;
   speed.angular.z = 0;
    chassis_executor_ptr_->Execute(speed); 
    //loopRate.sleep();
}

void Chase(){
  geometry_msgs::PoseStamped enemy_pose;
  enemy_pose = blackboard_ptr_->HandleEnemyPose();
  geometry_msgs::PoseStamped robot_map_pose = blackboard_ptr_->GetRobotMapPose();
  float threa,self_threa;
       threa = blackboard_ptr_->GetEnemyThrea();
       self_threa = blackboard_ptr_->GetRobotThrea();
  auto dx = enemy_pose.pose.position.x - robot_map_pose.pose.position.x;
  auto dy = enemy_pose.pose.position.y - robot_map_pose.pose.position.y;
  tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, threa);
  enemy_pose.pose.orientation.x = quaternion.x();
  enemy_pose.pose.orientation.y = quaternion.y();
  enemy_pose.pose.orientation.z = quaternion.z();
  enemy_pose.pose.orientation.w = quaternion.w();
  /*if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2.5 && abs(self_threa-threa) <= 0.1) {
        chassis_executor_ptr_->Cancel();
      }
  else{
    chassis_executor_ptr_->Execute(enemy_pose);
  }*/
  if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2 && abs(self_threa-threa) <= 0.1) {
        chassis_executor_ptr_->Cancel();
      }
  else{
    if(std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 2 ){
    robot_map_pose.pose.orientation.x = quaternion.x();
  robot_map_pose.pose.orientation.y = quaternion.y();
  robot_map_pose.pose.orientation.z = quaternion.z();
  robot_map_pose.pose.orientation.w = quaternion.w();
    chassis_executor_ptr_->Execute(robot_map_pose);
    }
    else{
      chassis_executor_ptr_->Execute(enemy_pose);
    }
  }
}

void  Swing(){
geometry_msgs::Twist speed; 
ros::Rate   r(3);
for(int i = 0;i<=16;i++)
{
  switch(i%8){
    case 0 :
        speed.linear.x = 0.5;
        speed.linear.y = 0;
        break;
    case 1 :
        speed.linear.x = -0.5;
        speed.linear.y = 0;
        break;
  case 2 :
        speed.linear.x = -0.5;
        speed.linear.y = 0;
        break;
  case 3 :
        speed.linear.x = 0.5;
        speed.linear.y = 0;
        break;  
   case 4 :
        speed.linear.x = 0;
        speed.linear.y = 0.5;
        break;
    case 5 :
        speed.linear.x = 0;
        speed.linear.y = -0.5;
        break;
  case 6 :
        speed.linear.x = 0;
        speed.linear.y = -0.5;
        break;
  case 7 :
        speed.linear.x = 0;
        speed.linear.y = 0.5;
        break;  
  }
    speed.linear.z = 0;
   speed.angular.x = 0;
   speed.angular.y = 0;
   speed.angular.z = 0;
    chassis_executor_ptr_->Execute(speed); 
   	 r.sleep();
		}
}



void CancelGoal() {
      ROS_INFO("Cancel Goal!");
      blackboard_ptr_->SetCancelFlag(false);
      // TODO 这两行实际是需要的，在和路径规划整合代码的时候取消规划的内容(不一定是下面这两句)
      //chassis_executor_ptr_->cancelGlobalGoal();
      //chassis_executor_ptr_->cancelLocalGoal();
      blackboard_ptr_->SetActionState(BehaviorState::IDLE);
    }

    // 放弃云台控制权, 取消转动云台进行扫描的行为
    void CancelScan(){
        if(blackboard_ptr_->IsGimbalView()){
            blackboard_ptr_->SetGimbalScan(false);
            ROS_INFO("cancel gimbal-scan action .");
        }
    }//cancel scan

    void CancelChassis(){
      // FIXME 这块的作用是禁止地盘的运动
      chassis_executor_ptr_->Cancel();
    }

    void CancelGimbal(){
      // FIXME 这块作用是让云台不动，所以也要让视觉不能动云台
      gimbal_executor_ptr_ -> Cancel();
    }

  };

}
#endif 
