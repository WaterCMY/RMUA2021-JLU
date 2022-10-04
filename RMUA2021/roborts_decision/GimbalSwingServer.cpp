#include<ros/ros.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<geometry_msgs/PoseStamped.h>
#include "executor/gimbal_executor.h"
#include "sdk.h"
#include "gimbal.h"

//#include "blackboard/blackboard.h"

//#include "../behavior_tree/behavior_node.h"
//#include "../behavior_tree/behavior_state.h"

#include "io/io.h"
#include "proto/decision.pb.h"

#include <actionlib/server/simple_action_server.h>
#include "roborts_msgs/GimbalSwingAction.h"
//#include "goal_factory.h"
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalActionlib.h"

 typedef actionlib::SimpleActionServer<roborts_msgs::GimbalSwingAction> Server;
 ros::Publisher scan_pub_;
 //std::shared_ptr<tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
 //roborts_decision::Blackboard::Ptr blackboard_;
 //auto goalfactory_ = std::make_shared<roborts_decision::GoalFactory>(blackboard_);


// 转动云台以扫描视野范围内是否有敌人
//bool ScanView(roborts_msgs::GimbalSwing::Request &req, roborts_msgs::GimbalSwing::Response& res)
//void ScanView()
//std::shared_ptr<tf::TransformListener> GetTFptr(){
 //       return tf_ptr_;
  //    }

int games_tatus=4;
bool camera_lost=false;
bool is_scan=false;
float angle_yaw=0;

void ActionlibCallBack(const roborts_msgs::GimbalActionlib::ConstPtr & actionlib_info){
  ROS_INFO("yawangle:%f",actionlib_info->yawangle);
  games_tatus=static_cast<unsigned int>(actionlib_info->gamestatus); 
  camera_lost=actionlib_info->cameralost;
  is_scan=actionlib_info->isscan; 
  angle_yaw=actionlib_info->yawangle; 
}

void ScanView(const roborts_msgs::GimbalSwingGoalConstPtr & goal, Server * as)
    { 
      std::shared_ptr<tf::TransformListener> tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
      //std::string path_=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
      //auto blackboard_ = std::make_shared<roborts_decision::Blackboard>(path_);
      //auto goalfactory_ = std::make_shared<roborts_decision::GoalFactory>(blackboard_);
      //roborts_decision::Blackboard::Ptr blackboard_;
      //auto goalfactory_ = std::make_shared<roborts_decision::GoalFactory>(blackboard_);
      roborts_msgs::GimbalSwingFeedback feedback;
      ROS_INFO("ScanView is working, goal->rate:%f",goal->rate);

      double angle_min = -1.50;
      double angle_max = 1.50;                // 这两个参数待定,云台的可运动范围
      double angle = 0.02;                       // 云台转动时单次运动角度
      short dir = 1;                          // 转动方向
      /*ros::NodeHandle nh_scan;
      ros::Publisher scan_pub_ =  nh_scan.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 30);
      ROS_INFO("scan_pub");
      ros::NodeHandle nh_actionlib;   
      ros::Subscriber actionlib_sub = nh_actionlib.subscribe<roborts_msgs::GimbalActionlib>("robort_info",30, &ActionlibCallBack);*/
      ROS_INFO("action_sub: cameralost=%d  isscan=%d  yaw_angle=%f  gamestatus=%d",camera_lost,is_scan,angle_yaw,games_tatus);

      roborts_msgs::GimbalAngle gimbal_angle_msg;
      gimbal_angle_msg.pitch_mode = false;
      gimbal_angle_msg.pitch_angle = 0;
      gimbal_angle_msg.yaw_mode = false;
      gimbal_angle_msg.yaw_angle = angle_yaw; 

        /*tf::Stamped<tf::Pose> gimbal_tf_pose;
        gimbal_tf_pose.setIdentity();
        gimbal_tf_pose.frame_id_ = "gimbal";  // 以上声明一个 gimbal 中的位置变量,默认原点
        gimbal_tf_pose.stamp_ = ros::Time();
        geometry_msgs::PoseStamped gimbal_base_pose;
        try{
            geometry_msgs::PoseStamped gimbal_pose;
            tf::poseStampedTFToMsg(gimbal_tf_pose, gimbal_pose);
            // 从 gimbal 转换到 base_link
            //blackboard_->GetTFptr()->transformPose("base_link", gimbal_pose, gimbal_base_pose);
            tf_ptr_->transformPose("base_link", gimbal_pose, gimbal_base_pose);
        }
        catch(tf::LookupException& e){
            ROS_ERROR("Transform Error looking up robot pose: %s", e.what());
        }
        tf::Quaternion q;
        tf::quaternionMsgToTF(gimbal_base_pose.pose.orientation, q);
        double r=0, p=0, y=0;
        tf::Matrix3x3(q).getRPY(r, p, y);
        if(-1.5 <= y && 1.5 >= y){
          gimbal_angle_msg.yaw_angle = y;
        }*/

        //while(GetGameStatus()!=3){
          //if(IsGimbalView()&&GetCameraLost()){
        ros::Rate rate(goal->rate);
        // while(games_tatus!=结束)
        while(games_tatus!=3){
          ROS_INFO("in while");
          if(camera_lost&&is_scan){
            ROS_INFO("in first IF");
            if(gimbal_angle_msg.yaw_angle+dir*angle <= -1.50 || gimbal_angle_msg.yaw_angle+dir*angle >= 1.50){
              dir *= -1;
            }
            gimbal_angle_msg.yaw_angle += dir*angle;
            //scan_pub_.publish(gimbal_angle_msg);
            ROS_INFO("scan_pub published: yaw_angle=%f",gimbal_angle_msg.yaw_angle);
            //goalfactory_->ScanPub(gimbal_angle_msg);
            // 按照频率发布进度feedback
            feedback.is_swing = true; 
          }else feedback.is_swing = false;
          as->publishFeedback(feedback);
          rate.sleep();
        }

        // 当action完成后，向客户端返回结果
	      ROS_INFO("ScanView finishes working ");
	      as->setSucceeded();
      //return true;
    }


int main(int argc , char ** argv){
	  ros::init(argc, argv, "gimbal_swing_server");
    ros::NodeHandle nh_scan;
    scan_pub_ =  nh_scan.advertise<roborts_msgs::GimbalAngle>("cmd_gimbal_angle", 30);
    ROS_INFO("scan_pub");
    ros::NodeHandle nh_actionlib;   
    ros::Subscriber actionlib_sub = nh_actionlib.subscribe<roborts_msgs::GimbalActionlib>("GimbalActionlib",30, ActionlibCallBack);
    ROS_INFO("action_sub: cameralost=%d  isscan=%d  yaw_angle=%f  gamestatus=%d",camera_lost,is_scan,angle_yaw,games_tatus);
    ros::NodeHandle nh_gimbal;
	  // 定义一个服务器
	  Server server(nh_gimbal, "gimbal_swing_", boost::bind(&ScanView, _1, &server), false);
	  // 服务器开始运行
	  server.start();
	
	  ros::spin();
	
	  return 0;
}