#ifndef ROBORTS_DECISION_BLACKBOARD_H
#define ROBORTS_DECISION_BLACKBOARD_H
#include <actionlib/client/simple_action_client.h>
#include<tf/tf.h>
#include<tf/transform_listener.h>
#include<ros/ros.h>
#include<geometry_msgs/PoseStamped.h>
#include "costmap/costmap_interface.h"


//#include "roborts_msgs/ArmorDetection.h"
#include "roborts_msgs/GameResult.h"
#include "roborts_msgs/GameStatus.h"
#include "roborts_msgs/RobotStatus.h"
#include "roborts_msgs/RobotDamage.h"
#include "roborts_msgs/RobotHeat.h"
#include "roborts_msgs/RobotShoot.h"
#include "roborts_msgs/GameRobotHP.h"
#include "roborts_msgs/GameRobotBullet.h"
#include "roborts_msgs/ShootInfo.h"
#include "roborts_msgs/GimbalMode.h"
#include "roborts_msgs/ShootCmd.h"
#include "roborts_msgs/FricWhl.h"
//#include "roborts_msgs/GameSurvivor.h"
#include "roborts_msgs/SentryInfo.h"
#include "roborts_msgs/PunishInfo.h"
#include "roborts_msgs/RobotInfo.h"
#include "roborts_msgs/TreeStatus.h"
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/GameZone.h"

#include "roborts_msgs/FricWhl.h"
#include "roborts_msgs/PyArmorInfo.h"
#include "roborts_msgs/interact.h"

#include "roborts_msgs/GoGoal.h"

#include "roborts_msgs/sg.h"

#include "../action_node/line_iterator.h"

#include "../proto/decision.pb.h"
#include "../executor/chassis_executor.h"
#include "../executor/gimbal_executor.h"
#include "sdk.h"
//#include "gimbal.h"

namespace roborts_decision{
    class Blackboard
    {

    public:
    typedef std::shared_ptr<Blackboard> Ptr;
    typedef roborts_costmap::CostmapInterface CostMap;
    typedef roborts_costmap::Costmap2D CostMap2D;
    
     explicit Blackboard(const std::string &proto_file_path):
     proto_file_path_(proto_file_path),
    mate_punished_(false),        
    remain_hp_(2000),
    last_check_attacked_time_(ros::Time::now()),
    search_count_(0),
    search_points_size_(0),
    escape_region_(0),
    camera_lost_(true),
    camera_z_(0),
    camera_x_(0),
    camera_y_(0),
    c_angle_(0),
    is_scan_(false),
    is_swing_(false), 
    camera_count(0),
    has_followed_(false), 
    enemy1_hp_(2000),
    enemy2_hp_(2000),
    gain_bullets_flag(false),
    gain_blood_flag(false),
    cls_num_(0),
    sentry_lost_(false),
    damage_type_(0), 
    damage_source_(-1), 
    game_status_(0),
    remaining_time_(180),       
    teammate_hp_(2000), 
    last_hp_(2000), 
    self_blps_(0),
    teammate_bullet_(0), 
    remain_bullet_(50),
    mate_gimbal_(false),
    has_chase_enemy_(false),
    mate_chassis_(false),
    mate_tree_running_(false),
    mate_chase_enemy_(false),
    robot_heat_(0),
    near_enemy_num_(0),
    is_master_(true),
    action_state_(BehaviorState::IDLE),     
    cancel_flag_(true),
    is_go_goal_(false),
    is_go_buff_(false),
    sentry_enemyone_lost(false),
    sentry_enemytwo_lost(false),
   is_follow_(false),
   if_punished_(false),
    gimbal_punished_(false),
    chassis_punished_(false),
    is_search_(false) ,
    hp_count(0),
    red1_hp(2000),
    red2_hp(2000),
    blue1_hp(2000),
    blue2_hp(2000),
    blood_active_(1),
    bullet_active_(1),
    is_blood_lose_fast(0),
    go_goal_y_(0.5),
    go_goal_x_(0.5)  
     {
        tf_ptr_ = std::make_shared<tf::TransformListener>(ros::Duration(10));
        chassis_executor_ptr_=std::make_shared<ChassisExecutor>();

        std::string map_path = ros::package::getPath("roborts_costmap") + \
        "/config/costmap_parameter_config_for_decision.prototxt";
        costmap_ptr_ = std::make_shared<CostMap>("decision_costmap", *tf_ptr_,
                                             map_path);
        charmap_ = costmap_ptr_->GetCostMap()->GetCharMap();
        costmap_2d_ = costmap_ptr_->GetLayeredCostmap()->GetCostMap();


        ros::NodeHandle sentry_nh;
        sentry_sub = sentry_nh.subscribe<roborts_msgs::sg>("sg",30, &Blackboard::SentryCallBack, this);


        ros::NodeHandle camera_nh;
        camera_sub = camera_nh.subscribe<roborts_msgs::PyArmorInfo>("PyArmorInfo",30,&Blackboard::CameraCallBack,this);

        ros::NodeHandle referee_nh;
        game_status_sub_ = referee_nh.subscribe<roborts_msgs::GameStatus>("game_status",30, &Blackboard::GameStatusCallback, this);
        game_result_sub_ =  referee_nh.subscribe<roborts_msgs::GameResult>("game_result", 30,&Blackboard::GameResultCallback, this);
        robot_hp_sub_ = referee_nh.subscribe<roborts_msgs::GameRobotHP>("game_robot_hp", 30,&Blackboard::RobotHPCallback, this);
        robot_bullet_sub_ = referee_nh.subscribe<roborts_msgs::GameRobotBullet>("game_robot_bullet", 30,&Blackboard::RobotBulletCallback, this);
        game_zone_array_status_sub_ = referee_nh.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 30,&Blackboard::GameZoneArrayCallback, this);
        ros_robot_status_sub_ = referee_nh.subscribe<roborts_msgs::RobotStatus>("robot_status", 30,&Blackboard::RobotStatusCallback, this);
        robot_heat_sub_ = referee_nh.subscribe<roborts_msgs::RobotHeat>("robot_heat",30,&Blackboard::RobotHeatCallback, this);
        robot_damage_sub_ = referee_nh.subscribe<roborts_msgs::RobotDamage>("robot_damage" ,30,&Blackboard::RobotDamageCallback,this);
        robot_shoot_sub_ = referee_nh.subscribe<roborts_msgs::RobotShoot>("robot_shoot", 30,&Blackboard::RobotShootCallback, this);



        ros::NodeHandle interact_nh;   
        //mate_info_sub = interact_nh.subscribe<roborts_msgs::RobotInfo>("robort_info",30, &Blackboard::MateInfoCallBack, this);
        mate_punish_sub = interact_nh.subscribe<roborts_msgs::PunishInfo>("punish",30, &Blackboard::MatePubnishCallBack, this);
        mate_treestatus_sub = interact_nh.subscribe<roborts_msgs::TreeStatus>("status",30, &Blackboard::MateTreeStatusCallBack, this);
        companion_info_sub = interact_nh.subscribe<roborts_msgs::interact>("mate_info",30, &Blackboard::CompanionCallBack, this);        



        ros::NodeHandle gogoal_nh;

        gogoal_sub_ = gogoal_nh.subscribe<roborts_msgs::GoGoal>("gogoal_pose_pub", 30,&Blackboard::GoGoalCallback, this);

      for(int i=0 ;i++;i<2){
        self_blood_loss_per_second_[i] = 0;
        mate_blood_loss_per_second_[i] = 0;
        enemy1_blood_loss_per_second_[i] = 0;
        enemy2_blood_loss_per_second_[i] = 0;
      }

        buff_pose_.header.frame_id = "map";
        buff_pose_.pose.position.x = 0;
        buff_pose_.pose.position.y = 0;
        buff_pose_.pose.position.z = 0;
        buff_pose_.pose.orientation.x = 0;
        buff_pose_.pose.orientation.y = 0;
        buff_pose_.pose.orientation.z = 0;
        buff_pose_.pose.orientation.w = 1;

        follow_pose_.header.frame_id = "map";
        follow_pose_.pose.position.x = 0;
        follow_pose_.pose.position.y = 0;
        follow_pose_.pose.position.z = 0;
        follow_pose_.pose.orientation.x = 0;
        follow_pose_.pose.orientation.y = 0;
        follow_pose_.pose.orientation.z = 0;
        follow_pose_.pose.orientation.w = 1;

        cls_.resize(5);
        for(int i=0;i<5;i++) cls_[i]=0;
        deck_time_.resize(5);
        for(int i=0;i<5;i++) deck_time_[i]=0;
         chase_goal_.resize(2);
        chase_goal_[0].header.frame_id = "map";
        chase_goal_[0].pose.orientation.x = 0;
        chase_goal_[0].pose.orientation.y = 0;
        chase_goal_[0].pose.orientation.z = 0;
        chase_goal_[0].pose.orientation.w = 1;
        chase_goal_[0].pose.position.x = 0;
        chase_goal_[0].pose.position.y = 0;
        chase_goal_[0].pose.position.z = 0;
        chase_goal_[1].header.frame_id = "map";
        chase_goal_[1].pose.orientation.x = 0;
        chase_goal_[1].pose.orientation.y = 0;
        chase_goal_[1].pose.orientation.z = 0;
        chase_goal_[1].pose.orientation.w = 1;
        chase_goal_[1].pose.position.x = 0;
        chase_goal_[1].pose.position.y = 0;
        chase_goal_[1].pose.position.z = 0;


        LoadParam(proto_file_path);
     }
    
      ~Blackboard() = default;
    


    /*  ——————————————————Get   Set   Is函数————————————————*/
  void SetHasChaseEnemy(bool has_chase_enemy){
    has_chase_enemy_ = has_chase_enemy;
  }

  bool GetHasChaseEnemy(){
    return has_chase_enemy_;
  }

  void SetGainBulletsFlag(bool flag){
    gain_bullets_flag = flag;
  }

  void SetGainBloodFlag(bool flag){
    gain_blood_flag = flag;
  }


    void SetRobotGoal(geometry_msgs::PoseStamped robort_goal){
        robort_goal_ = robort_goal;
      }

    geometry_msgs::PoseStamped GetRobotGoal(){
        return robort_goal_;
      } 
      
    bool IsGimbalView(){
        return is_scan_;
      }

    std::shared_ptr<ChassisExecutor> GetChassisExecutor(){
        return chassis_executor_ptr_;
      }

    std::shared_ptr<GimbalExecutor> GetGimbalExecutor(){
        return gimbal_executor_ptr_;
      }
    
    geometry_msgs::PoseStamped GetBuffPose(){
        return buff_pose_;
      }

    geometry_msgs::PoseStamped GetFollowPose(){
        return follow_pose_;
      }
    
    void SetFollowPose(geometry_msgs::PoseStamped follow){
      follow_pose_ = follow;
    }

    std::shared_ptr<tf::TransformListener> GetTFptr(){
        return tf_ptr_;
      }

      BehaviorState GetActionState(){
        return action_state_;
      }

    void SetActionState(BehaviorState action_state){
        action_state_=action_state;
      }

      void SetSwing(bool flag){
        is_swing_ = flag;
      }

     bool IsSwing(){
        return is_swing_;
      }

    bool IfPunished(){
        return if_punished_;
      }   

      bool IfTeammatePunished(){
        return mate_punished_;
      }

     bool GimbalPunished(){
        return gimbal_punished_;
      }

      bool ChassisPunished(){
        return chassis_punished_;
      }

      int GetNearEnemyNum(){
        return near_enemy_num_;
      } 

      void SetNearEnemyNum(int num){
        near_enemy_num_ = num;
      }

      void SetBuffPose(geometry_msgs::PoseStamped &buff_pose){
        buff_pose_ = buff_pose;
      }

      bool IsMaster(){
        return is_master_;
      }

      void SetMaster(bool master){
        is_master_ = master;
      }

      int GetRobotId(){
      	return id_;
      }

    void SetRobotId(int id){
      	 id_ = id;
      }  

      bool GetCameraLost(){
        return camera_lost_;
      }

      void SetCameraLost(bool e_lost_){
        camera_lost_ = e_lost_;
      }

      bool IsEnemyLost(){
        if(sentry_lost_){
          return true;
        } else{
          return false;
        }
      }
      
    void SetCompositeAngle(double c_angle){
        c_angle_ = c_angle;
      }

    double GetCompositeAngle(){
        return c_angle_;
        ROS_INFO("c_angle_: %f",c_angle_);
      }

    void SetGimbalScan(bool is_scan){
        is_scan_ = is_scan;
      }

    bool DeckChangeSlow(){
        return deck_change_slow_;
      }

    void SetCancelFlag(bool flag){
        cancel_flag_ = flag;
      }

    bool GetCancelFlag(){
        return cancel_flag_;
      }

       void SetGoGoal(bool is_go_goal){
        ROS_INFO("set gogoal:%d",is_go_goal);
        is_go_goal_=is_go_goal;
      }

      bool IsGoGoal(){
	      ROS_INFO("is gogoal:%d",is_go_goal_);
	      return is_go_goal_;
      }

        void SetGoBuff(bool is_go_buff){
        ROS_INFO("set gobuff:%d",is_go_buff);
        is_go_buff_=is_go_buff;
      }

      bool IsGoBuff(){
	      ROS_INFO("is gogoal:%d",is_go_buff_);
	      return is_go_buff_;
      }


        void SetFollow(bool is_follow){
        ROS_INFO("set follow:%d",is_follow);
        is_follow_=is_follow;
      }

      bool IsFollow(){
	      ROS_INFO("is follow:%d",is_follow_);
	      return is_follow_;
      }



      bool IsNearRefreshTime(){
        if((remaining_time_ > 120.5) && (remaining_time_<=122)){
          return true;
        }
        if((remaining_time_ > 60.5 ) && (remaining_time_<=62)){
          return true;
        }
        else{
          return false;
          }
      }


      bool IsNearRefresh(){
        if(GetBloodActive() || GetBulletActive()){
          return true;
        }
        else{
          return false;
        }
      }


       bool IsBloodAdvantage(){
       /*  int max_hp_ = (enemy1_hp_> enemy2_hp_ ? enemy1_hp_ : enemy2_hp_);
        bool x = ((int)remain_hp_-(int)max_hp_ >50);
        if(teammate_hp_ == 0 && !x) {return false;}*/
        if(((int)remain_hp_ + (int)teammate_hp_ - (int)enemy1_hp_ - (int)enemy2_hp_ >= 400) || (remain_hp_ >500)){
          return true;
        }
        else{
          return false;
        }
      }

      bool IsBulletAdvantage(){
        /*int max_bullet_ = (enemy1_bullet_> enemy2_bullet_ ? enemy1_bullet_ : enemy2_bullet_);
        bool x = ((int)remain_bullet_-(int)max_bullet_ >10);
        if((teammate_hp_ == 0 && !x )     ||   (teammate_hp_ == 0 && remain_bullet_<=80)) {return false;}*/
        if((int)remain_bullet_ + (int)teammate_bullet_ - (int)enemy1_bullet_ - (int)enemy2_bullet_  >= 150){
          return true;
        }
        else{
          return false;
        }
      }

    /*bool IsBloodLossFast(){
      double hurtedpersecond_ = HurtedPerSecond();
      if(hurtedpersecond_>=50){
        return true;
      }
      else {
        return false;
      }
    }*/

    bool GetHasFollowed(){
      return has_followed_;
    }

    void SetHasFollowed(bool has_follow){
         has_followed_=has_follow;
    }

    bool IsEnemyNear(){
      if(GetEnemyDistance()<=2.5 && GetEnemyDistance()!= -1 ){return true;}   //FIXME  4待定
      else{return false;} 
    }


    bool IsBloodEndangered(){
      if(remain_hp_ < 300){return true;}   //FIXME
      else{return false;}
    }


    bool IsBulletsEndangered(){
      if(remain_bullet_ == 0){return true;}   //FIXME
      else{return false;}
    }





/*  ——————————————————search 相关函数————————————————*/
    int GetSearchCount(){
        return search_count_;
      }

    void SetSearchCount(unsigned int count){
        ROS_INFO("set search count: %d",search_count_);
        search_count_ = count;
      }

    int GetSearchPointSize(){
        return search_points_size_;
      }

    geometry_msgs::PoseStamped GetSearchPose(int s_count){
        ROS_INFO("get x:%f,y:%f",search_point[s_count].pose.position.x,search_point[s_count].pose.position.y);
        return search_point[s_count];
      }


      void SetSearch(bool is_search){
        ROS_INFO("set search:%d",is_search);
        is_search_=is_search;
      }

      bool IsSearch(){
	      ROS_INFO("is escape:%d",is_search_);
	      return is_search_;
      }
/*  ——————————————————escape相关函数————————————————*/
    geometry_msgs::PoseStamped GetEscapePoints(int e_count){
        ROS_INFO("get escpae x:%f,y:%f",escape_points_[e_count].pose.position.x,escape_points_[e_count].pose.position.y);
        return escape_points_[e_count];
      }

    geometry_msgs::PoseStamped GetBuffPoints(int b_count){//b_count 0 对应F1
        //ROS_INFO("get  buff x:%f,y:%f",buff_debuff_points_[b_count].pose.position.x,buff_debuff_points_[b_count].pose.position.y);
        return buff_debuff_points_[b_count];
      }


      void SetEscapeRegion(int escape_region){
          escape_region_ = escape_region;
      }

      int GetEscapeRegion(){
          return escape_region_;
      }

      void UpdateEscapeReg(){
        int reg = GetCurrentRegion();
        geometry_msgs::PoseStamped e_pose =GetEscapePoints(reg);
        geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
        auto dx = current_pose.pose.position.x - e_pose.pose.position.x;        
        auto dy = current_pose.pose.position.y - e_pose.pose.position.y;        
        double e_distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        if(e_distance<0.5){
          SetEscapeRegion((reg+2)%4);
        }
      }

      int GetCurrentRegion(){
          geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
          double x, y;
          x = current_pose.pose.position.x;
          y = current_pose.pose.position.y;
          if(x <= 4.04){
            if(y > 2.24){
              return 0;
            }else
              {
                return 3;
              }
          }else{
            if(y > 2.4){
              return 1;
            }else{
              return 2;
            }
          }
        }

      void UpdateSearchCount(){
        geometry_msgs::PoseStamped current_pose =GetRobotMapPose();
        double x, y;
        x = current_pose.pose.position.x;
        y = current_pose.pose.position.y;
        if(x <= 4.04){
          if(y > 2.24){
            SetSearchCount(0);
            ROS_INFO("update search count 0");
          }else
          {
            SetSearchCount(1);
            ROS_INFO("update search count 1");
          }
        }else{
          if(y > 2.4){
            SetSearchCount(6);
            ROS_INFO("update search count 6");
          }else{
            SetSearchCount(4);
            ROS_INFO("update search count 4");
          }
        }
      }


      /*  ——————————————————其他函数————————————————*/

  void SetBloodPose(geometry_msgs::PoseStamped blood_pose){
    blood_pose_ = blood_pose;
  }

  void SetEnemyBloodPose(geometry_msgs::PoseStamped blood_pose){
    enemy_blood_pose_ = blood_pose;
  }

  geometry_msgs::PoseStamped GetEnemyBloodPose(){
    return enemy_blood_pose_;
  }


  geometry_msgs::PoseStamped GetBloodPose(){
    return blood_pose_;
  }

  void SetBulletPose(geometry_msgs::PoseStamped bullet_pose){
    bullet_pose_ = bullet_pose;
  }

  void SetEnemyBulletPose(geometry_msgs::PoseStamped blood_pose){
    enemy_bullet_pose_ = blood_pose;
  }

  geometry_msgs::PoseStamped GetBulletPose(){
    return bullet_pose_;
  }

  geometry_msgs::PoseStamped GetEnemyBulletPose(){
    return enemy_bullet_pose_;
  }

  const std::shared_ptr<CostMap> GetCostMap(){
    return costmap_ptr_;
  }

  const CostMap2D* GetCostMap2D() {
    return costmap_2d_;
  }      

void CostDetect(geometry_msgs::PoseStamped goal_pose,geometry_msgs::PoseStamped robot_map_pose) {
    auto executor_state =  chassis_executor_ptr_->Update();

    if (executor_state != BehaviorState::RUNNING) {

      auto dx = goal_pose.pose.position.x - robot_map_pose.pose.position.x;
      auto dy = goal_pose.pose.position.y - robot_map_pose.pose.position.y;
      auto yaw = std::atan2(dy, dx);

      if (std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) >= 0.05 && std::sqrt(std::pow(dx, 2) + std::pow(dy, 2)) <= 0.2) {  
        if (GetCancelFlag()) {
          chassis_executor_ptr_->Cancel();
          SetCancelFlag(false);
        }
        return;

      } else {

        auto orientation = tf::createQuaternionMsgFromYaw(yaw);
        geometry_msgs::PoseStamped reduce_goal;
        reduce_goal.pose.orientation = robot_map_pose.pose.orientation;

        reduce_goal.header.frame_id = "map";
        reduce_goal.header.stamp = ros::Time::now();
        //reduce_goal.pose.position.x = goal_pose.pose.position.x - 1.2 * cos(yaw);
        //reduce_goal.pose.position.y = goal_pose.pose.position.y - 1.2 * sin(yaw);
        reduce_goal.pose.position.x = goal_pose.pose.position.x ;
        reduce_goal.pose.position.y = goal_pose.pose.position.y ;
        auto enemy_x = reduce_goal.pose.position.x;
        auto enemy_y = reduce_goal.pose.position.y;
        reduce_goal.pose.position.z = 1;
        unsigned int goal_cell_x, goal_cell_y;

        auto get_enemy_cell =GetCostMap2D()->World2Map(enemy_x,
                                                                   enemy_y,
                                                                   goal_cell_x,
                                                                   goal_cell_y);

        if (!get_enemy_cell) {
          return;
        }

        auto robot_x = robot_map_pose.pose.position.x;
        auto robot_y = robot_map_pose.pose.position.y;
        unsigned int robot_cell_x, robot_cell_y;
        double goal_x, goal_y;
        GetCostMap2D()->World2Map(robot_x,
                                              robot_y,
                                              robot_cell_x,
                                              robot_cell_y);

        if (GetCostMap2D()->GetCost(goal_cell_x, goal_cell_y) >= 253) {

          bool find_goal = false;
          for(FastLineIterator line( goal_cell_x, goal_cell_y, robot_cell_x, robot_cell_y); line.IsValid(); line.Advance()) {

            auto point_cost = GetCostMap2D()->GetCost((unsigned int) (line.GetX()), (unsigned int) (line.GetY())); 

            if(point_cost >= 253){
              continue;

            } else {
              find_goal = true;
              GetCostMap2D()->Map2World((unsigned int) (line.GetX()),
                                                     (unsigned int) (line.GetY()),
                                                     goal_x,
                                                     goal_y);

              reduce_goal.pose.position.x = goal_x;
              reduce_goal.pose.position.y = goal_y;
              break;
            }

          }
          if (find_goal) {
            SetCancelFlag(true);
            std::cout<<"x pose:"<<reduce_goal.pose.position.x<<"y pose:"<<reduce_goal.pose.position.y<<std::endl;
            chassis_executor_ptr_->Execute(reduce_goal);
          } else {
            if (GetCancelFlag()) {
              chassis_executor_ptr_->Cancel();
              SetCancelFlag(false);
            }
            return;
          }

        } else {
          SetCancelFlag(true);
          std::cout<<"x pose:"<<reduce_goal.pose.position.x<<"y pose:"<<reduce_goal.pose.position.y<<std::endl;
          chassis_executor_ptr_->Execute(reduce_goal);
        }
      }
    }
  }



    double HurtedPerSecond() {
        auto reduce_hp_ = last_hp_ - remain_hp_;         
        auto time_diff = (ros::Time::now()-last_check_attacked_time_).toSec();
        if (time_diff > 0.5) {           
          self_blps_ = reduce_hp_ / time_diff;
          last_hp_ = remain_hp_;
          last_check_attacked_time_ = ros::Time::now();
          return self_blps_;
        } else {
            return self_blps_;
          }
      }



      geometry_msgs::PoseStamped HandleEnemyPose(){
        auto robot_map_pose = GetRobotMapPose();
        if(sentry_enemyone_lost){
          return enemy_pose2_;
        }
        if(sentry_enemytwo_lost){
          return enemy_pose1_;
        }                                 
        auto dx = robot_map_pose.pose.position.x - enemy_pose1_.pose.position.x;        
        auto dy = robot_map_pose.pose.position.y - enemy_pose1_.pose.position.y; 
        double dis_one = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
        auto dp = robot_map_pose.pose.position.x - enemy_pose2_.pose.position.x;        
        auto dq = robot_map_pose.pose.position.y - enemy_pose2_.pose.position.y; 
        double dis_two = std::sqrt(std::pow(dp, 2) + std::pow(dq, 2));
        if(dis_one<=dis_two){
          SetNearEnemyNum(1);   
          return enemy_pose1_;
        }else{
          SetNearEnemyNum(2);
          return enemy_pose2_;
        }
      }

      geometry_msgs::PoseStamped GetEnemyPose(){
        geometry_msgs::PoseStamped enemy_pose;
        enemy_pose.header.frame_id = "map";
        enemy_pose.pose.position.x = 0;
        enemy_pose.pose.position.y = 0;
        enemy_pose.pose.position.z = 0;
        tf::Quaternion quaternion = tf::createQuaternionFromRPY(0, 0, 0);
        enemy_pose.pose.orientation.x = quaternion.x();
        enemy_pose.pose.orientation.y = quaternion.y();
        enemy_pose.pose.orientation.z = quaternion.z();
        enemy_pose.pose.orientation.w = quaternion.w();
        try{
          enemy_pose = HandleEnemyPose();
        }
        catch(std::exception& e){
          ROS_WARN("handle enemy pose error : %s", e.what());
        }
        return enemy_pose;
      }


      double GetEnemyDistance(){
        if(!sentry_lost_){
          auto robot_map_pose = GetRobotMapPose();             
          auto enemy_map_pose = HandleEnemyPose();                                  
          auto dx = robot_map_pose.pose.position.x - enemy_map_pose.pose.position.x;        
          auto dy = robot_map_pose.pose.position.y - enemy_map_pose.pose.position.y;        
          double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
          return distance;
        }
        else{
          return -1;  
        }
      }


     geometry_msgs::PoseStamped LoadBootPosition(){
        int num = 0;
        if(GetRobotId()>10){         
          num = 1;
        }
        return boot_position_[num];
      }


    geometry_msgs::PoseStamped GetRobotMapPose(){
        UpdateRobotPose();
        return robot_map_pose_;
      }


     void UpdateRobotPose(){
        tf::Stamped<tf::Pose> robot_tf_pose;
        robot_tf_pose.setIdentity();
        robot_tf_pose.frame_id_ = "base_link";  
        robot_tf_pose.stamp_ = ros::Time();
        try{
            geometry_msgs::PoseStamped robot_pose;
            tf::poseStampedTFToMsg(robot_tf_pose, robot_pose);
            tf_ptr_->transformPose("map", robot_pose, robot_map_pose_);
        }
        catch(tf::LookupException& e){
            ROS_ERROR("Transform Error looking up robot pose: %s", e.what());
        }
      }

    int UpdateDeckInfo(){
        int a =0;
        float t=deck_time_[0];
        for(int i=0;i<5;i++){
          if(cls_[0]!=cls_[i]){
            float diff = fabs(deck_time_[0]-deck_time_[i]);
            if(diff < 0.5) deck_change_slow_ = false;
            else deck_change_slow_ = true;
          }
          if(t<deck_time_[i]){
            t=deck_time_[i];
            a=i;
          }
        }
        return a;
      }




/*  ——————————————————从config中加载参数————————————————*/

 void LoadParam(const std::string &proto_file_path) {
        if (!roborts_common::ReadProtoFromTextFile(proto_file_path, &decision_config)) {
            ROS_ERROR("Load param failed !");
            return ;
        }
/*  ——————————————————boot参数————————————————*/
        if(IsMaster()){
          boot_position_.resize(decision_config.master_bot().size());
          for (int i = 0; i != decision_config.master_bot().size(); i++) {
            boot_position_[i].header.frame_id = "map";
            boot_position_[i].pose.position.x = decision_config.master_bot(i).x();
            boot_position_[i].pose.position.z = decision_config.master_bot(i).z();
            boot_position_[i].pose.position.y = decision_config.master_bot(i).y();
            tf::Quaternion master_quaternion = tf::createQuaternionFromRPY(decision_config.master_bot(i).roll(),
                                                                          decision_config.master_bot(i).pitch(),
                                                                          decision_config.master_bot(i).yaw());
            boot_position_[i].pose.orientation.x = master_quaternion.x();
            boot_position_[i].pose.orientation.y = master_quaternion.y();
            boot_position_[i].pose.orientation.z = master_quaternion.z();
            boot_position_[i].pose.orientation.w = master_quaternion.w();
          }
        }else{
          boot_position_.resize(decision_config.auxe_bot().size());
          for (int i = 0; i != decision_config.auxe_bot().size(); i++) {
            boot_position_[i].header.frame_id = "map";
            boot_position_[i].pose.position.x = decision_config.auxe_bot(i).x();
            boot_position_[i].pose.position.z = decision_config.auxe_bot(i).z();
            boot_position_[i].pose.position.y = decision_config.auxe_bot(i).y();
            tf::Quaternion auxe_quaternion = tf::createQuaternionFromRPY(decision_config.auxe_bot(i).roll(),
                                                                          decision_config.auxe_bot(i).pitch(),
                                                                          decision_config.auxe_bot(i).yaw());
            boot_position_[i].pose.orientation.x = auxe_quaternion.x();
            boot_position_[i].pose.orientation.y = auxe_quaternion.y();
            boot_position_[i].pose.orientation.z = auxe_quaternion.z();
            boot_position_[i].pose.orientation.w = auxe_quaternion.w();
          }
        }
        /*  ——————————————————站桩输出位置参数————————————————*/\
        static_points_size_ = decision_config.point().size();
          static_position_.resize(static_points_size_);
          for (int i = 0; i != static_points_size_; i++) {
            static_position_[i].header.frame_id = "map";
            static_position_[i].pose.position.x = decision_config.point(i).x();
            static_position_[i].pose.position.z = decision_config.point(i).z();
            static_position_[i].pose.position.y = decision_config.point(i).y();
            tf::Quaternion master_quaternion = tf::createQuaternionFromRPY(decision_config.point(i).roll(),
                                                                          decision_config.point(i).pitch(),
                                                                          decision_config.point(i).yaw());
            static_position_[i].pose.orientation.x = master_quaternion.x();
            static_position_[i].pose.orientation.y = master_quaternion.y();
            static_position_[i].pose.orientation.z = master_quaternion.z();
            static_position_[i].pose.orientation.w = master_quaternion.w();
          }
        /*  ——————————————————search参数————————————————*/
         search_points_size_ = decision_config.search_path().size();
        search_point.resize(search_points_size_);
        for (int i = 0; i != search_points_size_; i++) {
            search_point[i].header.frame_id = "map";
            search_point[i].pose.position.x = decision_config.search_path(i).x();
            search_point[i].pose.position.y = decision_config.search_path(i).y();
            search_point[i].pose.position.z = decision_config.search_path(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.search_path(i).roll(),
                                                                    decision_config.search_path(i).pitch(),
                                                                    decision_config.search_path(i).yaw());
            search_point[i].pose.orientation.x = quaternion.x();
            search_point[i].pose.orientation.y = quaternion.y();
            search_point[i].pose.orientation.z = quaternion.z();
            search_point[i].pose.orientation.w = quaternion.w();
           //ROS_INFO("get search x:%f,search y:%f",search_point[i].pose.position.x,search_point[i].pose.position.y); 
        }
     /*  ——————————————————escape参数————————————————*/
        escape_points_.resize((unsigned int)(decision_config.escape().size()));
        for (int i = 0; i != (unsigned int)(decision_config.escape().size()); i++) {
            escape_points_[i].header.frame_id = "map";
            escape_points_[i].pose.position.x = decision_config.escape(i).x();
            escape_points_[i].pose.position.y = decision_config.escape(i).y();
            escape_points_[i].pose.position.z = decision_config.escape(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.escape(i).roll(),
                                                                    decision_config.escape(i).pitch(),
                                                                    decision_config.escape(i).yaw());
            escape_points_[i].pose.orientation.x = quaternion.x();
            escape_points_[i].pose.orientation.y = quaternion.y();
            escape_points_[i].pose.orientation.z = quaternion.z();
            escape_points_[i].pose.orientation.w = quaternion.w();
            //ROS_INFO("get escape x:%f,escape y:%f",escape_points_[i].pose.position.x,escape_points_[i].pose.position.y);
        }
     /*  ——————————————————F1-F6参数————————————————*/
        buff_debuff_points_.resize((unsigned int)(decision_config.buff_point().size()));
        for (int i = 0; i != (unsigned int)(decision_config.buff_point().size()); i++) {
            buff_debuff_points_[i].header.frame_id = "map";
            buff_debuff_points_[i].pose.position.x = decision_config.buff_point(i).x();
            buff_debuff_points_[i].pose.position.y = decision_config.buff_point(i).y();
            buff_debuff_points_[i].pose.position.z = decision_config.buff_point(i).z();

            tf::Quaternion quaternion = tf::createQuaternionFromRPY(decision_config.buff_point(i).roll(),
                                                                    decision_config.buff_point(i).pitch(),
                                                                    decision_config.buff_point(i).yaw());
            buff_debuff_points_[i].pose.orientation.x = quaternion.x();
            buff_debuff_points_[i].pose.orientation.y = quaternion.y();
            buff_debuff_points_[i].pose.orientation.z = quaternion.z();
            buff_debuff_points_[i].pose.orientation.w = quaternion.w();
            //ROS_INFO("get buff x:%f,buff y:%f",buff_debuff_points_[i].pose.position.x,buff_debuff_points_[i].pose.position.y);
        }
         int region = GetCurrentRegion();
         if(region == 1 || region == 2 ){
         tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,3.14);
          buff_debuff_points_[2].pose.orientation.x = quaternion.x();
          buff_debuff_points_[2].pose.orientation.y = quaternion.y();
          buff_debuff_points_[2].pose.orientation.z = quaternion.z();
          buff_debuff_points_[2].pose.orientation.w = quaternion.w();
          buff_debuff_points_[3].pose.orientation.x = quaternion.x();
          buff_debuff_points_[3].pose.orientation.y = quaternion.y();
          buff_debuff_points_[3].pose.orientation.z = quaternion.z();
          buff_debuff_points_[3].pose.orientation.w = quaternion.w();
         }
 }

geometry_msgs::PoseStamped GetLeastStaticPose(){
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  double distance[static_points_size_];
  for(int i = 0;i<static_points_size_;i++){
    double dx = robot_map_pose.pose.position.x - static_position_[i].pose.position.x;        
    double dy = robot_map_pose.pose.position.y - static_position_[i].pose.position.y;        
    distance[i] = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
  }
  int min_distance_flag = 0;
  double min_distance = distance[0];
  for(int i = 0 ;i<static_points_size_;i++){ if(distance[i]<min_distance) { min_distance =  distance[i]; min_distance_flag = i;}}
  return static_position_[min_distance_flag];
}
/*  ——————————————————回调函数————————————————*/
  void RobotStatusCallback(const roborts_msgs::RobotStatus::ConstPtr & robot_status){
     id_ = static_cast<int>(robot_status->id);
     if(id_ == 1 || id_ == 101){
       SetMaster(true);
     }
     else{SetMaster(false);
     }
    // remain_hp_ = static_cast<int>(robot_status->remain_hp);
     gimbal_punished_ = !(static_cast<bool>(robot_status->gimbal_enable));
     chassis_punished_ = !(static_cast<bool>(robot_status->chassis_enable));
     if (gimbal_punished_ || chassis_punished_){
       if_punished_ = true;
     }
     
  }

  int GetRemainHp(){
    return remain_hp_;
  }

  int GetRemainBullet(){
    return remain_bullet_;
  }

  void GameResultCallback(const roborts_msgs::GameResult::ConstPtr & game_result){
      game_result_ = static_cast<int>(game_result->result);
   }

   int GetGameResult()
   {
      return game_result_;
   }


  void RobotHPCallback(const roborts_msgs::GameRobotHP::ConstPtr & robot_hp){//101、102蓝方  1、2红方  1为主车
      ros::Time now_time_ = ros::Time::now();
      int robot_id_ = GetRobotId();
      red1_hp = static_cast<int>(robot_hp->red1);
      red2_hp = static_cast<int>(robot_hp->red2);
      blue1_hp = static_cast<int>(robot_hp->blue1);
      blue2_hp = static_cast<int>(robot_hp->blue2);
      if(1 == robot_id_){
        self_hp_delta_ = remain_hp_ - red1_hp;
        remain_hp_ = red1_hp;
        mate_hp_delta_ = teammate_hp_ - red2_hp;
        teammate_hp_ = red2_hp;
        enemy1_hp_delta_ = enemy1_hp_ - blue1_hp;
        enemy1_hp_ = blue1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - blue2_hp;
        enemy2_hp_ = blue2_hp;
      }
      if(2 == robot_id_){
        self_hp_delta_ = remain_hp_ - red2_hp;
        remain_hp_ = red2_hp;
        mate_hp_delta_ = teammate_hp_ - red1_hp;
        teammate_hp_ = red1_hp;
        enemy1_hp_delta_ = enemy1_hp_ - blue1_hp;
        enemy1_hp_ = blue1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - blue2_hp;
        enemy2_hp_ = blue2_hp;
      }
      if(101 == robot_id_){
        self_hp_delta_ = remain_hp_ - blue1_hp;
        remain_hp_ = blue1_hp;
        mate_hp_delta_ = teammate_hp_ - blue2_hp;
        teammate_hp_ = blue2_hp;
        enemy1_hp_delta_ = enemy1_hp_ - red1_hp;
        enemy1_hp_ = red1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - red2_hp;
        enemy2_hp_ = red2_hp;
      }
      if(102 == robot_id_){
        self_hp_delta_ = remain_hp_ - blue2_hp;
        remain_hp_ = blue2_hp;
        mate_hp_delta_ = teammate_hp_ - blue1_hp;
        teammate_hp_ = blue1_hp;
        enemy1_hp_delta_ = enemy1_hp_ - red1_hp;
        enemy1_hp_ = red1_hp;
        enemy2_hp_delta_ = enemy2_hp_ - red2_hp;
        enemy2_hp_ = red2_hp;
      }
      time_delta_ = (now_time_-last_time_).toSec();
      last_time_= now_time_;
      self_blood_loss_per_second_[hp_count]= self_hp_delta_ / time_delta_;
      mate_blood_loss_per_second_[hp_count]= mate_hp_delta_ / time_delta_;
      enemy1_blood_loss_per_second_[hp_count]= enemy1_hp_delta_ / time_delta_;
      enemy2_blood_loss_per_second_[hp_count]= enemy2_hp_delta_ / time_delta_;
      hp_count = (hp_count+1)%2;
   }

double SelfHurted3Second(){
  return self_blood_loss_per_second_[0]+self_blood_loss_per_second_[1];
}

double EnemyHurted3Second(){
  double enemy1 = enemy1_blood_loss_per_second_[0]+ enemy1_blood_loss_per_second_[1];
  double enemy2 = enemy2_blood_loss_per_second_[0]+ enemy2_blood_loss_per_second_[1];
  return (enemy1 > enemy2 ? enemy1 : enemy2);
}

bool IsBloodLossFast(){
      if(is_blood_lose_fast == true){
        if(SelfHurted3Second()<=80.0){ 
           is_blood_lose_fast = false;
             return false;
        }
        else {
         is_blood_lose_fast = true;
        return true;
      }
      }
      if(SelfHurted3Second()>=280.0 && SelfHurted3Second() > EnemyHurted3Second()){
        is_blood_lose_fast = true;
        return true;
      }
      else {
         is_blood_lose_fast = false;
        return false;
      }
}


bool IsMoreBloodThanMate(){
  return(remain_hp_>=teammate_hp_);
}

bool IsMateDie(){
  if((int)teammate_hp_ == 0) return true;
  else{ return false;}
}

 void RobotBulletCallback(const roborts_msgs::GameRobotBullet::ConstPtr & robot_bullet){
      int robot_id_ = GetRobotId();
      red1_bullet = static_cast<int>(robot_bullet->red1);
      red2_bullet = static_cast<int>(robot_bullet->red2);
      blue1_bullet = static_cast<int>(robot_bullet->blue1);
      blue2_bullet = static_cast<int>(robot_bullet->blue2);
      if(1 == robot_id_){
        remain_bullet_ = red1_bullet;
        teammate_bullet_ = red2_bullet;
        enemy1_bullet_ = blue1_bullet;
        enemy2_bullet_ = blue2_bullet;
      }
      if(2 == robot_id_){
        remain_bullet_ = red2_bullet;
        teammate_bullet_ = red1_bullet;
        enemy1_bullet_ = blue1_bullet;
        enemy2_bullet_ = blue2_bullet;
      }
      if(101 == robot_id_){
        remain_bullet_ = blue1_bullet;
        teammate_bullet_ = blue2_bullet;
        enemy1_bullet_ = red1_bullet;
        enemy2_bullet_ = red2_bullet;
      }
      if(102 == robot_id_){
        remain_bullet_ = blue2_bullet;
        teammate_bullet_ = blue1_bullet;
        enemy1_bullet_ = red1_bullet;
        enemy2_bullet_ = red2_bullet;
      }
   }

bool IsMoreBulletThanMate(){
  return(remain_bullet_>=teammate_bullet_);
}

bool IsLessBulletThanMate(){
  return(remain_bullet_<=teammate_bullet_);
}

void SetBloodActive(bool active){
  blood_active_ = active;
}

void SetEnemyBloodActive(bool active){
  enemy_blood_active_ = active;
}

bool GetBloodActive(){
  return blood_active_;
}


void SetBulletActive(bool active){
  bullet_active_ = active;
}

void SetEnemyBulletActive(bool active){
  enemy_bullet_active_ = active;
}

bool GetBulletActive(){
  return bullet_active_;
}

int GetEnemyBulletZone(){
  return enemy_bullet_zone;
}


void GameZoneArrayCallback(const roborts_msgs::GameZoneArray::ConstPtr & game_zone_array){
      int robot_id_ = GetRobotId();
      F1_zone_status = static_cast<int>(game_zone_array->zone[0].type);
      F2_zone_status = static_cast<int>(game_zone_array->zone[1].type);
      F3_zone_status = static_cast<int>(game_zone_array->zone[2].type);
      F4_zone_status = static_cast<int>(game_zone_array->zone[3].type);
      F5_zone_status = static_cast<int>(game_zone_array->zone[4].type);
      F6_zone_status = static_cast<int>(game_zone_array->zone[5].type);
      F1_zone_active = static_cast<bool>(game_zone_array->zone[0].active);
      F2_zone_active = static_cast<bool>(game_zone_array->zone[1].active);
      F3_zone_active = static_cast<bool>(game_zone_array->zone[2].active);
      F4_zone_active = static_cast<bool>(game_zone_array->zone[3].active);
      F5_zone_active = static_cast<bool>(game_zone_array->zone[4].active);
      F6_zone_active = static_cast<bool>(game_zone_array->zone[5].active);
      if(1 == robot_id_ || 2 == robot_id_){
        if(1 == F1_zone_status){SetBloodPose(GetBuffPoints(0));SetBloodActive(F1_zone_active);
        SetEnemyBloodPose(GetBuffPoints(5));SetEnemyBloodActive(F6_zone_active);}
        if(2 == F1_zone_status){SetBulletPose(GetBuffPoints(0));SetBulletActive(F1_zone_active);
        enemy_bullet_zone = 6;
        SetEnemyBulletPose(GetBuffPoints(5));SetEnemyBulletActive(F6_zone_active);}
        if(1 == F2_zone_status){SetBloodPose(GetBuffPoints(1));SetBloodActive(F2_zone_active);
        SetEnemyBloodPose(GetBuffPoints(4));SetEnemyBloodActive(F5_zone_active);}
        if(2 == F2_zone_status){SetBulletPose(GetBuffPoints(1));SetBulletActive(F2_zone_active);
        enemy_bullet_zone = 5;
        SetEnemyBulletPose(GetBuffPoints(4));SetEnemyBulletActive(F5_zone_active);}
        if(1 == F3_zone_status){SetBloodPose(GetBuffPoints(2));SetBloodActive(F3_zone_active);
        SetEnemyBloodPose(GetBuffPoints(3));SetEnemyBloodActive(F4_zone_active);}
        if(2 == F3_zone_status){SetBulletPose(GetBuffPoints(2));SetBulletActive(F3_zone_active);
        enemy_bullet_zone = 4;
        SetEnemyBulletPose(GetBuffPoints(3));SetEnemyBulletActive(F4_zone_active);}
        if(1 == F4_zone_status){SetBloodPose(GetBuffPoints(3));SetBloodActive(F4_zone_active);
        SetEnemyBloodPose(GetBuffPoints(2));SetEnemyBloodActive(F3_zone_active);}
        if(2 == F4_zone_status){SetBulletPose(GetBuffPoints(3));SetBulletActive(F4_zone_active);
        enemy_bullet_zone = 3;
        SetEnemyBulletPose(GetBuffPoints(2));SetEnemyBulletActive(F3_zone_active);}
        if(1 == F5_zone_status){SetBloodPose(GetBuffPoints(4));SetBloodActive(F5_zone_active);
        SetEnemyBloodPose(GetBuffPoints(1));SetEnemyBloodActive(F2_zone_active);}
        if(2 == F5_zone_status){SetBulletPose(GetBuffPoints(4));SetBulletActive(F5_zone_active);
        enemy_bullet_zone = 2;
        SetEnemyBulletPose(GetBuffPoints(1));SetEnemyBulletActive(F2_zone_active);}
        if(1 == F6_zone_status){SetBloodPose(GetBuffPoints(5));SetBloodActive(F6_zone_active);
        SetEnemyBloodPose(GetBuffPoints(0));SetEnemyBloodActive(F1_zone_active);}
        if(2 == F6_zone_status){SetBulletPose(GetBuffPoints(5));SetBulletActive(F6_zone_active);
        enemy_bullet_zone = 1;
        SetEnemyBulletPose(GetBuffPoints(0));SetEnemyBulletActive(F1_zone_active);}
      }
      if(101 == robot_id_ || 102 == robot_id_){
        if(3 == F1_zone_status){SetBloodPose(GetBuffPoints(0));SetBloodActive(F1_zone_active);
        SetEnemyBloodPose(GetBuffPoints(5));SetEnemyBloodActive(F6_zone_active);}
        if(4 == F1_zone_status){SetBulletPose(GetBuffPoints(0));SetBulletActive(F1_zone_active);
        enemy_bullet_zone = 6;
        SetEnemyBulletPose(GetBuffPoints(5));SetEnemyBulletActive(F6_zone_active);}
        if(3 == F2_zone_status){SetBloodPose(GetBuffPoints(1));SetBloodActive(F2_zone_active);
        SetEnemyBloodPose(GetBuffPoints(4));SetEnemyBloodActive(F5_zone_active);}
        if(4 == F2_zone_status){SetBulletPose(GetBuffPoints(1));SetBulletActive(F2_zone_active);
        enemy_bullet_zone = 5;
        SetEnemyBulletPose(GetBuffPoints(4));SetEnemyBulletActive(F5_zone_active);}
        if(3 == F3_zone_status){SetBloodPose(GetBuffPoints(2));SetBloodActive(F3_zone_active);
        SetEnemyBloodPose(GetBuffPoints(3));SetEnemyBloodActive(F4_zone_active);}
        if(4 == F3_zone_status){SetBulletPose(GetBuffPoints(2));SetBulletActive(F3_zone_active);
        enemy_bullet_zone = 4;
        SetEnemyBulletPose(GetBuffPoints(3));SetEnemyBulletActive(F4_zone_active);}
        if(3 == F4_zone_status){SetBloodPose(GetBuffPoints(3));SetBloodActive(F4_zone_active);
        SetEnemyBloodPose(GetBuffPoints(2));SetEnemyBloodActive(F3_zone_active);}
        if(4 == F4_zone_status){SetBulletPose(GetBuffPoints(3));SetBulletActive(F4_zone_active);
        enemy_bullet_zone = 3;
        SetEnemyBulletPose(GetBuffPoints(2));SetEnemyBulletActive(F3_zone_active);}
        if(3 == F5_zone_status){SetBloodPose(GetBuffPoints(4));SetBloodActive(F5_zone_active);
        SetEnemyBloodPose(GetBuffPoints(1));SetEnemyBloodActive(F2_zone_active);}
        if(4 == F5_zone_status){SetBulletPose(GetBuffPoints(4));SetBulletActive(F5_zone_active);
        enemy_bullet_zone = 2;
        SetEnemyBulletPose(GetBuffPoints(1));SetEnemyBulletActive(F2_zone_active);}
        if(3 == F6_zone_status){SetBloodPose(GetBuffPoints(5));SetBloodActive(F6_zone_active);
        SetEnemyBloodPose(GetBuffPoints(0));SetEnemyBloodActive(F1_zone_active);}
        if(4 == F6_zone_status){SetBulletPose(GetBuffPoints(5));SetBulletActive(F6_zone_active);
        enemy_bullet_zone = 1;
        SetEnemyBulletPose(GetBuffPoints(0));SetEnemyBulletActive(F1_zone_active);}
      }   
   }


geometry_msgs::PoseStamped GetStopPose(){
   int robot_id_ = GetRobotId();
  if(1 == robot_id_ ){
       stop_pose_.pose.position.x = 5.14;
       stop_pose_.pose.position.y = 1.59;
  geometry_msgs::PoseStamped enemy_pose =HandleEnemyPose();
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  double threa;
  if(enemy_pose.pose.position.x > robot_map_pose.pose.position.x) threa = 0;
  else threa = 3.14;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,threa);
          stop_pose_.pose.orientation.x = quaternion.x();
          stop_pose_.pose.orientation.y = quaternion.y();
          stop_pose_.pose.orientation.z = quaternion.z();
          stop_pose_.pose.orientation.w = quaternion.w();
  }
    if(2 == robot_id_ ){
       stop_pose_.pose.position.x = 5.14;
       stop_pose_.pose.position.y = 2.89;
  geometry_msgs::PoseStamped enemy_pose =HandleEnemyPose();
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  double threa;
  if(enemy_pose.pose.position.x > robot_map_pose.pose.position.x) threa = 0;
  else threa = 3.14;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,threa);
          stop_pose_.pose.orientation.x = quaternion.x();
          stop_pose_.pose.orientation.y = quaternion.y();
          stop_pose_.pose.orientation.z = quaternion.z();
          stop_pose_.pose.orientation.w = quaternion.w();
  }
    if(101 == robot_id_ ){
       stop_pose_.pose.position.x = 3.54;
       stop_pose_.pose.position.y = 2.89;
  geometry_msgs::PoseStamped enemy_pose =HandleEnemyPose();
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  double threa;
  if(enemy_pose.pose.position.x > robot_map_pose.pose.position.x) threa = 0;
  else threa = 3.14;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,threa);
          stop_pose_.pose.orientation.x = quaternion.x();
          stop_pose_.pose.orientation.y = quaternion.y();
          stop_pose_.pose.orientation.z = quaternion.z();
          stop_pose_.pose.orientation.w = quaternion.w();
  }
    if(102 == robot_id_ ){
       stop_pose_.pose.position.x = 3.54;
       stop_pose_.pose.position.y = 1.59;
  geometry_msgs::PoseStamped enemy_pose =HandleEnemyPose();
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  double threa;
  if(enemy_pose.pose.position.x > robot_map_pose.pose.position.x) threa = 0;
  else threa = 3.14;
      tf::Quaternion quaternion = tf::createQuaternionFromRPY(0,0,threa);
          stop_pose_.pose.orientation.x = quaternion.x();
          stop_pose_.pose.orientation.y = quaternion.y();
          stop_pose_.pose.orientation.z = quaternion.z();
          stop_pose_.pose.orientation.w = quaternion.w();
  }
  return stop_pose_;
}




   double GetBloodBuffDistance(){
    geometry_msgs::PoseStamped buff_goal = GetBloodPose();
    geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();                                  
    auto dx = robot_map_pose.pose.position.x - buff_goal.pose.position.x;        
    auto dy = robot_map_pose.pose.position.y - buff_goal.pose.position.y;        
    double distance = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
    return distance;
   }


bool IsAllBuffDistanceNear(){
    double distance[6];
    geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();    
    for(int i=0;i<5;i++){                          
    auto dx = robot_map_pose.pose.position.x - buff_debuff_points_[i].pose.position.x;        
    auto dy = robot_map_pose.pose.position.y - buff_debuff_points_[i].pose.position.y;        
    distance[i] = std::sqrt(std::pow(dx, 2) + std::pow(dy, 2));
   }
    for(int i=0;i<5;i++){                          
        if(distance[i] < 0.5){
            return true;
        }
   }
   return false;
}



 void RobotHeatCallback(const roborts_msgs::RobotHeat::ConstPtr & robot_heat){
     robot_heat_=static_cast<int>(robot_heat->shooter_heat);
 }

int GetRobotHeat()
{
   return robot_heat_;
}


void RobotShootCallback(const roborts_msgs::RobotShoot::ConstPtr &  robot_shoot){
    robot_shoot_frequency_=static_cast<int>(robot_shoot->frequency);
     robot_shoot_speed_=static_cast<double>(robot_shoot->speed);
 }

 int GetShootFrequency()
{
  return robot_shoot_frequency_;
}

double GetShootSpeed()
{
  return robot_shoot_speed_;
}

 void RobotDamageCallback(const roborts_msgs::RobotDamage::ConstPtr & robot_damage)
  {
      damage_type_ = static_cast<int>(robot_damage->damage_type);
      damage_source_ = static_cast<int>(robot_damage->damage_source);
  }

int GetDamageType(){
  return damage_type_;
}

void SetDamageType(){
  damage_type_ = -1;
}

void SetDamageSourece(){
  damage_source_ = -1;
}

int GetDamageSource(){
  return damage_source_;
}

bool IsDamaged(){
  if(0 == damage_type_) return true;
}

void GameStatusCallback(const roborts_msgs::GameStatus::ConstPtr & game_info)
  {
      game_status_ = static_cast<int>(game_info->game_status);
      remaining_time_ = static_cast<unsigned int>(game_info->remaining_time);
      if(remaining_time_ == 121 || remaining_time_ == 61){
        LoadParam(proto_file_path_);
        SetGainBulletsFlag(false);
        SetGainBloodFlag(false);
        SetHasChaseEnemy(false);
        SetHasFollowed(false);
      }
  }

 int GetGameStatus(){
        return game_status_;
}

 int GetRemainingTime(){
        return remaining_time_;
}

void CompanionCallBack(const roborts_msgs::interact::ConstPtr & interact_info){
        teammate_hp_ = static_cast<int>(interact_info->hp);
        //teammate_bullet_ = static_cast<unsigned int>(interact_info->bullets);
        teammate_pose_x_ = interact_info->robot_pose_x;
        teammate_pose_y_ = interact_info->robot_pose_y;
        teammate_goal_.pose.position.x = interact_info->shoot_goal_x;
        teammate_goal_.pose.position.y = interact_info->shoot_goal_y;
        mate_chase_enemy_ = interact_info->mate_chase_enemy;
        geometry_msgs::PoseStamped pose_;
        pose_.pose.position.x = teammate_pose_x_;
        pose_.pose.position.y = teammate_pose_y_;
        SetFollowPose(pose_);
      }

geometry_msgs::PoseStamped GetTeammateGoal(){
        return teammate_goal_;
    } 

bool GetMateHasChaseEnemy(){
  return mate_chase_enemy_;
}      


unsigned int GetTeammateHP(){
        ROS_INFO("teammate_remain_hp_: %d",teammate_hp_);
        return teammate_hp_;
}

unsigned int GetTeammateBullets(){
        ROS_INFO("teammate_remain_bullets_: %d",teammate_bullet_);
        return teammate_bullet_;
 }



void MatePubnishCallBack(const roborts_msgs::PunishInfo::ConstPtr & punish_info){
        mate_gimbal_ = punish_info->on_gimbal;
        mate_chassis_ = punish_info->on_chassis;
        if(mate_gimbal_ || mate_chassis_){
          mate_punished_ = true;
        }else{
          mate_punished_ = false;
        }
      }

bool GetMateGimbal(){
        ROS_INFO("mate_gimbal_: %d",mate_gimbal_);
        return mate_gimbal_;
 }

bool GetMateChassis(){
        ROS_INFO("mate_chassis_: %d",mate_chassis_);
        return mate_chassis_;
 }

void MateTreeStatusCallBack(const roborts_msgs::TreeStatus::ConstPtr & tree_status){
        mate_status_=static_cast<unsigned int>(tree_status->status); 
        mate_tree_running_=tree_status->is_running;
        mate_goal_=tree_status->goal; 
      }

int GetMateStatus(){
        ROS_INFO("mate_status_: %d",mate_status_);
        return mate_status_;
 }

 bool GetMateTreeRunning(){
        ROS_INFO("mate_tree_running_: %d",mate_tree_running_);
        return mate_tree_running_;
 }

 geometry_msgs::PoseStamped GetMateGoal(){
        return mate_goal_;     
      }


void CameraCallBack(const roborts_msgs::PyArmorInfo::ConstPtr & camera_info){
        if(camera_lost_ == 0){
        camera_count = (camera_count+1)%50;
        if(camera_count%50 == 0){
        camera_lost_ = (!camera_info->is_enemy);  
        }
        }
        else{
          camera_lost_ = (!camera_info->is_enemy);
        }
      }



void SentryCallBack(const roborts_msgs::sg::ConstPtr & sentry_info){
      int id_ = GetRobotId();
      if(id_ == 101 || id_ == 102){
      sentry_enemyone_lost = !sentry_info->car3;
      sentry_enemytwo_lost = !sentry_info->car4;
      sentry_lost_ = sentry_enemyone_lost & sentry_enemytwo_lost;
      if(sentry_enemyone_lost == 0){
      enemy_pose1_.pose.position.x = (sentry_info->car3_x)/100;  
      enemy_pose1_.pose.position.y = (sentry_info->car3_y)/100;
      }
      /*else{
      enemy_pose1_.pose.position.x = 0;  
      enemy_pose1_.pose.position.y = 0;
      }*/
      if(sentry_enemytwo_lost == 0){
      enemy_pose2_.pose.position.x = (sentry_info->car4_x)/100;
      enemy_pose2_.pose.position.y = (sentry_info->car4_y)/100;
      }
      /*else{
      enemy_pose2_.pose.position.x = 0;  
      enemy_pose2_.pose.position.y = 0;
      }*/
      }
      if(id_ == 1 || id_ == 2){
      sentry_enemyone_lost = !sentry_info->car1;
      sentry_enemytwo_lost = !sentry_info->car2;
      if(sentry_enemyone_lost == 0){
      enemy_pose1_.pose.position.x = (sentry_info->car1_x)/100;
      enemy_pose1_.pose.position.y = (sentry_info->car1_y)/100;
      }
      /*else{
      enemy_pose1_.pose.position.x = 0;  
      enemy_pose1_.pose.position.y = 0; 
      }*/
      if(sentry_enemytwo_lost == 0){
      enemy_pose2_.pose.position.x = (sentry_info->car2_x)/100;
      enemy_pose2_.pose.position.y = (sentry_info->car2_y)/100;
      }
      /*else{
      enemy_pose2_.pose.position.x = 0;  
      enemy_pose2_.pose.position.y = 0; 
      }*/
      }
      if(id_ != 1 && id_ != 2 && id_ != 101 && id_ != 102){
          ROS_INFO("can't get enemy pose from sentry !");      
      }
    }


bool GetEnemyOneLost(){
  return sentry_enemyone_lost;
}

bool GetEnemyTwoLost(){
  return sentry_enemytwo_lost;
}


geometry_msgs::PoseStamped GetEnemy1pose(){
  return enemy_pose1_;
}

geometry_msgs::PoseStamped GetEnemy2pose(){
  return enemy_pose2_;
}
float GetEnemyThrea(){
  geometry_msgs::PoseStamped enemy_pose;
  enemy_pose = HandleEnemyPose();
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  float threa;
      if (robot_map_pose.pose.position.x < enemy_pose.pose.position.x)
       {
         threa = atan((robot_map_pose.pose.position.y-enemy_pose.pose.position.y)/(robot_map_pose.pose.position.x-enemy_pose.pose.position.x));
         
       }
    if (robot_map_pose.pose.position.x >= enemy_pose.pose.position.x)
       {
         if (robot_map_pose.pose.position.y < enemy_pose.pose.position.y)
            {
              threa = 3.14 + atan((robot_map_pose.pose.position.y-enemy_pose.pose.position.y)/(robot_map_pose.pose.position.x-enemy_pose.pose.position.x));
            }
         if (robot_map_pose.pose.position.y >= enemy_pose.pose.position.y)
            {
              threa = atan((robot_map_pose.pose.position.y-enemy_pose.pose.position.y)/(robot_map_pose.pose.position.x-enemy_pose.pose.position.x)) - 3.14;
            }
       }
  return threa;
}

float GetRobotThrea(){
  geometry_msgs::PoseStamped robot_map_pose = GetRobotMapPose();
  tf::Quaternion cur_q;
  tf::quaternionMsgToTF(robot_map_pose.pose.orientation, cur_q);
  double r, p, y;
  tf::Matrix3x3(cur_q).getRPY(r, p, y); 
  return y;
}

void GoGoalCallback(const roborts_msgs::GoGoal::ConstPtr &  goal_pose){
     go_goal_x_=goal_pose->go_goal_x;
      go_goal_y_=goal_pose->go_goal_y;
 }

double GetGoGoalX()
{
  return go_goal_x_;
}

double GetGoGoalY()
{
  return go_goal_y_;
}




    private:
      std::shared_ptr<tf::TransformListener> tf_ptr_;        
      std::shared_ptr<ChassisExecutor> chassis_executor_ptr_;
      std::shared_ptr<GimbalExecutor>  gimbal_executor_ptr_;
      const std::string &proto_file_path_;

    std::shared_ptr<CostMap> costmap_ptr_;
    CostMap2D* costmap_2d_;
    unsigned char* charmap_;
    unsigned int search_count_;
    unsigned int search_points_size_;
    unsigned int static_points_size_;
    int escape_region_;
    bool camera_lost_;        
    double camera_z_; 
    double camera_x_;
    double camera_y_;   
    std::vector<int> cls_;          //敌人装甲板方位（1前;2侧;3后）
    std::vector<float> deck_time_;    
    int cls_num_;      
    double c_angle_;            //相机与装甲板之间的角度
    bool is_scan_;                     // 是否进行旋转扫描
    bool deck_change_slow_;            //判断敌人装甲板变化是否小于一定频率
    bool sentry_lost_;       
    unsigned int near_enemy_num_;         
    bool is_master_;   
    roborts_decision::DecisionConfig decision_config;
    bool is_search_;         
    bool is_go_goal_;
    bool is_go_buff_;
    bool  is_follow_;
    BehaviorState action_state_;        
      bool cancel_flag_;                            // 是否需要采取取消行为
      bool mate_punished_;       
      double go_goal_x_;
      double go_goal_y_;
      bool is_swing_;
      bool if_punished_;
      bool gimbal_punished_;      
      bool chassis_punished_;     
      ros::Time last_check_attacked_time_;    
      ros::Time last_time_;
      unsigned int last_hp_;  
      double self_blps_;  
     // bool is_mate_search_enemy_;
      bool has_followed_;
      bool gain_bullets_flag;
      bool gain_blood_flag;
      bool has_chase_enemy_;     
      bool is_blood_lose_fast;

      /*——————————————————与队友交互的参数————————————*/
     int teammate_hp_;   
     int teammate_bullet_;   
    bool mate_gimbal_;        
    bool mate_chassis_;      
    unsigned int mate_status_;    
    bool mate_tree_running_;     

    geometry_msgs::PoseStamped teammate_goal_;
    double teammate_pose_x_;
    double teammate_pose_y_;
    bool mate_chase_enemy_;


      /*——————————————————从裁判系统获得的参数————————————*/
       int id_;  
       int camera_count;
       int remain_bullet_;  
       int remain_hp_;            
       int game_result_; 
       bool red3_survival_;
       bool  red4_survival_;
       bool  blue3_survival_;
       bool blue4_survival_;
       int red_bonus_status_;
       int blue_bonus_status_;
       int  supplier_status_;
       int  robot_heat_;
       bool robot_bonus_;
       int robot_shoot_frequency_;
       double robot_shoot_speed_;
      int damage_type_;       
      int damage_source_;     
    unsigned int game_status_;        
    unsigned int remaining_time_;     

    int enemy_bullet_zone;
     int red1_hp;  
     int red2_hp; 
     int blue1_hp;
     int blue2_hp;  
     int red1_bullet; 
     int red2_bullet;
     int blue1_bullet;
     int blue2_bullet;
    unsigned int F1_zone_status;
    unsigned int F2_zone_status;
    unsigned int F3_zone_status;
    unsigned int F4_zone_status;
    unsigned int F5_zone_status;
    unsigned int F6_zone_status;
    bool F1_zone_active;
    bool F2_zone_active;
    bool F3_zone_active;
    bool F4_zone_active;
    bool F5_zone_active;
    bool F6_zone_active;
     int enemy1_bullet_;
     int enemy2_bullet_;
     int enemy1_hp_;
     int enemy2_hp_;
    bool blood_active_;
    bool bullet_active_;
    bool enemy_blood_active_;
    bool enemy_bullet_active_;

    double  self_hp_delta_;
    double  mate_hp_delta_;
    double  enemy1_hp_delta_;
    double  enemy2_hp_delta_;
    double time_delta_;
    double self_blood_loss_per_second_[2];
    double mate_blood_loss_per_second_[2];
    double enemy1_blood_loss_per_second_[2];
    double enemy2_blood_loss_per_second_[2];
    int hp_count;

      /*——————————————————敌人交互的信息————————————*/ 


      /*——————————————————与哨岗交互的参数————————————*/
      bool sentry_enemyone_lost;
      bool sentry_enemytwo_lost;
      geometry_msgs::PoseStamped enemy_pose1_;    
      geometry_msgs::PoseStamped enemy_pose2_;  



      /*——————————————————关于位置的变量————————————*/
      std::vector<geometry_msgs::PoseStamped> boot_position_;  
      std::vector<geometry_msgs::PoseStamped> static_position_;  
      std::vector<geometry_msgs::PoseStamped> search_point;
      std::vector<geometry_msgs::PoseStamped> escape_points_;  
      std::vector<geometry_msgs::PoseStamped> buff_debuff_points_;    
      geometry_msgs::PoseStamped buff_pose_;    
      geometry_msgs::PoseStamped follow_pose_;    
      geometry_msgs::PoseStamped robot_map_pose_;     
      std::vector<geometry_msgs::PoseStamped> chase_goal_;   
      geometry_msgs::PoseStamped mate_goal_;  
      geometry_msgs::PoseStamped enemy_pose_;  
      geometry_msgs::PoseStamped robort_goal_; 
      geometry_msgs::PoseStamped blood_pose_;
      geometry_msgs::PoseStamped enemy_blood_pose_;
      geometry_msgs::PoseStamped enemy_bullet_pose_;
      geometry_msgs::PoseStamped bullet_pose_; 
      geometry_msgs::PoseStamped stop_pose_; 


      /*——————————————————订阅者的定义——————————————*/
     // refreee
      ros::Subscriber game_result_sub_;
      ros::Subscriber bonus_status_sub_;
      ros::Subscriber supplier_status_sub_;
      ros::Subscriber robot_heat_sub_;
      ros::Subscriber robot_bonus_sub_;
      ros::Subscriber robot_shoot_sub_;
      ros::Subscriber game_status_sub_;
      ros::Subscriber robot_damage_sub_;


      ros::Subscriber robot_hp_sub_;
      ros::Subscriber robot_bullet_sub_;
      ros::Subscriber game_zone_array_status_sub_;
      ros::Subscriber ros_robot_status_sub_;
      //camera
      ros::Subscriber camera_sub;

      ros::Subscriber gogoal_sub_;

      //mate interact
      ros::Subscriber mate_info_sub;
      ros::Subscriber mate_punish_sub;
      ros::Subscriber mate_treestatus_sub;
      ros::Subscriber companion_info_sub;


      //sentry 
      ros::Subscriber enemy_pose_sub;
      ros::Subscriber sentry_sub;
    };
    
  

}
#endif 
