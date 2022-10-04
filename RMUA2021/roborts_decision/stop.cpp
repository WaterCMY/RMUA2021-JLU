#include "behavior_tree/behavior_tree.h"

//include  Action Header File
#include "action_node/BackBootArea.h"
#include "action_node/ChaseAction.h"
#include "action_node/ChassisLimited.h"
#include "action_node/DefendAction.h"
#include "action_node/Escape.h"
#include "action_node/Follow.h"
#include "action_node/FrozenAction.h"
#include "action_node/TurnDefend.h"
#include "action_node/GainBullets.h"
#include "action_node/GainBlood.h"
#include "action_node/GimbalLimited.h"
#include "action_node/SearchAction.h"
#include "action_node/ShootAction.h"
#include "action_node/GoStaticPose.h"

//include  mate interact Header File
#include "interact/correspond.h"


int main(int argc, char **argv){
    ros::init(argc, argv, "decision_node");
    ros::Time::init();
    //define date interact  ptr
    std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
    auto goal_factory_ = std::make_shared<roborts_decision::GoalFactory>(blackboard_ptr_);
    auto correspond_ptr_ = std::make_shared<roborts_decision::CorrespondBoard>(blackboard_ptr_);

// define action  node ptr
    auto back_boot_area_=std::make_shared<roborts_decision::BackBootArea>(blackboard_ptr_,goal_factory_);       
    auto search_action_=std::make_shared<roborts_decision::SearchAction>(blackboard_ptr_,goal_factory_);         
    auto defend_action_=std::make_shared<roborts_decision::DefendAction>(blackboard_ptr_,goal_factory_);
    auto turn_defend_action_=std::make_shared<roborts_decision::TurnDefend>(blackboard_ptr_,goal_factory_);       
    //auto go_buff_action_=std::make_shared<roborts_decision::GoBuff>(blackboard_ptr_,goal_factory_);   
    auto follow_action_=std::make_shared<roborts_decision::Follow>(blackboard_ptr_,goal_factory_);   
    auto frozen_action_=std::make_shared<roborts_decision::FrozeAction>(blackboard_ptr_,goal_factory_); 
    auto chase_action_=std::make_shared<roborts_decision::ChaseAction>(blackboard_ptr_,goal_factory_); 
    auto shoot_action_=std::make_shared<roborts_decision::ShootAction>(blackboard_ptr_,goal_factory_); 
    auto gimbal_limited_action_ = std::make_shared<roborts_decision::GimbalLimited>(blackboard_ptr_, goal_factory_); 
    auto chassis_limited_action_=std::make_shared<roborts_decision::ChassisLimited >(blackboard_ptr_,goal_factory_);              
    auto escape_action_=std::make_shared<roborts_decision::Escape >(blackboard_ptr_,goal_factory_); 
    auto gain_bullets_action_ = std::make_shared<roborts_decision::GainBullets>(blackboard_ptr_, goal_factory_); 
    auto gain_blood_action_=std::make_shared<roborts_decision::GainBlood >(blackboard_ptr_,goal_factory_);      
    auto go_static_action=std::make_shared<roborts_decision::GoStaticPose >(blackboard_ptr_,goal_factory_);      


//the first  SelectorNode
    auto game_status_selector_=std::make_shared<roborts_decision::SelectorNode>("game_status_selector",blackboard_ptr_);



    auto game_not_start_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game_not_start_condition", blackboard_ptr_,
        [&]() {
            if ((blackboard_ptr_->GetGameStatus()!=4) && (blackboard_ptr_->GetGameStatus()!=5)){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);
    game_not_start_condition_->SetChild(frozen_action_);

       auto game_over_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game_over_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->GetGameStatus()==5){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    game_over_condition_->SetChild(back_boot_area_);
    
//the second SelectorNode
    auto game_start_selector_=std::make_shared<roborts_decision::SelectorNode>("game_start_selector",blackboard_ptr_);
    
        auto game_start_condition_ = std::make_shared<roborts_decision::PreconditionNode>("game_start_condition", blackboard_ptr_,
        [&]() {
            if (blackboard_ptr_->GetGameStatus()==4){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

    game_start_condition_->SetChild(game_start_selector_);

    game_status_selector_->AddChildren(game_not_start_condition_);
    game_status_selector_->AddChildren(game_over_condition_);
    game_status_selector_->AddChildren(game_start_condition_); 

//if under punish precondition node
    auto under_punish_condition_ = std::make_shared<roborts_decision::PreconditionNode>("under_punish_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IfPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);

//the third SelectorNode
    auto under_punish_selector_=std::make_shared<roborts_decision::SelectorNode>("under_punish_selector",blackboard_ptr_);


    under_punish_condition_->SetChild(under_punish_selector_);

    auto gimbal_pubnished_condition_ = std::make_shared<roborts_decision::PreconditionNode>("gimbal_pubnished_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->GimbalPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    gimbal_pubnished_condition_->SetChild(escape_action_);  


    auto chassis_pubnished_condition_ = std::make_shared<roborts_decision::PreconditionNode>("chassis_pubnished_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->ChassisPunished()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    chassis_pubnished_condition_->SetChild(shoot_action_);  

    under_punish_selector_->AddChildren(gimbal_pubnished_condition_);
    under_punish_selector_->AddChildren(chassis_pubnished_condition_);

//if buff fresh precondition node
    auto near_buff_refresh_time_condition_= std::make_shared<roborts_decision::PreconditionNode>("near_buff_refresh_time_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsNearRefresh()){  
               
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

//the fourth SelectorNode
    auto buff_fresh_selector_=std::make_shared<roborts_decision::SelectorNode>("buff_fresh_selector",blackboard_ptr_);


    near_buff_refresh_time_condition_->SetChild(buff_fresh_selector_);


    auto gain_blood_condition_ = std::make_shared<roborts_decision::PreconditionNode>("gain_blood_condition", blackboard_ptr_,
        [&]() {
            if (!blackboard_ptr_->IsBloodAdvantage() && blackboard_ptr_->GetBloodActive()){  
//&& (blackboard_ptr_->IsMoreBloodThanMate() || blackboard_ptr_->IsMateDie())){  
                //&&   (blackboard_ptr_->IsMoreBloodThanMate() || blackboard_ptr_->IsMateDie())
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    gain_blood_condition_->SetChild(gain_blood_action_);  



    auto gain_bullet_condition_ = std::make_shared<roborts_decision::PreconditionNode>("gain_bullet_condition", blackboard_ptr_,
        [&]() {
 		    if (!blackboard_ptr_->IsBulletAdvantage() && blackboard_ptr_->GetBulletActive()){  
//&& (blackboard_ptr_->IsMoreBulletThanMate() || blackboard_ptr_->IsMateDie())){  
                 //&&  (blackboard_ptr_->IsMoreBulletThanMate() || blackboard_ptr_->IsMateDie())
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    gain_bullet_condition_->SetChild(gain_bullets_action_); 

    buff_fresh_selector_->AddChildren(gain_blood_condition_);
    buff_fresh_selector_->AddChildren(gain_bullet_condition_);

    auto is_near_buff_condition_ = std::make_shared<roborts_decision::PreconditionNode>("is_near_buff_condition_", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsAllBuffDistanceNear() && blackboard_ptr_->IsNearRefreshTime()){       
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    is_near_buff_condition_->SetChild(search_action_); 

    auto blood_loss_fast_condition_= std::make_shared<roborts_decision::PreconditionNode>("blood_loss_fast_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsBloodLossFast()){  
        // if (blackboard_ptr_->GetDamageType() == 0 && blackboard_ptr_->GetDamageSource() == 2){      
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    blood_loss_fast_condition_->SetChild(defend_action_); 

    
    auto blood_endangered_condition_= std::make_shared<roborts_decision::PreconditionNode>("blood_endangered_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsBloodEndangered() && !blackboard_ptr_->GetBloodActive() && blackboard_ptr_->GetDamageType() == 0){  
             //&&(blackboard_ptr_->GetRemainingTime()>=59)){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    blood_endangered_condition_->SetChild(escape_action_); 



    auto bullets_endangered_condition_= std::make_shared<roborts_decision::PreconditionNode>("bullets_endangered_condition", blackboard_ptr_,
        [&]() {
 		    if (blackboard_ptr_->IsBulletsEndangered() && !blackboard_ptr_->GetBulletActive()){  
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH);  

    bullets_endangered_condition_->SetChild(defend_action_);     

    auto enemy_not_lost_condition_= std::make_shared<roborts_decision::PreconditionNode>("enemy_not_lost_condition_", blackboard_ptr_,
        [&]() {
            if (!blackboard_ptr_->GetCameraLost()){ 
                return true;
            } else {
                return false;
                }
        },
        roborts_decision::AbortType::BOTH); 
        enemy_not_lost_condition_->SetChild(frozen_action_); 

    game_start_selector_->AddChildren(under_punish_condition_);
    game_start_selector_->AddChildren(near_buff_refresh_time_condition_);
    game_start_selector_->AddChildren(is_near_buff_condition_);
    //game_start_selector_->AddChildren(blood_loss_fast_condition_);
    game_start_selector_->AddChildren(blood_endangered_condition_);
    game_start_selector_->AddChildren(bullets_endangered_condition_);
    game_start_selector_->AddChildren(enemy_not_lost_condition_);
    game_start_selector_->AddChildren(go_static_action);


//..................................................................


  ros::Rate rate(30);
    roborts_decision::BehaviorTree root_(game_status_selector_, 100);
    while(ros::ok()){
        root_.Run();
        if(!blackboard_ptr_->IsMaster()){ 
        correspond_ptr_->send();
        correspond_ptr_->recive();
        correspond_ptr_->publish();
        }
        else{
        correspond_ptr_->recive();
        correspond_ptr_->send();
        correspond_ptr_->publish();
        }
        ros::spinOnce();
        rate.sleep();
    }
}
