#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "action_node/GoGoal.h"
#include "action_node/SearchAction.h"
#include "action_node/BackBootArea.h"
#include "action_node/DefendAction.h"
#include "action_node/SwingDefend.h"
#include "action_node/FrozenAction.h"
#include "action_node/Follow.h"
#include "action_node/Escape.h"
#include "action_node/ChaseAction.h"
#include "action_node/TurnDefend.h"
#include "action_node/GoStaticPose.h"


void Command();
char command = '0';

int main(int argc, char **argv) {
  ros::init(argc, argv, "test_node");
      ros::Time::init();
std::string file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
auto black_ptr = std::make_shared<roborts_decision::Blackboard>(file_path);
auto g_factory = std::make_shared<roborts_decision::GoalFactory>(black_ptr);



 auto go_goal=std::make_shared<roborts_decision::GoGoal>(black_ptr,g_factory);    
 auto search=std::make_shared<roborts_decision::SearchAction>(black_ptr,g_factory);    
 auto back_boot_area=std::make_shared<roborts_decision::BackBootArea>(black_ptr,g_factory);    
 auto defend=std::make_shared<roborts_decision::DefendAction>(black_ptr,g_factory);    
 auto swingdefend=std::make_shared<roborts_decision::SwingDefend>(black_ptr,g_factory);   
 auto follow=std::make_shared<roborts_decision::Follow>(black_ptr,g_factory);   
auto frozen=std::make_shared<roborts_decision::FrozeAction>(black_ptr,g_factory);   
auto escape=std::make_shared<roborts_decision::Escape>(black_ptr,g_factory);   
auto chase=std::make_shared<roborts_decision::ChaseAction>(black_ptr,g_factory);
auto turn_defend=std::make_shared<roborts_decision::TurnDefend>(black_ptr,g_factory);
auto go_static_action=std::make_shared<roborts_decision::GoStaticPose >(black_ptr,g_factory);      



auto game_status_selector=std::make_shared<roborts_decision::SelectorNode>("game_status_selector",black_ptr);


roborts_decision::BehaviorTree root(game_status_selector, 100);
 //command=argv[1][0];
  auto command_thread= std::thread(Command);
  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    switch (command) {
      case '1':
  {  
    game_status_selector->AddChildren(go_goal);
        root.Run();
        break;
  }
      case '2':
      {
    game_status_selector->AddChildren(search);
         root.Run();
        break;
      }
      case '3':
      {
    game_status_selector->AddChildren(back_boot_area);
       root.Run();
        break;
      }
     case '4':
      {
geometry_msgs::PoseStamped pose;
 pose = black_ptr->GetRobotMapPose();
  std::cout <<"x:"<< pose.pose.position.x << std::endl;
  std::cout <<"y:"<< pose.pose.position.y<< std::endl;
  std::cout <<"x:"<< pose.pose.orientation.x << std::endl;
  std::cout <<"y:"<< pose.pose.orientation.y<< std::endl;
   std::cout <<"z:"<< pose.pose.orientation.z << std::endl;
  std::cout <<"w:"<< pose.pose.orientation.w<< std::endl;
        break;
      }
      case '5':
      {
     game_status_selector->AddChildren(defend);
           root.Run();
        break;
       }
        case '6':
game_status_selector->AddChildren(swingdefend);
           root.Run();
        break;
         case '7':

        break;
            case '8':
game_status_selector->AddChildren(follow);
           root.Run();
        break;
         case '9':
game_status_selector->AddChildren(chase);
           root.Run();
        break;   
          case 'a':
game_status_selector->AddChildren(escape);
           root.Run(); 
        break;    
       case 'b':
game_status_selector->AddChildren(turn_defend);
           root.Run(); 
        break;  
            case 'c':
game_status_selector->AddChildren(go_static_action);
           root.Run(); 
        break;  
        /*    case 27:
                if (command_thread.joinable()){
                    command_thread.join();
                }
                return 0;*/
            default:
                break;
    }
    rate.sleep();
  }

  return 0;
}

void Command() {

  while (command != 27) {
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "*********************************please send a command********************************" << std::endl;
    std::cout << "1: gogoal" << std::endl
              << "2: search" << std::endl
              << "3: back  boot  area" << std::endl
              << "4: get pose" << std::endl
              << "5: rolldefend" << std::endl
              << "6: swingdefend" << std::endl
              << "7: .." << std::endl
              << "8: follow" << std::endl
              << "9: chase" << std::endl 
              << "a: escape" << std::endl 
              << "b: turndefend" << std::endl        
              << "c: staticpose" << std::endl        
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != '1' && command != '2' && command != '3' && command != '4' && command != '5' && command != '6' && command != '7' 
        &&command != '8' &&command != '9' &&command != 'a'  &&command != 'b'  && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}
