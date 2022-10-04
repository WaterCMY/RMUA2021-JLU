#include <ros/ros.h>

#include "executor/chassis_executor.h"
#include "behavior_tree/behavior_tree.h"
#include "action_node/GoGoal.h"
#include "action_node/SearchAction.h"
#include "action_node/BackBootArea.h"
#include "action_node/DefendAction.h"
#include "action_node/SwingDefend.h"
#include "zcx/pathn.h"
//../../devel/include/
void Command();
bool ServiceBack(zcx::pathn::Request  &req , zcx::pathn::Response &res);
int command = 0;
int mode_number_;
double x_;
double y_;


int main(int argc, char **argv) {
  ros::init(argc, argv, "vd_node");
      ros::Time::init();
std::string file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
auto black_ptr = std::make_shared<roborts_decision::Blackboard>(file_path);
auto g_factory = std::make_shared<roborts_decision::GoalFactory>(black_ptr);


 auto go_goal=std::make_shared<roborts_decision::GoGoal>(black_ptr,g_factory);    
 auto search=std::make_shared<roborts_decision::SearchAction>(black_ptr,g_factory);    
 auto back_boot_area=std::make_shared<roborts_decision::BackBootArea>(black_ptr,g_factory);    
 auto defend=std::make_shared<roborts_decision::DefendAction>(black_ptr,g_factory);    
 auto swingdefend=std::make_shared<roborts_decision::SwingDefend>(black_ptr,g_factory);   

auto game_status_selector=std::make_shared<roborts_decision::SelectorNode>("game_status_selector",black_ptr);

roborts_decision::BehaviorTree root(game_status_selector, 100);

 ros::NodeHandle n;
 ros::ServiceServer service = n.advertiseService("mode", ServiceBack);
 //command=argv[1][0];
  //auto command_thread= std::thread(Command);
  command = mode_number_;
  ros::Rate rate(10);
  while(ros::ok()){
    ros::spinOnce();
    command = mode_number_;
    switch (command) {
      case 1:
  {  
    game_status_selector->AddChildren(go_goal);
        root.Run();
        break;
  }
      case 2:
      {
    game_status_selector->AddChildren(search);
         root.Run();
        break;
      }
      case 3:
      {
    game_status_selector->AddChildren(back_boot_area);
       root.Run();
        break;
      }
     case 4:
      {
geometry_msgs::PoseStamped pose;
 pose = black_ptr->GetRobotMapPose();
  std::cout <<"x:"<< pose.pose.position.x << std::endl;
  std::cout <<"y:"<< pose.pose.position.y<< std::endl;
        break;
      }
      case 5:
      {
     game_status_selector->AddChildren(defend);
           root.Run();
        break;
       }
        case 6:
game_status_selector->AddChildren(swingdefend);
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


  
bool ServiceBack(zcx::pathn::Request  &req , zcx::pathn::Response &res)
{
  res.result = 666;
    std::cout <<"mode:"<< req.mode << std::endl;
   std::cout <<"x:"<< req.x << std::endl;
  std::cout <<"y:"<< req.y<< std::endl;
  x_ = req.x;
  y_ = req.y;
mode_number_ = req.mode;
  std::cout <<"result:"<< res.result<< std::endl;
  return true;
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
              << "esc: exit program" << std::endl;
    std::cout << "**************************************************************************************" << std::endl;
    std::cout << "> ";
    std::cin >> command;
    if (command != 1 && command != 2 && command != 3 && command != 4 && command != 5 && command != 6 && command != 27) {
      std::cout << "please input again!" << std::endl;
      std::cout << "> ";
      std::cin >> command;
    }

  }
}
