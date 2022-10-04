#include "blackboard/blackboard.h"
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include <roborts_msgs/sg.h>

int main(int argc, char **argv){
    ros::init(argc, argv, "sentry_node");
    ros::Time::init();
    std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
    ros::Rate loop_rate(1);
    double x,y;
    std::cout<<"shurux"<<std::endl;
    std::cin>>x;
    std::cout<<"shuruy"<<std::endl;
    std::cin>>y;
    while (ros::ok())
    {
             geometry_msgs::PoseStamped enemy = blackboard_ptr_-> GetRobotMapPose();
        unsigned int robot_cell_x, robot_cell_y;
            blackboard_ptr_->GetCostMap2D()->World2Map(x,
                                              y,
                                              robot_cell_x,
                                              robot_cell_y);
        double w =  blackboard_ptr_->GetCostMap2D()->GetCost(robot_cell_x, robot_cell_y);
         std::cout<<w<<"w"<<std::endl;
/*   double x =blackboard_ptr_-> SelfHurted3Second();
    double y =blackboard_ptr_->EnemyHurted3Second();
    bool z =blackboard_ptr_->IsBloodLossFast();
    std::cout<<x<<"x"<<std::endl;
    std::cout<<y<<"y"<<std::endl;
    std::cout<<z<<"z"<<std::endl;*/

    /*bool one_lost_  =  blackboard_ptr_->GetEnemyOneLost();
    bool two_lost_  =  blackboard_ptr_->GetEnemyTwoLost();
    geometry_msgs::PoseStamped enemy = blackboard_ptr_-> HandleEnemyPose();
    geometry_msgs::PoseStamped enemy1 = blackboard_ptr_-> GetEnemy1pose();
    geometry_msgs::PoseStamped enemy2 = blackboard_ptr_-> GetEnemy2pose();
    std::cout<<enemy1.pose.position.x<<std::endl;
    std::cout<<enemy1.pose.position.y<<std::endl;
    std::cout<<enemy2.pose.position.x<<std::endl;
    std::cout<<enemy2.pose.position.y<<std::endl;
    std::cout<<enemy.pose.position.x<<"wwwww"<<std::endl;
    std::cout<<enemy.pose.position.y<<"wwwww"<<std::endl;
    std::cout<<blackboard_ptr_->GetCameraLost()<<"lost"<<std::endl;
    std::cout<<one_lost_<<std::endl;
    std::cout<<two_lost_<<std::endl;*/
    ros::spinOnce();
    loop_rate.sleep();
    }
}
