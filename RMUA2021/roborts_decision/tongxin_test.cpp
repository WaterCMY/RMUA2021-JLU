#include "interact/mutualboard.h"

#include "behavior_tree/behavior_tree.h"




int main(int argc, char **argv){
    ros::init(argc, argv, "tongxin_test_node");
    ros::Time::init();

    std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
    //auto goal_factory_ = std::make_shared<roborts_decision::GoalFactory>(blackboard_ptr_);
    auto mutualboard_ptr_ = std::make_shared<roborts_decision::MutualBoard>(blackboard_ptr_);    

    ros::Rate rate(30);

    while(ros::ok()){
        ROS_INFO("start");
        mutualboard_ptr_->ExchangeData();
        ROS_INFO("end");
        //blackboard_ptr_->PublishGimcontrol();
        ros::spinOnce();
        rate.sleep();
    }

}