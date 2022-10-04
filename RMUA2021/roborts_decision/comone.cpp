#include "interact/correspond.h"
#include "blackboard/blackboard.h"
int main(int argc, char **argv){
    ros::init(argc, argv, "comone_node");
    ros::Time::init();

    //define date interact  ptr
    std::string config_file_path=ros::package::getPath("roborts_decision")+ "/config/decision.prototxt";
    auto blackboard_ptr_ = std::make_shared<roborts_decision::Blackboard>(config_file_path);
 blackboard_ptr_->SetMaster(1);
     auto correspond_ptr_ = std::make_shared<roborts_decision::CorrespondBoard>(blackboard_ptr_);  
//服务端
        correspond_ptr_->recive();
        correspond_ptr_->send();
        correspond_ptr_->publish();

}