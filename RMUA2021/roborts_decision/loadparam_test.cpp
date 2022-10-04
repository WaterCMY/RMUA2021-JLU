#include<string.h>
#include "ros/ros.h"
#include "action_node/threadtest.h"


/*class hello{
public:
	hello() { }
	void world1(){
        for(int i =0; i<5; i++){
		std::cout << "Hello world" << std::endl;
        }
    }
	void world2(std::string text){
		std::cout << "Hello world, " << text << std::endl;
	}
};*/

int main(int argc, char **argv){ 
    //auto test_ptr = std::make_shared<roborts_decision::threadtest>;    
    //roborts_decision::threadtest thread_;
    //std::thread t1(&thread_::world1, &thread_);
    //std::thread t1(test_ptr->world1());


 
	//hello h;
	//std::thread t1(&hello::world1, &h);//必须使用&，表明是同一个对象
	//std::thread t2(&hello::world2, &h, "lee");
	

    ros::init(argc, argv, "loadparam_test_node"); 
    ros::NodeHandle paramtest_nh;
    std::string paramtest_a;

    int paramtest_b=55;
    paramtest_nh.getParam("paramtest_a", paramtest_a);
    paramtest_nh.getParam("paramtest_b", paramtest_b);

    //ros::param::get("~paramtest_a",paramtest_a);
    //ros::param::get("~paramtest_b",paramtest_b);
    ROS_INFO("paramtest_a:%s",paramtest_a.c_str());
    ROS_INFO("paramtest_b:%d",paramtest_b);

    //if (t1.joinable()) t1.join();
	//if (t2.joinable()) t2.join();

    return 0;
}
