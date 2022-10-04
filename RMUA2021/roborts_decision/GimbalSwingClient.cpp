#include <unistd.h>      
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include "blackboard/blackboard.h"
#include "goal_factory.h"
#include "roborts_msgs/GimbalSwingAction.h"


typedef actionlib::SimpleActionClient<roborts_msgs::GimbalSwingAction> Client;

// 当action完成后会调用该回调函数一次
void doneCallback(const actionlib::SimpleClientGoalState & state,const roborts_msgs::GimbalSwingResultConstPtr & result)
{
	ROS_INFO("gimbal_swing finished");
	//ros::shutdown();
    }

// 当action激活后会调用该回调函数一次
void activeCallback()
{
	ROS_INFO("Goal just went active");
}

// 收到feedback后调用该回调函数
void feedbackCallback(const roborts_msgs::GimbalSwingFeedbackConstPtr & feedback)
{
	ROS_INFO("[feedback] gimbal is swinging or not: %d", feedback->is_swing);
}


int main(int argc, char ** argv)
{
	ros::init(argc, argv, "gimbal_swing_client");
	
	// 定义一个客户端
	Client client("gimbal_swing_", true);
	
	// 等待服务器端
	ROS_INFO("waiting for action server to start.");
	client.waitForServer();
	ROS_INFO("Action server started, sending goal.");
	
	//创建一个action的goal
	roborts_msgs::GimbalSwingGoal goal;
	goal.rate = 30;
	
	// 发送aciton的goal给服务器端
	client.sendGoal(goal, &doneCallback, &activeCallback, &feedbackCallback);

    ros::spin();

	return 0;
}
