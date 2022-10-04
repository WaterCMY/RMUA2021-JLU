#include "ros/ros.h"
#include "zcx/path.h"
#include "zcx2.h"

int main(int argc,char **argv)
{
    ros::init(argc,argv,"talker");
	ros::NodeHandle n;
	ros::Publisher zcx_pub = n.advertise<zcx::path>("mode",1000);
	ros::Rate loop_rate(10);

	int count=0;
	while(ros::ok())
	{
		zcx::path msg;
		msg.mode =2;
		msg.x =4;
		msg.y =5;

		zcx_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
	return 0;
}