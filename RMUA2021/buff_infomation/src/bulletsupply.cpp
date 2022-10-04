#include "ros/ros.h"
#include "std_msgs/String.h"
#include"std_msgs/Int8.h"
#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/RobotStatus.h"

		int a;
		int roborts_id_;
		int b;
		geometry_msgs::PoseStamped msg2;
		unsigned int F1_zone_status;
        unsigned int F2_zone_status;
        unsigned int F3_zone_status;
        unsigned int F4_zone_status;
        unsigned int F5_zone_status;
        unsigned int F6_zone_status;
        unsigned int F1_zone_active;
        unsigned int F2_zone_active;
        unsigned int F3_zone_active;
        unsigned int F4_zone_active;
        unsigned int F5_zone_active;
        unsigned int F6_zone_active;

		void GameZoneArray3Callback(const roborts_msgs::GameZoneArray::ConstPtr & game_zone_array)
{
            F1_zone_status = static_cast<int>(game_zone_array->zone[0].type);
            F2_zone_status = static_cast<int>(game_zone_array->zone[1].type);
            F3_zone_status = static_cast<int>(game_zone_array->zone[2].type);
            F4_zone_status = static_cast<int>(game_zone_array->zone[3].type);
            F5_zone_status = static_cast<int>(game_zone_array->zone[4].type);
            F6_zone_status = static_cast<int>(game_zone_array->zone[5].type);
            F1_zone_active = static_cast<int>(game_zone_array->zone[0].active);
            F2_zone_active = static_cast<int>(game_zone_array->zone[1].active);
            F3_zone_active = static_cast<int>(game_zone_array->zone[2].active);
            F4_zone_active = static_cast<int>(game_zone_array->zone[3].active);
            F5_zone_active = static_cast<int>(game_zone_array->zone[4].active);
			F6_zone_active = static_cast<int>(game_zone_array->zone[5].active);

			ros::param::get("robort_id",roborts_id_);
			if((roborts_id_==1)||(roborts_id_==2)){b=1;}
			if((roborts_id_==101)||(roborts_id_==102)){b=2;}
			switch(b){
				case 1 :
				if(F1_zone_status == 4 && F1_zone_active == true){
					a = 1;
					}
				else if(F2_zone_status == 4 && F2_zone_active == true){
					a = 2;}
				else if(F3_zone_status == 4 && F3_zone_active == true){
					a = 3;
					}
				else if(F4_zone_status == 4 && F4_zone_active == true){
					a = 4;
					}
				else if(F5_zone_status == 4 && F5_zone_active == true){
					a = 5;
					}
				else if(F6_zone_status == 4 && F6_zone_active == true){
					a = 6;
					}
				else {
					a = 7;
					}
				break;

				case 2 :
				if(F1_zone_status == 2 && F1_zone_active == true){
					a = 1;
					}
				else if(F2_zone_status == 2 && F2_zone_active == true){
					a = 2;}
				else if(F3_zone_status == 2 && F3_zone_active == true){
					a = 3;
					}
				else if(F4_zone_status == 2 && F4_zone_active == true){
					a = 4;
					}
				else if(F5_zone_status == 2 && F5_zone_active == true){
					a = 5;
					}
				else if(F6_zone_status == 2 && F6_zone_active == true){
					a = 6;
					}
				else {
					a = 7;
					}
					break;
			}

		
}

	
	int main(int argc, char **argv){
    ros::Time::init();
     ros::init(argc, argv, "BULLET_SUPPLY");	
	 ros::NodeHandle n;	
	 ros::Subscriber game_zone_array_status_sub_;
     game_zone_array_status_sub_ = n.subscribe<roborts_msgs::GameZoneArray>("game_zone_array_status", 30,GameZoneArray3Callback);
		ros::Publisher pub = n.advertise<geometry_msgs::PoseStamped>("bullet_supply_pose", 10);	
		ros::Rate loop_rate(10);
		while(ros::ok()){
                       switch (a)
			{
			case 1:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 0.40;
				msg2.pose.position.y = 2.89;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			case 2:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 1.90;
				msg2.pose.position.y = 1.55;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			case 3:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 4.04;
				msg2.pose.position.y = 4.04;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			case 4:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 4.04;
				msg2.pose.position.y = 0.45;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			case 5:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 6.18;
				msg2.pose.position.y = 2.93;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			case 6:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 7.68;
				msg2.pose.position.y = 1.60;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			case 7:
				msg2.header.seq = 0;
				msg2.header.stamp =ros::Time::now();
				msg2.header.frame_id = "map";
				msg2.pose.position.x = 0;
				msg2.pose.position.y = 0;
				msg2.pose.position.z = 0.0;
				msg2.pose.orientation.x = 0.0;
				msg2.pose.orientation.y = 0.0;
				msg2.pose.orientation.w = 0;
				break;
			}
			ros::spinOnce();
        	pub.publish(msg2);
			ROS_INFO("set bullet supply pose successed");	
			loop_rate.sleep();
			}
			return 0;
			}

