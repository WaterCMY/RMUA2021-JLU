#ifndef BUFF_INFOMATION
#define BUFF_INFOMATION
#include "ros/ros.h"
#include "std_msgs/String.h"
#include"std_msgs/Int8.h"
#include <geometry_msgs/PoseStamped.h>
#include "roborts_msgs/GameZoneArray.h"
#include "roborts_msgs/RobotStatus.h"
namespace debuffpose{
    class DebuffPose{
        public:
        int a;
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


        private:



    }



    
}
#endif
