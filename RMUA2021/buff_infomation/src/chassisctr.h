#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "roborts_msgs/ChassisCtrAction.h"
#include "roborts_msgs/TwistAccel.h"
#define PI 3.1415927
namespace chassisctr{
    class ChassisCtr{
        private:

        public:
        typedef actionlib::SimpleActionServer<roborts_msgs::ChassisCtrAction> ChassisServer;
        ros::NodeHandle nh_;
        ChassisServer as_;
        ros::Publisher cmd_vel_pub_;
        ros::Publisher cmd_vel_acc_pub_;
        roborts_msgs::TwistAccel cmd_vel_;
        int n;
        float x;
        float ct;
        ChassisCtr();
        ~ChassisCtr();
        void GoalCallback(const roborts_msgs::ChassisCtrGoal::ConstPtr &goal);
    };
}