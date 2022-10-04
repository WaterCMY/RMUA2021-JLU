#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include "roborts_msgs/GimbalAngle.h"
#include "roborts_msgs/GimbalCtrAction.h"
#include "roborts_msgs/PyArmorInfo.h"
#define PI 3.1415927
namespace gimbalctr{
    class GimbalCtr{
        private:

        public:
        typedef actionlib::SimpleActionServer<roborts_msgs::GimbalCtrAction> GimbalServer;
        ros::NodeHandle nh_;
        GimbalServer as_;
        ros::Publisher cmd_gim_pub_;
        roborts_msgs::GimbalAngle cmd_gim_;
        int n;
        double x;
        float ct;
        float pitch;
        bool flag;
        GimbalCtr();
        ~GimbalCtr();
        void GoalCallback(const roborts_msgs::GimbalCtrGoal::ConstPtr &goal);
    };
}