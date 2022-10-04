#include"gimbalctr.h"
namespace gimbalctr{
    GimbalCtr::GimbalCtr():n(0),x(0),ct(0),pitch(0),flag(true),
    as_(nh_,"gimbalswing",boost::bind(&GimbalCtr::GoalCallback,this,_1),false){
        cmd_gim_pub_ = nh_.advertise<roborts_msgs::GimbalAngle>("/cmd_gimbal_angle", 1);
        as_.start();
        ROS_INFO("action start");
    }

    void GimbalCtr::GoalCallback(const roborts_msgs::GimbalCtrGoal::ConstPtr &goal){
        ros::Rate r(40);
        x=(goal->angle)/180*3.14;
        //if(goal->angle>0){flag = true;}
        //if(goal->angle<=0){flag = false;}
        while(ros::ok()){
            ROS_INFO("start callback");           
            if(flag){
                x+=0.04;
                cmd_gim_.yaw_mode = false;
                cmd_gim_.pitch_mode = false;
                cmd_gim_.yaw_angle = x;
                cmd_gim_.pitch_angle = pitch;
            }
            if(!flag){
                x-=0.04;
                cmd_gim_.yaw_mode = false;
                cmd_gim_.pitch_mode = false;
                cmd_gim_.yaw_angle = x;
                cmd_gim_.pitch_angle = pitch;
            }
            if(x >=1.2){pitch=0.02;flag = false;}
            if(x <= -1.2){pitch=0.24;flag = true;}     
            cmd_gim_pub_.publish(cmd_gim_);
            if (as_.isPreemptRequested()){
            as_.setPreempted();
            ROS_INFO("stop swing");
            break;
            }
            r.sleep();
        }
    }

GimbalCtr::~GimbalCtr() {}
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "gimbalswing");
  gimbalctr::GimbalCtr gimbal_ctr;
  ros::spin();
  return 0;
}
