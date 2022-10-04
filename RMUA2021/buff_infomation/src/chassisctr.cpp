#include"chassisctr.h"
namespace chassisctr{
    ChassisCtr::ChassisCtr():n(0),x(0),ct(0),
    as_(nh_,"chassisdefend",boost::bind(&ChassisCtr::GoalCallback,this,_1),false){
        cmd_vel_acc_pub_ = nh_.advertise<roborts_msgs::TwistAccel>("/cmd_vel_acc", 1);
        //cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        as_.start();
    }

    void ChassisCtr::GoalCallback(const roborts_msgs::ChassisCtrGoal::ConstPtr &goal){
        ros::Rate r(720);
        while(ros::ok()){
            if(goal->model==1){
                cmd_vel_.twist.linear.x = 0;
                cmd_vel_.twist.linear.y = 0;
                cmd_vel_.twist.angular.z = 3*sin(x);
                cmd_vel_.accel.linear.x = 0;
                cmd_vel_.accel.linear.y = 0;
                cmd_vel_.accel.angular.z = 0;
            }
            else if(goal->model==2){
                cmd_vel_.twist.linear.x = 0;
                cmd_vel_.twist.linear.y = 0;
                cmd_vel_.twist.angular.z = 2*PI;
                cmd_vel_.accel.linear.x = 0;
                cmd_vel_.accel.linear.y = 0;
                cmd_vel_.accel.angular.z = 0;
            }
            ct+=0.5;
            x=(ct/180*PI);
            if(ct>360){ct = 0;}
            cmd_vel_acc_pub_.publish(cmd_vel_);
            if (as_.isPreemptRequested()){
            as_.setPreempted();
            break;
            }
            r.sleep();
        }
    }

ChassisCtr::~ChassisCtr() {}
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "chassisdefend");
  chassisctr::ChassisCtr chassis_ctr;
  ros::spin();
  return 0;
}