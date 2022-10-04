#ifndef CORRESPOND_H_
#define CORRESPOND_H_


#include "ros/ros.h"
#include "../blackboard/blackboard.h"
 #include <stdio.h>
#include <memory>
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <arpa/inet.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <iostream>
#include "roborts_msgs/interact.h"

#define MAXLINE 4096
 #define UDPPORT 8001
 #define SERVERIP "192.168.1.120"


 namespace roborts_decision{
class CorrespondBoard{
        protected:
            Blackboard::Ptr blackboard_ptr_;

         public:
            CorrespondBoard(Blackboard::Ptr& blackboard):
            blackboard_ptr_(blackboard),
            send_length(0),
            recv_length(0)
             {
            UDPSocketInit();
            ros::NodeHandle n;   
            mate_info = n.advertise<roborts_msgs::interact>("mate_info",30);
             }

        ~CorrespondBoard(){
             if(!blackboard_ptr_->IsMaster()){ 
            close(confd);
             }
             else{
            close(serverfd);
        }
        }

        void UDPSocketInit(){
          if(!blackboard_ptr_->IsMaster()){           
            if( (confd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
             perror("socket() error");
             exit(1);
             }
            bzero(&serveraddr, sizeof(serveraddr));
            serveraddr.sin_family = AF_INET;
            serveraddr.sin_addr.s_addr = inet_addr(SERVERIP);
            serveraddr.sin_port = htons(UDPPORT);
            addr_length = sizeof(serveraddr);
          }

          else{
            if( (serverfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
            perror("socket() error");
            exit(1);
            }
            bzero(&serveraddr,sizeof(serveraddr));
            serveraddr.sin_family = AF_INET;
            serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
             serveraddr.sin_port = htons(UDPPORT);
            server_addr_length = sizeof(serveraddr);

            if( bind(serverfd, (struct sockaddr *) &serveraddr, server_addr_length) < 0){
                perror("bind() error");
                exit(1); 
                }
          }           
        }


void send(){
    if(!blackboard_ptr_->IsMaster()){//if 1 is client
    //sprintf(sendline,"hello server!");
     //send_length = sendto(confd, sendline, sizeof(sendline), 0, (struct sockaddr *) &serveraddr, addr_length);
     getselfinfo();
    send_length = sendto(confd, (char*)&interact1, sizeof(interact1), 0, (struct sockaddr *) &serveraddr, addr_length);
     if(send_length < 0 ){
         perror("sendto() error");
         exit(1);
     }
    }
    else{
     //sprintf(sendline, "hello client !");
    getselfinfo();
     send_length = sendto(serverfd,  (char*)&interact2, sizeof(interact2), 0, (struct sockaddr *) &clientaddr, client_addr_length);
    // send_length = sendto(serverfd, sendline, sizeof(sendline), 0, (struct sockaddr *) &clientaddr, client_addr_length);
     if( send_length < 0){
         perror("sendto() error");
         exit(1);
     }
     //std::cout << "send_length = "<< send_length <<std::endl;
    }
}

void recive(){
    if(!blackboard_ptr_->IsMaster()){       
    //recv_length = recvfrom(confd, recvline, sizeof(recvline), 0, (struct sockaddr *) &serveraddr, &addr_length);
    recv_length = recvfrom(confd, (char*)&interact1, sizeof(interact1), 0, (struct sockaddr *) &serveraddr, &addr_length);
     std::cout << "recv_length = " << recv_length <<std::endl;
     //std::cout << recvline << std::endl; 
    }
    else{
    client_addr_length = sizeof(sockaddr_in);
    recv_length = recvfrom(serverfd, (char*)&interact2, sizeof(interact2), 0,  (struct sockaddr *) &clientaddr, &client_addr_length);
     //recv_length = recvfrom(serverfd, recvline, sizeof(recvline), 0, (struct sockaddr *) &clientaddr, &client_addr_length);
    std:: cout << "recv_length = "<< recv_length <<std::endl;
     //std::cout << recvline << std::endl;
    }
}

void getselfinfo(){
    if(!blackboard_ptr_->IsMaster()){
    shoot_goal_  = blackboard_ptr_->GetRobotGoal();
    robot_pose_  = blackboard_ptr_->GetRobotMapPose();
    interact2.robot_pose_x_ =  robot_pose_.pose.position.x;
    interact2.robot_pose_y_ =  robot_pose_.pose.position.y;
    interact2.shoot_goal_x_ =  shoot_goal_.pose.position.x;
    interact2.shoot_goal_y_ =  shoot_goal_.pose.position.y;
    interact2.hp_ = blackboard_ptr_->GetRemainHp();
    interact2.bullets_ = blackboard_ptr_->GetRemainBullet();
    interact2.mate_chase_enemy_ =  blackboard_ptr_->GetHasChaseEnemy();
    }
    else{
    shoot_goal_  = blackboard_ptr_->GetRobotGoal();
    robot_pose_  = blackboard_ptr_->GetRobotMapPose();
    interact1.robot_pose_x_ =  robot_pose_.pose.position.x;
    interact1.robot_pose_y_ =  robot_pose_.pose.position.y;
    interact1.shoot_goal_x_ =  shoot_goal_.pose.position.x;
    interact1.shoot_goal_y_ =  shoot_goal_.pose.position.y;
    interact1.hp_ = blackboard_ptr_->GetRemainHp();
    interact1.bullets_ = blackboard_ptr_->GetRemainBullet();
    interact1.mate_chase_enemy_ =  blackboard_ptr_->GetHasChaseEnemy();
    }
} 


void publish(){
    if(!blackboard_ptr_->IsMaster()){
    msg1.hp=interact1.hp_;
    msg1.bullets=interact1.bullets_;
    msg1.robot_pose_x=interact1.robot_pose_x_;
    msg1.robot_pose_y=interact1.robot_pose_y_;
    msg1.shoot_goal_x=interact1.shoot_goal_x_;
    msg1.shoot_goal_y=interact1.shoot_goal_y_;
    msg1.mate_chase_enemy=interact1.mate_chase_enemy_;
    mate_info.publish(msg1);
    }
    else{
    msg2.hp=interact2.hp_;
    msg2.bullets=interact2.bullets_;
    msg2.robot_pose_x=interact2.robot_pose_x_;
    msg2.robot_pose_y=interact2.robot_pose_y_;
    msg2.shoot_goal_x=interact2.shoot_goal_x_;
    msg2.shoot_goal_y=interact2.shoot_goal_y_;
    msg1.mate_chase_enemy=interact1.mate_chase_enemy_;
    mate_info.publish(msg2);
    }
}

 private:
        int confd;
        int serverfd;
        unsigned int addr_length;
        unsigned int server_addr_length, client_addr_length;
        char recvline[MAXLINE];
        char sendline[MAXLINE];
        int send_length;
        int recv_length;
        struct sockaddr_in serveraddr , clientaddr;
        struct InteractDate
        {
            int hp_;
            int bullets_;
            double  robot_pose_x_;
            double  robot_pose_y_;
            double  shoot_goal_x_;
            double  shoot_goal_y_;
            bool mate_chase_enemy_;
        }interact1,interact2;

        geometry_msgs::PoseStamped shoot_goal_;
        geometry_msgs::PoseStamped robot_pose_;
        roborts_msgs::interact msg1,msg2;

        ros::Publisher mate_info;

    };
 }


#endif
