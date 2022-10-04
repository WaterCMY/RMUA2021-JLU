#include <unistd.h>
#include <string.h>
#include <stdio.h>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <arpa/inet.h>
#include "ros/ros.h"
#include "roborts_msgs/sg.h"
  
#define IP_FOUND "IP_FOUND"
#define IP_FOUND_ACK "IP_FOUND_ACK"
#define MAXLINE 100
#define MCAST_ADDR "224.0.0.87"
 
 using namespace std;
 
 struct SgData 
{
    bool car1;
    bool car2;
    bool car3;
    bool car4;
    float car1_x;
    float car1_y;
    float car2_x;
    float car2_y;
    float car3_x;
    float car3_y;
    float car4_x;
    float car4_y;
}sgdata;

 int main(int argc,char **argv){
    ros::init(argc, argv, "sgdata");
    ros::NodeHandle n;
     int confd,ret;
     unsigned int addr_length;
     char recvline[MAXLINE];
     char sendline[MAXLINE];
     struct sockaddr_in serveraddr,our_addr,recvaddr;
    int so_broadcast=1;

    socklen_t  socklen;

    ros::Publisher sg2_pub = n.advertise<roborts_msgs::sg>("sg",1000);
    ros::Rate loop_rate(10);

     // 使用socket()，生成套接字文件描述符；
     if( (confd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
         perror("socket() error");
         exit(1);
     }
    
     //通过struct sockaddr_in 结构设置服务器地址和监听端口；
     bzero(&serveraddr, sizeof(serveraddr));
     serveraddr.sin_family = AF_INET;
     serveraddr.sin_addr.s_addr = inet_addr(MCAST_ADDR);
     serveraddr.sin_port = htons(6665);
     addr_length = sizeof(serveraddr);

     //客户端绑定通信端口，否则系统自动分配
        memset(&our_addr,0,sizeof(our_addr));
        our_addr.sin_family = AF_INET;
        our_addr.sin_port = htons(7777);
        our_addr.sin_addr.s_addr = htonl(INADDR_ANY); //MCAST_ADDR

        ret = bind(confd, (struct sockaddr *)&our_addr, sizeof(our_addr) );
        if(ret == -1)
        {
                perror("bind !");
        }

  //socklen = sizeof(struct sockaddr);
    //    strncpy(send_buf,IP_FOUND,strlen(IP_FOUND)+1);

     int send_length = 0;
     sprintf(sendline,"hello server!");
     send_length = sendto(confd, sendline, sizeof(sendline), 0, (struct sockaddr *) &serveraddr, addr_length);
    if(send_length < 0 ){
         perror("sendto() error");
         exit(1);
     }
     //cout << "send_length = " << send_length << endl;
     

    while(ros::ok())
    {
        roborts_msgs::sg msg;
     // 向服务器发送数据，sendto() ；
    
    
     
    //cout<<"1"<<endl;
     // 接收服务器的数据，recvfrom() ；
     int recv_length = 0;
     recv_length = recvfrom(confd, (char*)&sgdata, sizeof(sgdata), 0, (struct sockaddr *) &serveraddr, &addr_length);
     //cout<<"2"<<endl;
    msg.car1=sgdata.car1;
    msg.car2=sgdata.car2;
    msg.car3=sgdata.car3;
    msg.car4=sgdata.car4;
    msg.car1_x=sgdata.car1_x;
    msg.car1_y=sgdata.car1_y;
    msg.car2_x=sgdata.car2_x;
    msg.car2_y=sgdata.car2_y;
    msg.car3_x=sgdata.car3_x;
    msg.car3_y=sgdata.car3_y;
    msg.car4_x=sgdata.car4_x;
    msg.car4_y=sgdata.car4_y;
    ROS_INFO("start reception！");
   cout<<3<<endl;
  if(msg.car1){
    cout << "car1_x= "<< msg.car1_x<<endl;
    cout << "car1_y = "<< msg.car1_y<<endl;
  }
  if(msg.car2){
    cout << "car2_x = "<< msg.car2_x<<endl;
    cout << "car2_y = "<< msg.car2_y<<endl;
    }
    if(msg.car3){
    cout << "car3_x = "<< msg.car3_x<<endl;
    cout << "car3_y = "<< msg.car3_y<<endl;
    }
    if(msg.car4){
    cout << "car4_x = "<< msg.car4_x<<endl;
    cout << "car4_y = "<< msg.car4_y<<endl;
    }
    //cout<<3<<endl;
    sg2_pub.publish(msg);
    //cout<<"4"<<endl;

    
    
   // cout<<"5"<<endl;
    //loop_rate.sleep();
    ros::spinOnce();
   // cout<<"6"<<endl;

    }

     // 关闭套接字，close() ；
     close(confd);
    
     return 0;
 }
