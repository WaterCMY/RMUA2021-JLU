  #include <unistd.h>
  #include <stdio.h>
  #include <iostream>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <sys/types.h>
  #include <arpa/inet.h>
  #include<ros/ros.h>
  //#include<roborts_msgs/sg.h>
  #include<roborts_msgs/sg.h>

 //收
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

         setvbuf(stdout,NULL,_IONBF,0);
         fflush(stdout);
	


         struct sockaddr_in addrto;
         bzero(&addrto,sizeof(struct sockaddr_in));
         addrto.sin_addr.s_addr=htonl(INADDR_ANY);
         addrto.sin_family=AF_INET;
         addrto.sin_port=htons(6000);

         struct sockaddr_in from;
         bzero(&from,sizeof(struct sockaddr_in));
         from.sin_addr.s_addr=htonl(INADDR_ANY);
         from.sin_family=AF_INET;
         from.sin_port=htons(6000);

         int sock = -1;
         if((sock = socket(AF_INET,SOCK_DGRAM,0))==-1)
         {
                 cout<<"socket error"<<endl;
         }

	struct timeval timeout;
	timeout.tv_sec = 5;
	timeout.tv_usec =0;

	if(setsockopt(sock,SOL_SOCKET,SO_RCVTIMEO,&timeout,sizeof(timeout))==-1){
		cout<<"setsockpot failed!"<<endl;
	}


         const int opt =-1;
         int nb=0;
         nb=setsockopt(sock,SOL_SOCKET,SO_BROADCAST,(char*)&opt,sizeof(opt));

         if(nb==1)
         {
                 cout<<"set socket error"<<endl;
         }

         if(bind(sock,(struct sockaddr*)&(addrto),sizeof(struct sockaddr_in))==-1)
         {
                 cout<<"bind error"<<endl;
         }

         int len =sizeof(sockaddr_in);

     

        ros::init(argc, argv, "sgdata");
        ros::NodeHandle n;

	ros::Publisher sg2_pub = n.advertise<roborts_msgs::sg>("sg",1000);
        ros::Rate loop_rate(10);

	roborts_msgs::sg msg;



     while(ros::ok()){
     // 接收客户端的数据，使用recvfrom() 函数接收客户端的网络数据；

     int ret;
     ret = recvfrom(sock, (char*)&sgdata, sizeof(sgdata), 0, (struct sockaddr *) &from, (socklen_t*)&len);
     if(sgdata.car1==false&&sgdata.car2==false&&sgdata.car3==false&&sgdata.car4==false){
     ROS_INFO("time out");
     }

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
    if(msg.car1){ 
	    ROS_INFO("car1_x = %f ",sgdata.car1_x);
	    ROS_INFO("car1_y = %f",sgdata.car1_y);

    }
    if(msg.car2){
	     ROS_INFO("car2_x = %f ",sgdata.car2_x);
	      ROS_INFO("car2_y = %f",sgdata.car2_y);

    }
    if(msg.car3){
	     ROS_INFO("car3_x = %f",sgdata.car3_x);
             ROS_INFO("car3_y = %f",sgdata.car3_y);

    }
    if(msg.car4){
	    ROS_INFO("car4_x = %f",sgdata.car4_x);
	    ROS_INFO("car4_y = %f",sgdata.car4_y);

    }
    sg2_pub.publish(msg);


        msg.car1=false;
    msg.car2=false;
    msg.car3=false;
    msg.car4=false;
    msg.car1_x=0.0;
    msg.car1_y=0.0;
    msg.car2_x=0.0;
    msg.car2_y=0.0;
    msg.car3_x=0.0;
    msg.car3_y=0.0;
    msg.car4_x=0.0;
    msg.car4_y=0.0;
    sgdata.car1=false;
    sgdata.car2=false;
    sgdata.car3=false;
    sgdata.car4=false;
    sgdata.car1_x=0.0;
    sgdata.car1_y=0.0;
    sgdata.car2_x=0.0;
    sgdata.car2_y=0.0;
    sgdata.car3_x=0.0;
    sgdata.car3_y=0.0;
    sgdata.car4_x=0.0;
    sgdata.car4_y=0.0;
    ros::spinOnce();
    loop_rate.sleep();
 }
     
     //关闭套接字，使用close() 函数释放资源；
     close(sock);
 
     return 0;
 }
     
