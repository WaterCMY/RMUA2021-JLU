#include<sys/select.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <string>
#include <cstring>
#include <algorithm>
#include <sstream>
#include "ros/ros.h"
#include <zcx/path.h>
#include "stdlib.h"
#include "stdio.h"


using namespace std;

class data
{
	public:

		struct protogroup pretreat();

		int send();//

		struct receivedata receive();//

		int come();//

		int edit();

		int editpart();
	private:
		
		struct proto;

		struct receivedata;
		
		struct protogroup;

};
//接受数据
struct receivedata
{
	bool link_state;//检测是否收到
	bool decision_flag;//检测是否启动决策节点
	bool path_flag;//检测是否需要修改路径
	bool vision_flag;//检测是否需要路径数据
	bool map_flag;//检测地图数据是否需要修改
	int decison_node;
	double decision_x;
	double decision_y;
    double goal_angle_tolerance;
    double goal_search_tolerance;//123
    double max_vel_x;
    double max_vel_x_backwards;
    double max_vel_y;
    double max_vel_theta;
    double acc_lim_x;
    double acc_lim_y;
    double acc_lim_theta;
    double min_obstacle_dist;
    double robot_vertices_x;
    double robot_vertices_y;
};

//进行处理的结构体
struct proto
{
	string name;
	int number;
	string config;
	string fileaddr;
	string filename;

};

struct decision_node{
	int node_number;
	double x;
	double y;
};


struct protogroup
{
	struct decision_node dec;
	struct proto goal_angle_tolerance;	//1
    struct proto goal_search_tolerance;	//2
    struct proto max_vel_x;				//3
    struct proto max_vel_x_backwards;	//4
    struct proto max_vel_y;				//5
    struct proto max_vel_theta;			//6
    struct proto acc_lim_x;				//7
    struct proto acc_lim_y;				//8
    struct proto acc_lim_theta; 		//9
    struct proto min_obstacle_dist;		//10
  //  struct proto robot_vertices_x;		//11
  //  struct proto robot_vertices_y;		//12
};


void num2string(double num, string &str)
{
	stringstream ss;
	ss << num;
	str = ss.str();
}

//用于终端能够输出的函数
int Test(string cmd)
{
	
   char line[300];
   FILE *fp;
   string cmdPort = cmd;
    // 系统调用
   const char *sysCommand = cmdPort.data();
    //如果没有打开端口
   if ((fp = popen(sysCommand, "r")) == NULL)
    {
	   cout << "error" << endl;
      return 1;
    }
   //如果端口号打开了，
   while (fgets(line, sizeof(line)-1, fp) != NULL)
    {
    	cout << line ;
    }
 
    pclose(fp);

	return 0;
}



//回到编辑的地方。不同的车上需要修改进入的路径。
int come()
{
	string cmd;
	cmd = " cd /home/kid/djiroborts_ws/src/RoboRTS/"; //换到小车上需要修改。
	Test(cmd);
	return 0;	
	
}
//将发来的结构体处理成可以我们使用的结构体。
struct protogroup pretreat(receivedata a)
{
	struct protogroup b;

	b.dec.node_number=a.decison_node;
	b.dec.x=a.decision_x;
	b.dec.y=a.decision_y;

//	num2string(a.decison_node,b.dec.node_number);

	num2string(a.goal_angle_tolerance,b.goal_angle_tolerance.config);
//	b.goal_angle_tolerance.config=a.goal_angle_tolerance;
	b.goal_angle_tolerance.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/global_planner/config";
	b.goal_angle_tolerance.filename="global_planner_config.prototxt";
	b.goal_angle_tolerance.name="goal_angle_tolerance";
	b.goal_angle_tolerance.number=1;

	num2string(a.goal_search_tolerance,b.goal_search_tolerance.config);
//  b.goal_search_tolerance.config=a.goal_search_tolerance;
	b.goal_search_tolerance.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/global_planner/config";
	b.goal_search_tolerance.filename="global_planner_config.prototxt";
	b.goal_search_tolerance.name="goal_angle_tolerance";
	b.goal_search_tolerance.number=2;

	num2string(a.max_vel_x,b.max_vel_x.config);
//	b.max_vel_x.config=a.max_vel_x;
	b.max_vel_x.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_x.filename="timed_elastic_band.prototxt";
	b.max_vel_x.name="max_vel_x";
	b.max_vel_x.number=3;

	num2string(a.max_vel_x_backwards,b.max_vel_x_backwards.config);
//	b.max_vel_x_backwards.config=a.max_vel_x_backwards;
	b.max_vel_x_backwards.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_x_backwards.filename="timed_elastic_band.prototxt";
	b.max_vel_x_backwards.name="max_vel_x_backwards";
	b.max_vel_x_backwards.number=4;

	num2string(a.max_vel_y,b.max_vel_y.config);
//	b.max_vel_y.config=a.max_vel_y;
	b.max_vel_y.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_y.filename="timed_elastic_band.prototxt";
	b.max_vel_y.name="max_vel_y";
	b.max_vel_y.number=5;

	num2string(a.max_vel_theta,b.max_vel_theta.config);
//	b.max_vel_theta.config=a.max_vel_theta;
	b.max_vel_theta.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_theta.filename="timed_elastic_band.prototxt";
	b.max_vel_theta.name="max_vel_theta";
	b.max_vel_theta.number=6;

	num2string(a.acc_lim_x,b.acc_lim_x.config);
//	b.acc_lim_x.config= a.acc_lim_x;
	b.acc_lim_x.fileaddr = "/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.acc_lim_x.filename="timed_elastic_band.prototxt";
	b.acc_lim_x.name="acc_lim_x";
	b.acc_lim_x.number=6;

	num2string(a.acc_lim_y,b.acc_lim_y.config);
//	b.acc_lim_y.config=a.acc_lim_y;
	b.acc_lim_y.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.acc_lim_y.filename="timed_elastic_band.prototxt";
	b.acc_lim_y.name="acc_lim_y";
	b.acc_lim_y.number=7;

	num2string(a.acc_lim_theta,b.acc_lim_theta.config);
//	b.acc_lim_theta.config=a.acc_lim_theta;
	b.acc_lim_theta.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.acc_lim_theta.filename="timed_elastic_band.prototxt";
	b.acc_lim_theta.name="acc_lim_theta";
	b.acc_lim_theta.number=8;

	num2string(a.min_obstacle_dist,b.min_obstacle_dist.config);
//	b.min_obstacle_dist.config=a.min_obstacle_dist;
	b.min_obstacle_dist.fileaddr="/home/kid/djiroborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.min_obstacle_dist.filename="timed_elastic_band.prototxt";
	b.min_obstacle_dist.name="min_obstacle_dist";
	b.min_obstacle_dist.number=9;

	return b;
	
}

//发送一些反馈。
bool send(const char* a)
{
	int port_out = 8888;
	int sockfd;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(-1 == sockfd)
	{
		return false;
	}

	struct sockaddr_in addr;
	int addr_len=sizeof(addr);

	memset(&addr,0,sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port_out);
	addr.sin_addr.s_addr = inet_addr("192.168.1.113");// 添加qt的ip地址。  

	sockaddr_in client;


	if (bind(sockfd, (struct sockaddr*)&addr, addr_len) == -1){
//        printf("Failed to bind socket on port %d\n", port_in);
        close(sockfd);
        return false;
    }

	  sendto(sockfd, a, sizeof(a), 0, (sockaddr*)&client, sizeof(client));

	close(sockfd);
	return true;

}
//接收结构体。
struct receivedata receive()
{
	//int port_in = 12321;//接收端口号。
	int port_out = 15555;
	int sockfd;
	
	struct receivedata a;

	a.link_state =false;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(-1 == sockfd)
	{
		cout<<"socket wrong"<<endl;
		//return false;//出错后如何维护出错和输出报错信息现在很麻烦。
	}

	struct sockaddr_in addr;
    socklen_t addr_len=sizeof(addr);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;       // Use IPV4
    addr.sin_port   = htons(port_out);    //
    addr.sin_addr.s_addr = htonl(INADDR_ANY);


    // Bind 端口，用来接受之前设定的地址与端口发来的信息,作为接受一方必须bind端口，并且端口号与发送方一致
    if (bind(sockfd, (struct sockaddr*)&addr, addr_len) == -1){
     //   printf("Failed to bind socket on port %d\n", port_out);
        close(sockfd);
    cout<<"bind wrong"<<endl;
	return a;
    }


    //int counter = 0;
    do
	{
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        memset(&src, 0, sizeof(src));
		
		cout<<"recvfrom"<<endl;
		
		int sz = recvfrom(sockfd, (char*)&a, sizeof(a), 0, (sockaddr*)&src, &src_len);
        //int sz = recvfrom(sockfd, buffer, 128, 0, (sockaddr*)&src, &src_len);
        if (sz > 0){
            cout<<a.decison_node<<endl;
        }
		cout<<"recvfromed"<<endl;
		//counter = sz;
		/*
        else{
            puts("timeout");
		*/	
	}while(!a.link_state);
    
	
 	close(sockfd);
	return a;
}

//用于修改.ptototxt里的参数，.axml里的参数待会根据里的函数直接改。
int edit(struct proto a)
{
	string cmd,cmd1,cmd2,cmd3,cmd4,cmd5,cmd6;
//	char name[100];
//	char proto[100];
	
	cmd1 ="sed -i 's/";
//	cmd2 =s1;
	cmd2 = a.name;
	cmd3 =a.config;
	cmd4 ="/g' ";
	cmd5 =a.fileaddr;
	cmd6 =a.filename;
	cmd = cmd1+cmd2+": .*/"+cmd2+" : "+cmd3+cmd4+cmd5+"/"+cmd6;
	cout<<cmd<<endl;

	Test(cmd);

	return 0;
	
}


int editpart(struct protogroup a)
{
	//cout<<123<<endl;
	edit(a.acc_lim_theta);
	edit(a.acc_lim_x);
	edit(a.acc_lim_y);
	edit(a.goal_angle_tolerance);
	edit(a.goal_search_tolerance);
	edit(a.max_vel_theta);
	edit(a.max_vel_x);
	edit(a.max_vel_x_backwards);
	edit(a.max_vel_y);
	edit(a.min_obstacle_dist);

	return 1;
}
/*
int decision(int argc ,char **argv)
{
	ros::init(argc,argv,"talker");
	ros::NodeHandle n;
	ros::Publisher zcx_pub n.advertise<zcx::path>("mode",1000);
	ros::Rate loop_rate(10);

	int count=0;
	while(ros::ok)
	{
		zcx::path msg;
		msg.mode =
		msg.x =
		msg.y =

		zcx_pub.publish(msg);
		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}
}
*/
