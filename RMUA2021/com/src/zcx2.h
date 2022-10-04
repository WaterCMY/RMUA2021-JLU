#ifndef  COM_BOARD_H_
#define COM_BOARD_H_

#include<sys/select.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <thread>
#include <algorithm>
#include <sstream>
#include "ros/ros.h"
#include<signal.h>

//使用string类型
#include  <string> 
using namespace std;


namespace com_ws{
    void Stop(int signo) 
{
    printf("oops! stop!!!\n");
    _exit(0);
}

    //进行行为修改的中间结构体
struct decision_node{
	int node_number;
	double x;
	double y;
};

//进行参数修改时使用的中间结构体
    struct proto
{
	string name;
	int number;
	string config;
	string fileaddr;
	string filename;
};

//进行操作的中间结构体
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
};   
   
    //与服务器传输的数据
    struct  ReceiveData
    {
        bool end_flag;//检测是否结束
	    bool link_state;//检测是否收到
	    bool decision_flag;//检测是否启动决策节点
	    bool path_flag;//检测是否需要修改路径
        bool vision_flag;//检测是否需要路径数据
        bool map_flag;//检测地图数据是否需要修改
        int decison_mode;
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
        bool operator==(const ReceiveData& rhs) // 操作运算符重载
    {
        return( end_flag == rhs.end_flag) && ( link_state== rhs.link_state)
        &&(decision_flag==rhs.decision_flag)
        &&( path_flag==rhs. path_flag)
        &&(vision_flag==rhs.vision_flag)
         &&(map_flag==rhs.map_flag)
        &&(decison_mode==rhs.decison_mode)
        &&(decision_x==rhs.decision_x)
        &&(decision_y==rhs.decision_y)
        &&(goal_angle_tolerance==rhs.goal_angle_tolerance)
        &&(goal_search_tolerance==rhs.goal_search_tolerance)
        &&(max_vel_x==rhs.max_vel_x)
        &&(max_vel_x_backwards==rhs.max_vel_x_backwards)
        &&(max_vel_y==rhs.max_vel_y)
        &&(max_vel_theta==rhs.max_vel_theta)
        &&(acc_lim_x==rhs.acc_lim_x) 
        &&(acc_lim_y==rhs.acc_lim_y)
        &&(acc_lim_theta==rhs.acc_lim_theta)
        &&(min_obstacle_dist==rhs.min_obstacle_dist);
     }

    };

    //与队友传递的数据
    struct MateData
    {
    };

    //与哨岗传输的数据
    struct ShaoData
    {};

//修脚函数
   void num2string(double num, string &str)
    {
        stringstream ss;
        ss << num;
        str = ss.str();
    }
    //中间转换函数
struct protogroup pretreat(struct ReceiveData a)
{
	struct protogroup b;

	b.dec.node_number=a.decison_mode;
    b.dec.x=a.decision_x;
    b.dec.y=a.decision_y;
	//num2string(a.decision_x,b.dec.x );
	//num2string(a.decision_y,b.dec.y );	
    //cout<<"mode:"<<a.decison_mode<<endl<<"x:"<<a.decision_x<<endl<<"y:"<<b.dec.y<<endl;
	num2string(a.goal_angle_tolerance,b.goal_angle_tolerance.config);
//	b.goal_angle_tolerance.config=a.goal_angle_tolerance;
	b.goal_angle_tolerance.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/global_planner/config";
	b.goal_angle_tolerance.filename="global_planner_config.prototxt";
	b.goal_angle_tolerance.name=" goal_angle_tolerance";
	b.goal_angle_tolerance.number=1;

	num2string(a.goal_search_tolerance,b.goal_search_tolerance.config);
//  b.goal_search_tolerance.config=a.goal_search_tolerance;
	b.goal_search_tolerance.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/global_planner/a_star_planner/config";
	b.goal_search_tolerance.filename="a_star_planner_config.prototxt";
	b.goal_search_tolerance.name=" goal_search_tolerance";
	b.goal_search_tolerance.number=2;

	num2string(a.max_vel_x,b.max_vel_x.config);
//	b.max_vel_x.config=a.max_vel_x;
	b.max_vel_x.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_x.filename="timed_elastic_band.prototxt";
	b.max_vel_x.name=" max_vel_x";
	b.max_vel_x.number=3;

	num2string(a.max_vel_x_backwards,b.max_vel_x_backwards.config);
//	b.max_vel_x_backwards.config=a.max_vel_x_backwards;
	b.max_vel_x_backwards.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_x_backwards.filename="timed_elastic_band.prototxt";
	b.max_vel_x_backwards.name=" max_vel_x_backwards";
	b.max_vel_x_backwards.number=4;

	num2string(a.max_vel_y,b.max_vel_y.config);
//	b.max_vel_y.config=a.max_vel_y;
	b.max_vel_y.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_y.filename="timed_elastic_band.prototxt";
	b.max_vel_y.name=" max_vel_y";
	b.max_vel_y.number=5;

	num2string(a.max_vel_theta,b.max_vel_theta.config);
//	b.max_vel_theta.config=a.max_vel_theta;
	b.max_vel_theta.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.max_vel_theta.filename="timed_elastic_band.prototxt";
	b.max_vel_theta.name=" max_vel_theta";
	b.max_vel_theta.number=6;

	num2string(a.acc_lim_x,b.acc_lim_x.config);
//	b.acc_lim_x.config= a.acc_lim_x;
	b.acc_lim_x.fileaddr = "/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.acc_lim_x.filename="timed_elastic_band.prototxt";
	b.acc_lim_x.name=" acc_lim_x";
	b.acc_lim_x.number=6;

	num2string(a.acc_lim_y,b.acc_lim_y.config);
//	b.acc_lim_y.config=a.acc_lim_y;
	b.acc_lim_y.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.acc_lim_y.filename="timed_elastic_band.prototxt";
	b.acc_lim_y.name=" acc_lim_y";
	b.acc_lim_y.number=7;

	num2string(a.acc_lim_theta,b.acc_lim_theta.config);
//	b.acc_lim_theta.config=a.acc_lim_theta;
	b.acc_lim_theta.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.acc_lim_theta.filename="timed_elastic_band.prototxt";
	b.acc_lim_theta.name=" acc_lim_theta";
	b.acc_lim_theta.number=8;

	num2string(a.min_obstacle_dist,b.min_obstacle_dist.config);
//	b.min_obstacle_dist.config=a.min_obstacle_dist;
	b.min_obstacle_dist.fileaddr="/home/dji/roborts_ws/src/RoboRTS/roborts_planning/local_planner/timed_elastic_band/config";
	b.min_obstacle_dist.filename="timed_elastic_band.prototxt";
	b.min_obstacle_dist.name=" min_obstacle_dist";
	b.min_obstacle_dist.number=9;

	return b;
}


//工具函数
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



//用于修改.ptototxt里的参数，.axml里的参数待会根据里的函数直接改。
int edit(struct proto a)
{
	string cmd,cmd1,cmd2,cmd3,cmd4,cmd5,cmd6;
	cmd1 ="sed -i 's/";
	cmd2 = a.name;
	cmd3 =a.config;
	cmd4 ="/g' ";
	cmd5 =a.fileaddr;
	cmd6 =a.filename;
	cmd = cmd1+cmd2+": .*/"+cmd2+": "+cmd3+cmd4+cmd5+"/"+cmd6;
	cout<<cmd<<endl;
	Test(cmd);
	return 0;
}

//void init()
//{
    //
    //接受数据，开启3个线程，定义个一个变量判断是否是最新的指令
    //判断
    //进入模式
    //线程一：先判断是否是执行该线程，是就执行静态下修改参数再退出，否则直接退出；
    //线程二：先判断是否动态下执行发布指令；
    //主线程判断：是否收到结束的信号，退出程序。
    //循环

//}

struct ReceiveData receive()
{
	int port_in = 7000;//接收端口号。
    int port_out = 8888;
	int sockfd;
	struct ReceiveData a;

  //  cout<<"1"<<endl;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(-1 == sockfd)
	{
        cout<<"socket wrong"<<endl;
		//return false;//出错后如何维护出错和输出报错信息现在很麻烦。
	}

   // cout<<"2"<<endl;
	struct sockaddr_in addr;
    socklen_t addr_len=sizeof(addr);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;       // Use IPV4
    addr.sin_port   = htons(port_in);    //
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
   // cout<<"3"<<endl;
    // Bind 端口，用来接受之前设定的地址与端口发来的信息,作为接受一方必须bind端口，并且端口号与发送方一致
    if (bind(sockfd, (struct sockaddr*)&addr, addr_len) == -1){
     //   printf("Failed to bind socket on port %d\n", port_out);
        close(sockfd);
  //  cout<<"4"<<endl;
	return a;
    }

    //int counter = 0;
    do
	{
      //  cout<<"5"<<endl;
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        memset(&src, 0, sizeof(src));
     //   cout<<"6"<<endl;
		int sz = recvfrom(sockfd, (char*)&a, sizeof(a), 0, (sockaddr*)&src, &src_len);
     //   cout<<"7"<<endl;
        //int sz = recvfrom(sockfd, buffer, 128, 0, (sockaddr*)&src, &src_len);
        if (sz > 0){
           // cout<<a.decison_mode<<endl;
         //  cout<<"8"<<endl;
           break;
        }
	}while(!a.link_state);
 	close(sockfd);
	return a;
}

//部分修改
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
void th1(struct protogroup b, struct ReceiveData a)
{
    if(a.decision_flag==true)
       {
           cout<<b.dec.node_number<<endl<<b.dec.x<<endl<<b.dec.y<<endl;
            //ros::init(argc,argv,"talker");
	        ros::NodeHandle n;
	        ros::Publisher zcx_pub = n.advertise<zcx::path>("mode",1000);
	        ros::Rate loop_rate(10);

            int count=0;
            cout<<"before while"<<endl;
	        while(count<100)
	        {
		        zcx::path msg;
		        msg.mode =b.dec.node_number;
		        msg.x =b.dec.x;
		        msg.y =b.dec.y;

		        zcx_pub.publish(msg);
		        ros::spinOnce();

		        loop_rate.sleep();
		        ++count;
	        }
            cout<<"whiled"<<endl;

       }
}

void th2(struct protogroup b, struct ReceiveData a)
{
      if(a.path_flag==true)
       {
           editpart(b);
       }
}

void Init(){
    //
    //接受数据，开启3个线程，定义个一个变量判断是否是最新的指令
    struct ReceiveData  receivedata;
    receivedata.end_flag=false;
    do
    {
        receivedata.link_state =false;
        receivedata  = receive();
        ROS_INFO("Connecting");//测试时使用，正式使用后删除。
        if(receivedata.link_state==true)
       {
               struct protogroup b; 
               b=pretreat(receivedata);
                thread myobj1(th1,b,receivedata);
                thread myobj2(th2,b,receivedata);

                myobj1.join();
                myobj2.join();
       }
    } while (!receivedata.end_flag);
    //判断
    //进入模式
    //线程一：先判断是否是执行该线程，是就执行静态下修改参数再退出，否则直接退出；
    //线程二：先判断是否动态下执行发布指令；
    //主线程判断：是否收到结束的信号，退出程序。
    //循环
}   

*/





/*
    class ComBoard{
        protected:

        private:
        //传出
        struct ReceiveData fadata;
        struct MateData matedata;
        struct ShaoData shaodata;
        //传入
        struct ReceiveData receivedata;
        //中间
        struct proto a;
        struct protogroup b;
        //发布者
        ros::Publisher zcx_pub ;
        //订阅者
        //声明消息
      //  zcx::path msg;
        //SOCKET
        int port_out ;
        int port_in ;
        int sockfd;

        public:
        struct receivedata receive();
        Init();
        ~ComBoard(){
            close(sock_);
            close(sock);
        }
    };

     void Init(){}；

}*/
}

struct feelback 
{
    string data;
    int num;
};

bool send(struct feelback  a)
{
    int port_in =7000;
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
	addr.sin_port = htons(port_in);
	addr.sin_addr.s_addr = inet_addr(INADDR_ANY);// 添加qt的ip地址。  

	//sockaddr_in client;


	 while(1000)
    {
        addr.sin_family = AF_INET;
        addr.sin_port   = htons(port_out);
        addr.sin_addr.s_addr = inet_addr("127.0.0.1");
        cout<<"before send"<<endl;
	    sendto(sockfd, (char*)&a, sizeof(a), 0, (sockaddr*)&addr, sizeof(addr));
        cout<<"sended"<<endl;
        sleep(1);
    }
	close(sockfd);
	return true;

}



#endif