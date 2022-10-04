#include "ros/ros.h"
#include "com/pathn.h"
#include "zcx2.h"

using namespace com_ws;

void th1(struct protogroup b,  ReceiveData a)
{
    if(a.decision_flag==true)
       {
          cout<<"mode:"<<b.dec.node_number<<endl<<"x:"<<b.dec.x<<endl<<"y:"<<b.dec.y<<endl;
           // ros::init(argc,argv,"talker");
	        ros::NodeHandle n;
	        ros::ServiceClient zcx_client = n.serviceClient<com::pathn>("mode");
		        com::pathn srv;
		        srv.request.mode =b.dec.node_number;
		        srv.request.x =b.dec.x;
		        srv.request.y =b.dec.y;		        
                if(zcx_client.call(srv))
                {
                    ROS_INFO("result:%d",srv.response.result);
                }else{
                    ROS_ERROR("wrong call");
                    //return -1;
                }
       }
}

void th2(struct protogroup b, struct ReceiveData a)
{
      if(a.path_flag==true)
       {
           editpart(b);
           ROS_INFO("edited");
       }
}

int main(int argc,char**argv){
    //
    //接受数据，开启3个线程，定义个一个变量判断是否是最新的指令
    struct ReceiveData  receivedata;
    struct ReceiveData  olddata;
    receivedata.end_flag=false;
    ros::init(argc,argv,"talker");
    //receivadata.end_flag=false;
    receivedata.link_state=false;//检测是否收到
	receivedata.decision_flag=false;//检测是否启动决策节点
	receivedata.path_flag=false;//检测是否需要修改路径
	receivedata.vision_flag=false;//检测是否需要路径数据
	receivedata.map_flag=false;//检测地图数据是否需要修改
	receivedata.decison_mode=1;
    receivedata.decision_x=7;
    receivedata.decision_y=8;
    receivedata.goal_angle_tolerance=0;
    receivedata.goal_search_tolerance=0;//123
    receivedata.max_vel_x=0;
    receivedata.max_vel_x_backwards=0;
    receivedata.max_vel_y=0;
    receivedata.max_vel_theta=0;
    receivedata.acc_lim_x=0;
    receivedata.acc_lim_y=0;
    receivedata.acc_lim_theta=0;
    receivedata.min_obstacle_dist=0;
    do
    {
        struct feelback 
{
    string data;
    int num;
};
/*
            struct feelback  line1;
            line1.data="ready";
            line1.num=0;
        send(line1);
        */
        receivedata.link_state =true;
        receivedata  = receive();
        receivedata.link_state =true;
        ROS_INFO("Connecting");//测试时使用，正式使用后删除。
         signal(SIGINT, Stop); 
        if(olddata == receivedata)
        {
            continue;
        }
        olddata =receivedata;
        if(receivedata.link_state==true)
       {
               struct protogroup b; 
               b=pretreat(receivedata);
                thread myobj1(th1,b,receivedata);
                thread myobj2(th2,b,receivedata);
                myobj1.detach();
                myobj2.detach();
       }
    } while (!receivedata.end_flag);
    //判断
    //进入模式
    //线程一：先判断是否是执行该线程，是就执行静态下修改参数再退出，否则直接退出；
    //线程二：先判断是否动态下执行发布指令；
    //主线程判断：是否收到结束的信号，退出程序。
    //循环
}   