#include "zcx2.h"
using namespace com_ws;
int main()
{

    struct ReceiveData a;
    a.end_flag=false;
    a.link_state=true;//检测是否收到
	a.decision_flag=true;//检测是否启动决策节点
	a.path_flag=false;//检测是否需要修改路径
	a.vision_flag=false;//检测是否需要路径数据
	a.map_flag=false;//检测地图数据是否需要修改
	a.decison_mode=1;
    a.decision_x=7;
    a.decision_y=8;
    a.goal_angle_tolerance=0;
    a.goal_search_tolerance=0;//123
    a.max_vel_x=0;
    a.max_vel_x_backwards=0;
    a.max_vel_y=0;
    a.max_vel_theta=0;
    a.acc_lim_x=0;
    a.acc_lim_y=0;
    a.acc_lim_theta=0;
    a.min_obstacle_dist=0;
    //a.robot_vertices_x=0;
    //a.robot_vertices_y=0;

    int port_in =7000;
    int port_out = 8888;
	int sockfd;
    
	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(-1 == sockfd)
	{
		return false;
        cout<<"socket wrong"<<endl;
	}

	struct sockaddr_in addr;
	socklen_t addr_len=sizeof(addr);

	memset(&addr,0,sizeof(addr));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(port_in);
	addr.sin_addr.s_addr = htonl(INADDR_ANY);// 添加qt的ip地址。  

	//sockaddr_in client;

	if (bind(sockfd, (struct sockaddr*)&addr, addr_len) == -1){
//        printf("Failed to bind socket on port %d\n", port_in);
        cout<<"bind wrong"<<endl;
        close(sockfd);
        return false;
    }
    
    while(1)
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
    
	return 0;
}
