#include<sys/select.h>
#include<unistd.h>
#include<sys/types.h>
#include<sys/socket.h>
#include<arpa/inet.h>
#include<netinet/in.h>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include<thread>
#include <algorithm>
#include <sstream>
#include "zcx2.h"

using namespace com_ws;

char* receivetest()
{
	int port_in =8888;//接收端口号。
    int port_out = 7000;
	int sockfd;
	char* a;

    cout<<"1"<<endl;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(-1 == sockfd)
	{
        cout<<"socket wrong"<<endl;
		//return false;//出错后如何维护出错和输出报错信息现在很麻烦。
	}

    cout<<"2"<<endl;
	struct sockaddr_in addr;
    socklen_t addr_len=sizeof(addr);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;       // Use IPV4
    addr.sin_port   = htons(port_out);    //
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    cout<<"3"<<endl;
    // Bind 端口，用来接受之前设定的地址与端口发来的信息,作为接受一方必须bind端口，并且端口号与发送方一致
    if (bind(sockfd, (struct sockaddr*)&addr, addr_len) == -1){
     //   printf("Failed to bind socket on port %d\n", port_out);
        close(sockfd);
    cout<<"4"<<endl;
	return a;
    }

    //int counter = 0;
    do
	{
        cout<<"5"<<endl;
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        memset(&src, 0, sizeof(src));
        cout<<"6"<<endl;
		int sz = recvfrom(sockfd, (char*)&a, sizeof(a), 0, (sockaddr*)&src, &src_len);
        cout<<"7"<<endl;
        //int sz = recvfrom(sockfd, buffer, 128, 0, (sockaddr*)&src, &src_len);
        if (sz > 0){
           // cout<<a.decison_mode<<endl;
           cout<<"8"<<endl;
        }
	}while(1000);
 	close(sockfd);
	return a;
}

int main(int argc,char**argv){
    	int port_in =8888;//接收端口号。
    int port_out = 7000;
	int sockfd;
	char* a;

    cout<<"1"<<endl;

	sockfd = socket(AF_INET,SOCK_DGRAM,0);
	if(-1 == sockfd)
	{
        cout<<"socket wrong"<<endl;
		//return false;//出错后如何维护出错和输出报错信息现在很麻烦。
	}

    cout<<"2"<<endl;
	struct sockaddr_in addr;
    socklen_t addr_len=sizeof(addr);

    memset(&addr, 0, sizeof(addr));
    addr.sin_family = AF_INET;       // Use IPV4
    addr.sin_port   = htons(port_out);    //
    addr.sin_addr.s_addr = htonl(INADDR_ANY);
    cout<<"3"<<endl;
    // Bind 端口，用来接受之前设定的地址与端口发来的信息,作为接受一方必须bind端口，并且端口号与发送方一致
    if (bind(sockfd, (struct sockaddr*)&addr, addr_len) == -1){
     //   printf("Failed to bind socket on port %d\n", port_out);
        close(sockfd);
    cout<<"4"<<endl;
	return 0;
    }

    //int counter = 0;
    do
	{
        cout<<"5"<<endl;
        struct sockaddr_in src;
        socklen_t src_len = sizeof(src);
        memset(&src, 0, sizeof(src));
        cout<<"6"<<endl;
		int sz = recvfrom(sockfd, (char*)&a, sizeof(a), 0, (sockaddr*)&src, &src_len);
        cout<<"7"<<endl;
        //int sz = recvfrom(sockfd, buffer, 128, 0, (sockaddr*)&src, &src_len);
        if (sz > 0){
           // cout<<a.decison_mode<<endl;
           cout<<"8"<<endl;
           cout<<a<<endl;
        }
	}while(1000);
 	close(sockfd);
	return 0;

}