  #include <unistd.h>
  #include <string.h>
  #include <stdio.h>
  #include <iostream>
  #include <netinet/in.h>
  #include <sys/socket.h>
  #include <sys/types.h>
  #include <arpa/inet.h>
  
#define IP_FOUND "IP_FOUND"
#define IP_FOUND_ACK "IP_FOUND_ACK"

#define MCAST_ADDR "224.0.0.88"
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




 int main(){
     int serverfd;
     unsigned int server_addr_length, client_addr_length;
     char recvline[20];
     char sendline[20];
     struct sockaddr_in serveraddr , clientaddr,our_addr;
    
     int so_broadcast=1;

     socklen_t  socklen;
    cout<<1<<endl;
    //取sgdata，暂时还无，因此在这里写成假获取。
    sgdata.car1=true;
    sgdata.car2=true;
    sgdata.car3=true;
    sgdata.car4=true;
    sgdata.car1_x=1;
    sgdata.car1_y=1;
    sgdata.car2_x=2;
    sgdata.car2_y=2;
    sgdata.car3_x=3;
    sgdata.car3_y=3;
    sgdata.car4_x=4;
    sgdata.car4_y=4;

     // 使用函数socket()，生成套接字文件描述符；
     if( (serverfd = socket(AF_INET, SOCK_DGRAM, 0)) < 0 ){
         perror("socket() error");
         exit(1);
     }
 
     // 通过struct sockaddr_in 结构设置服务器地址和监听端口；
     bzero(&serveraddr,sizeof(serveraddr));
     serveraddr.sin_family = AF_INET;
     serveraddr.sin_addr.s_addr = htonl(INADDR_ANY);
     serveraddr.sin_port = htons(6666);
     server_addr_length = sizeof(serveraddr);
    cout<<2<<endl;
    //客户端绑定通信端口，否则系统自动分配
        memset(&our_addr,0,sizeof(our_addr));
        our_addr.sin_family = AF_INET;
        our_addr.sin_port = htons(7777);
        our_addr.sin_addr.s_addr = htonl(INADDR_ANY); //MCAST_ADDR

     // 使用bind() 函数绑定监听端口，将套接字文件描述符和地址类型变量（struct sockaddr_in ）进行绑定；
     if( bind(serverfd, (struct sockaddr *) &serveraddr, server_addr_length) < 0){
         perror("bind() error");
         exit(1); 
     }
     cout<<3<<endl;
     socklen = sizeof(struct sockaddr);
        strncpy(sendline,IP_FOUND,strlen(IP_FOUND)+1);

     // 接收客户端的数据，使用recvfrom() 函数接收客户端的网络数据；
     client_addr_length = sizeof(sockaddr_in);
     int recv_length = 0;
     recv_length = recvfrom(serverfd, recvline, sizeof(recvline), 0, (struct sockaddr *) &clientaddr, &client_addr_length);
     cout << "recv_length = "<< recv_length <<endl;
     cout << recvline << endl;
     cout<<4<<endl;
      
     // 向客户端发送数据，使用sendto() 函数向服务器主机发送数据；
    int send_length = 0;
     //sprintf(sendline, "hello client !");
     send_length = sendto(serverfd,(char*)& sgdata, sizeof(sgdata), 0, (struct sockaddr *) &clientaddr, client_addr_length);
     if( send_length < 0){
         perror("sendto() error");
         exit(1);
     }
    cout << "car1 = "<< sgdata.car1<<endl;
    cout <<"car2 = "<< sgdata.car2<<endl;
    cout << "car3 = "<< sgdata.car3<<endl;
    cout << "car4 = "<< sgdata.car4<<endl;
    cout << "car1_x= "<< sgdata.car1_x<<endl;
    cout << "car1_y = "<< sgdata.car1_y<<endl;
    cout << "car2_x = "<< sgdata.car2_x<<endl;
    cout << "car2_y = "<< sgdata.car2_y<<endl;
    cout << "car3_x = "<< sgdata.car3_x<<endl;
    cout << "car3_y = "<< sgdata.car3_y<<endl;
    cout << "car4_x = "<< sgdata.car4_x<<endl;
    cout << "car4_y = "<< sgdata.car4_y<<endl;
                

     //关闭套接字，使用close() 函数释放资源；
     close(serverfd);
 
     return 0;
	}
