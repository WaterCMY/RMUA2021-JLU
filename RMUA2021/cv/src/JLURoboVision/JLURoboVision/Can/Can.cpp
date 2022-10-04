//#include "Can.h"
//
//Can::Can()
//{
//}
//
//Can::~Can()
//{
//}
////vision_re vision_receive = {0};
//
//int16_t adata[8] = {0};
//uint8_t bdata[2] = {0,0};
//
//uint8_t buf[4] = {0};
//
//int Can::canMode(double yaw,double pitch,double distance,char num,bool find, Parse_vision_re vision_get_main)
//{
//    int fd = -1;
//
//    yaw *= 30;
//    yaw += 12;
//    pitch *= 30;
//    //distance = 0;
//
//
//
//    fd = socket_connect("can0");
//
//    //send data
//    struct can_frame vision_s = {0};
//    //rec data
//    struct can_frame vision_r = {0};
//
//    while(1){
//        int len_receive = 0;
//        //生成报文
//        vision_s.can_id = 0xAA;
//        vision_s.can_dlc = 8;
//
//        Int16ToUint8_p((int16_t)yaw,bdata);
//        vision_s.data[0] = bdata[0];//yaw1
//        vision_s.data[1] = bdata[1];//yaw2
//
//        Int16ToUint8_p((int16_t)pitch,bdata);
//        vision_s.data[2] = bdata[0];//pitch1
//        vision_s.data[3] = bdata[1];//pitch1
//
//        Int16ToUint8_p((int16_t)distance,bdata);
//        vision_s.data[4] = bdata[0];//distance1
//        vision_s.data[5] = bdata[1];//distance2
//
//        vision_s.data[6] = (uint8_t)num;
//
//
////        sleep(1);//yanshi
//        //***************发送**************//
//        Can_Send(fd, vision_s, sizeof(vision_s));
//
//        //***************接收**************//
//        fd = Can_Filter(fd);
//
//        len_receive = Can_recv( fd, &vision_r, sizeof(vision_r));
//
//        if (len_receive > 0) {
//            printf("len_receive : %x\n", len_receive);
//            //printf("raw socket read ((successful))\n");
//        }else{
//            printf("can receive date fail !!!!\n");
//        }
//
//        //********************use to test*******************//
//        //select zhentou
//        if(vision_r.can_id == 0x206 ){
//            printf("vision_r.id : %x\t",vision_r.can_id);
//            printf("vision_r.can_dlc : %x\t",vision_r.can_dlc);
//            printf("vision_r.date[0] : %x\t",vision_r.data[0]);
//            printf("vision_r.date[1] :%x\t",vision_r.data[1]);
//            printf("vision_r.date[2] :%x\n",vision_r.data[2]);
//            printf("vision_r.date[3] :%x\t",vision_r.data[3]);
//            printf("vision_r.date[4] :%x\t",vision_r.data[4]);
//            printf("vision_r.date[5] :%x\t",vision_r.data[5]);
//            printf("vision_r.date[6] :%x\t",vision_r.data[6]);
//            printf("vision_r.date[7] :%x\n",vision_r.data[7]);
//
//            vision_re vision_get = {0};
//
//            select_data(&vision_get, vision_r);
//
//            vision_get.Enemy_color = 0x03;
//
//            //getVolume(&vision_get_main, &vision_get);
//
//
//            vision_get_main->Enemy_color = vision_get.Enemy_color;
//
//            vision_get_main->hero_HP = vision_get.hero_HP;
//            vision_get_main->engineer_HP = vision_get.engineer_HP;
//            vision_get_main->infantry1_HP = vision_get.infantry1_HP;
//            vision_get_main->infantry2_HP = vision_get.infantry2_HP;
//            vision_get_main->infantry3_HP = vision_get.infantry3_HP;
//            vision_get_main->sentinel_HP = vision_get.sentinel_HP;
//            vision_get_main->outpost_HP = vision_get.outpost_HP;
//
//            vision_get_main->Bullet_speed = vision_get.Bullet_speed;
//
//            vision_get_main->mode = vision_get.mode;
//
//        }
//        break;
//    }
//    close(fd);
//}
//
//
//int Can::socket_connect(const char*m_port)
//{
//    int fd = -1,ret = 0;
//    struct sockaddr_can addr;//can总线的地址 同socket编程里面的 socketaddr结构体 用来设置can外设的信息
//    struct ifreq ifr;//接口请求结构体
//
//    //要发送的buffer
//    //创建socket套接字
//    //PF_CAN 为域位 同网络编程中的AF_INET 即ipv4协议
//    //SOCK_RAW使用的协议类型 SOCK_RAW表示原始套接字 报文头由自己创建
//    //CAN_RAW为使用的具体协议 为can总线协议
//
//    fd = socket(PF_CAN, SOCK_RAW, CAN_RAW);//创建套接字
//    if (fd < 0) {
//        perror("socket PF_CAN failed");
//        return -1;
//    }
//    strcpy(ifr.ifr_name, m_port);//"can0" );
//
//    //指定 can0 设备
//    ret = ioctl(fd, SIOCGIFINDEX, &ifr); //成功时返回 0，失败则返回 -1
//
//    if (ret < 0) {
//        perror("ioctl failed");
//        return -1;
//    }
//
//    /* 设置CAN协议 */
//    addr.can_family = AF_CAN;//协议类型
//    addr.can_ifindex = ifr.ifr_ifindex;//can总线外设的具体索引 类似 ip地址
//    ret = bind(fd, (struct sockaddr *)&addr, sizeof(addr));//将套接字与 can0 绑定
//    if (ret < 0) {
//        perror("bind failed");
//        return -1;
//    }
//
//    return fd;
//}
//
//int Can::Can_Filter(const int fd)//设置滤波
//{
//    struct can_filter rfilter[1];
//    rfilter[0].can_id   = 0x206;//定义接收规则
//    rfilter[0].can_mask = CAN_EFF_MASK;
//    //setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));//设置过滤规则
//
//    setsockopt(fd, SOL_CAN_RAW, CAN_RAW_FILTER, NULL, 0);
//
//    //nonblock
//    int flag = fcntl(fd,F_GETFL, 0);
//    flag |= O_NONBLOCK;
//    fcntl(fd,F_SETFL,flag);
//
//    return fd;
//
//}
//
//int Can::Can_Send(int fd, can_frame vision_s, const int count)
//{
//    int nbytes = write(fd, &vision_s, count); //发送 frame[0]
//
//    cout << "send_len : "<< nbytes << endl;
//    if(nbytes > 0){
//        printf("send success !");
//    }else{
//        printf("send erro\n");
//    }
//
//    if(nbytes != count)
//    {
//        printf("Send Error vision_s!\n");
//    }
//}
//
//int Can::Can_recv(int fd, can_frame* rcv_buf, const int count)
//{
//    struct timeval tv_timeout;
//
//    tv_timeout.tv_sec  = 0;
//    tv_timeout.tv_usec = 1;
//    fd_set fs_read;
//
//    FD_ZERO(&fs_read);
//    FD_SET(fd, &fs_read);	//如果fd == -1, FD_SET将在此阻塞
//
//
//    ssize_t ret;
//    ret= select(fd + 1, &fs_read, NULL, NULL, &tv_timeout);
//
//    printf("ret = %d\n",ret);
//    if (ret == 0) // recv 超时
//    {
//        return  0;
//    }
//    if (ret < 0) // select 错误
//    {
//        return  ret;
//    }
//
//    ret = read(fd, rcv_buf, count);
//    //read函数返回-1，说明不是read失败，而是read在以非阻塞方式读一个设备文件（网络文件），并且文件无数据
//
//    //    ret = recv(fd, (char*)rcv_buf, count, 0);
//
//    if (ret <= 0)
//    {
//        return  -1;
//    }
//
//    return  ret;
//}
//
//int16_t Can::Int16ToUint8_p(int16_t adata, uint8_t bdata[])
//{
//    bdata[0] = (adata >>8 &0x00FF);
//    bdata[1] = (adata & 0x00FF);
//
//    return bdata[0];
//}
//
//
//int Can::select_data(Parse_vision_re vision_get, can_frame vision_r)
//{
//    if( vision_r.data[1] == (uint8_t)0x01 )
//    {
//        vision_get->Enemy_color = vision_r.data[2];
//        printf(" get.Enemy_color :%d \n",vision_get->Enemy_color);
//    }
//    if(vision_r.data[1] == (uint8_t)0x02)
//    {
//        vision_get->hero_HP = ((vision_r.data[2]<< 8) & vision_r.data[3]);
//        vision_get->engineer_HP = ((vision_r.data[4]<< 8) & vision_r.data[5]);
//        vision_get->infantry1_HP = ((vision_r.data[6]<< 8) & vision_r.data[7]);
//        printf("  get.hero_HP :%d      ",vision_get->hero_HP);
//        printf("\n get.engineer_HP :%d",vision_get->engineer_HP);
//        printf("get.infantry1_HP :%d \t",vision_get->infantry1_HP);
//    }
//    if( vision_r.data[1] == (uint8_t)0x03 )
//    {
//        vision_get->infantry2_HP = ((vision_r.data[2]<< 8) & vision_r.data[3]);
//        vision_get->infantry3_HP = ((vision_r.data[4]<< 8) & vision_r.data[5]);
//        vision_get->sentinel_HP = ((vision_r.data[6]<< 8) & vision_r.data[7]);
//    }
//    if( vision_r.data[1] == (uint8_t)0x04 )
//    {
//        //#include <QByteArray>
//        //#include <QDebug>
//        //QByteArray buf;
//        buf[0] = vision_r.data[2];
//        buf[1] = vision_r.data[3];
//        buf[2] = vision_r.data[4];
//        buf[3] = vision_r.data[5];
//        memcpy(&(vision_get->Bullet_speed), buf, 4);//sizeof(vision_get.Bullet_speed));转化为float数组
//
//        printf("\n get.Bullet_speed :%f",vision_get->Bullet_speed);
//    }
//    if( vision_r.data[1] == (uint8_t)0x05 )
//    {
//        vision_get->mode= vision_r.data[2];
//        printf("\n!!get.mode :%d",vision_get->mode);
//    }
//
//}
//
////printf("\n get.Enemy_color :%d \t", vision_get.Enemy_color);
////printf("get.hero_HP :%d",vision_get.hero_HP);
////printf("\n get.engineer_HP :%d",vision_get.engineer_HP);
////printf("\tget.infantry1_HP :%d \t",vision_get.infantry1_HP);
////printf("\n get.Bullet_speed :%f",vision_get.Bullet_speed);
////printf("\n get.mode :%d \n",vision_get.mode);
//
//void getVolume(Parse_vision_re s,Parse_vision_re receive_buf)
//{
//    s->Enemy_color = receive_buf->Enemy_color;
//
//    s->hero_HP = receive_buf->hero_HP;
//    s->engineer_HP = receive_buf->engineer_HP;
//    s->infantry1_HP = receive_buf->infantry1_HP;
//    s->infantry2_HP = receive_buf->infantry2_HP;
//    s->infantry3_HP = receive_buf->infantry3_HP;
//    s->sentinel_HP = receive_buf->sentinel_HP;
//
//    s->outpost_HP = receive_buf->outpost_HP;
//
//    s->Bullet_speed = receive_buf->Bullet_speed;
//
//    s->mode = receive_buf->mode;
//
//}
//
