//#ifndef CAN_H
//#define CAN_H
//
//#include <stdio.h>
//#include <stdlib.h>
//#include <string.h>
//#include <unistd.h>
//#include <net/if.h>
//#include <sys/ioctl.h>
//#include <sys/socket.h>
//#include <linux/can.h>
//#include <linux/can/raw.h>
//#include <iostream>
//#include <fcntl.h>      /*文件控制定义*/
//
//using namespace std;
//
///**
//* @brief vision_get_struct
//*/
//typedef struct vision_get_struct
//{
//   uint8_t Enemy_color;
//
//   int16_t hero_HP;
//   int16_t engineer_HP;
//   int16_t infantry1_HP;
//   int16_t infantry2_HP;
//   int16_t infantry3_HP;
//   int16_t sentinel_HP;
//   int16_t outpost_HP;
//   int16_t base_HP;
//
//   float Bullet_speed;//单位 m/s
//
//   uint8_t mode;
//
//}vision_re, *Parse_vision_re;
//
//
//
//class Can
//{
//public:
//    Can();
//    ~Can();
//
//    /**
//    * @brief Can main function
//    * @param fire: Whether to fire
//    * @param find: whether to find armor
//    */
//    int canMode(double yaw,double pitch,double distance,char num,bool find, Parse_vision_re vision_get);
//
//    /**
//    * @brief
//    * @param
//    */
//    int socket_connect(const char*m_port);
//
//    /**
//    * @brief
//    * @param
//    */
//    int Can_Filter(const int fd);
//
//    /**
//    * @brief
//    * @param
//    */
//    int Can_Send(int fd, can_frame vision_s, const int count);
//
//    /**
//    * @brief
//    * @param
//    */
//    int Can_recv(int fd,can_frame* rcv_buf, const int count);
//
//    /**
//    * @brief 16位数据转2个8位数据
//    * @param
//    */
//    int16_t Int16ToUint8_p(int16_t adata, uint8_t bdata[]);
//
//    /**
//    * @brief select data
//    * @param
//    */
//    int select_data(Parse_vision_re vision_get, can_frame vision_r);
//
//
//
//};
//
//#endif // CAN_H
