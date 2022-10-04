//#include     "../Serial/Serial.h"
//int SerialSend(double yaw, double pitch, double distance, char armor_change, bool find)
//{
//    if(find == 0){
//        armor_change = 0.0;
//        yaw = 0.0;
//        pitch = 0.0;
//        distance = 0.0;
//    }else if ( find == 1 && (yaw < 0.5 || pitch < 0.5 ) ) {
//        find = 0;
//    }

//    uint16_t u_yaw, u_pitch, u_distance;
//    u_yaw = (int16_t)(yaw *100);
//    u_pitch = (int16_t)(pitch*100);
//    u_distance = (int16_t)distance;

//    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
//    int err;               //返回调用函数的状态
//    int len;
//    int flag=0;

//    const char *dev[]  = {"/dev/ttyUSB0"};
//    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
//    if(-1 == fd)
//    {
//        perror("Can't Open Serial Port");
//        return (0);
//    }else{
//        flag=1;//Seral open permit to trans
//    }

//    do{
//        err = UART0_Init(fd,57600,0,8,2,'N');
//    }while(FALSE == err || FALSE == fd);

//    while(1)//serial fasong shuju
//    {
//        uint8_t uart0_send_buf[7];
//        uart0_send_buf[0] = (uint8_t)0xAA;
//        uart0_send_buf[1] = (u_yaw >> 8) & 0xff;
//        uart0_send_buf[2] = (u_yaw) & 0xff;
//        uart0_send_buf[3] = (u_pitch >> 8) & 0xff;
//        uart0_send_buf[4] = (u_pitch) & 0xff;
//        uart0_send_buf[5] = 0x02;
//        uart0_send_buf[6] = (uint8_t)find;

//        printf("u_yaw : %d\n", u_yaw);
//        printf("u_yaw : %d\n", u_pitch);
//        printf("u_yaw : %d", u_distance);

//        if (flag)
//        {
//            //***************发送**************//
//            len = UART0_Send(fd, uart0_send_buf, 7);
//            if(len > 0)
//            {
//                printf("time send %d data successful\n",len);
//            }else{
//                printf("send data failed!\n");
//            }

//            flag = fcntl(fd,F_GETFL, 0);
//            flag |= O_NONBLOCK;
//            fcntl(fd,F_SETFL,flag);

//            break;
//        }
//    }
//    close(fd);
//}
