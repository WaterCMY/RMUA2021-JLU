//#include     "../Serial/Serial.h"
//int INFOSIZE = 18;  //接收最大数据包的大小

//int SerialReceive(Parse_vision_re vision_get_main)
//{

//    int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
//    int err;               //返回调用函数的状态
//    int flag=0;

//    //收到的数据
//    uint8_t uart0_recv_buf[INFOSIZE];
//    //串口接收的数据.
//    uint8_t packages[INFOSIZE*2];

//    const char *dev[]  = {"/dev/ttyUSB0"};
//    fd = open(dev[0],O_RDWR | O_NOCTTY | O_NDELAY); //打开串口，返回文件描述符
//    if(-1 == fd)
//    {
//        perror("Can't Open Serial Port");
//        return 0;
//    }else{
//        flag=1;//Seral open permit to trans
//    }

//    do{
//        err = UART0_Init(fd,57600,0,8,2,'N');
//    }while(FALSE == err || FALSE == fd);

//    while(1)
//    {
//        if (flag)
//        {
//            flag = fcntl(fd,F_GETFL, 0);
//            flag |= O_NONBLOCK;
//            fcntl(fd,F_SETFL,flag);

//            //***************接收**************//
//            int len_receive = UART0_Recv(fd, (char *)packages, sizeof(packages));

//            if (len_receive > 0)
//            {
//                printf("\n receive %d data successful\n", len_receive);
//            }else {
//                printf("\n receive data failed!\n");
//            }

//            //提取数据包
//            bool readable = get_one_in_packages(uart0_recv_buf, packages);
//            printf("readable : %d\n", readable);

//            //解包——>得到一个完整的数据结构体vision_receive
//            if (readable) {
//                buff_to_vision_receive(uart0_recv_buf, vision_get_main);
//            }

//            printf("##########Enemy_color:%d\n",vision_get_main->Enemy_color);
//            printf("\n********get_yaw : %f\t", vision_get_main->little_vision.yaw_now);
//            printf("********get_pitch : %f\t", vision_get_main->little_vision.pitch_now);
//            printf("********get_Bullet_speed : %f\t", vision_get_main->little_vision.Bullet_speed);
//            printf("########position:%d\n",vision_get_main->little_vision.position);
//            printf("\n******move_v : %f\t", vision_get_main->little_vision.move_v);
//            printf("********infantry3_HP : %d\t", vision_get_main->infantry3_HP);
//            printf("********infantry4_HP : %d\t", vision_get_main->infantry4_HP);
//            printf("********Enemy_color : %d\t", vision_get_main->Enemy_color);

////                vision_get_main->Enemy_color = vision_r.Enemy_color;

////                vision_get_main->hero_HP = vision_r.hero_HP;
////                vision_get_main->engineer_HP = vision_r.engineer_HP;
////                vision_get_main->infantry1_HP = vision_r.infantry1_HP;
////                vision_get_main->infantry2_HP = vision_r.infantry2_HP;
////                vision_get_main->infantry3_HP = vision_r.infantry3_HP;
////                vision_get_main->sentinel_HP = vision_r.sentinel_HP;
////                vision_get_main->outpost_HP = vision_r.outpost_HP;

////            vision_get_main->mode = vision_r.mode;

//            break;
//        }
//    }
//    close(fd);
//}
