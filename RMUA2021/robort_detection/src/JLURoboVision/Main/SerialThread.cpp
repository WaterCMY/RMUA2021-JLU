#include"../General/General.h"
#include"../Serial/Serial.h"
int INFOSIZE = 9;  //接收max数据bao的大小

//import Communication
Serial serial;
//Can can;

int SerialReceive()
{
    while(1){

        //cout<<"!@#$%^&*()_+asdfghjklzxcvbnm,"<<endl<<endl;
        int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
        int err;               //返回调用函数的状态
        int flag=0;

        //收到的数据
        uint8_t uart0_recv_buf[INFOSIZE];
        //串口接收的数据.
        uint8_t packages[INFOSIZE*2];
        uint8_t color = 0;

        const char *dev[]  = {"/dev/ttyUSB0"};
        fd = open(dev[0],O_RDWR | O_NOCTTY ); //打开串口，返回文件描述符
        if(-1 == fd)
        {
            perror("Can't Open Serial Port");
            return 0;
        }else{
            flag=1;//Seral open permit to trans
        }

        do{
            err = serial.UART0_Init(fd,115200,0,8,1,'N');
        }while(false == err || false == fd);

        while(1)
        {
            if (flag)
            {
                flag = fcntl(fd,F_GETFL, 0);
                flag |= O_NONBLOCK;
                fcntl(fd,F_SETFL,flag);

                //***************接收**************//
                int len_receive;
                len_receive = serial.UART0_Recv(fd, (char *)packages, sizeof(packages));
                //len_receive = 0;
                if (len_receive > 0)
                {
                    printf("\n receive %d data successful\n", len_receive);
                }else {
                    printf("\n receive data failed!\n");
                }

                //提取数据包
                bool readable = serial.get_one_in_packages(uart0_recv_buf, packages);
    //            printf("readable : %d\n", readable);

                SerialReceiveLock.lock();
                //解包——>得到一个完整的数据结构体vision_receive
                if (readable) {
                    serial.buff_to_vision_receive(uart0_recv_buf, &receive_data, &color);
                }

                if(receive_data.little_vision.yaw_now>180.0){
                    receive_data.little_vision.yaw_now = 360.0-receive_data.little_vision.yaw_now;
                }

                //                send_data->Enemy_color = vision_r.Enemy_color;

                //                send_data->hero_HP = vision_r.hero_HP;
                //                send_data->engineer_HP = vision_r.engineer_HP;

                //            send_data->mode = vision_r.mode;

                if(color == 1){
                    receive_data.Enemy_color = 1;
                }else if(color == 0){
                    receive_data.Enemy_color = 2;
                }
                SerialReceiveLock.unlock();
                break;
            }
        }
        close(fd);

        if (1) {
            std::unique_lock <std::mutex> lck(SerialReceiveLock);
            receiveNow = true;
            SerialReceiveCond.notify_one();
        }
    }

}

int SerialSend()
{
    //cout<<"sennnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnnd"<<endl<<endl;
    while(1){

        if (1) {
            std::unique_lock <std::mutex> lck(SerialSendLock);
            while (!sendNow) {
                SerialSendCond.wait(lck);
            }
            sendNow = false;
        }

        SerialSendLock.lock();
        if(send_data.find == 0){
            send_data.yaw = 0.0;
            send_data.pitch = 0.0;
            send_data.distance = 0.0;
        }

        if(send_data.yaw > 180.0){
            send_data.yaw -= 360.0 ;
        }
        uint16_t u_yaw, u_pitch, u_distance, u_filter;
        uint8_t fire = 0, u_find;
        u_yaw = (int16_t)(send_data.yaw *100);
        u_pitch = (int16_t)(send_data.pitch*100);
        u_distance = (int16_t)send_data.distance;
        u_filter = (int16_t)(send_data.filter*100);
        u_find = (uint8_t)send_data.find ;
        SerialSendLock.unlock();

        //printf(" ^^^^^^^^^^^^^^^^^^^^^^: %f", u_pitch );

        int fd = -1;           //文件描述符，先定义一个与程序无关的值，防止fd为任意值导致程序出bug
        int err;               //返回调用函数的状态
        int len;
        int flag=0;

        const char *dev[]  = {"/dev/ttyUSB0"};
        fd = open(dev[0],O_RDWR | O_NOCTTY ); //打开串口，返回文件描述符
        if(-1 == fd)
        {
            perror("Can't Open Serial Port");
            return (0);
        }else{
            flag=1;//Seral open permit to trans
        }

        do{
            err = serial.UART0_Init(fd,115200,0,8,1,'N');
        }while(false == err || false == fd);

        while(1)//serial fasong shuju
        {
    #define UART_SEND_ARR_SIZE 13
            uint16_t sum = 0;
            uint8_t uart0_send_buf[UART_SEND_ARR_SIZE];
            uart0_send_buf[0] = (uint8_t)0xAA;
            uart0_send_buf[1] = (u_yaw >> 8) & 0xff;
            uart0_send_buf[2] = (u_yaw) & 0xff;
            uart0_send_buf[3] = (u_pitch >> 8) & 0xff;
            uart0_send_buf[4] = (u_pitch) & 0xff;
            uart0_send_buf[5] = (u_distance >> 8) & 0xff;
            uart0_send_buf[6] = (u_distance) & 0xff;
            uart0_send_buf[7] = (u_filter >> 8) & 0xff;
            uart0_send_buf[8] = (u_filter) & 0xff;
            uart0_send_buf[9] = 0x00;
            uart0_send_buf[9] |= (fire<<6) & 0x40;
            uart0_send_buf[9] |= (u_find<<7) & 0x80;
            for(uint8_t sum_index = 0; sum_index < 10; sum_index++)
            {
                sum += uart0_send_buf[sum_index];
            }
            uart0_send_buf[10] = (sum >> 8) & 0xff;
            uart0_send_buf[11] = (sum) & 0xff;
            uart0_send_buf[12] = (uint8_t)0xBB;

    //        printf("u_yaw : %d\n", u_yaw);
    //        printf("u_pitch : %d\n", u_pitch);
    //        printf("u_distance : %d", u_distance);

            if (flag)
            {
    //            flag = fcntl(fd,F_GETFL, 0);
    //            flag |= O_NONBLOCK;
    //            fcntl(fd,F_SETFL,flag);
                //***************发送**************//
                len = serial.UART0_Send(fd, uart0_send_buf, UART_SEND_ARR_SIZE);
                if(len > 0)
                {
                    printf("\ntime send %d data successful\n",len);
                }else{
                    printf("send data failed!\n");
                }
                break;
            }
        }
        close(fd);
    }
}
