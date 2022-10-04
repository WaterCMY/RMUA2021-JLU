#include     "../Serial/Serial.h"

Serial::Serial()
{
}

Serial::~Serial()
{
}

/*
 优先目标数字 - targetNum - 前三位（数字1 - 8——1英雄，2工程，345步兵，6哨兵，7前哨战，8基地）
 模式选择mode - 后五位（0 不处理，1 - 8留作模式选择，1为手动开火，2为自瞄，3为小符，4为大符）
 */
//int INFOSIZE = 9;  //接收max数据bao的大小


//int Serial::UART0_Open(int fd,char*port)
//{
//    fd = open( port, O_RDWR|O_NOCTTY|O_NDELAY);
//    if (fd<0)
//    {
//        perror("Can't Open Serial Port");
//        return(FALSE);
//    }
//    //恢复串口为阻塞状态
//    if(fcntl(fd, F_SETFL, FNDELAY) < 0)
//    {
//        printf("fcntl failed!\n");
//        return(FALSE);
//    }
//    else
//    {
//        printf("fcntl=%d\n",fcntl(fd, F_SETFL,0));
//    }
//    //测试是否为终端设备
//    if(0 == isatty(STDIN_FILENO))
//    {
//        printf("standard input is not a terminal device\n");
//        return(FALSE);
//    }
//    else
//    {
//        printf("isatty success!\n");
//    }
//    printf("fd->open=%d\n",fd);
//    return fd;
//}

int Serial::UART0_Set(int fd,int speed,int flow_ctrl,int databits,int stopbits,int parity)
{

    int   i;
    int   status;
    int   speed_arr[] = { B115200,B57600, B38400,B19200,B9600, B4800, B2400, B1200, B300};
    int   name_arr[] = {115200, 57600,38400, 19200,  9600,  4800,  2400,  1200,  300};

    struct termios options;

    /*  tcgetattr(fd,&options)得到与fd指向对象的相关参数，并将它们保存于options,该函数还可以测试配置是否正确，
        该串口是否可用等。若调用成功，函数返回值为0，若调用失败，函数返回值为1.  */
    if( tcgetattr( fd,&options)  !=  0)
    {
        perror("SetupSerial 1");
        return false;
    }

    //设置串口输入波特率和输出波特率
    for ( i= 0;  i < sizeof(speed_arr) / sizeof(int);  i++)
    {
        if  (speed == name_arr[i])
        {
            cfsetispeed(&options, speed_arr[i]);
            cfsetospeed(&options, speed_arr[i]);
        }
    }

    //修改控制模式，保证程序不会占用串口
    options.c_cflag |= CLOCAL;
    //修改控制模式，使得能够从串口中读取输入数据
    options.c_cflag |= CREAD;

    //设置数据流控制
    switch(flow_ctrl)
    {

    case 0 ://不使用流控制
        options.c_cflag &= ~CRTSCTS;
        break;

    case 1 ://使用硬件流控制
        options.c_cflag |= CRTSCTS;
        break;
    case 2 ://使用软件流控制
        options.c_cflag |= IXON | IXOFF | IXANY;
        break;
    }
    //设置数据位
    //屏蔽其他标志位
    options.c_cflag &= ~CSIZE;
    switch (databits)
    {
    case 5    :
        options.c_cflag |= CS5;
        break;
    case 6    :
        options.c_cflag |= CS6;
        break;
    case 7    :
        options.c_cflag |= CS7;
        break;
    case 8:
        options.c_cflag |= CS8;
        break;
    default:
        //fprintf(stderr,"Unsupported data size\n");
        return false;
    }
    //设置校验位
    switch (parity)
    {
    case 'n':
    case 'N': //无奇偶校验位。
        options.c_cflag &= ~PARENB;
        options.c_iflag &= ~INPCK;
        break;
    case 'o':
    case 'O'://设置为奇校验
        options.c_cflag |= (PARODD | PARENB);
        options.c_iflag |= INPCK;
        break;
    case 'e':
    case 'E'://设置为偶校验
        options.c_cflag |= PARENB;
        options.c_cflag &= ~PARODD;
        options.c_iflag |= INPCK;
        break;
    case 's':
    case 'S': //设置为空格
        options.c_cflag &= ~PARENB;
        options.c_cflag &= ~CSTOPB;
        break;
    default:
        //fprintf(stderr,"Unsupported parity\n");
        return false;
    }
    // 设置停止位
    switch (stopbits)
    {
    case 1:
        options.c_cflag &= ~CSTOPB; break;
    case 2:
        options.c_cflag |= CSTOPB; break;
    default:
        //fprintf(stderr,"Unsupported stop bits\n");
        return false;
    }

    //修改输出模式，原始数据输出
    options.c_oflag &= ~OPOST;

    options.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
    //options.c_lflag &= ~(ISIG | ICANON);

    //设置等待时间和最小接收字符
    options.c_cc[VTIME] = 1; /* 读取一个字符等待1*(1/10)s */
    options.c_cc[VMIN] = 1; /* 读取字符的最少个数为1 */

    //如果发生数据溢出，接收数据，但是不再读取 刷新收到的数据但是不读
    tcflush(fd,TCIFLUSH);

    //激活配置 (将修改后的termios数据设置到串口中）
    if (tcsetattr(fd,TCSANOW,&options) != 0)
    {
        perror("com set error!\n");
        return false;
    }
    return true;
}

int Serial::UART0_Init(int fd, int speed,int flow_ctrl,int databits,int stopbits,int parity)
{
    int err=0;
    //设置串口数据帧格式
    if (UART0_Set(fd,115200,0,8,2,'N') == false)
    {
        return false;
    }
    else
    {
        return  true;
    }
}

int Serial::UART0_Recv(int fd, char *rcv_buf,int data_len)
{
    int len,fs_sel;
    fd_set fs_read;

    struct timeval time;

    FD_ZERO(&fs_read);
    FD_SET(fd,&fs_read);    //如果fd == -1, FD_SET将在此阻塞

    time.tv_sec = 0;
    time.tv_usec = 10000;

    //串口的多路通信
    fs_sel = select(fd+1,&fs_read,NULL,NULL,&time);
    //printf("fs_sel = %d\n",fs_sel);
    if(fs_sel)
    {
        len = read(fd,rcv_buf,data_len);
        return len;
    }
    else
    {
        return false;
    }
}

int Serial::UART0_Send(int fd, uint8_t *send_buf,int data_len)
{
    int len = 0;

    len = write(fd,send_buf,data_len);
    if (len == data_len )
    {
        //printf("send data is %d\n",send_buf);
        return len;
    }
    else
    {
        tcflush(fd,TCOFLUSH);
        return false;
    }
}

bool Serial::get_one_in_packages(uint8_t * infoArray, uint8_t * packages)
{
    int ptr;

    for (ptr = 0; ptr < 9 * 2; ptr++) {
        if (packages[ptr] == 0xA7){
            for (int i = 0; i < 9; i++){
                infoArray[i] = packages[ptr + i];
            }
            return true;
        }
    }

    return false;
}

void Serial::buff_to_vision_receive(uint8_t * uart0_recv_buf,Parse_vision_re vision_receive, uint8_t * color)
{
    if (uart0_recv_buf[8] == 0xBB) {
        //        memcpy( &(receive_buf->little_vision) , &(uart0_recv_buf[2]) , INFOSIZE-2);
        vision_receive->little_vision.yaw_now = \
                ((int16_t)(( ((uint16_t)uart0_recv_buf[1] << 8) & 0xFF00) | ((uint16_t)uart0_recv_buf[2]) & 0x00FF))*0.01; //Enemy_color
        vision_receive->little_vision.pitch_now = \
                ((int16_t)(( ((uint16_t)uart0_recv_buf[3] << 8) & 0xFF00) | ((uint16_t)uart0_recv_buf[4]) & 0x00FF))*0.01;
        vision_receive->little_vision.Bullet_speed = \
                ((int16_t)(( ((uint16_t)uart0_recv_buf[5] << 8) & 0xFF00) | ((uint16_t)uart0_recv_buf[6]) & 0x00FF))*0.01;
        *color = ((uint8_t)uart0_recv_buf[7] & 0x80)>>7; //Enemy_color
        vision_receive->If_Press_Down = ((uint8_t)uart0_recv_buf[7] & 0x40)>>6; //If_Press_Down
        vision_receive->wind_change = ((uint8_t)uart0_recv_buf[7] & 0x30)>>4;
    }
}

