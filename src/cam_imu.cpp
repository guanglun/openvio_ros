//#include <ros/ros.h>
#include <stdio.h>
#include <libusb.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include "cam_imu.h"
#include "find_str.h"

using namespace std;
// using namespace cv;
// using namespace ros;

#define CTRL_EPADDR     0x01
#define CAM_EPADDR      0x81
#define IMU_EPADDR      0x82

#define USB_TIMEOUT     1000 //传输数据的时间延迟

// #define GX_OFFSET       0
// #define GY_OFFSET       0
// #define GZ_OFFSET       0

#define GX_OFFSET       0.045
#define GY_OFFSET       0.001
#define GZ_OFFSET       -0.009

#define IMU_BUF_SIZE    24
#define IMG_BUF_SIZE    752 * 480
#define M_PI            3.14159265358979323846

#define REQUEST_CAMERA_START    0xA0
#define REQUEST_CAMERA_STOP     0xA1
#define REQUEST_IMU_START       0xB0
#define REQUEST_IMU_STOP        0xB1

int img_cnt = 0, imu_cnt = 0;
float imu_buffer[6];
libusb_device_handle *dev_handle;

unsigned char imu_flag=0,img_flag=0;

Mat img_cam(Size(752, 480), CV_8UC1);

ros::Time imu_time;
ros::Time img_time;

#define IMG_FRAME_SIZE_MAX 30

#define IMG_FRAME_SIZE_MAX 30
#define IMU_FRAME_SIZE_MAX 100

unsigned char img[IMG_FRAME_SIZE_MAX][IMG_BUF_SIZE];
unsigned char img_time_buf[IMG_FRAME_SIZE_MAX][6];

unsigned char imu_buf[IMU_FRAME_SIZE_MAX][IMU_BUF_SIZE];
unsigned char imu_time_buf[IMU_FRAME_SIZE_MAX][6];

int frame_fps=0,recv_count_1s=0;

float pitch, yaw, roll;
float g_x,g_y,g_z;
float a_x,a_y,a_z;

int fd_vn100;
int vn100_count=0;
Time img_get(Mat img_data)
{
    static uint32_t timer;
    static uint32_t t1,t1_old;
    static uint16_t t2,t2_old;
    static bool is_first = true;
    static float d_time = 0;

    t1 = (uint32_t)(img_time_buf[img_flag][0]<<24);
    t1 |= (uint32_t)(img_time_buf[img_flag][1]<<16);
    t1 |= (uint32_t)(img_time_buf[img_flag][2]<<8);
    t1 |= (uint32_t)(img_time_buf[img_flag][3]<<0);

    t2 = (uint16_t)(img_time_buf[img_flag][4]<<8);
    t2 |= (uint16_t)(img_time_buf[img_flag][5]<<0);

    if(is_first == true)
    {
        is_first = false;
        t1_old = t1;
        t2_old = t2;
        return img_time;
    }

    if(t2 > t2_old)
    {
        timer = t2-t2_old;
    }else{
        timer = (uint32_t)t2 + 50000 - t2_old;
    }

    t1_old = t1;
    t2_old = t2;

    d_time = timer*0.00001;
    //printf("cam %d\t%d\t%d\t%f\r\n",t1,t2,timer,d_time);

    img_cnt++;
    memcpy(img_data.data, img[img_flag], IMG_BUF_SIZE);

    img_time = ros::Time(t1/2, (t1%2*50000+t2)*10000);//ros::Time::now();

    //printf("cam %f\r\n",img_time.toSec());
    return img_time;
}

Time imu_get_data(float *buf)
{
    static uint32_t timer;
    static uint32_t t1,t1_old;
    static uint16_t t2,t2_old;
    static bool is_first = true;
    static float d_time = 0;

    t1 = (uint32_t)(imu_buf[imu_flag][0]<<24);
    t1 |= (uint32_t)(imu_buf[imu_flag][1]<<16);
    t1 |= (uint32_t)(imu_buf[imu_flag][2]<<8);
    t1 |= (uint32_t)(imu_buf[imu_flag][3]<<0);

    t2 = (uint16_t)(imu_buf[imu_flag][4]<<8);
    t2 |= (uint16_t)(imu_buf[imu_flag][5]<<0);

    if(is_first == true)
    {
        is_first = false;
        t1_old = t1;
        t2_old = t2;
        return imu_time;
    }

    if(t2 > t2_old)
    {
        timer = t2-t2_old;
    }else{
        timer = (uint32_t)t2 + 50000 - t2_old;
    }

    t1_old = t1;
    t2_old = t2;

    d_time = timer*0.00001;
    //printf("cam %d\t%d\t%d\t%f\r\n",t1,t2,timer,d_time);

    imu_cnt++;
    for (unsigned char i = 0; i < 6; i++)
    {
        buf[i] = imu_buffer[i];
    }

    imu_time = ros::Time(t1/2, (t1%2*50000+t2)*10000);

    printf("IMU:%f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t\r\n",
                    imu_time.toSec(),
                    imu_buffer[0],imu_buffer[1],imu_buffer[2],
                    imu_buffer[3],imu_buffer[4],imu_buffer[5]);


    
    return imu_time;
}


void *timer_thread_run(void *)
{
    while(1)
    {
        sleep(1);
        printf("%d %0.2f %d\r\n",frame_fps,((float)recv_count_1s)/1024/1024,vn100_count);
        frame_fps = 0;
        recv_count_1s = 0;
        vn100_count = 0;
    }
    
}


void *cam_catch_thread(void *)
{
    int ret = 0,findRet=0;
    static int img_recv_index = 0,img_index = 0;
    int camRecvLen;
    int recv_head_status = 0;
    uint8_t head_tmp[1024];
    uint32_t fcount,cfcount;

    printf("cam_catch_thread start\r\n");

    while (1)
    {
        ret = libusb_bulk_transfer(dev_handle, CAM_EPADDR, (unsigned char *)(img[img_index] + img_recv_index), 512 * 1024, &camRecvLen, 1000);

        if (ret < 0)
        {
            if (ret != -7)
            {
                printf("cam recv error %d\r\n", ret);
                break;
            }
        }
        else
        {
        
            recv_count_1s += camRecvLen;

            if ((recv_head_status < 3) && (camRecvLen == 90240))
            {
                recv_head_status++;
                img_recv_index += camRecvLen;
            }else if ((recv_head_status == 3) && (camRecvLen == 90248))
            {
                recv_head_status = 0;
                
                unsigned char *head = img[img_index] + img_recv_index + 90240;

                if(*(head+0) == 'C' &&
                        *(head+1) == 'A' &&
                        *(head+2) == 'M' &&
                        *(head+3) == 'E')
                {
                    fcount = *(uint32_t *)(head+4);
                    if(cfcount == fcount)
                    {
                        cfcount++;
                        frame_fps++;
                        //check_count[img_index] = fcount;

                        img_index++;
                        if (img_index >= IMG_FRAME_SIZE_MAX)
                        {
                            img_index = 0;
                        }

                    }else{
                        cfcount = fcount + 1;
                        printf("cam cfcount ERROR %d %d\r\n",fcount,cfcount);
                    }


                }else{
                    printf("cam Check ERROR !! %C%C%C%C\r\n",*(head+0),*(head+1),*(head+2),*(head+3));
                }

                img_recv_index = 0;

            }else{
                if(camRecvLen == 90248)
                {
                    recv_head_status = 0;
                    img_recv_index = 0;
                }
                printf("cam ERROR !!\r\n");
            }

        }
    }

    printf("cam_catch_thread exit\r\n");
    pthread_exit(NULL);
}



float buf_data[3][10];
unsigned char lvbo_cnt=0;
float lvbo_data[3],lvbo_gyro_data[3];

#define OX -0.0731
#define OY -0.2291
#define OZ 0.1741
#define RX 1.0013
#define RY 1.0018
#define RZ 1.0144

int set_port_attr(int fd,
                  int vtime, int vmin)
{
    struct termios opt;
    tcgetattr(fd, &opt);

    opt.c_cflag |= CLOCAL | CREAD; /* | CRTSCTS */
    // 其它设置
    opt.c_oflag = 0;

    opt.c_lflag &= ~(ICANON | ECHO | ECHOE |
                     ECHOK | ECHONL |
                     ISIG | IEXTEN | ECHOCTL | ECHOKE);
    opt.c_iflag &= ~(IXON | IXOFF | IXANY | ICRNL);
    opt.c_oflag &= ~OPOST;
    opt.c_cc[VTIME] = vtime;
    opt.c_cc[VMIN] = vmin;
    tcflush(fd, TCIFLUSH);
    tcsetattr(fd, TCSANOW, &opt);

    return 0;
}

// Calculate the 16-bit CRC for the given ASCII or binary message.
unsigned short calculate_imu_crc(unsigned char data[], unsigned int length)
{
    unsigned int i;
    unsigned short crc = 0;
    for (i = 0; i < length; i++)
    {
        crc = (unsigned char)(crc >> 8) | (crc << 8);
        crc ^= data[i];
        crc ^= (unsigned char)(crc & 0xff) >> 4;
        crc ^= crc << 12;
        crc ^= (crc & 0x00ff) << 5;
    }
    return crc;
}

void float_to_mem(unsigned char *buf,float v)
{
    buf[0] = *((unsigned char*)&v+0);
    buf[1] = *((unsigned char*)&v+1);
    buf[2] = *((unsigned char*)&v+2);
    buf[3] = *((unsigned char*)&v+3);
}

int imu_init(char *dev)
{
	char set[1024];
	sprintf(set, "stty -F %s speed 921600 min 1 time 1 ignbrk -brkint -icrnl -imaxbel -opost -onlcr -isig -icanon -iexten -echo -echoe -echok -echoctl -echoke", dev);
	system(set);
	system(set);
	fd_vn100 = open(dev, O_RDWR);
	if (fd_vn100 <= 0)
	{
		printf("%s open fail\r\n", dev);
	}

	set_port_attr(fd_vn100, 1000, 1000);
	return 0;
}

void *imu_catch_thread(void *)
{
    int imuRecvLen;
    printf("imu_catch_thread start\r\n");

    int rlen;
    unsigned char rdata[1024];
    int sync = 1;

    imu_init((char *)"/dev/ttyUSB0");

    while (1)
    {
        sync = 1;
        while (sync)
        {
            rlen = read(fd_vn100, rdata, 1);
            if (rlen > 0)
            {
                if (rdata[0] == 0xFA)
                {
                    sync = 0;
                }
            }
        }
        rlen = read(fd_vn100, rdata, 41);
        if (rlen > 0)
        {

            if (calculate_imu_crc(rdata, 39) == (unsigned short)(rdata[40] | rdata[39] << 8))
            {
                vn100_count++;
                for (int i = 0; i < 4; i++)
                {
                    *((unsigned char *)&yaw + i) = rdata[3 + i];
                    *((unsigned char *)&pitch + i) = rdata[7 + i];
                    *((unsigned char *)&roll + i) = rdata[11 + i];
                    *((unsigned char *)&g_x + i) = rdata[15 + i];
                    *((unsigned char *)&g_y + i) = rdata[19 + i];
                    *((unsigned char *)&g_z + i) = rdata[23 + i];
                    *((unsigned char *)&a_x + i) = rdata[27 + i];
                    *((unsigned char *)&a_y + i) = rdata[31 + i];
                    *((unsigned char *)&a_z+ i)  = rdata[35 + i];
                }
 
				// printf("%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\t%f\r\n",
				// 	   a_x, a_y, a_z,
				// 	   g_x, g_y, g_z,
				// 	   roll, pitch, yaw);
				//printf("%f\t%f\t%f\t%f\t%f\t%f\r\n",a_y, a_x, -a_z, g_y, g_x, -g_z);
				//printf("%0.2f\t%0.2f\t%0.2f\r\n",g_y, g_x, -g_z);
            }
        }

        usleep(1 * 10);
    }
    return NULL;


    printf("imu_catch_thread exit\r\n");
    pthread_exit(NULL);
}
void *img_show_thread(void *)
{
    while (1)
    {

    }
    pthread_exit(NULL);
}

//int main()
int openvio_init(unsigned char flag)
{
    unsigned char ctrl_buffer[10];
    libusb_device **devs;
    libusb_context *ctx = NULL;
    int r;


    r = libusb_init(&ctx);
    if (r < 0)
    {
        printf("Init Error\r\n");
        return 1;
    }

    dev_handle = libusb_open_device_with_vid_pid(ctx, 0x07DC, 0x07DC);
    if (dev_handle == NULL)
    {
        printf("Not found 0x07DC 0x07DC device,exit.\r\n");
        libusb_exit(ctx);
        return 1;
    }
    else
    {
        printf("found 0x07DC 0x07DC device.\r\n");
    }

    r = libusb_claim_interface(dev_handle, 0); //声明设备接口
    if (r < 0)
    {
        printf("Error:libusb_claim_interface\r\n");
        return 1;
    }

    r = libusb_control_transfer(dev_handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_ENDPOINT_IN, REQUEST_CAMERA_START, 0, 0, ctrl_buffer, 128, 1000);
    if (r < 0)
    {
        printf("cam libusb_control_transfer fail\r\n");
        return 1;
    }
    else
    {
        printf("cam libusb_control_transfer success %c\r\n", ctrl_buffer[0]);
    }

    libusb_control_transfer(dev_handle, LIBUSB_REQUEST_TYPE_VENDOR + LIBUSB_ENDPOINT_IN, REQUEST_IMU_START, 0, 0, ctrl_buffer, 128, 1000);
    if (r < 0)
    {
        printf("imu_buf libusb_control_transfer fail\r\n");
        return 1;
    }
    else
    {
        printf("imu_buf libusb_control_transfer success %c\r\n", ctrl_buffer[0]);
    }

    pthread_t cam_thread;
    if (pthread_create(&cam_thread, NULL, cam_catch_thread, NULL))
        printf("Failed to create thread cam_catch_thread\r\n");

    pthread_t imu_thread;
    if (pthread_create(&imu_thread, NULL, imu_catch_thread, NULL))
        printf("Failed to create thread imu_catch_thread\r\n");

    pthread_t timer_thread;
    if (pthread_create(&timer_thread, NULL, timer_thread_run, NULL))
        printf("Failed to create thread timer_thread_run\r\n");

    //pthread_t img_s_thread;
    //if(pthread_create(&img_s_thread, NULL, img_show_thread, NULL))
    //	printf("Failed to create thread img_show_thread\r\n");

    // while (1)
    // {
    //     sleep(1);
    //     printf("%dfps\t%dHz\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t%0.3f\t\r\n",
    //                 img_cnt, imu_cnt,
    //                 imu_buffer[0],imu_buffer[1],imu_buffer[2],
    //                 imu_buffer[3],imu_buffer[4],imu_buffer[5]);

    //     img_cnt = 0;
    //     imu_cnt = 0;
    // }

    //libusb_close(dev_handle);
    //libusb_exit(ctx);
    return 0;
}

//g++ cam_imu.cpp -o cam_imu -I /usr/include/libusb-1.0 /usr/local/lib/libusb-1.0.so /usr/local/lib/libopencv_core.so /usr/local/lib/libopencv_highgui.so -lpthread -I /opt/ros/kinetic/include/opencv-3.3.1-dev

