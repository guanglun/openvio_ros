//#include <ros/ros.h>
#include <stdio.h>
#include <libusb.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
#include <termios.h>
#include <fcntl.h>
#include <time.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>


#include "cam_imu.h"
#include "find_str.h"

extern image_transport::Publisher img_pub;
extern ros::Publisher imu_pub;

using namespace std;
// using namespace cv;
// using namespace ros;

#define CTRL_EPADDR     0x01
#define CAM_EPADDR      0x81
#define IMU_EPADDR      0x82
#define M_PI            3.14159265358979323846

int img_cnt = 0, imu_cnt = 0;
float imu_buffer[6];
libusb_device_handle *dev_handle;

unsigned char imu_flag=0,img_flag=0;

Mat img_cam(Size(752, 480), CV_8UC1);

ros::Time imu_time;
ros::Time img_time,img_time_last;

#define IMU_BUF_SIZE    24
#define IMG_BUF_SIZE    752 * 480 * 2

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

struct timeval tstart;
struct timeval tend;
unsigned long timer;

int img_pass_num = 80;

void hex_show(uint8_t *sdata,int len)
{
    for(int i=0;i<len;i++)
    {
        printf("%02X ",sdata[i]);
    }
}

void *timer_thread_run(void *)
{
    while(1)
    {
        sleep(1);
        //printf("%d %0.2f %d\r\n",frame_fps,((float)recv_count_1s)/1024/1024,vn100_count);
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

    std_msgs::Header header;
    cv::Mat image(cv::Size(752,480),CV_8UC1);
    sensor_msgs::ImagePtr msg;
    ros::Time last_time;

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
            }else if ((recv_head_status == 3) && (camRecvLen == 90256))
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
        //gettimeofday(&tstart, NULL);
                        uint64_t data_time = *(uint64_t *)(head+8);
                        ros::Time now = ros::Time((uint32_t)(data_time/1000000000),(uint32_t)(data_time%1000000000));

                        // hex_show(head+8,8);
                        // printf("%ld\n",*(uint64_t *)(head+8));

                        

                        img_time = now;
                        header.seq = 0;
                        header.frame_id = "img";
                        header.stamp = img_time_last;
                        img_time_last = img_time;
                        memcpy(image.data, img[img_index], IMG_BUF_SIZE/2);
                        msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();

                        if(img_pass_num <= 0)
                            img_pub.publish(msg);
                        else
                            img_pass_num--;

                        // char imgname[128];
                        // sprintf(imgname,"img-%d.bmp",fcount);
                        // cv::imwrite(imgname, image);

        // gettimeofday(&tend, NULL);
        // timer = 1000000 * (tend.tv_sec-tstart.tv_sec) + tend.tv_usec-tstart.tv_usec;
        // printf("%ldus\n", timer);


                        int64_t error_time = now.toNSec() - last_time.toNSec() - 25000000;
                        //printf("%ld\n",now.toNSec() - last_time.toNSec());
                    
                        if(error_time != 0 && error_time != 1000 && error_time != -1000 && error_time != 2000 && error_time != -2000)
                        {
                            printf("cam error %ld\n",error_time);
                        }
                        last_time = now;

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
        return -1;
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
    ros::Time last_time;
    sensor_msgs::Imu imu;


    int ret = imu_init((char *)"/dev/ttyUSB0");
    if(ret < 0)
    {
        ret = imu_init((char *)"/dev/ttyUSB1");
        if(ret < 0)
        {
            goto exit;
        }
    }

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
        rlen = read(fd_vn100, rdata, 49);
        if (rlen > 0)
        {
            if (calculate_imu_crc(rdata, 47) == (unsigned short)(rdata[48] | rdata[47] << 8))
            {
                uint64_t data_time = *(uint64_t *)(rdata+3);
                ros::Time now = ros::Time((uint32_t)(data_time/1000000000),(uint32_t)(data_time%1000000000));//ros::Time::now();
                imu.header.stamp = now;
                //printf("%ld\n",now.toNSec());

                //hex_show(rdata+3,8);
                //printf("%ld\n",*(uint64_t *)(rdata+3));
                vn100_count++;
                for (int i = 0; i < 4; i++)
                {
                    *((unsigned char *)&yaw + i) = rdata[3 + 8 + i];
                    *((unsigned char *)&pitch + i) = rdata[7 + 8 + i];
                    *((unsigned char *)&roll + i) = rdata[11 + 8 + i];
                    *((unsigned char *)&g_x + i) = rdata[15 + 8 + i];
                    *((unsigned char *)&g_y + i) = rdata[19 + 8 + i];
                    *((unsigned char *)&g_z + i) = rdata[23 + 8 + i];
                    *((unsigned char *)&a_x + i) = rdata[27 + 8 + i];
                    *((unsigned char *)&a_y + i) = rdata[31 + 8 + i];
                    *((unsigned char *)&a_z+ i)  = rdata[35 + 8 + i];
                }
 
                imu.header.frame_id = "imu";

                imu.orientation.x = 0.0;//imu_data[7];
                imu.orientation.y = 0.0;//imu_data[9];
                imu.orientation.y = 0.0;//imu_data[9];
                imu.orientation.w = 1.0;//imu_data[8];

                imu.angular_velocity.x = g_x;
                imu.angular_velocity.y = g_y;    
                imu.angular_velocity.z = g_z;

                imu.linear_acceleration.x = a_x;
                imu.linear_acceleration.y = a_y;
                imu.linear_acceleration.z = a_z;

                imu_pub.publish(imu);


                int64_t error_time = now.toNSec() - last_time.toNSec() - 2500000;
                //printf("%ld\n",now.toNSec() - last_time.toNSec());
            
                if(error_time != 0 && error_time != 1000 && error_time != -1000 && error_time != 2000 && error_time != -2000)
                {
                    printf("imu error %ld\n",error_time);
                }
                last_time = now;

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

exit:
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

