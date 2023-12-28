//#include <ros/ros.h>
#include <stdio.h>
#include <libusb.h>
#include <malloc.h>
#include <unistd.h>
#include <pthread.h>
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
                frame_fps++;


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

                        //check_count[img_index] = fcount;

                        img_index++;
                        if (img_index >= IMG_FRAME_SIZE_MAX)
                        {
                            img_index = 0;
                        }

                    }else{
                        cfcount = fcount + 1;
                        printf("cam cfcount ERROR %d %d",fcount,cfcount);
                    }


                }else{
                    printf("cam Check ERROR !! %C%C%C%C",*(head+0),*(head+1),*(head+2),*(head+3));
                }

                img_recv_index = 0;

            }else{
                if(camRecvLen == 90248)
                {
                    recv_head_status = 0;
                    img_recv_index = 0;
                }
                printf("cam ERROR !!");
            }





/////////////////////////////////
            // if ((recv_head_status == 0) && (camRecvLen == 12))
            // {
            //     findRet = find_str("CAMERA",img[img_index]+img_recv_index, camRecvLen);
            //     if (findRet > 0)
            //     {
            //         recv_head_status = 1;
            //         memcpy(img_time_buf[img_index], img[img_index]+img_recv_index + 6, 6);
            //     }
            // }
            // else if (camRecvLen == 12)
            // {
            //     findRet = find_str("CAMERA",img[img_index]+img_recv_index, camRecvLen);
            //     if (findRet > 0)
            //     {
            //         memcpy(img_time_buf[img_index], img[img_index]+img_recv_index + 6, 6);
            //         img_recv_index = 0;
            //         recv_head_status = 1;
            //     }
            //     else
            //     {
            //         img_recv_index = 0;
            //         recv_head_status = 0;
            //     }
            // }
            // else if (recv_head_status == 0)
            // {
            //     //DBG("cam recv error len %d", camRecvLen);
            //     //emit disconnectSignals();
            //     //break;
            // }
            // else
            // {
            //     img_recv_index += camRecvLen;
                
            //     if (img_recv_index >= 752*480)
            //     {
            //         //printf("cat success:%d\r\n",img_index);
            //         img_recv_index = 0;
            //         recv_head_status = 0;
            //         //img_time = ros::Time::now();
            //         img_flag = img_index;
            //         img_index++;
            //         if (img_index >= IMG_FRAME_SIZE_MAX)
            //         {
            //             img_index = 0;
            //         }
            //     }
            // }

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



void *imu_catch_thread(void *)
{
    int imuRecvLen;
    printf("imu_catch_thread start\r\n");

    float acc_cal = 9.8f*8.0f/65535*2;
    float buffer_tmp[3];
    uint8_t IMU_DATA_INDEX = 6;
    static int imu_recv_index = 0,imu_index = 0;
    int ret = 0;

    while (1)
    {

        ret = libusb_bulk_transfer(dev_handle, IMU_EPADDR, imu_buf[imu_index], IMU_BUF_SIZE, &imuRecvLen, USB_TIMEOUT);

        if (ret < 0)
        {

            if (ret != -7)
            {
                printf("imu_buf recv error %d", ret);
                //emit disconnectSignals();
                break;
            }
            else
            {
                //DBG("imu_buf recv time out");
            }
        }
        else
        {

            if (imuRecvLen == IMU_BUF_SIZE)
            {

                int16_t gx = (((0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 3 * 2]) << 8) | 0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 3 * 2 + 1]);
                int16_t gy = (((0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 4 * 2]) << 8) | 0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 4 * 2 + 1]);
                int16_t gz = (((0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 5 * 2]) << 8) | 0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 5 * 2 + 1]);

                imu_buffer[3] = gx * (4000.0 / 65536.0) * (M_PI / 180.0);// - GX_OFFSET;
                imu_buffer[4] = gy * (4000.0 / 65536.0) * (M_PI / 180.0);// - GY_OFFSET;
                imu_buffer[5] = gz * (4000.0 / 65536.0) * (M_PI / 180.0);// - GZ_OFFSET;

                // get acelerometer values
                int16_t ax = (((0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 0 * 2]) << 8) | 0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 0 * 2 + 1]);
                int16_t ay = (((0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 1 * 2]) << 8) | 0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 1 * 2 + 1]);
                int16_t az = (((0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 2 * 2]) << 8) | 0xff & (unsigned char)imu_buf[imu_index][IMU_DATA_INDEX + 2 * 2 + 1]);

                buffer_tmp[0] = ax*acc_cal;
                buffer_tmp[1] = ay*acc_cal;
                buffer_tmp[2] = az*acc_cal;

                // imu_buffer[0] = (buffer_tmp[0]-OX)/RX;
                // imu_buffer[1] = (buffer_tmp[1]-OY)/RY;
                // imu_buffer[2] = (buffer_tmp[2]-OZ)/RZ;

                imu_buffer[0] = buffer_tmp[0];
                imu_buffer[1] = buffer_tmp[1];
                imu_buffer[2] = buffer_tmp[2];

                imu_flag = imu_index;
                imu_index++;

                if (imu_index >= IMU_FRAME_SIZE_MAX)
                {
                    imu_index = 0;
                }
            }
        }
    }

        // imu_time = ros::Time::now();
        // //imu_time_cnt = (unsigned int)(imu_buf[14] << (8 * 3) | imu_buf[15] << (8 * 2) | imu_buf[16] << (8 * 1) | imu_buf[17]);
        // //img_time_cnt = (unsigned int)(imu_buf[19] << (8 * 3) | imu_buf[20] << (8 * 2) | imu_buf[21] << (8 * 1) | imu_buf[22]);

        // // ros::Time ttime((double)imu_time_cnt / 1000);
        // // imu_time = ttime;
        // // ros::Time tttime((double)img_time_cnt / 1000);
        // // img_time = tttime;

        // int16_t gx = (((0xff & (char)imu_buf[IMU_DATA_INDEX + 3 * 2]) << 8) | 0xff & (char)imu_buf[IMU_DATA_INDEX + 3 * 2 + 1]);
        // int16_t gy = (((0xff & (char)imu_buf[IMU_DATA_INDEX + 4 * 2]) << 8) | 0xff & (char)imu_buf[IMU_DATA_INDEX + 4 * 2 + 1]);
        // int16_t gz = (((0xff & (char)imu_buf[IMU_DATA_INDEX + 5 * 2]) << 8) | 0xff & (char)imu_buf[IMU_DATA_INDEX + 5 * 2 + 1]);

        // imu_buffer[3] = gx * (500.0 / 65536.0) * (M_PI / 180.0) - GX_OFFSET;
        // imu_buffer[4] = gy * (500.0 / 65536.0) * (M_PI / 180.0) - GY_OFFSET;
        // imu_buffer[5] = gz * (500.0 / 65536.0) * (M_PI / 180.0) - GZ_OFFSET;

        // // get acelerometer values
        // int16_t ax = (((0xff & (char)imu_buf[IMU_DATA_INDEX + 0 * 2]) << 8) | 0xff & (char)imu_buf[IMU_DATA_INDEX + 0 * 2 + 1]);
        // int16_t ay = (((0xff & (char)imu_buf[IMU_DATA_INDEX + 1 * 2]) << 8) | 0xff & (char)imu_buf[IMU_DATA_INDEX + 1 * 2 + 1]);
        // int16_t az = (((0xff & (char)imu_buf[IMU_DATA_INDEX + 2 * 2]) << 8) | 0xff & (char)imu_buf[IMU_DATA_INDEX + 2 * 2 + 1]);

        // buffer_tmp[0] = ax*acc_cal;
        // buffer_tmp[1] = ay*acc_cal;
        // buffer_tmp[2] = az*acc_cal;

        // // imu_buffer[0] = (buffer_tmp[0]-OX)/RX;
        // // imu_buffer[1] = (buffer_tmp[1]-OY)/RY;
        // // imu_buffer[2] = (buffer_tmp[2]-OZ)/RZ;

        // imu_buffer[0] = buffer_tmp[0];
        // imu_buffer[1] = buffer_tmp[1];
        // imu_buffer[2] = buffer_tmp[2];

        // // imu_buffer[0] =  1.0019*buffer_tmp[0]-0.0134*buffer_tmp[1]+0.0212*buffer_tmp[2];
        // // imu_buffer[1] =  0.0569*buffer_tmp[0]+1.0190*buffer_tmp[1]+0.0180*buffer_tmp[2];
        // // imu_buffer[2] = -0.0263*buffer_tmp[0]-0.0086*buffer_tmp[1]+0.9741*buffer_tmp[2];

        // // calculate accelerations in m/s²
        // // imu_buffer[0] = ax * (16.0 / 65536.0) * 9.8015f;
        // // imu_buffer[1] = ay * (16.0 / 65536.0) * 9.8015f;
        // // imu_buffer[2] = az * (16.0 / 65536.0) * 9.8015f;

        // // buf_data[0][lvbo_cnt]=imu_buffer[0];
        // // buf_data[1][lvbo_cnt]=imu_buffer[1];
        // // buf_data[2][lvbo_cnt]=imu_buffer[2];

        // // lvbo_data[0]=0.0;
        // // lvbo_data[1]=0.0;
        // // lvbo_data[2]=0.0;

        // // for(int ii=0;ii<10;ii++)
        // // {
        // //     lvbo_data[0]+=buf_data[0][ii];
        // //     lvbo_data[1]+=buf_data[1][ii];
        // //     lvbo_data[2]+=buf_data[2][ii];
        // // }

        // // imu_buffer[0]=(lvbo_data[0]/10);
        // // imu_buffer[1]=(lvbo_data[1]/10);
        // // imu_buffer[2]=(lvbo_data[2]/10);
        // // lvbo_cnt++;
        // // if(lvbo_cnt==10)
        // //     lvbo_cnt=0;

        // // imu_buffer[0] = 0.9953f * imu_buffer[0] - 0.3479f;
        // // imu_buffer[1] = 0.9952f * imu_buffer[1] + 0.1601f;
        // // imu_buffer[2] = 0.9868f * imu_buffer[2] - 0.0253f;

        // imu_flag=1;

        // imu_cnt++;
    //}

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

