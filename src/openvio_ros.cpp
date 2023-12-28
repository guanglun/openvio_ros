#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <sensor_msgs/Imu.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <unistd.h>
#include <pthread.h>
#include "cam_imu.h"

extern unsigned char imu_flag,img_flag;

float imu_data[6];
std_msgs::Header header;
sensor_msgs::Imu imu;
sensor_msgs::ImagePtr msg;
ros::Publisher imu_pub;
bool imu_start_flag=false;
void* imu_thread(void *)
{
    while(imu_start_flag)
    {
        if(imu_flag != 0)
        {
            header.seq=0;
            imu.header.stamp = imu_get_data(imu_data);//ros::Time::now();

            imu.header.frame_id = "imu4";

            imu.orientation.x = 0.0;//imu_data[7];
            imu.orientation.y = 0.0;//imu_data[9];
            imu.orientation.y = 0.0;//imu_data[9];
            imu.orientation.w = 1.0;//imu_data[8];

            //imu.orientation_covariance0=99999.9;
            //imu.orientation_covariance4=99999.9;
            //imu.orientation_covariance8=99999.9;

            /*imu.angular_velocity.x = imu_data[3];
            imu.angular_velocity.y = imu_data[5];
            imu.angular_velocity.z = -imu_data[4];

            imu.linear_acceleration.x = imu_data[0];
            imu.linear_acceleration.y = imu_data[2];
            imu.linear_acceleration.z = -imu_data[1];*/
            imu.angular_velocity.x = imu_data[3];
            imu.angular_velocity.y = imu_data[5];    //IMU body
            imu.angular_velocity.z = -imu_data[4];

            imu.linear_acceleration.x = imu_data[0];
            imu.linear_acceleration.y = imu_data[2];
            imu.linear_acceleration.z = -imu_data[1];

            imu_pub.publish(imu);

            imu_flag=0;
            //printf("IMU:%d %d\r\n",imu.header.stamp.sec,imu.header.stamp.nsec);
        }
    }
    pthread_exit(NULL);
}

int main(int argc, char **argv)
{
    ros::init(argc,argv,"openvio");

    cv::Mat image(cv::Size(752,480),CV_8UC1);
    image_transport::Publisher img_pub;
    ros::NodeHandle node;

    imu_pub = node.advertise<sensor_msgs::Imu>("/imu0", 1000);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_pub = it.advertise("/cam0/image_raw", 1);
    ros::Rate loop_rate(100);

    if (openvio_init(1) == 1)
    {
        printf("Error exit\r\n");
        return 0;
    }

    pthread_t new_thread;
    pthread_create(&new_thread, NULL, imu_thread, NULL);


    while(node.ok())
    {
        imu_start_flag = 1;
        if(img_flag != 0)
        {
            header.seq = 0;
            header.frame_id = "img";
            header.stamp = img_get(image);

            msg = cv_bridge::CvImage(header, "mono8", image).toImageMsg();
            img_pub.publish(msg);

            img_flag = 0;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
