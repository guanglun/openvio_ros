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

image_transport::Publisher img_pub;
ros::Publisher imu_pub;

int main(int argc, char **argv)
{
    ros::init(argc,argv,"openvio");

    cv::Mat image(cv::Size(752,480),CV_8UC1);
    
    ros::NodeHandle node;

    imu_pub = node.advertise<sensor_msgs::Imu>("/imu0", 1000);
    ros::NodeHandle nh;
    image_transport::ImageTransport it(nh);
    img_pub = it.advertise("/cam0/image_raw", 1);
    ros::Rate loop_rate(1000);

    if (openvio_init(1) == 1)
    {
        printf("Error exit\r\n");
        return 0;
    }


    while(node.ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
