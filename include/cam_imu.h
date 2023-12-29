#ifndef __CAMERA_H
#define __CAMERA_H

#include <ros/ros.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;  
using namespace cv;
using namespace ros;

int openvio_init(unsigned char flag);
Time imu_get_data(float *buf);

#endif
