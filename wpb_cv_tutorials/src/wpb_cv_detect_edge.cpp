/*********************************************************************
* Software License Agreement (BSD License)
* 
*  Copyright (c) 2017-2020, Waterplus http://www.6-robot.com
*  All rights reserved.
* 
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
* 
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the WaterPlus nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
* 
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  FOOTPRINTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/
/*!******************************************************************
 @author     WuAnran
 ********************************************************************/
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp> 

using namespace cv;
using namespace std;
void Cam_RGB_Callback(const sensor_msgs::ImageConstPtr& msg)
{
    //ROS_INFO("callbackRGB");
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    Mat imgOriginal = cv_ptr->image;

    int lowThreshold = 200;
	int highThreshold =300;
	int kernel_size = 5;

     //将RGB图片转换成灰度图
    Mat imgGRAY;
    Mat imgEDGES_gray;
    Mat imgEDGES_rgb;

    cvtColor(imgOriginal, imgGRAY, COLOR_BGR2GRAY);
    //将灰度图进行均值滤波处理
    cv::blur( imgGRAY, imgEDGES_gray, cv::Size(5,5) );
    //用canny函数进行边缘检测
    cv::Canny( imgEDGES_gray, imgEDGES_gray, lowThreshold, highThreshold, kernel_size );

    imgEDGES_rgb = cv::Scalar::all(0);
    imgOriginal.copyTo( imgEDGES_rgb, imgEDGES_gray );
    //显示处理结果
    imshow("RGB", imgOriginal);
    imshow("GRAY", imgGRAY);
    imshow("Result_gray", imgEDGES_gray);
    imshow("Result_rgb", imgEDGES_rgb);
    cv::waitKey(5);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_cv_detect_edge_node");
   
    ros::NodeHandle nh;
    ros::Subscriber rgb_sub = nh.subscribe("/camera/color/image_raw", 1 , Cam_RGB_Callback);

    ros::Rate loop_rate(30);
    while( ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
}