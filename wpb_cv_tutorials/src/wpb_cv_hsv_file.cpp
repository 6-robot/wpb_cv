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
 @author     ZhangWanjie
 ********************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "wpb_cv_hsv_file");

    ros::Rate loop_rate(30);

    //读取图片文件
    Mat imgOriginal;
    imgOriginal = imread("/home/robot/camera_rgb.jpg");

    //生成图像显示和参数调节的窗口空见
    namedWindow("Control", CV_WINDOW_AUTOSIZE);

    int iLowH = 20;
    int iHighH = 80;

    int iLowS = 90; 
    int iHighS = 255;

    int iLowV = 50;
    int iHighV = 255;

    cvCreateTrackbar("LowH", "Control", &iLowH, 179); //Hue (0 - 179)
    cvCreateTrackbar("HighH", "Control", &iHighH, 179);

    cvCreateTrackbar("LowS", "Control", &iLowS, 255); //Saturation (0 - 255)
    cvCreateTrackbar("HighS", "Control", &iHighS, 255);

    cvCreateTrackbar("LowV", "Control", &iLowV, 255); //Value (0 - 255)
    cvCreateTrackbar("HighV", "Control", &iHighV, 255);

    namedWindow("RGB"); 
    namedWindow("HSV"); 
    namedWindow("Result"); 
    while( ros::ok())
    {
        //将RGB图片转换成HSV
        Mat imgHSV;
        vector<Mat> hsvSplit;
        cvtColor(imgOriginal, imgHSV, COLOR_BGR2HSV);

        //在HSV空间做直方图均衡化
        split(imgHSV, hsvSplit);
        equalizeHist(hsvSplit[2],hsvSplit[2]);
        merge(hsvSplit,imgHSV);
        Mat imgThresholded;

        //使用上面的Hue,Saturation和Value的阈值范围对图像进行二值化
        inRange(imgHSV, Scalar(iLowH, iLowS, iLowV), Scalar(iHighH, iHighS, iHighV), imgThresholded); 

        //开操作 (去除一些噪点)
        Mat element = getStructuringElement(MORPH_RECT, Size(5, 5));
        morphologyEx(imgThresholded, imgThresholded, MORPH_OPEN, element);

        //闭操作 (连接一些连通域)
        morphologyEx(imgThresholded, imgThresholded, MORPH_CLOSE, element);

        //遍历二值化后的图像数据
        int nTargetX = 0;
        int nTargetY = 0;
        int nPixCount = 0;
        int nImgWidth = imgThresholded.cols;
        int nImgHeight = imgThresholded.rows;
        int nImgChannels = imgThresholded.channels();
        printf("w= %d   h= %d   size = %d\n",nImgWidth,nImgHeight,nImgChannels);
        for (int y = 0; y < nImgHeight; y++)
        {
            for(int x = 0; x < nImgWidth; x++)
            {
                //printf("%d  ",imgThresholded.data[y*nImgWidth + x]);
                if(imgThresholded.data[y*nImgWidth + x] == 255)
                {
                    nTargetX += x;
                    nTargetY += y;
                    nPixCount ++;
                }
            }
        }
        if(nPixCount > 0)
        {
            nTargetX /= nPixCount;
            nTargetY /= nPixCount;
            printf("颜色质心坐标( %d , %d )  点数 = %d\n",nTargetX,nTargetY,nPixCount);
            //画坐标
            Point line_begin = Point(nTargetX-10,nTargetY);
            Point line_end = Point(nTargetX+10,nTargetY);
            line(imgOriginal,line_begin,line_end,Scalar(255,0,0));
            line_begin.x = nTargetX; line_begin.y = nTargetY-10; 
            line_end.x = nTargetX; line_end.y = nTargetY+10; 
            line(imgOriginal,line_begin,line_end,Scalar(255,0,0));
        }
        else
        {
            printf("目标颜色消失...\n");
        }

        //显示处理结果
        imshow("RGB", imgOriginal);
        imshow("HSV", imgHSV);
        imshow("Result", imgThresholded);
        cv::waitKey(5);
        ros::spinOnce();
        loop_rate.sleep();
    }
}