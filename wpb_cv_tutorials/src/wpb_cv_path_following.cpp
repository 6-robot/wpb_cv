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
#include <std_msgs/Int32MultiArray.h>
#include <geometry_msgs/Twist.h>

geometry_msgs::Twist vel_cmd;   //速度消息包
static int arADVal[15];

void AD_Callback(const std_msgs::Int32MultiArray msg)
{
    if(msg.data.size() < 15)
        return;

    //获取AD值
    for(int i=0;i<15;i++)
    {
        arADVal[i] = msg.data[i];
    }
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_cv_avoid_obstacles");

    ros::NodeHandle n;
    ros::Subscriber sub_ad = n.subscribe("/wpb_cv/ad", 100, AD_Callback);
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    vel_cmd.linear.x = 0;
    vel_cmd.linear.y = 0;
    vel_cmd.linear.z = 0;
    vel_cmd.angular.x = 0;
    vel_cmd.angular.y = 0;
    vel_cmd.angular.z = 0;
    vel_pub.publish(vel_cmd);

    ros::Rate loop_rate(30);
    int nCountToStop = 0;   //计时然后停止

    while(ros::ok())
    {
        //显示AD值
        printf("[AD]");
        for(int i=5;i<7;i++)
        {
            printf(" ad%d=%d ",i+1,arADVal[i]);
        }
        printf("\n");

        //默认速度值(如果灰度传感器没有检测到黑线,默认机器人正沿着线走,则直行即可)
        vel_cmd.linear.x = 0.05;
        vel_cmd.angular.z = 0;

        //检测左前灰度传感器是否检测到黑线
        if(arADVal[5] < 1000)
        {
            vel_cmd.linear.x = 0.05;
            vel_cmd.angular.z = 0.1;
        }

        //检测右前灰度传感器是否检测到黑线
        if(arADVal[6] < 1000)
        {
            vel_cmd.linear.x = 0.05;
            vel_cmd.angular.z = -0.1;
        }

        //运行10秒后自动停止(速度全赋值0)
        nCountToStop ++;
        if(nCountToStop > 300)
        {
            vel_cmd.linear.x = 0;
            vel_cmd.angular.z = 0;
            ROS_WARN("Stop!");
        }

        //向底盘发送速度值
        vel_pub.publish(vel_cmd);

        //延时
        ros::spinOnce();
        loop_rate.sleep();
    }


    return 0;
}