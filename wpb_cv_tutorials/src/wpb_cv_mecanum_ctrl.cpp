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
#include <geometry_msgs/Twist.h>

static int state = 0;

int main(int argc, char** argv)
{
    ros::init(argc, argv, "wpb_cv_mecanum_ctrl");

    ros::NodeHandle n;
    ros::Publisher vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 10);

    ros::Rate loop_rate(0.2);

    geometry_msgs::Twist vel_cmd;


    while(ros::ok())
    {
        switch(state)
        {
            case 0:
                vel_cmd.linear.x = 0;
                vel_cmd.linear.y = 0;
                vel_cmd.linear.z = 0;
                vel_cmd.angular.x = 0;
                vel_cmd.angular.y = 0;
                vel_cmd.angular.z = 0;
                vel_pub.publish(vel_cmd);
                printf("[启智CV] 停止\n");
                state ++;
                break;
            case 1:
                vel_cmd.linear.x = 0.1;
                vel_pub.publish(vel_cmd);
                printf("[启智CV] 前进\n");
                state ++;
                break;
            case 2:
                vel_cmd.linear.x = 0;
                vel_cmd.linear.y = -0.1;
                vel_pub.publish(vel_cmd);
                printf("[启智CV] 右平移\n");
                state ++;
                break;
            case 3:
                vel_cmd.linear.x = -0.1;
                vel_cmd.linear.y = 0;
                vel_pub.publish(vel_cmd);
                printf("[启智CV] 后退\n");
                state ++;
                break;
            case 4:
                vel_cmd.linear.x = 0;
                vel_cmd.linear.y = 0.1;
                vel_pub.publish(vel_cmd);
                printf("[启智CV] 左平移\n");
                state ++;
                break;
            case 5:
                vel_cmd.linear.x = 0;
                vel_cmd.linear.y = 0;
                vel_pub.publish(vel_cmd);
                printf("[启智CV] 结束\n");
                break;
        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
