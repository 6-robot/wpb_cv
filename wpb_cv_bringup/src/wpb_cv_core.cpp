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
#include <tf/transform_broadcaster.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Int32MultiArray.h>
#include "driver/WPB_driver.h"
#include <math.h>

/*!******************************************************************
 底盘模式选择
 ********************************************************************/

#define MODE_MOTORS     0   //电机控制模式
#define MODE_2WD        1   //双轮差动底盘模式
#define MODE_4WD        2   //四轮差动底盘模式
#define MODE_OMNI       3   //三轮全向底盘模式
#define MODE_MECANUM    4   //麦克纳姆轮全向底盘模式

static int nMotionMode = MODE_MOTORS;
/*!******************************************************************
 
 ********************************************************************/
static CWPB_driver m_wpb_cv;
static geometry_msgs::Twist lastVel;
static int nLastMotorPos[4];
static bool bAD_publish = true;

void motorCtrlCallback(const std_msgs::Int32MultiArray& msg)
{
    if(msg.data.size() < 4)
    {
        return;
    }
    //ROS_INFO("[wpb_cv_motor] %d , %d , %d , %d", msg.data[0] , msg.data[1] , msg.data[2] , msg.data[3]);
    switch(nMotionMode)
    {
        case MODE_MOTORS:
            m_wpb_cv.SendMotors(msg.data[0] , msg.data[1] , msg.data[2] , msg.data[3]);
            m_wpb_cv.arLastMotorVelocity[0] = msg.data[0];
            m_wpb_cv.arLastMotorVelocity[1] = msg.data[1];
            m_wpb_cv.arLastMotorVelocity[2] = msg.data[2];
            m_wpb_cv.arLastMotorVelocity[3] = msg.data[3];
            break;
        case MODE_2WD:
            m_wpb_cv.SendMotors(m_wpb_cv.arLastMotorVelocity[0] , m_wpb_cv.arLastMotorVelocity[1] , msg.data[2] , msg.data[3]);
            m_wpb_cv.arLastMotorVelocity[2] = msg.data[2];
            m_wpb_cv.arLastMotorVelocity[3] = msg.data[3];
            break;
        case MODE_4WD:
            m_wpb_cv.SendMotors(m_wpb_cv.arLastMotorVelocity[0] , m_wpb_cv.arLastMotorVelocity[1] , m_wpb_cv.arLastMotorVelocity[2] , m_wpb_cv.arLastMotorVelocity[3]);
            break;
        case MODE_OMNI:
            m_wpb_cv.SendMotors(m_wpb_cv.arLastMotorVelocity[0] , m_wpb_cv.arLastMotorVelocity[1] , m_wpb_cv.arLastMotorVelocity[2] , msg.data[3]);
            m_wpb_cv.arLastMotorVelocity[3] = msg.data[3];
            break;
        case MODE_MECANUM:
            m_wpb_cv.SendMotors(m_wpb_cv.arLastMotorVelocity[0] , m_wpb_cv.arLastMotorVelocity[1] , m_wpb_cv.arLastMotorVelocity[2] , m_wpb_cv.arLastMotorVelocity[3]);
            break;
    }
}

void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    //ROS_INFO("[wpb_cv] liner(%.2f %.2f) angular(%.2f)", msg->linear.x,msg->linear.y,msg->angular.z);
    switch(nMotionMode)
    {
        case MODE_2WD:
            m_wpb_cv.Velocity2WD(msg->linear.x,msg->angular.z);
            break;
        case MODE_4WD:
            m_wpb_cv.Velocity4WD(msg->linear.x,msg->angular.z);
            break;
        case MODE_OMNI:
            m_wpb_cv.Omni(msg->linear.x,msg->linear.y,msg->angular.z);
            break;
        case MODE_MECANUM:
            m_wpb_cv.Mecanum(msg->linear.x,msg->linear.y,msg->angular.z);
            break;
    }

    lastVel.linear.x = msg->linear.x;
    if(nMotionMode == MODE_OMNI || nMotionMode == MODE_MECANUM)
    {
        lastVel.linear.y = msg->linear.y;
    }
    lastVel.angular.z = msg->angular.z; 
}


static float fKVx = 1.0f/sqrt(3.0f);
static float fKVy = 2.0f/3.0f;
static float fKVz = 1.0f/3.0f;
static float fSumX =0;
static float fSumY =0;
static float fSumZ =0;
static float fOdomX =0;
static float fOdomY =0;
static float fOdomZ =0;
static geometry_msgs::Pose2D lastPose;
int main(int argc, char** argv)
{
    ros::init(argc,argv,"wpb_cv_core");
    ros::NodeHandle n;
    ros::Subscriber cmd_vel_sub = n.subscribe("cmd_vel",10,&cmdVelCallback);
    ros::Subscriber motor_ctrl_sub = n.subscribe("motor_ctrl",10,&motorCtrlCallback);
    ros::Publisher imu_pub = n.advertise<sensor_msgs::Imu >("imu/data_raw", 100);
    ros::Publisher ad_pub = n.advertise<std_msgs::Int32MultiArray>("/wpb_cv/ad", 100);

    ros::NodeHandle n_param("~");
    std::string strSerialPort;
    n_param.param<std::string>("serial_port", strSerialPort, "/dev/ttyUSB0");
    m_wpb_cv.Open(strSerialPort.c_str(),115200);
    std::string chassis_type;
    if (n_param.getParam("chassis_type", chassis_type))
    {
        printf("[设置]底盘类型= %s \n",chassis_type.c_str());
        if(chassis_type == "2wd")
        {
            printf("[设置]双轮差动底盘 \n");
            nMotionMode = MODE_2WD;
        }
        if(chassis_type == "4wd")
        {
            printf("[设置]四轮差动底盘 \n");
            nMotionMode = MODE_4WD;
        }
        if(chassis_type == "omni")
        {
            printf("[设置]三轮全向底盘 \n");
            nMotionMode = MODE_OMNI;
        }
        if(chassis_type == "mecanum")
        {
            printf("[设置]麦克纳姆轮底盘 \n");
            nMotionMode = MODE_MECANUM;
        }
    }
    else
    {
        printf("[设置]未设置底盘类型,默认为电机控制模式 \n");
        nMotionMode = MODE_MOTORS;
    }

    bool bImuOdom;
    n_param.param<bool>("imu_odom", bImuOdom, false);
    
    ros::Time current_time, last_time;
    current_time = ros::Time::now();
    last_time = ros::Time::now();
    ros::Rate r(100.0);
    std_msgs::Int32MultiArray ad_val;

    ros::Publisher odom_pub;
    geometry_msgs::TransformStamped odom_trans;
    tf::TransformBroadcaster broadcaster;
    nav_msgs::Odometry odom;
    geometry_msgs::Quaternion odom_quat;

    odom_pub = n.advertise<nav_msgs::Odometry>("odom",2);
    odom_trans.header.frame_id = "odom";
    odom_trans.child_frame_id = "base_footprint";
    odom.header.frame_id = "odom";
    odom.child_frame_id = "base_footprint";
    odom_trans.transform.translation.x = 0;
    odom_trans.transform.translation.y = 0;
    odom_trans.transform.translation.z = 0;
    odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(0);
    odom.pose.pose.position.x = 0;
    odom.pose.pose.position.y = 0;
    odom.pose.pose.position.z = 0.0;
    odom.pose.pose.orientation = odom_quat;
    odom.twist.twist.linear.x = 0;
    odom.twist.twist.linear.y = 0;
    odom.twist.twist.linear.z = 0;
    odom.twist.twist.angular.x = 0;
    odom.twist.twist.angular.y = 0;
    odom.twist.twist.angular.z = 0;

    lastPose.x = lastPose.y = lastPose.theta = 0;
    lastVel.linear.x = lastVel.linear.y = lastVel.linear.z = lastVel.angular.x = lastVel.angular.y = lastVel.angular.z = 0;
    nLastMotorPos[0] = nLastMotorPos[1] = nLastMotorPos[2] = 0;
    while(n.ok())
    {
        m_wpb_cv.ReadNewData();
        m_wpb_cv.nParseCount ++;
        //ROS_INFO("[m_wpb_cv.nParseCount]= %d",m_wpb_cv.nParseCount);
        if(m_wpb_cv.nParseCount > 100)
        {
            m_wpb_cv.arMotorPos[0] =0; nLastMotorPos[0] = 0;
            m_wpb_cv.arMotorPos[1] =0; nLastMotorPos[1] = 0;
            m_wpb_cv.arMotorPos[2] =0; nLastMotorPos[2] = 0;
            m_wpb_cv.nParseCount = 0;
            //ROS_INFO("empty");
        }
        
        last_time = current_time;
        current_time = ros::Time::now();

        if(bImuOdom == false)
        {
            double fVx,fVy,fVz;
            double fPosDiff[3];
            //if(nLastMotorPos[0] != m_wpb_cv.arMotorPos[0] || nLastMotorPos[1] != m_wpb_cv.arMotorPos[1] || nLastMotorPos[2] != m_wpb_cv.arMotorPos[2] || nLastMotorPos[3] != m_wpb_cv.arMotorPos[3])
            if(true)
            {
                double fTimeDur = current_time.toSec() - last_time.toSec();

                // 方案二 直接把下发速度当作里程计积分依据
                fVx = lastVel.linear.x;
                fVy = lastVel.linear.y;
                if(nMotionMode == MODE_2WD || nMotionMode == MODE_4WD)
                {
                    fVy = lastVel.linear.y = 0;
                }
                fVz = lastVel.angular.z;
                ///////////////////////////////////
               // ROS_INFO("[vel] liner(%.2f %.2f) angular(%.2f)",fVx,fVy,fVz);
                double dx = (lastVel.linear.x*cos(lastPose.theta) - lastVel.linear.y*sin(lastPose.theta))*fTimeDur;
                double dy = (lastVel.linear.x*sin(lastPose.theta) + lastVel.linear.y*cos(lastPose.theta))*fTimeDur;

                //lastPose中就是积分出来的里程计数据
                lastPose.x += dx;
                lastPose.y += dy;
                lastPose.theta += (fVz*fTimeDur);
                //ROS_INFO("[odom] (%.2f %.2f) theta(%.2f)",lastPose.x,lastPose.y,lastPose.theta);
                
                odom_quat = tf::createQuaternionMsgFromRollPitchYaw(0,0,lastPose.theta);
                //updata transform
                odom_trans.header.stamp = current_time;
                odom_trans.transform.translation.x = lastPose.x;
                odom_trans.transform.translation.y = lastPose.y;
                odom_trans.transform.translation.z = 0;
                odom_trans.transform.rotation = tf::createQuaternionMsgFromYaw(lastPose.theta);

                //filling the odometry
                odom.header.stamp = current_time;
                //position
                odom.pose.pose.position.x = lastPose.x;
                odom.pose.pose.position.y = lastPose.y;
                odom.pose.pose.position.z = 0.0;
                odom.pose.pose.orientation = odom_quat;

                //velocity
                odom.twist.twist.linear.x = fVx;
                odom.twist.twist.linear.y = fVy;
                odom.twist.twist.linear.z = 0;
                odom.twist.twist.angular.x = 0;
                odom.twist.twist.angular.y = 0;
                odom.twist.twist.angular.z = fVz;

                //plublishing the odometry and new tf
                broadcaster.sendTransform(odom_trans);
                odom_pub.publish(odom);

                nLastMotorPos[0] = m_wpb_cv.arMotorPos[0];
                nLastMotorPos[1] = m_wpb_cv.arMotorPos[1];
                nLastMotorPos[2] = m_wpb_cv.arMotorPos[2];
                nLastMotorPos[3] = m_wpb_cv.arMotorPos[3];
            }
            else
            {
                odom_trans.header.stamp = ros::Time::now();
                //plublishing the odometry and new tf
                broadcaster.sendTransform(odom_trans);
                odom_pub.publish(odom);
                //ROS_INFO("[odom] zero");
            }
        }
        //else
        {
            //imu
            sensor_msgs::Imu imu_msg = sensor_msgs::Imu();	
            imu_msg.header.stamp = ros::Time::now();
            imu_msg.header.frame_id = "imu";
            imu_msg.orientation.x = m_wpb_cv.fQuatX;
            imu_msg.orientation.y = m_wpb_cv.fQuatY;
            imu_msg.orientation.z = m_wpb_cv.fQuatZ;
            imu_msg.orientation.w = m_wpb_cv.fQuatW;

            imu_msg.angular_velocity.x = m_wpb_cv.fGyroX;
            imu_msg.angular_velocity.y = m_wpb_cv.fGyroY;
            imu_msg.angular_velocity.z = m_wpb_cv.fGyroZ;

            imu_msg.linear_acceleration.x = m_wpb_cv.fAccX;
            imu_msg.linear_acceleration.y = m_wpb_cv.fAccY;
            imu_msg.linear_acceleration.z = m_wpb_cv.fAccZ;

            imu_pub.publish(imu_msg);
        }

        if(bAD_publish == true && m_wpb_cv.bNewAD_0_4 == true && m_wpb_cv.bNewAD_5_9 == true && m_wpb_cv.bNewAD_10_14 == true)
        {
            ad_val.data.clear();
            for(int i=0;i<15;i++)
            {
                ad_val.data.push_back(m_wpb_cv.arValAD[i]); 
            }
            ad_pub.publish(ad_val);
            m_wpb_cv.bNewAD_0_4 = false;
            m_wpb_cv.bNewAD_5_9 = false;
            m_wpb_cv.bNewAD_10_14 = false;
        }

        ros::spinOnce();
        r.sleep();
    }
}