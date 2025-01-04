#include <sstream>
#include <iostream>
#include <serial/serial.h>
#include <float.h>
#include <math.h>

using namespace std;
#include <geometry_msgs/Twist.h>
#include <ros/ros.h>

#define MAX_PACKET_SIZE             26
#define RMID                        183
#define TMID                        183


serial::Serial ser;

double goal_cmd_speed;             // m/sec
double goal_cmd_ang_speed;         // radian/sec

uint8_t serial_comm_snd_buff[MAX_PACKET_SIZE];

void cmdVelCallBack(const geometry_msgs::Twist& keyVel)   //from turtlebot3_teleop_key node
{
    goal_cmd_speed = keyVel.linear.x;
    goal_cmd_ang_speed = -keyVel.angular.z;

#if 0
        ROS_INFO("Goal cmd_vel(m/sec): lin: %f, ang: %f", keyVel.linear.x, keyVel.angular.z);
#endif

    return;
}

uint8_t CalCheckSum(uint8_t *pData)
{
    uint8_t sum;

    sum = 0;
    for(int i = 0; i < 8; i++) {
        sum += *pData++;
    }

    sum = ~sum + 1; //check sum

    return sum;
}

void PutMdData()
{
    uint8_t *p;
    uint16_t len;

    len = 0;
    serial_comm_snd_buff[len++] = RMID;
    serial_comm_snd_buff[len++] = TMID;
    serial_comm_snd_buff[len++] = 1;
    serial_comm_snd_buff[len++] = 130;
    serial_comm_snd_buff[len++] = 2;
    serial_comm_snd_buff[len++] = 30;
    serial_comm_snd_buff[len++] = 0;
    serial_comm_snd_buff[len++] = CalCheckSum(serial_comm_snd_buff);

#if 0
    {
        if(fgInitsetting == INIT_SETTING_STATE_NONE) {
            for(int i = 0; i < len; i++) {
                ROS_INFO("%2d: 0x%02x, %3d", i, serial_comm_snd_buff[i], serial_comm_snd_buff[i]);
            }
        }
    }
#endif

    if(ser.isOpen() == true) {
        ser.write(serial_comm_snd_buff, len);
    }

    return;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "cmdNteleop"); //Node name initialization.
    ros::NodeHandle nh;        //Node handle declaration for communication with ROS system.

    ros::Subscriber keyboard_sub = nh.subscribe("cmd_vel", 10, cmdVelCallBack);

}