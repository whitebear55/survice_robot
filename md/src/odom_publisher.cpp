#include "ros/ros.h"
#include "std_msgs/Int32.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <cmath>
#include <md/Pose.h>
#include <tf/transform_broadcaster.h>


using namespace std;

typedef struct {

   float PosX;
   float PosY;
   float Theta;
   float RealLinearVel, RealAngularVel;

}MotorDriver;
MotorDriver Md;

//tf_prefix add
// std::string tf_prefix_;
// bool has_prefix;

//_localization
// bool m_b_option = false;

class MERO
{
private:

public:
   bool first;
   double prev_coordinates[5];
   double coordinates[5]; //x,y,theta

   ros::NodeHandle pnh;
   ros::Publisher odom_publisher;
   // tf::TransformBroadcaster odom_broadcaster;
   ros::Time current_time, last_time;

   MERO()
   {
       first = true;
       odom_publisher = pnh.advertise<nav_msgs::Odometry>("odom", 50);
       printf("Create MERO Class\n");

       current_time = ros::Time::now();
       last_time = ros::Time::now();
       for (int i = 0; i < 5; i++)
       {
           prev_coordinates[i] = 0;
           coordinates[i] = 0;
       }

   }
   ~MERO()
   {
       printf("Distroy MERO Class\n");
   }

   void read() {

       if (first)
       {
           current_time = ros::Time::now();
           last_time = current_time;
           first = false;
       }
       else
       {
           current_time = ros::Time::now();
           pub();
       }

   }


   void pub()
   {
       // double dt=(current_time-last_time).toSec();

       for (int i = 0; i < 5; i++)
       {
           prev_coordinates[i] = coordinates[i];
       }


       geometry_msgs::Quaternion odom_quat =
tf::createQuaternionMsgFromYaw(coordinates[2]);

       nav_msgs::Odometry odom;
       odom.header.stamp = current_time;
       for (int i = 0; i < 36; i++) {
           if (i == 0 || i == 7 || i == 14) {
               odom.pose.covariance[i] = .01;
           }
           else if (i == 21 || i == 28 || i == 35) {
               odom.pose.covariance[i] += 0.1;
           }
           else {
               odom.pose.covariance[i] = 0;
           }
       }
       odom.header.frame_id = "odom";
       odom.child_frame_id = "base_footprint";

       //pose
       odom.pose.pose.position.x = coordinates[0];
       odom.pose.pose.position.y = coordinates[1];
       odom.pose.pose.position.z = 0.0;
       odom.pose.pose.orientation = odom_quat;
       odom.twist.twist.linear.x = coordinates[3];
       odom.twist.twist.linear.y = 0.0;
       odom.twist.twist.linear.z = 0.0;
       odom.twist.twist.angular.z = coordinates[4];
       odom_publisher.publish(odom);

       last_time = current_time;
   }
};
void md_callBack(const md::Pose::ConstPtr& msg)
{
   std::cout << "x : " << msg->x << std::endl;
   Md.PosX = msg->x;
   Md.PosY = msg->y;
   Md.Theta = msg->theta;
   // Md.fTheta             = (float)(msg->theta)/10;
   Md.RealLinearVel = msg->linear_velocity;
   Md.RealAngularVel = msg->angular_velocity;
}
int main(int argc, char** argv) {



   // Launch ROS and create a node
   ros::init(argc, argv, "odom_pub");
   ros::NodeHandle node;


   ros::Subscriber submd = node.subscribe("robot_pose", 10,md_callBack);    //Subscriber declaration.

   ros::Rate loop_rate(100);
   MERO mero;

   while (ros::ok()) {


       mero.coordinates[0] = Md.PosX;
       mero.coordinates[1] = Md.PosY;
       mero.coordinates[2] = Md.Theta;
       mero.coordinates[3] = Md.RealLinearVel;
       mero.coordinates[4] = Md.RealAngularVel;
       mero.read();

       ros::spinOnce();
       loop_rate.sleep();
   }

   return 0;
}