#include "ros/ros.h"  
#include "geometry_msgs/PoseStamped.h" 
#include <mavros/SetPointLocal.h>
#include <mavros/Vector3.h>
#include "sensor_msgs/Imu.h"
#include <geometry_msgs/Vector3.h>
#include <Eigen/Dense>
#include <sstream>
#include <math.h>  
#include <iostream>
#include <mavros/State.h>
#include "std_msgs/Float32.h"
#include "mavros_extras/PositionSetpoint.h"

using Eigen::MatrixXd;

geometry_msgs::PoseStamped p;
geometry_msgs::Vector3 v;
sensor_msgs::Imu a;
mavros::State m;
std_msgs::Float32 h;

int counter = 0;

void imitate_p(const mavros_extras::PositionSetpoint msg);
void imitate_v(const mavros::Vector3 msg);
void imitate_a(const mavros::Vector3 msg);

int main(int argc, char **argv)  
{
	ros::init(argc, argv, "status_imitate");  

    ros::NodeHandle nh;
    ros::Subscriber p_sub = nh.subscribe("/offboard/setpoints_local", 500, imitate_p);
    ros::Subscriber v_sub = nh.subscribe("/offboard/velocity_test", 500, imitate_v);
    ros::Subscriber a_sub = nh.subscribe("/offboard/acceleration_test", 500, imitate_a);

    ros::Publisher p_pub = nh.advertise<geometry_msgs::PoseStamped>("offboard/position_imitate", 500);
    ros::Publisher v_pub = nh.advertise<geometry_msgs::Vector3>("offboard/velocity_imitate",500);
    ros::Publisher a_pub = nh.advertise<sensor_msgs::Imu>("offboard/acceleration_imitate", 500);
    ros::Publisher m_pub = nh.advertise<mavros::State>("offboard/mode_imitate", 500);
    ros::Publisher lidar_pub = nh.advertise<std_msgs::Float32>("offboard/lidar_imitate", 500);

    p.pose.position.x = 1;
    p.pose.position.y = 0;
    p.pose.position.z = 3;
    v.x = 0;
    v.y = 0;
    v.z = 0;
    a.linear_acceleration.x = 0;
	a.linear_acceleration.y = 0;
	a.linear_acceleration.z = 0;

    h.data = 2;  //start height

    m.mode = "MANUAL";

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
    	p_pub.publish(p);
    	v_pub.publish(v);
    	a_pub.publish(a);
        m_pub.publish(m);
        lidar_pub.publish(h);

        counter ++;
        if(counter > 200) m.mode = "OFFBOARD";

    	ros::spinOnce();  
    	loop_rate.sleep();
    }

    return 0;
}

void imitate_p(const mavros_extras::PositionSetpoint msg)
{
    p.pose.position.x = msg.px;
    p.pose.position.y = msg.py;
    p.pose.position.z = msg.ph;
    h.data = msg.ph;
}
void imitate_v(const mavros::Vector3 msg)
{
	v.x = msg.x;
	v.y = msg.y;
	v.z = msg.z;
}
void imitate_a(const mavros::Vector3 msg)
{
	a.linear_acceleration.x = msg.x;
	a.linear_acceleration.y = msg.y;
	a.linear_acceleration.z = msg.z;
}
