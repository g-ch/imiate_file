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
#include "mavros_extras/LaserDistance.h"
#define Pi 3.1415926

using Eigen::MatrixXd;

geometry_msgs::PoseStamped p;
geometry_msgs::Vector3 v;
sensor_msgs::Imu a;
mavros::State m;
std_msgs::Float32 h;
mavros_extras::LaserDistance o;


int counter = 0;
float p_last[3];
float p_last_2[3];

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
    ros::Publisher obs_pub = nh.advertise<mavros_extras::LaserDistance>("/laser_send", 500);

    p.pose.position.x = 1;
    p.pose.position.y = 0;
    p.pose.position.z = 3;

    float yaw = Pi/2;
    p.pose.orientation.x = 0.0;
    p.pose.orientation.y = 0.0;
    p.pose.orientation.z = sin(yaw/2);
    p.pose.orientation.w = cos(yaw/2);

    v.x = 0;
    v.y = 0;
    v.z = 0;
    a.linear_acceleration.x = 0;
    a.linear_acceleration.y = 0;
    a.linear_acceleration.z = 0;

    o.min_distance = 600;
    o.angle = 0;

    h.data = -2;  //start height

    m.mode = "MANUAL";

    ros::Rate loop_rate(20);

    while(ros::ok())
    {
    	p_pub.publish(p);
    	v_pub.publish(v);
    	a_pub.publish(a);

        if(counter % 20 == 1) m_pub.publish(m);
        if(counter % 5 == 1)
        {
            lidar_pub.publish(h);
            obs_pub.publish(o);
        }

        counter ++;
        if(counter > 200) m.mode = "OFFBOARD";

        if(counter > 280 && counter < 500){
        	o.min_distance = 200;
            o.angle = 120;
        }
        else
        {
            o.min_distance = 600;
            o.angle = 0;
        }

        if(counter % 3 == 1)h.data -= 0.01;
        if(h.data < -2.17) h.data = -2;
    	ros::spinOnce();  
    	loop_rate.sleep();
    }

    return 0;
}

void imitate_p(const mavros_extras::PositionSetpoint msg)
{
    p.pose.position.x = p_last_2[0];
    p.pose.position.y = p_last_2[1];
    p_last_2[0] = p_last[0];
    p_last_2[1] = p_last[1];
    p_last[0] = msg.px;
    p_last[1] = msg.py;

    p.pose.position.z = 2;
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
