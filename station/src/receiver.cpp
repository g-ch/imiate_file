//*******receiver.cpp**********

#include "receiver.h"
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <math.h>
#include <mavros/State.h>
#include <mavros/Mavlink.h>
#include <mavros/BatteryStatus.h>
#include <mavros/RadioStatus.h>
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Temperature.h"
#include "sensor_msgs/TimeReference.h"
#include "std_msgs/Float64.h"
#include "geometry_msgs/PoseStamped.h"
#include "geometry_msgs/Vector3Stamped.h"
#include "geometry_msgs/Vector3.h"
#include "geometry_msgs/TwistStamped.h"
#include <mavros_extras/OpticalFlowRad.h>
#include "ros/time.h"
#include <mavros_extras/SonarDistance.h>
#include <mavros_extras/LaserDistance.h>
#include <mavros_extras/OffboardRoutePoints.h>
#include <mavros_extras/OffboardRoutePointsConfirm.h>
#include <mavros_extras/PumpStatus.h>
#include <mavros_extras/PumpController.h>
#include <mavros_extras/ExtraFunction.h>
#include <sstream>
#include <math.h>
#include <ros/console.h>
#include <iostream>

using namespace std;

extern MavrosMessage message;
extern bool send_button_pressed;
extern float route_p_send[MAX_POINT_NUM+2][3];// (x,y,z)
extern int route_p_send_total;//number of points to send
extern float yaw_set;
extern int computer_flag;
extern bool test_mode;
extern bool imitate_mode;
extern float start_x;
extern float start_y;

mavros_extras::PumpController pump_msg;
mavros_extras::ExtraFunction extra_msg;
mavros_extras::OffboardRoutePoints route_points_msg;
bool f_equal(float x, float y);
bool send_ok = false;
int transmitted_num = 0;

void chatterCallback_Mode(const mavros::State &msg);
void chatterCallback_Mavlink(const mavros::Mavlink &msg);
void chatterCallback_GlobalPosition_RelAlt(const std_msgs::Float64 &msg);
void chatterCallback_LocalPosition_Local(const geometry_msgs::PoseStamped &msg);
void chatterCallback_WindEstimation(const geometry_msgs::TwistStamped &msg);
void chatterCallback_Battery(const mavros::BatteryStatus &msg);
void chatterCallback_RadioStatus(const mavros::RadioStatus &msg);
void chatterCallback_GPS_Fix(const sensor_msgs::NavSatFix &msg);
void chatterCallback_Imu_Temperature(const sensor_msgs::Temperature &msg);
void chatterCallback_Sonar(const mavros_extras::SonarDistance &msg);
void chatterCallback_Laser(const mavros_extras::LaserDistance &msg);
void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg);
void chatterCallback_Imu_Data(const sensor_msgs::Imu &msg);
void chatterCallback_GlobalPosition_GpVel(const geometry_msgs::TwistStamped &msg);
void chatterCallback_Optical_Flow(const mavros_extras::OpticalFlowRad &msg);
void chatterCallback_Pump_Status(const mavros_extras::PumpStatus &msg);
void chatterCallback_Setpoints_Confirm(const mavros_extras::OffboardRoutePointsConfirm &msg);

void MavrosMessage::run()
{
    //initialize values

    route_points_msg.px_1 = 0.0;
    route_points_msg.py_1 = 0.0;
    route_points_msg.ph_1 = 6.0;
    route_points_msg.px_2 = 0.0;
    route_points_msg.py_2 = 0.0;
    route_points_msg.ph_2 = 6.0;
    route_points_msg.yaw = 0.0;
    route_points_msg.seq = 0;
    route_points_msg.total = 0;


    //接收以下话题
    ros::NodeHandle n;
    ros::Subscriber sub1 = n.subscribe("/offboard/mode_imitate", 10, chatterCallback_Mode);
    //ros::Subscriber sub1 = n.subscribe("/mavros/state", 10, chatterCallback_Mode);
    ros::Subscriber sub2 = n.subscribe("/mavlink/from", 20, chatterCallback_Mavlink);
    ros::Subscriber sub3 = n.subscribe("/mavros/imu/data", 20, chatterCallback_Imu_Data);
    ros::Subscriber sub4 = n.subscribe("/mavros/global_position/rel_alt", 5,chatterCallback_GlobalPosition_RelAlt);
    ros::Subscriber sub5 = n.subscribe("/mavros/global_position/raw/gps_vel", 10,chatterCallback_GlobalPosition_GpVel);
    ros::Subscriber sub6 = n.subscribe("/offboard/position_imitate", 20,chatterCallback_LocalPosition_Local);
    //ros::Subscriber sub6 = n.subscribe("/mavros/local_position/local", 20,chatterCallback_LocalPosition_Local);
    ros::Subscriber sub7 = n.subscribe("/mavros/wind_estimation",10,chatterCallback_WindEstimation);
    ros::Subscriber sub8 = n.subscribe("/mavros/battery",5,chatterCallback_Battery);
    ros::Subscriber sub9 = n.subscribe("/mavros/radio_status",10,chatterCallback_RadioStatus);
    ros::Subscriber sub10 = n.subscribe("/mavros/global_position/raw/fix",20,chatterCallback_GPS_Fix);
    ros::Subscriber sub11 = n.subscribe("/mavros/px4flow/raw/optical_flow_rad",10,chatterCallback_Optical_Flow);
    ros::Subscriber sub12 = n.subscribe("/mavros/imu/temperature",100,chatterCallback_Imu_Temperature);
    //ros::Subscriber sub13 = n.subscribe("/mavros/sonar_receiver/sonar_receiver",20,chatterCallback_Sonar);
    ros::Subscriber sub14 = n.subscribe("/mavros/laser_receiver/laser_receiver",20,chatterCallback_Laser);
    ros::Subscriber sub15 = n.subscribe("/mavros/local_position/local_velocity",20,chatterCallback_local_velocity);
    ros::Subscriber sub16 = n.subscribe("/offboard_route_points_confirm",30,chatterCallback_Setpoints_Confirm);
    //ros::Subscriber sub16 = n.subscribe("/mavros/offboard_route_points_confirm_receiver/offboard_route_points_confirm_receiver",30,chatterCallback_Setpoints_Confirm);
    ros::Subscriber sub17 = n.subscribe("/mavros/pump_status/pump_status", 200,chatterCallback_Pump_Status);

    //Publish Topic
    ros::Publisher offboard_setpoint_pub = n.advertise<mavros_extras::OffboardRoutePoints>("offboard_route_points", 30);
    ros::Publisher pump_controller_pub = n.advertise<mavros_extras::PumpController>("pump_controller",10);
    ros::Publisher extra_function_pub = n.advertise<mavros_extras::ExtraFunction>("extra_function",10);

    ros::Rate check_loop_rate(4);

    while(ros::ok())
    {
        //other sendings
        pump_msg.pump_speed_sp = message.pump.pump_speed_sp;
        pump_msg.spray_speed_sp = message.pump.spray_speed_sp;
        pump_controller_pub.publish(pump_msg);

        extra_msg.laser_height_enable = message.extra_function.laser_height_enable;
        extra_msg.obs_avoid_enable = message.extra_function.obs_avoid_enable;
        extra_msg.add_one = message.extra_function.add_one;
        extra_msg.add_two = message.extra_function.add_two;
        extra_msg.add_three = message.extra_function.add_three;
        extra_function_pub.publish(extra_msg);
        //cout<<pump_msg.spray_speed_sp<<endl;

        //set field values

        if(send_button_pressed)
        {   
            send_ok = false;
            transmitted_num = 0;
            message.success_counter = 0;
            ros::Rate send_loop_rate(12);
            int other_sending_counter = 0;

            while(ros::ok())
            {
                int counter = 0;
                bool failure = false;

                while (ros::ok()&& !send_ok)
                {
                  //other sendings
                  other_sending_counter ++;
                  if(other_sending_counter == 4)
                  {
                      pump_msg.pump_speed_sp = message.pump.pump_speed_sp;
                      pump_msg.spray_speed_sp = message.pump.spray_speed_sp;
                      pump_controller_pub.publish(pump_msg);

                      extra_msg.laser_height_enable = message.extra_function.laser_height_enable;
                      extra_msg.obs_avoid_enable = message.extra_function.obs_avoid_enable;
                      extra_msg.add_one = message.extra_function.add_one;
                      extra_msg.add_two = message.extra_function.add_two;
                      extra_msg.add_three = message.extra_function.add_three;
                      extra_function_pub.publish(extra_msg);

                      other_sending_counter = 0;
                  }

                  //route, x:E->N y:N->W
                  route_points_msg.px_1 = route_p_send[transmitted_num][1] + start_x;
                  route_points_msg.py_1 = -route_p_send[transmitted_num][0] + start_y;
                  route_points_msg.ph_1 = route_p_send[transmitted_num][2];
                  route_points_msg.px_2 = route_p_send[transmitted_num+1][1] + start_x;
                  route_points_msg.py_2 = -route_p_send[transmitted_num+1][0] + start_y;
                  route_points_msg.ph_2 = route_p_send[transmitted_num+1][2];
                  route_points_msg.yaw = yaw_set;
                  route_points_msg.seq = transmitted_num;
                  route_points_msg.total = route_p_send_total;
                  //cout<<"route_p_send[transmitted_num+1][0]"<<route_p_send[transmitted_num+1][0]<<endl;
                  offboard_setpoint_pub.publish(route_points_msg);

                  counter ++;
                  if(f_equal(message.setpoints_receive.px_1,route_points_msg.px_1) && f_equal(message.setpoints_receive.py_1,route_points_msg.py_1)
                          && f_equal(message.setpoints_receive.px_2,route_points_msg.px_2) && f_equal(message.setpoints_receive.py_2,route_points_msg.py_2))
                  {
                      //send successfully
                      transmitted_num += 2;
                      message.success_counter += 2;
                      break;

                  }

                  else if(counter > 100) //send failed
                  {
                      message.success_counter = 0;
                      failure = true;
                      break;
                  }
                  else ;

                  ros::spinOnce();
                  send_loop_rate.sleep();
                }

                message.msg_Send_Setpoints_Confirm();
                if(failure) break;
                if((success_counter-1)==route_p_send_total) break;
            }

             transmitted_num -= 1;
             send_button_pressed = false;
        }

        ros::spinOnce();
        check_loop_rate.sleep();
    }

}

void chatterCallback_Mode(const mavros::State &msg)//模式
{  
  //cout<<msg.mode;
  if(msg.mode=="MANUAL") message.mode="手动模式";
  else if(msg.mode=="ALTCTRL")message.mode="高度控制";
  else if(msg.mode=="POSCTRL") message.mode="位置控制";
  else if(msg.mode=="AUTOCTRL") message.mode="自动模式";
  else if(msg.mode=="LOITER") message.mode="悬停模式";
  else if(msg.mode=="OFFBOARD") message.mode="自动喷洒";
  else  message.mode=msg.mode;
  message.msg_Send_State();
}  

void chatterCallback_Mavlink(const mavros::Mavlink &msg) //mavlink原始信息
{
    if(msg.msgid==24)
       {
        //cout<<msg<<endl;
        message.global_position.gps.satellites=((msg.payload64[3]>>40)&0xFF);
       // cout<<message.global_position.gps.satellites<<endl;
       }
    //if(msg.msgid==105)cout<<msg<<endl;
    //if(msg.msgid==212||msg.msgid==213)cout<<msg<<endl;
    message.msg_Send_GPS_Satellites();
    //if(msg.msgid==225)cout<<"redsadasa"<<endl;
}
void chatterCallback_Imu_Data(const sensor_msgs::Imu &msg)//传感器信息
{
;
}
void chatterCallback_Imu_Temperature(const sensor_msgs::Temperature &msg)//飞控温度
{
    message.temperature=msg.temperature;
    message.msg_Send_Temperature();
}
void chatterCallback_GlobalPosition_RelAlt(const std_msgs::Float64 &msg)//相对高度
{
    //cout<<"A"<<msg.data<<endl;
    message.global_position.rel_altitude=msg.data;
    message.msg_Send_Rel_Alt();
}

void chatterCallback_GlobalPosition_GpVel(const geometry_msgs::TwistStamped &msg)//速度
{
    //cout<<"V"<<msg.vector<<endl;
    message.global_position.vel.x =msg.twist.linear.x;
    message.global_position.vel.y =msg.twist.linear.y;
    message.global_position.vel.z =msg.twist.linear.z;
    message.msg_Send_Velocity();
}
void chatterCallback_LocalPosition_Local(const geometry_msgs::PoseStamped &msg)//当地信息
{
    double q2=msg.pose.orientation.x;
    double q1=msg.pose.orientation.y;
    double q0=msg.pose.orientation.z;
    double q3=msg.pose.orientation.w;
    //cout<<"Speed "<<msg.pose.position<<endl;
    //cout<<" ** "<<q0<<endl<<q1<<endl<<q2<<endl<<q3<<endl;
    message.local_position.orientation.pitch = asin(2*q0*q2-2*q1*q3 );//弧度制
    message.local_position.orientation.roll  = atan2(2*q2*q3 + 2*q0*q1, 1-2*q1*q1-2*q2*q2);
    message.local_position.orientation.yaw = -atan2(2*q1*q2 - 2*q0*q3, -2*q1*q1 - 2*q3*q3 + 1);
    message.local_position.orientation.pitchd=message.local_position.orientation.pitch*57.3;//换为角度制
    message.local_position.orientation.rolld=message.local_position.orientation.roll*57.3;
    message.local_position.orientation.yawd=message.local_position.orientation.yaw*57.3;

    message.global_position.rel_altitude=msg.pose.position.z;
    message.local_position.position.x = msg.pose.position.x;
    message.local_position.position.y = msg.pose.position.y;
    message.local_position.position.z = msg.pose.position.z;
    //cout<<"x"<<msg.pose.position.x<<endl;
    //cout<<"yaw"<<message.local_position.orientation.yaw+3.14159<<endl;
    message.msg_Send_Rel_Alt();
    message.msg_Send_Orientation();

}

void chatterCallback_WindEstimation(const geometry_msgs::TwistStamped &msg)//风速信息
{
    //cout<<"WIND"<<msg.twist<<endl;
    message.wind_speed.linear.x=msg.twist.linear.x;
    message.wind_speed.linear.y=msg.twist.linear.y;
    message.wind_speed.linear.z=msg.twist.linear.z;
    message.wind_speed.angular.x=msg.twist.angular.x;
    message.wind_speed.angular.y=msg.twist.angular.y;
    message.wind_speed.angular.z=msg.twist.angular.z;
}

void chatterCallback_Battery(const mavros::BatteryStatus &msg)//电量信息
{
    //cout<<"Battery"<<msg.voltage<<endl;
    message.battery_voltage=msg.voltage;
    message.msg_Send_Battery();
}
void chatterCallback_RadioStatus(const mavros::RadioStatus &msg)//信号质量
{
    //cout<<"Signal Strength"<<msg.rssi_dbm<<endl;
    message.radio_rssi=msg.rssi_dbm;
    message.msg_Send_Radio();
}
void chatterCallback_GPS_Fix(const sensor_msgs::NavSatFix &msg)//GPS
{
    //cout<<msg<<endl;
    message.global_position.gps.status=msg.status.status;
    message.global_position.gps.x=msg.latitude;
    message.global_position.gps.y=msg.longitude;
    message.global_position.gps.z=msg.altitude;
    message.msg_Send_GPS();
}
void chatterCallback_Optical_Flow(const mavros_extras::OpticalFlowRad &msg)//光流
{
    message.optical_flow.quality=msg.quality;
    message.optical_flow.distance=msg.distance;
    message.optical_flow.temperature=msg.temperature;
    //cout<<msg;
}

void chatterCallback_Sonar(const mavros_extras::SonarDistance &msg)
{
    message.optical_flow.distance=msg.sonar_up/100.0; //changed to sonar msg, clarence, 2015.12.2
    message.optical_flow.temperature=0;
    //cout<<msg;
}

void chatterCallback_Laser(const mavros_extras::LaserDistance &msg)
{
    message.laser_distance.min_distance =msg.min_distance;
    message.laser_distance.angle =msg.angle;
    message.laser_distance.laser_x =msg.laser_x;
    message.laser_distance.laser_y =msg.laser_y;
    message.msg_Send_Laser_Distance();
}

void chatterCallback_local_velocity(const geometry_msgs::Vector3 &msg)
{
    message.local_position.speed.x=msg.x;
    message.local_position.speed.y=msg.y;
    message.local_position.speed.z=msg.z;

    message.msg_Send_Orientation();
}

void chatterCallback_Setpoints_Confirm(const mavros_extras::OffboardRoutePointsConfirm &msg)
{
    message.setpoints_receive.px_1 = msg.px_1;
    message.setpoints_receive.py_1 = msg.py_1;
    message.setpoints_receive.ph_1 = msg.ph_1;
    message.setpoints_receive.px_2 = msg.px_2;
    message.setpoints_receive.py_2 = msg.py_2;
    message.setpoints_receive.ph_2= msg.ph_2;
    message.setpoints_receive.seq = msg.seq;
    message.setpoints_receive.num = msg.total;

    computer_flag = 1;

    cout<<"seq="<<message.setpoints_receive.seq<<endl;

    //message.msg_Send_Setpoints_Confirm();
}


bool f_equal(float x, float y)
{
  if(fabs(x-y)<0.01) return true;
  else return false;
}

void chatterCallback_Pump_Status(const mavros_extras::PumpStatus &msg)
{
    message.pump.pump_speed=msg.pump_speed;
    message.pump.spray_speed=msg.spray_speed;
    message.msg_Send_Pump_Status();
}


