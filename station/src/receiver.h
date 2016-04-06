//*******receiver.h******

#ifndef RECEIVER_H
#define RECEIVER_H

#include "mainwindow.h"
#include <iostream>
#include <string>

#include <QApplication>
#include <QThread>

using namespace std;

struct Imu_Data
{
};
struct Extra_Function
{
    unsigned short obs_avoid_enable;
    unsigned short laser_height_enable;
    unsigned short add_one;
    unsigned short add_two;
    unsigned short add_three;
};

struct Laser_Distance
{
    float min_distance;
    float angle;
    float laser_x;
    float laser_y;
};

struct Pump
{
    float pump_speed_sp;
    float spray_speed_sp;
    float pump_speed;
    float spray_speed;
};

struct Global_Position
{
    struct GPS
    {
      double x;
      double y;
      double z;
      int status;
      unsigned int satellites;
    }gps;
    struct Velocity//by FCU
    {
        double x;
        double y;
        double z;
    }vel;
    double rel_altitude;
    double compass_hdg;
};

struct Local_Position
{
    struct Orientation
    {
        double x;//四元数
        double y;
        double z;
        double w;
        double pitch,pitchd;//d为角度制
        double roll,rolld;
        double yaw,yawd;
    }orientation;
    struct Position
    {
        double x;
        double y;
        double z;
    }position;
    struct Speed
    {
        double x;
        double y;
        double z;
    }speed;
};

struct Wind_Speed
{
  struct Linear
  {
      double x;
      double y;
      double z;
  }linear;
  struct Angular
  {
      double x;
      double y;
      double z;
  }angular;
};

struct Optical_Flow
{
    double distance;
    double temperature;
    double quality;
};

struct Offboard_Setpoints
{
    float px_1;
    float py_1;
    float ph_1;
    float px_2;
    float py_2;
    float ph_2;
    float yaw;
    int seq;
    int total;
};

struct Setpoints_Receive
{
    float px_1;
    float py_1;
    float ph_1;
    float px_2;
    float py_2;
    float ph_2;
    int seq;
    int num;
};

class MavrosMessage : public QThread
{
    Q_OBJECT
public:
    void run();

    /*********飞行参数变量**********/
    string mode;
    struct Imu_Data imu_data;
    struct Global_Position global_position;
    struct Local_Position local_position;
    double battery_voltage;
    double radio_rssi;
    double temperature;
    double time_now;
    double time_fromboost;
    struct Wind_Speed wind_speed;
    struct Optical_Flow optical_flow;

    struct Pump pump;
    struct Offboard_Setpoints setpoints_send;
    struct Setpoints_Receive setpoints_receive;
    struct Extra_Function extra_function;
    struct Laser_Distance laser_distance;

    int success_counter;


    /*******信号发射函数******/
    void msg_Send_State()const {emit state_Mode_Signal();}
    void msg_Send_GPS()const {emit global_GPS_Signal();}
    void msg_Send_Battery()const{emit battery_Signal();}
    void msg_Send_Radio()const{emit radio_Signal();}
    void msg_Send_Velocity()const{emit global_Velocity_Signal();}
    void msg_Send_Rel_Alt()const{emit global_Rel_Alt_Signal();}
    void msg_Send_Orientation()const{emit local_Orientation_Signal();}
    void msg_Send_GPS_Satellites()const{emit global_GPS_Satellites_Signal();}
    void msg_Send_Laser_Distance()const{emit laser_Distance_Signal();}
    void msg_Send_Imu_Data()const{emit imu_Data_Signal();}
    void msg_Send_Temperature()const{emit temperature_Signal();}
    void msg_Send_Time()const{emit time_Signal();}
    void msg_Send_Setpoints_Confirm()const{emit setpoints_Confirm_Signal();}
    void msg_Send_Pump_Status()const{emit pump_Status_Signal();}

    /**********信号**********/
signals:
    void state_Mode_Signal()const;
    void global_GPS_Signal()const;
    void global_GPS_Satellites_Signal()const;
    void battery_Signal()const;
    void radio_Signal()const;
    void global_Velocity_Signal()const;
    void global_Compass_Signal()const;
    void global_Rel_Alt_Signal()const;
    void local_Orientation_Signal()const;
    void laser_Distance_Signal()const;
    void imu_Data_Signal()const;
    void temperature_Signal()const;
    void time_Signal()const;
    void offboard_Set_Signal()const;
    void setpoints_Confirm_Signal()const;
    void pump_Status_Signal()const;
};


#endif
