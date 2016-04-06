//******mainwindow.h*******

//bug: 空中飞行时输入航点画实际轨迹有偏离

#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <string>
#include <QListWidgetItem>
#include <iostream>
#include <fstream>
#include "painterWiget.h"
#include <QTime>
#include <QTimer>
#include <QDateTime>
#include <QWheelEvent>
#include <QMouseEvent>
#include <QKeyEvent>
#include <QListWidgetItem>
#include <QLabel>
#include <QFileDialog>
#include <QPainter>
#include <math.h>
#include <QBitmap>
#include <QPainter>
#include <QMessageBox>
#include <QDir>
#include <QCloseEvent>


#define FLY_POSITION_LABEL_WIDTH 720
#define FLY_POSITION_LABEL_HEIGHT 540
#define FLY_ROUTE_LABEL_WIDTH 720
#define FLY_ROUTE_LABEL_HEIGHT 540

#define SUCCESS_COUNTER_INIT 100

#define NORTHERN_HEMISPHERE 1
#define EASTERN_HEMISPHERE 1

#define ROUTE_DRAW_AREA_WIDTH 720
#define ROUTE_DRAW_AREA_HEIGHT 540

#define DEG_TO_RAD 	0.01745329251994
#define RAD_TO_DEG 	57.2957795130823

#define PI 3.14159265358979323846
#define PI_2 1.57079632679489661923

#define DBL_EPSILON 2.2204460492503131e-16
#define CONSTANTS_RADIUS_OF_EARTH 6371393

#define MAX_POINT_NUM 1000
#define MAX_DIRACTION_POINT_NUM 200


using namespace std;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

    StatusPainter *status_painter;//用于画图的类指针
    QPixmap *compass_arrow_pixmap;

protected:
    void paintEvent(QPaintEvent *event);
    void closeEvent(QCloseEvent *event);
    void get_Painter_Address(StatusPainter *painter);//用于画图传参的函数

private slots:
    void init_paras();

    void state_Mode_Slot();//模式控制显示
    void battery_Slot();//电量显示
    void radio_Status_Slot();//信号强度显示
    void global_Velocity_Slot();//速度显示
    void global_GPS_Slot();//GPS信息显示
    void global_GPS_Satellites_Slot();//GPS星数
    void global_Rel_Alt_Slot();//相对高度显示
    void local_Position_Slot();//姿态显示
    void laser_Distance_Slot();//光流显示
    void temperature_Slot();//温度显示
    void time_Slot();//飞行时间显示
    void setpoints_Confirm_Slot();
    void pump_Status_Slot();
    void timer_Slot();

    //void on_comboBox_Num_currentIndexChanged(int index);

    void time_Update();

    void on_pushButton_Reset_FlyingTime_clicked();
    void on_pushButton_Route_Send_clicked();
    int on_pushButton_Route_Generate_clicked();

    void on_pushButton_Open_Fence_clicked();
    void on_pushButton_Open_Diraction_clicked();
    void on_pushButton_Delete_Point_clicked();
    void on_pushButton_Restore_Point_clicked();
    void on_listWidget_GPS_Point_itemClicked();

    void delete_point(int x);
    bool restore_point();
    void draw_gps_fence();
    void draw_route(int window); //window 0: draw in planning, window 1: draw in flying window, window 2: draw in time route in flying window, window 3: draw common route in planning window

    void gps_to_local(double lat, double lon, float *x, float *y);
    void local_to_gps(float x, float y, double *lat, double *lon);
    float point_dist(float x1, float y1, float x2, float y2);
    float point_line_dist(float m, float n, float k, float b);

    void record_home_gps();

    void turn_point_cal();

    int record_break_point();

    void record_start_p();

    void on_dial_Offset_Angle_valueChanged(int value);

    int on_pushButton_Save_Config_clicked();

    int on_pushButton_Open_Break_Point_clicked();

    void break_point_cal();

    void on_pushButton_Break_Paras_Update_clicked();

    int read_saved_paras();

    void on_pushButton_Restore_Config__clicked();

    void on_pushButton_Route_Generate_Common_clicked();

    void common_flight_cal(float length, float width, float height, int times, bool left_side);

private:
    Ui::MainWindow *ui;
    QTime system_time;
    QImage image_resize;

    QPoint mouse_pos;

    QListWidgetItem *item_cp1;
    QListWidgetItem *item_cp2;
    QListWidgetItem *item_cp3;

    QLabel *fly_position_label;
    QLabel *fly_route_label;


    /*fence*/
    double gps_fence[MAX_POINT_NUM][3]; //(lat, lon, initial_sequence)
    double gps_fence_cp1[MAX_POINT_NUM][3];
    double gps_fence_cp2[MAX_POINT_NUM][3];
    double gps_fence_cp3[MAX_POINT_NUM][3];

    int gps_num;//start from 0
    int gps_num_cp1;
    int gps_num_cp2;
    int gps_num_cp3;   
    float gps_fence_local[MAX_POINT_NUM][2]; //local: East->x, North->y

    /*diraction*/
    double gps_diraction[MAX_DIRACTION_POINT_NUM][2]; //(lat, lon)
    float diraction_k;
    int diraction_p_num;

    /*home position*/
    double home_lat;
    double home_lon;

    /*distance between lines*/
    float dist_between_lines;

    /*intersection points, local*/
    float intersection_p_local[MAX_POINT_NUM][2];
    float route_p_local[MAX_POINT_NUM][2];//local: East->x, North->y
    double route_p_gps[MAX_POINT_NUM][2];

    int intersection_num;

    /*for common flight mode*/
    float common_length;
    float common_width;
    float common_height;
    int common_times;
    bool common_side; //left: true; right: false
    bool common_mode; //used when drawing route to calculate different painting scale

    /*for break point*/
    bool break_point_flag1;
    bool break_point_flag2;
    int intersection_num_last;
    double route_p_gps_last[MAX_POINT_NUM][2];
    int gps_num_last;
    double gps_fence_last[MAX_POINT_NUM][3];
    double break_point_lat;
    double break_point_lon;
    int break_position_num;
    double route_p_gps_read[MAX_POINT_NUM][2];
    float route_p_local_read[MAX_POINT_NUM][2];
    int route_p_num_read;
    double break_point_lat_read;
    double break_point_lon_read;
    int break_point_seq_read;


    /*route offset*/
    float offset_angle_d;
    float offset_dist_m;
    float measure_compensation_m;
    float spray_length;
    float spray_width;

    /*item sequence for listwidget*/
    int list_seq;
    int list_seq_cp1;
    int list_seq_cp2;
    int list_seq_cp3;

    /*others*/
    int success_counter;

    int controller_flag;
    int controller_flag_last;

    double orientation_last;

    bool bool_flying;

    bool controller_working;

    float flying_height;
    float take_off_height;

    int flying_time;
    unsigned int flying_status_counter;
    unsigned int flying_status_counter_last;

    int time_counter;

    bool battery_low;

    int route_plan_mode; // 0: GPS fence mode 1:common mode 2:GPS fence break point mode 3:common break point mode

    //以下变量用于画路径图
    float paint_scale ;
    float real_position[6000][2];
    int position_num;
    int save_counter;

    float fly_distance;

};

#endif // MAINWINDOW_H
