//********mianwindow.cpp*********

#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "receiver.h"
#include "QDesktopWidget"


extern MavrosMessage message;

bool test_mode = true;
bool imitate_mode = false;

int choice=0;//选择显示的图片
int computer_flag = 0;//used in receiver.cpp, changed here
bool send_button_pressed = false;//used in receiver.cpp, changed here
float route_p_send[MAX_POINT_NUM+2][3];//used in receiver.cpp, changed here. (x,y,z)
int route_p_send_total = 0;//used in receiver.cpp, changed here. number of points to send
float yaw_set=0.0; //used in receiver.cpp, changed here. yaw to send
float start_x = 0.0; //used in receiver.cpp, changed here.
float start_y = 0.0; //used in receiver.cpp, changed here.
float draw_start_x = 0.0;
float draw_start_y = 0.0;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);

   /*
    QDesktopWidget *d = QApplication::desktop();//屏幕大小捕获
    int w = d->width();     // 返回桌面宽度
    int h = d->height();    // 返回桌面高度
    setMaximumSize(w,h);
    setMinimumSize(w,h);
    */

    //定时器
    QTimer *timer=new QTimer(this);
    timer->start(1000);//每秒更新一次

    //信号与槽连接
    QObject::connect(timer,SIGNAL(timeout()),this,SLOT(time_Update()));
    QObject::connect(&message,SIGNAL(state_Mode_Signal()),this,SLOT(state_Mode_Slot()));
    QObject::connect(&message,SIGNAL(global_GPS_Signal()),this,SLOT(global_GPS_Slot()));
    QObject::connect(&message,SIGNAL(global_GPS_Satellites_Signal()),this,SLOT(global_GPS_Satellites_Slot()));
    QObject::connect(&message,SIGNAL(battery_Signal()),this,SLOT(battery_Slot()));
    QObject::connect(&message,SIGNAL(radio_Signal()),this,SLOT(radio_Status_Slot()));
    QObject::connect(&message,SIGNAL(global_Velocity_Signal()),this,SLOT(global_Velocity_Slot()));
    QObject::connect(&message,SIGNAL(global_Rel_Alt_Signal()),this,SLOT(global_Rel_Alt_Slot()));
    QObject::connect(&message,SIGNAL(local_Orientation_Signal()),this,SLOT(local_Position_Slot()));
    QObject::connect(&message,SIGNAL(laser_Distance_Signal()),this,SLOT(laser_Distance_Slot()));
    QObject::connect(&message,SIGNAL(temperature_Signal()),this,SLOT(temperature_Slot()));
    QObject::connect(&message,SIGNAL(time_Signal()),this,SLOT(time_Slot()));
    QObject::connect(&message,SIGNAL(setpoints_Confirm_Signal()),this,SLOT(setpoints_Confirm_Slot()));
    QObject::connect(&message,SIGNAL(pump_Status_Signal()),this,SLOT(pump_Status_Slot()));


    init_paras();

    //set cout precision
    cout<<fixed;
    cout.precision(8);


    ui->tabWidget->setCurrentIndex(0);

    //pitch、roll、yaw绘图仪表初始化
    StatusPainter *painter = new StatusPainter();
    ui->tabWidget_PaintArea->addTab(painter,"仪表显示");//添加新的画图区widget new StatusPainter(QColor(141,238,238))
    //if(choice==0)ui->tabWidget_PaintArea->removeTab(0);
    ui->tabWidget_PaintArea->setCurrentIndex(1);//显示第二页
    get_Painter_Address(painter);

    //字体字号颜色设定
    QFont font1("宋体",12,QFont::Bold);
    ui->label_Mode->setFont(font1);


    QPalette palette1;//红色
    palette1.setColor(QPalette::Text,QColor(255,0,0));
    ui->label_Mode->setPalette(palette1);
    ui->label_Tips->setPalette(palette1);
    ui->label_Warning_Area->setPalette(palette1);

    QPalette palette2;//蓝色
    palette2.setColor(QPalette::Text,QColor(0,0,255));
    ui->lineEdit_GPS_X->setPalette(palette2);
    ui->lineEdit_GPS_Y->setPalette(palette2);
    ui->lineEdit_GPS_Z->setPalette(palette2);
    ui->lineEdit_Rel_Alt->setPalette(palette2);
    ui->lineEdit_GPS_Satellites->setPalette(palette2);
    ui->lineEdit_Battery->setPalette(palette2);


    //飞行路径图背景设置
    ui->frame_Fly_Route->setFrameStyle(1);
    ui->frame_Fly_Route->setFixedSize(FLY_ROUTE_LABEL_WIDTH,FLY_ROUTE_LABEL_HEIGHT);

    QPalette   palette3;
    QPixmap pixmap3(":/icon/Icons/grass-720x540-2.png");//背景图片
    palette3.setBrush(ui->frame_Fly_Route-> backgroundRole(),QBrush(pixmap3));
    ui->frame_Fly_Route->setPalette(palette3);
    ui->frame_Fly_Route->setMask(pixmap3.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Route->setAutoFillBackground(true);

    fly_route_label = new QLabel(ui->frame_Fly_Route);
    fly_route_label->setFixedWidth(FLY_ROUTE_LABEL_WIDTH);
    fly_route_label->setFixedHeight(FLY_ROUTE_LABEL_HEIGHT);
    fly_route_label->move(0,0);


    ui->frame_Fly_Position->setFrameStyle(1);
    ui->frame_Fly_Position->setFixedSize(FLY_POSITION_LABEL_WIDTH,FLY_POSITION_LABEL_HEIGHT);

    QPalette   palette4;
    QPixmap pixmap4(":/icon/Icons/grass-720x540-2.png");//背景图片
    palette4.setBrush(ui->frame_Fly_Position-> backgroundRole(),QBrush(pixmap4));
    ui->frame_Fly_Position->setPalette(palette4);
    ui->frame_Fly_Position->setMask(pixmap4.mask());  //可以将图片中透明部分显示为透明的
    ui->frame_Fly_Position->setAutoFillBackground(true);

    fly_position_label = new QLabel(ui->frame_Fly_Position);
    fly_position_label->setFixedWidth(FLY_POSITION_LABEL_WIDTH);
    fly_position_label->setFixedHeight(FLY_POSITION_LABEL_HEIGHT);
    fly_position_label->move(0,0);

    //设置是否可用
    ui->pushButton_Route_Generate->setEnabled(true);
    ui->pushButton_Route_Send->setEnabled(false);
    ui->pushButton_Delete_Point->setEnabled(false);
    ui->pushButton_Restore_Point->setEnabled(false);
    ui->pushButton_Break_Paras_Update->setEnabled(false);

    ui->progressBar_GPS->setRange(0,15);
    ui->progressBar_Battery->setRange(190,245);
    ui->progressBar_RC->setRange(0,200);
    ui->progressBar_RC->setValue(160);

    //set conection display
    ui->label_Controller->setStyleSheet("background-color:red");
    ui->label_Computer->setStyleSheet("background-color:red");

    //offset initialize
    ui->dial_Offset_Angle->setValue(180);
    ui->lineEdit_Offset_Dist->setText(QString::number(0.0));

    //other line edit
    ui->lineEdit_Flying_Height->setText(QString::number(flying_height));
    ui->lineEdit_Take_Off_Height->setText(QString::number(take_off_height));
    ui->lineEdit_Measure_Compensation->setText(QString::number(measure_compensation_m));
    ui->lineEdit_Spray_Width->setText(QString::number(spray_width));
    ui->lineEdit_Spray_Length->setText(QString::number(spray_length));

    //set limitation
    ui->lineEdit_Flying_Height->setValidator(new QDoubleValidator(0.0,6.0,2,this));
    ui->lineEdit_Take_Off_Height->setValidator(new QDoubleValidator(0.0,40.0,2,this));
    ui->lineEdit_Offset_Dist->setValidator(new QDoubleValidator(0.0,100.0,2,this));
    ui->lineEdit_Measure_Compensation->setValidator(new QDoubleValidator(-100.0,100.0,2,this));
    ui->lineEdit_Spray_Width->setValidator(new QDoubleValidator(0.0,10.0,2,this));
    ui->lineEdit_Spray_Length->setValidator(new QDoubleValidator(0.0,10.0,2,this));

    //set extra function
    ui->checkBox_Auto_Height->setChecked(message.extra_function.laser_height_enable);

    if(message.extra_function.obs_avoid_enable == 2)
        ui->radioButton_Auto_Avoid->setChecked(true);
    else if(message.extra_function.obs_avoid_enable == 1)
        ui->radioButton_Mannual_Avoid->setChecked(true);
    else ui->radioButton_Mannual_Avoid->setChecked(true);

    ui->horizontalSlider_Spray->setValue((int)(message.pump.pump_speed_sp*10));

    //set common flight settings
    ui->radioButton_Left_Common->setChecked(true);

}


MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::init_paras()
{
    //value initialize
    gps_num = 0;
    gps_num_cp1 = 0;
    gps_num_cp2 = 0;
    gps_num_cp3 = 0;

    battery_low = false;

    diraction_p_num = 0;
    diraction_k = 0.0;

    offset_angle_d = 30.0;
    offset_dist_m = 2.0;
    measure_compensation_m = 1.0;

    spray_length = 1.6;
    spray_width = 3.0;

    list_seq = 0;
    list_seq_cp1 = 0;
    list_seq_cp2 = 0;
    list_seq_cp3 = 0;

    if(!test_mode)
    {
        home_lat = 0.0;
        home_lon = 0.0;
    }
    else{
        home_lat = 31.027528;
        home_lon = 121.444228;
        break_point_lat = 31.027428;
        break_point_lon = 121.444350;
        break_position_num = 4;
    }


    dist_between_lines = spray_width;

    intersection_num = 0;

    success_counter = SUCCESS_COUNTER_INIT;

    take_off_height = 4.0;
    flying_height = 4.0;

    controller_flag=0;
    controller_flag_last=0;

    orientation_last=0;

    bool_flying=false;

    break_point_flag1 = false;
    break_point_flag2 = false;

    flying_time=0;
    flying_status_counter=0;
    flying_status_counter_last=0;

    time_counter = 0;

    //以下变量用于画路径图
    paint_scale = 1.0;
    position_num = 0;
    save_counter = 0;

    fly_distance = 0;

    message.extra_function.laser_height_enable = 0;
    message.extra_function.obs_avoid_enable = 0;
    message.extra_function.add_one = 0;
    message.extra_function.add_two = 0;
    message.extra_function.add_three = 0;

    message.pump.pump_speed_sp = 0.0;
    message.pump.spray_speed_sp = 0.6;

    message.global_position.gps.x = 0.0;
    message.global_position.gps.y =0.0;

    message.local_position.orientation.yaw = PI_2;


    gps_diraction[0][0] = 0.0;
    gps_fence[0][0] = 0.0;

    /*for common flight mode*/
    common_height = 2.5;
    common_length = -1.0;
    common_width = -1.0;
    common_times = -1;
    common_side = true;
    common_mode = false;

    route_plan_mode = 0;

    controller_working = false;

    read_saved_paras();
}

int MainWindow::read_saved_paras()
{
    char dir_path[80]="/home/cc/catkin_ws/src/break_point";
    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)
    {
        cout<<"no config file found!"<<endl;
        return 0;
    }

    QString fileName = "/home/cc/catkin_ws/src/break_point/config.txt";
    fstream config_f;
    char *path = fileName.toLatin1().data();
    config_f.open(path,ios::in);

    int read_counter=0;
    while(!config_f.eof())
    {   //while not the end of file
        char str[30];
        config_f >> str;
        /*values:  take_off_height spray_length spray_width
          message.extra_function.laser_height_enable message.extra_function.obs_avoid_enable
          message.pump.pump_speed_sp*/
        float fnum = str[1]-'0'+ (str[3]-'0')/10.0;
        switch(read_counter)
        {
        case 0:
            take_off_height = fnum;
            break;
        case 1:
            spray_length = fnum;
            break;
        case 2:
            spray_width = fnum;
            break;
        case 3:
            message.extra_function.laser_height_enable = (int)fnum;
            break;
        case 4:
            message.extra_function.obs_avoid_enable = (int)fnum;
            break;
        case 5:
            message.pump.pump_speed_sp = fnum;
            break;
        default:
            break;
        }
        read_counter ++;
    }
        config_f.close(); //reading finished
}

void MainWindow::closeEvent(QCloseEvent *event)
{
    QMessageBox::StandardButton button;
    button = QMessageBox::question(this, tr("退出程序"),
                                   QString(tr("确认退出程序?(请勿在自动喷洒过程中退出)")),
                                   QMessageBox::Yes | QMessageBox::No);
    if (button == QMessageBox::No) {
          event->ignore();  //忽略退出信号，程序继续运行
    }
    else if (button == QMessageBox::Yes) {
          event->accept();  //接受退出信号，程序退出
    }
}

/****显示消息槽****/
void MainWindow::state_Mode_Slot()
{
    ui->label_Mode->setText(QString::fromStdString(message.mode));
    if(controller_flag<=10000)controller_flag+=1;
    else controller_flag=1;

    if(message.mode=="自动喷洒")
    {
        break_point_flag1 = true;
        break_point_flag2 = false;
    }
    else break_point_flag2 = true;

    if(break_point_flag1 && break_point_flag2)
    {
        record_break_point();
        break_point_flag1 = false;
    }
}

void MainWindow::battery_Slot()
{
    ui->lineEdit_Battery->setText(QString::number(message.battery_voltage));
    ui->progressBar_Battery->setValue(((message.battery_voltage>24.5)?24.5:message.battery_voltage)*10);
    if(message.battery_voltage < 19.8) battery_low = true;
    else battery_low = false;
}

void MainWindow::radio_Status_Slot()
{
    ui->lineEdit_Radio->setText(QString::number(message.radio_rssi));
    ui->progressBar_RC->setValue(((message.radio_rssi+240)>200)?200:(message.radio_rssi+240));
}

void MainWindow::global_GPS_Slot()
{
    ui->lineEdit_GPS_X->setText(QString::number(message.global_position.gps.x));
    ui->lineEdit_GPS_Y->setText(QString::number(message.global_position.gps.y));
    ui->lineEdit_GPS_Z->setText(QString::number(message.global_position.gps.z));
}
void MainWindow::global_Velocity_Slot()
{

    ui->lineEdit_Velocity_X->setText(QString::number(message.global_position.vel.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.global_position.vel.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.global_position.vel.z));

}

void MainWindow::global_Rel_Alt_Slot()
{
    if(message.extra_function.laser_height_enable==0)ui->lineEdit_Rel_Alt->setText(QString::number(message.global_position.rel_altitude));
}

void MainWindow::global_GPS_Satellites_Slot()
{
    ui->lineEdit_GPS_Satellites->setText(QString::number(message.global_position.gps.satellites));
    ui->progressBar_GPS->setValue(((message.global_position.gps.satellites>15)?15:message.global_position.gps.satellites));
}

void MainWindow::local_Position_Slot()
{
    //去抖动
    if(fabs(message.local_position.orientation.pitchd+message.local_position.orientation.rolld+message.local_position.orientation.yawd-orientation_last)>=2.0)
    {
        status_painter->pitchd=message.local_position.orientation.pitchd;
        status_painter->rolld=message.local_position.orientation.rolld;
        status_painter->pitch=message.local_position.orientation.pitch;
        status_painter->roll=message.local_position.orientation.roll;
        status_painter->compassd=-message.local_position.orientation.yawd+180;
        status_painter->update();
    }

    ui->lineEdit_Orientation_Pitchd->setText(QString::number(message.local_position.orientation.pitchd));
    ui->lineEdit_Orientation_Rolld->setText(QString::number(message.local_position.orientation.rolld));
    ui->lineEdit_Orientation_Yawd->setText(QString::number(message.local_position.orientation.yawd));

    //竖直速度显示，这里的local_position.消息为速度
    ui->lineEdit_Velocity_X->setText(QString::number(message.local_position.speed.x));
    ui->lineEdit_Velocity_Y->setText(QString::number(message.local_position.speed.y));
    ui->lineEdit_Velocity_Z->setText(QString::number(message.local_position.speed.z));
    ui->lineEdit_Velocity->setText(QString::number(sqrt(message.local_position.speed.x*message.local_position.speed.x+message.local_position.speed.y*message.local_position.speed.y)));

    //判断是否起飞
    if(fabs(message.local_position.position.z)>=1.0)
        bool_flying=true;
    if(fabs(message.local_position.position.z)<1.0)
    {
        flying_status_counter+=1;
        if(flying_status_counter>200)
        {
            flying_status_counter=0;
            bool_flying = false;
        }
    }

    //local_position画当前位置图
    save_counter++;
    if(message.mode!="自动喷洒")
    {
        position_num = 0; //退出offboard则清零等待重新计数
        fly_distance = 0; //距离清零
    }
    else if(save_counter>=1) //draw every 2 points
    {
         //translate coordinate
         real_position[position_num][1]= message.local_position.position.x; //N: x->y
         real_position[position_num][0]= -message.local_position.position.y; //W->E: y->x
         cout<<"real_position[position_num][0]"<<real_position[position_num][0]<<endl;

         if(position_num>0) draw_route(2); //draw

         //计数器设置
         save_counter = 0;
         position_num += 1;

         //calculate total flying distance
         if(position_num>0)
         {
             fly_distance += point_dist(real_position[position_num][0],real_position[position_num][1],real_position[position_num+1][0],real_position[position_num+1][1]);
             ui->lineEdit_Total_Distance->setText(QString::number(fly_distance));            
         }
    }
    else ;
}

void MainWindow::laser_Distance_Slot()
{
    if(message.extra_function.laser_height_enable==1)
        ui->lineEdit_Rel_Alt->setText(QString::number(-message.laser_distance.laser_x));
    ui->lineEdit_Obstacle_Distance->setText(QString::number(message.laser_distance.min_distance/100));
}

void MainWindow::temperature_Slot()
{
    ui->lineEdit_Temperature->setText(QString::number(message.temperature));
}

void MainWindow::time_Slot()
{
;
}


void MainWindow::paintEvent(QPaintEvent *event) /*****主界面绘图槽******/
{
    //QPainter mainwindow_painter(this);
    //mainwindow_painter.drawLine(QPoint(0,0),QPoint(100,100));
}

void MainWindow::get_Painter_Address(StatusPainter *painter)
{
    status_painter=painter;
}

void MainWindow::time_Update()
{
    system_time=QTime::currentTime();
    char time_temp[20];
    sprintf(time_temp,"%d:%d:%d",system_time.hour(),system_time.minute(),system_time.second());
    ui->label_System_Time->setText(time_temp);

    int minutes;
    int seconds;
    if(bool_flying)//飞行时间显示
    {
        flying_time+=1;
        minutes=flying_time/60;
        seconds=flying_time%60;
        char flying_time_temp[20];
        sprintf(flying_time_temp,"%d:%d",minutes,seconds);
        ui->label_Flying_Time->setText(flying_time_temp);
    }
    time_counter ++;
    if(time_counter % 3 ==1)
    {
        timer_Slot();
    }
    if(time_counter % 5 ==1 && !test_mode)
    {
        home_lat = 0; //GPS connection lost
        time_counter = 0;
    }

    if(message.setpoints_receive.num == 0)
    {
        ui->textBrowser_Offboard_Message->append(tr("无航点信息，请点击发送!"));
    }

}

void MainWindow::timer_Slot()
{
    if(controller_flag != controller_flag_last)
    {
        ui->label_Controller->setStyleSheet("background-color:green");
        controller_working = true;
    }
    else
    {
        ui->label_Controller->setStyleSheet("background-color:red");
        computer_flag = 0;
        controller_working = false;
    }

    if(computer_flag != 0)
        ui->label_Computer->setStyleSheet("background-color:green");
    else
        ui->label_Computer->setStyleSheet("background-color:red");

    controller_flag_last=controller_flag;

    if(battery_low) ui->label_Warning_Area->setText("警告! 电量不足, 请立即返航");
    else ui->label_Warning_Area->setText(" ");
}

void MainWindow::on_pushButton_Reset_FlyingTime_clicked()
{
    flying_time=0;
    bool_flying=false;
}


int MainWindow::on_pushButton_Route_Generate_clicked()
{
    //record home gps position
    if(!test_mode) record_home_gps();

    if(home_lat == 0 || (!controller_working && !test_mode))
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未连接到飞机或无GPS信号", QMessageBox::Cancel, NULL);
        message_box.exec();
        return 1;
    }
    else
    {
        //判断是否所有值都写好
        if(gps_diraction[0][0]!=0.0 && gps_fence[0][0]!= 0)
        {
            //if distance from home to first gps fence point is too long, reject
            float gps_fence_local_x;
            float gps_fence_local_y;
            gps_to_local(gps_fence[0][0], gps_fence[0][1], &gps_fence_local_x, &gps_fence_local_y);

            //if distance from home to first diraction point is too long, reject
            float diraction_local_x;
            float diraction_local_y;
            gps_to_local(gps_diraction[0][0], gps_diraction[0][1], &diraction_local_x, &diraction_local_y);

            if(point_dist(0.0, 0.0, gps_fence_local_x, gps_fence_local_y) > 5000)
            {
                QMessageBox message_box(QMessageBox::Warning,"警告","GPS围栏距离太远(>5km)", QMessageBox::Cancel, NULL);
                message_box.exec();
                return 2;
            }
            else if(point_dist(0.0, 0.0, diraction_local_x, diraction_local_y)>5000)
            {
                QMessageBox message_box(QMessageBox::Warning,"警告","GPS方向点距离太远(>5km)", QMessageBox::Cancel, NULL);
                message_box.exec();
                return 3;
            }
            else  //all correct
            {
                offset_dist_m = ui->lineEdit_Offset_Dist->text().toFloat();
                flying_height = ui->lineEdit_Flying_Height->text().toFloat();
                int value = ui->dial_Offset_Angle->value();
                offset_angle_d = (float)(- value + 270);
                record_start_p();
                turn_point_cal(); //calculate
            }

        }
        else if(gps_fence[0][0]==0) {
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS围栏未导入", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
        else if(gps_diraction[0][0]==0){
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS方向未导入", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
    }


    ui->textBrowser_Offboard_Message->append("生成成功！");
    ui->pushButton_Route_Send->setEnabled(true);
    return 0;
}


void MainWindow::on_pushButton_Route_Send_clicked()
{
    if(message.mode=="自动喷洒")
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","自动喷洒中，无法发送，请先切换到手动！", QMessageBox::Cancel, NULL);
        message_box.exec();
        if(test_mode)record_break_point(); //for test
    }
    else
    {
        /*only points which have been sent to UAV can be used as break point record*/
        memcpy(gps_fence_last,gps_fence,24000);
        memcpy(route_p_gps_last,route_p_gps,16000);

        gps_num_last = gps_num;//store here for break point
        intersection_num_last = intersection_num;//store here for break point

        if(test_mode)record_break_point(); //for test
        //insert 2 take off setpoints
        route_p_send[0][0] = 0.0; //x
        route_p_send[0][1] = 0.0; //y
        route_p_send[0][2] = take_off_height; //h
        route_p_send[1][0] = route_p_local[0][0]; //x
        route_p_send[1][1] = route_p_local[0][1]; //y
        route_p_send[1][2] = take_off_height; //h
        route_p_send_total = intersection_num + 2;
        cout<<"route_p_send_total"<<route_p_send_total<<endl;

        //insert route points
        for(int i=0;i<=intersection_num;i++)
        {
           route_p_send[i+2][0] = route_p_local[i][0];
           route_p_send[i+2][1] = route_p_local[i][1];
           route_p_send[i+2][2] = flying_height;
           cout<<i<<" "<<route_p_send[i+2][0]<<endl;
        }
        //turn to even number to correct a bug, start from 0
        if(route_p_send_total % 2 == 0)
        {
            route_p_send_total += 1;
            route_p_send[route_p_send_total][0] = route_p_local[intersection_num][0];
            route_p_send[route_p_send_total][1] = route_p_local[intersection_num][1];
            route_p_send[route_p_send_total][2] = flying_height;
        }
        if(route_plan_mode==1 || route_plan_mode==3) common_mode = true;
        else common_mode = false;

        draw_route(1);

        send_button_pressed = true;
        ui->textBrowser_Offboard_Message->append("发送中...");
    }
}


void MainWindow::setpoints_Confirm_Slot()
{
    if(message.success_counter > 0 && message.success_counter < 4)
        ui->textBrowser_Offboard_Message->append(tr("发送中..")+QString::number(message.success_counter)+tr("/")+QString::number(route_p_send_total+1));
    else if(message.success_counter >=4 )
        ui->textBrowser_Offboard_Message->append(tr("可以起飞!")+QString::number(message.success_counter)+tr("/")+QString::number(route_p_send_total+1));
    else
        ui->textBrowser_Offboard_Message->append(tr("发送中断!"));
}

void MainWindow::pump_Status_Slot()
{
    ui->lineEdit_Spray_Speed->setText(QString::number(message.pump.spray_speed));
}

void MainWindow::draw_gps_fence()
{

    //find min,max and calculate scale
    double lat_max = gps_fence[0][0];
    double lat_min = gps_fence[0][0];
    double lon_max = gps_fence[0][1];
    double lon_min = gps_fence[0][1];
    for(int i=1;i<=gps_num;i++)
    {
        if(gps_fence[i][0]>lat_max) lat_max = gps_fence[i][0];
        if(gps_fence[i][0]<lat_min) lat_min = gps_fence[i][0];
        if(gps_fence[i][1]>lon_max) lon_max = gps_fence[i][1];
        if(gps_fence[i][1]<lon_min) lon_min = gps_fence[i][1];
    }
    float scale_lat = (FLY_ROUTE_LABEL_HEIGHT-80)/(lat_max-lat_min); //40 for edge
    float scale_lon = (FLY_ROUTE_LABEL_WIDTH-120)/(lon_max-lon_min); //60 for edge
    float scale = (scale_lat<scale_lon) ? scale_lat : scale_lon;

    //draw lines
    QPainter painter;
    QImage image("/home/cc/catkin_ws/src/station/src/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);
    painter.setPen(QPen(Qt::blue,4));

    for(int j=0;j<gps_num;j++)
    {
        float px=(gps_fence[j][1]-lon_min)*scale+60;
        float py=FLY_ROUTE_LABEL_HEIGHT-(gps_fence[j][0]-lat_min)*scale-60;
        float pnx=(gps_fence[j+1][1]-lon_min)*scale+60;
        float pny=FLY_ROUTE_LABEL_HEIGHT-(gps_fence[j+1][0]-lat_min)*scale-60;
        painter.drawLine(px,py,pnx,pny);
        painter.drawEllipse(px,py,10,10);
        QRectF rect(px+20, py, px+75, py-20);
        painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[j][2]+1));
    }

    //draw line between the last and the first point
    painter.drawLine((gps_fence[gps_num][1]-lon_min)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence[gps_num][0]-lat_min)*scale-60,(gps_fence[0][1]-lon_min)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence[0][0]-lat_min)*scale-60);

   //draw last point
    float last_point_x = (gps_fence[gps_num][1]-lon_min)*scale+60;
    float last_point_y = FLY_ROUTE_LABEL_HEIGHT-(gps_fence[gps_num][0]-lat_min)*scale-60;
    painter.drawEllipse(last_point_x,last_point_y,10,10);
    QRectF rect(last_point_x+20, last_point_y, last_point_x+75, last_point_y-20);
    painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[gps_num][2]+1));

    painter.end();
    fly_route_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片

}

void MainWindow::draw_route(int window)
{
    float scale = 0.0;
    float min_x = 0.0, max_x = 0.0, min_y = 0.0, max_y = 0.0;

    if((common_mode && window != 0) || window==3)
    {
        /*calculate scale with common flight position and home position*/

        for(int i =0;i<=intersection_num;i++)
        {
            if(min_x > route_p_local[i][0]) min_x = route_p_local[i][0];
            if(max_x < route_p_local[i][0]) max_x = route_p_local[i][0];
            if(min_y > route_p_local[i][1]) min_y = route_p_local[i][1];
            if(max_y < route_p_local[i][1]) max_y = route_p_local[i][1];
        }
        float scale_x = (FLY_ROUTE_LABEL_WIDTH - 120)/(max_x - min_x);
        float scale_y = (FLY_ROUTE_LABEL_HEIGHT - 80)/(max_y - min_y);
        scale = (scale_x < scale_y) ? scale_x : scale_y;
    }
    else
    {
        /*calculate scale with local position of fence and home position*/

        for(int i =0;i<=gps_num;i++)
        {
            if(min_x > gps_fence_local[i][0]) min_x = gps_fence_local[i][0];
            if(max_x < gps_fence_local[i][0]) max_x = gps_fence_local[i][0];
            if(min_y > gps_fence_local[i][1]) min_y = gps_fence_local[i][1];
            if(max_y < gps_fence_local[i][1]) max_y = gps_fence_local[i][1];
        }
        float scale_x = (FLY_ROUTE_LABEL_WIDTH - 120)/(max_x - min_x);
        float scale_y = (FLY_ROUTE_LABEL_HEIGHT - 80)/(max_y - min_y);
        scale = (scale_x < scale_y) ? scale_x : scale_y;
    }


    QPainter painter;
    QImage image("/home/cc/catkin_ws/src/station/src/Icons/grass-720x540-2.png");//定义图片，并在图片上绘图方便显示
    painter.begin(&image);
    painter.setPen(QPen(Qt::blue,4));

    if((!common_mode || window == 0 )&& window!=3)
    {
        /*draw fence*/
        for(int j=0;j<gps_num;j++)
        {
            float px=(gps_fence_local[j][0]-min_x)*scale+60;
            float py=(FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[j][1]-min_y)*scale-60);
            float pnx=(gps_fence_local[j+1][0]-min_x)*scale+60;
            float pny=(FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[j+1][1]-min_y)*scale-60);
            painter.drawLine(px,py,pnx,pny);
            painter.drawEllipse(px,py,10,10);
            QRectF rect(px+20, py, px+75, py-20);
            painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[j][2]+1));
        }

        //draw line between the last and the first point
        painter.drawLine((gps_fence_local[gps_num][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[gps_num][1]-min_y)*scale-60,(gps_fence_local[0][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[0][1]-min_y)*scale-60);

        //draw last point
        float last_point_x = (gps_fence_local[gps_num][0]-min_x)*scale+60;
        float last_point_y = FLY_ROUTE_LABEL_HEIGHT-(gps_fence_local[gps_num][1]-min_y)*scale-60;
        painter.drawEllipse(last_point_x,last_point_y,10,10);
        QRectF rect(last_point_x+20, last_point_y, last_point_x+75, last_point_y-20);
        painter.drawText(rect, Qt::AlignLeft,tr("Point")+QString::number(gps_fence[gps_num][2]+1));
    }

    /*draw route*/
    painter.setPen(QPen(Qt::yellow,3));
    painter.drawLine((0-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(0-min_y)*scale-60,(route_p_local[0][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(route_p_local[0][1]-min_y)*scale-60);
    for(int i=0;i<intersection_num;i++)
    {
        painter.drawLine((route_p_local[i][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(route_p_local[i][1]-min_y)*scale-60,(route_p_local[i+1][0]-min_x)*scale+60,FLY_ROUTE_LABEL_HEIGHT-(route_p_local[i+1][1]-min_y)*scale-60);
    }

    painter.setPen(QPen(Qt::red,6));
    float home_local_x = (0-min_x)*scale+60;
    float home_local_y = FLY_ROUTE_LABEL_HEIGHT-(0-min_y)*scale-60;
    painter.drawEllipse(home_local_x,home_local_y,5,5);

    if(window==0)
    {
        painter.end();
        fly_route_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
    else if(window==1)
    {
        painter.end();
        fly_position_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
    else if(window==2)
    {
        float home_draw_x = (real_position[0][0] - draw_start_x - min_x)*scale+60;
        float home_draw_y = FLY_ROUTE_LABEL_HEIGHT-(real_position[0][1] - draw_start_y - min_y)*scale-60;

        painter.setPen(QPen(Qt::red,3));
        painter.save();
        painter.translate(home_draw_x,home_draw_y);

        if(position_num > 0)
        {
            for(int n=0;n<position_num;n++)
            {
                 painter.drawLine((real_position[n][0]-real_position[0][0])*scale,-(real_position[n][1]-real_position[0][1])*scale,
                      (real_position[n+1][0]-real_position[0][0])*scale,-(real_position[n+1][1]-real_position[0][1])*scale);
            }
        }
        painter.restore();
        painter.end();
        fly_position_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
    else if(window==3)
    {
        painter.end();
        fly_route_label->setPixmap(QPixmap::fromImage(image));//在label上显示图片
    }
    else ;
}

void MainWindow::delete_point(int x) //x start with 0
{
    //store
    memcpy(gps_fence_cp3,gps_fence_cp2,16000);
    memcpy(gps_fence_cp2,gps_fence_cp1,16000);
    memcpy(gps_fence_cp1,gps_fence,16000);
    gps_num_cp3=gps_num_cp2;
    gps_num_cp2=gps_num_cp1;
    gps_num_cp1=gps_num;
    list_seq_cp3=list_seq_cp2;
    list_seq_cp2=list_seq_cp1;
    list_seq_cp1=list_seq;

    //delete
    for(int i=x;i<gps_num;i++)
    {
        gps_fence[i][0]=gps_fence[i+1][0];
        gps_fence[i][1]=gps_fence[i+1][1];
        gps_fence[i][2]=gps_fence[i+1][2];
    }
    gps_num -= 1;
}

bool MainWindow::restore_point()
{
    if(gps_fence_cp1[0][0]>0){
        memcpy(gps_fence,gps_fence_cp1,16000);
        gps_num=gps_num_cp1;

        memcpy(gps_fence_cp1,gps_fence_cp2,16000);
        gps_num_cp1=gps_num_cp2;

        memcpy(gps_fence_cp2,gps_fence_cp3,16000);
        gps_fence_cp3[0][0]=0.0;
        gps_num_cp2=gps_num_cp3;

        //cout<<gps_fence[2][0]<<endl;

        return true;
    }
    return false;
}

void MainWindow::gps_to_local(double lat, double lon, float *x, float *y) //y is North, x is East
{
    double lat_rad = lat * DEG_TO_RAD;
    double lon_rad = lon * DEG_TO_RAD;
    double home_lat_rad = home_lat * DEG_TO_RAD;
    double home_lon_rad = home_lon * DEG_TO_RAD;

    //algorithm from px4
    /*double sin_lat = sin(lat_rad);
    double cos_lat = cos(lat_rad);
    double sin_home_lat = sin(home_lat_rad);
    double cos_home_lat = cos(home_lon_rad);
    double cos_d_lon = cos(lon_rad - home_lon_rad);

    double arg = sin_home_lat * sin_lat + cos_home_lat * cos_lat * cos_d_lon;
    cout<<"arg="<<arg<<endl;

    if (arg > 1.0) {
            arg = 1.0;

    } else if (arg < -1.0) {
            arg = -1.0;
    }

    double c = acos(arg);
    double k = (fabs(c) < DBL_EPSILON) ? 1.0 : (c / sin(c));

    *x = k * (cos_home_lat * sin_lat - sin_home_lat * cos_lat * cos_d_lon) * CONSTANTS_RADIUS_OF_EARTH;
    *y = k * cos_lat * sin(lon_rad - home_lon_rad) * CONSTANTS_RADIUS_OF_EARTH;*/

    //easiest algorithm
    if(EASTERN_HEMISPHERE==1) *x = cos((lat_rad+home_lat_rad)/2)*CONSTANTS_RADIUS_OF_EARTH*(lon_rad - home_lon_rad); //East is positive
    else *x = -cos((lat_rad+home_lat_rad)/2)*CONSTANTS_RADIUS_OF_EARTH*(lon_rad - home_lon_rad);

    if(NORTHERN_HEMISPHERE==1) *y = (lat_rad - home_lat_rad)*CONSTANTS_RADIUS_OF_EARTH;  //NE Coodinate
    else *y = -(lat_rad - home_lat_rad)*CONSTANTS_RADIUS_OF_EARTH;

    //http://blog.sina.com.cn/s/blog_658a93570101hynw.html
}

void MainWindow::local_to_gps(float x, float y, double *lat, double *lon)
{
    double home_lat_rad = home_lat * DEG_TO_RAD;
    double home_lon_rad = home_lon * DEG_TO_RAD;

    double lat_rad = ((double)y) / CONSTANTS_RADIUS_OF_EARTH + home_lat_rad;
    double lon_rad = ((double)x)/(CONSTANTS_RADIUS_OF_EARTH*cos((lat_rad+home_lat_rad)/2.0))+home_lon_rad;

    *lat = lat_rad * RAD_TO_DEG;
    *lon = lon_rad * RAD_TO_DEG;
}

void MainWindow::turn_point_cal()
{
    //initialize
    intersection_num = 0;
    measure_compensation_m = ui->lineEdit_Measure_Compensation->text().toFloat();

    for(int i=0;i<MAX_POINT_NUM;i++)
    {
        route_p_local[i][0] = 0;
        route_p_local[i][1] = 0;
    }

    //calculate diraction k with the first and the last point
    float first_px = 0.0;
    float first_py = 0.0;
    float last_px = 0.0;
    float last_py = 0.0;
    gps_to_local(gps_diraction[0][0],gps_diraction[0][1],&first_px,&first_py);
    gps_to_local(gps_diraction[diraction_p_num][0],gps_diraction[diraction_p_num][1],&last_px,&last_py);
    diraction_k = (last_py-first_py)/(last_px-first_px);
    cout<<"diraction_k "<<diraction_k<<endl;


    //calculate fence local position
    if(gps_fence[0][0]>0){
        for(int i=0;i<=gps_num;i++)
        {
            gps_to_local(gps_fence[i][0],gps_fence[i][1],&gps_fence_local[i][0],&gps_fence_local[i][1]);
        }
        //cout<<gps_fence_local[0][0]<<" + "<<gps_fence_local[0][1]<<endl;
    }
    //calculate lines, form: y=kx+b
    float line_paras[gps_num+1][4]; //(k,b,x1,x2), para[0] is for the line between Point0 and Point1
    for(int i=0;i<=gps_num;i++)
    {
        if(i==gps_num)
        {
            line_paras[i][0] = (gps_fence_local[i][1]-gps_fence_local[0][1])/(gps_fence_local[i][0]-gps_fence_local[0][0]);
            line_paras[i][1] = gps_fence_local[i][1]-line_paras[i][0]*gps_fence_local[i][0];
            line_paras[i][2] = (gps_fence_local[i][0] < gps_fence_local[0][0]) ? gps_fence_local[i][0] : gps_fence_local[0][0];
            line_paras[i][3] = (gps_fence_local[i][0] > gps_fence_local[0][0]) ? gps_fence_local[i][0] : gps_fence_local[0][0];
        }
        else
        {
            line_paras[i][0] = (gps_fence_local[i][1]-gps_fence_local[i+1][1])/(gps_fence_local[i][0]-gps_fence_local[i+1][0]);
            line_paras[i][1] = gps_fence_local[i][1]-line_paras[i][0]*gps_fence_local[i][0];
            line_paras[i][2] = (gps_fence_local[i][0] < gps_fence_local[i+1][0]) ? gps_fence_local[i][0] : gps_fence_local[i+1][0];
            line_paras[i][3] = (gps_fence_local[i][0] > gps_fence_local[i+1][0]) ? gps_fence_local[i][0] : gps_fence_local[i+1][0];
        }
    }
    /*calculate turning points*/
    /*1.calculate proper y cross distance between lines*/
    float angle_k = atan(diraction_k);
    //if(angle_k<0) angle_k += PI;
    float cos_k = cos(angle_k);//= sqrt(1/(diraction_k*diraction_k+1));

    float min_b = 100000.0, max_b = -100000.0;

    for(int i=0;i<=gps_num;i++)
    {
        float b_temp = gps_fence_local[i][1] - diraction_k*gps_fence_local[i][0];
        min_b = (min_b > b_temp) ? b_temp : min_b;
        max_b = (max_b < b_temp) ? b_temp : max_b;
    }

    float delt_b_ideal = fabs(spray_width / cos_k); //positive

    float b_distance = max_b - min_b - measure_compensation_m*2/cos_k;
    float times_f = b_distance / delt_b_ideal;
    float delt_b = 0.0;
    int times = 0;

    if(fabs(b_distance/((int)times_f)-spray_width) < fabs(b_distance/((int)(times_f+1))-spray_width))
    {
        times = (int)times_f;
        delt_b = b_distance/times;
    }
    else {
        times = (int)(times_f+1);
        delt_b = b_distance/times;
    }
   //cout<<"delt_b="<<delt_b<<endl;

    dist_between_lines = delt_b * cos_k;
    cout<<"dist_between_lines="<<dist_between_lines<<endl;

    /*2.calculate local intersection point*/
    float b_start = min_b + delt_b/2 + measure_compensation_m/cos_k;
    for(int i=0;i<times;i++)
    {
        float intersection_temp[20][2]; //set 20 intersection points max for a line
        int intersection_num_temp = 0;
        //calculate
        for(int j=0;j<=gps_num;j++)
        {
            float x = (b_start - line_paras[j][1]) / (line_paras[j][0] - diraction_k);
            //cout<<"x="<<x<<endl;
            if(x>line_paras[j][2] && x<line_paras[j][3])
            {
                //cout<<"get in &&&"<<endl;
                intersection_temp[intersection_num_temp][0] = x;
                intersection_temp[intersection_num_temp][1] = diraction_k * x + b_start;
                intersection_num_temp ++;
            }
        }
        intersection_num_temp --;

        /*arrange intersection points sequence by value of x from small to large (point(x,y))*/
        float ex_x = 0.0, ex_y = 0.0;
        for(int m=0;m<=intersection_num_temp;m++)
        {
            for(int n=m+1;n<=intersection_num_temp;n++)
            {
                if(intersection_temp[m][0] > intersection_temp[n][0])
                {
                    ex_x = intersection_temp[m][0];
                    ex_y = intersection_temp[m][1];
                    intersection_temp[m][0] = intersection_temp[n][0];
                    intersection_temp[m][1] = intersection_temp[n][1];
                    intersection_temp[n][0] = ex_x;
                    intersection_temp[n][1] = ex_y;
                }
                //cout<<"intersection_temp[m][0]"<<intersection_temp[m][0]<<endl;
            }
        }
        //cout<<"intersection_temp[0][0]"<<intersection_temp[0][0]<<endl;

        /*store the first and the last intersection point on a line*/
        intersection_p_local[intersection_num][0] = intersection_temp[0][0];
        intersection_p_local[intersection_num][1] = intersection_temp[0][1];
        intersection_p_local[intersection_num+1][0] = intersection_temp[intersection_num_temp][0];
        intersection_p_local[intersection_num+1][1] = intersection_temp[intersection_num_temp][1];

        //cout<<"("<<intersection_p_local[intersection_num][0]<<","<<intersection_p_local[intersection_num][1]<<")"<<endl;
        //cout<<"("<<intersection_p_local[intersection_num+1][0]<<","<<intersection_p_local[intersection_num+1][1]<<")"<<endl;

        intersection_num += 2;

        b_start += delt_b;
    }
    intersection_num -= 1; //correct intersection_num

    /*eliminate the effect on the end of line from spray length and measurement error*/
    float e_x = (spray_length/2 + measure_compensation_m) * cos(angle_k);
    float e_y = (spray_length/2 + measure_compensation_m) * sin(angle_k);
    for(int i=0;i<= intersection_num;i+=2)
    {
        intersection_p_local[i][0] += e_x;
        intersection_p_local[i][1] += e_y;
        intersection_p_local[i+1][0] -= e_x;
        intersection_p_local[i+1][1] -= e_y;
    }

    /*3.arrange the best route points sequence according to home position*/
    float min_start_dist = 10000.0;
    float d[4];
    int method_num = -1;
    d[0] = point_dist(intersection_p_local[0][0], intersection_p_local[0][1], 0.0, 0.0);
    d[1] = point_dist(intersection_p_local[1][0], intersection_p_local[1][1], 0.0, 0.0);
    d[2] = point_dist(intersection_p_local[intersection_num-1][0], intersection_p_local[intersection_num-1][1], 0.0, 0.0);
    d[3] = point_dist(intersection_p_local[intersection_num][0], intersection_p_local[intersection_num][1], 0.0, 0.0);

    for(int i=0;i<4;i++)
    {
        if(d[i] < min_start_dist)
        {
            min_start_dist = d[i];
            method_num = i;
            cout<<i<<" min: "<<min_start_dist<<endl;
        }
    }
    cout<<"intersection_num="<<intersection_num<<endl;
    switch(method_num)
    {
        case 0:
        {
            int num = 0; //start from p0
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num += 1; break;
                    case 2: num += 2; break;
                    case 3: num -= 1; break;
                    case 0: num += 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        case 1:
        {
            int num = 1; //start from p1
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num -= 1; break;
                    case 2: num += 2; break;
                    case 3: num += 1; break;
                    case 0: num += 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        case 2:
        {
            int num = intersection_num-1; //start from p(n-1)
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num += 1; break;
                    case 2: num -= 2; break;
                    case 3: num -= 1; break;
                    case 0: num -= 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        case 3:
        {
            int num = intersection_num; //start from pn
            route_p_local[0][0] = intersection_p_local[num][0];
            route_p_local[0][1] = intersection_p_local[num][1];
            for(int i=1;i<=intersection_num;i++)
            {
                switch(i%4)
                {
                    case 1: num -= 1; break;
                    case 2: num -= 2; break;
                    case 3: num += 1; break;
                    case 0: num -= 2; break;
                }
               route_p_local[i][0] = intersection_p_local[num][0];
               route_p_local[i][1] = intersection_p_local[num][1];
            }
            break;
        }
        default: break;
    }

    /*4.translate to global coordnate, for flying back to break point, without wind effect*/
    for(int n=0;n<=intersection_num;n++)
    {
        local_to_gps(route_p_local[n][0],route_p_local[n][1],&route_p_gps[n][0],&route_p_gps[n][1]);
    }

    cout<<"route_p_local[0][0]"<<route_p_local[0][0]<<endl;
    cout<<"route_p_local[0][1]"<<route_p_local[0][1]<<endl;
    cout<<"route_p_local[1][0]"<<route_p_local[1][0]<<endl;
    cout<<"route_p_local[1][1]"<<route_p_local[1][1]<<endl;
    /*float local_tx;
    float local_ty;
    gps_to_local(route_p_gps[0][0],route_p_gps[0][1],&local_tx,&local_ty);
    cout<<"local-gps-local"<<local_tx<<"  "<<local_tx<<endl;*/

    /*offset to eliminate the effect from wind*/
    if(offset_dist_m != 0){
        float offset_x = offset_dist_m * cos(offset_angle_d*DEG_TO_RAD);
        float offset_y = offset_dist_m * sin(offset_angle_d*DEG_TO_RAD);
        for(int i=0;i<=intersection_num;i++)
        {
            route_p_local[i][0] += offset_x;
            route_p_local[i][1] += offset_y;
        }
    }

    /*calculate yaw*/
    float yaw_local = atan2(route_p_local[1][1]-route_p_local[0][1], route_p_local[1][0]-route_p_local[0][0]); //E is 0, N is Pi/2
    yaw_set = yaw_local - PI_2; //turn to: N is 0, W is Pi/2
    if(yaw_set < 0) yaw_set += 2*PI;
    cout<<"yaw_set="<<yaw_set<<endl; 

    route_plan_mode = 0;

    //draw
    draw_route(0);
}

float MainWindow::point_dist(float x1, float y1, float x2, float y2)
{
    return sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2));
}

float MainWindow::point_line_dist(float m, float n, float k, float b)
{
    return (fabs(k*m-n+b))/(sqrt(k*k+1));
}



void MainWindow::on_pushButton_Open_Fence_clicked()
{
    //open a file using QFileDialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "/home",  tr("track(*.gpx)"));

    if(fileName.length()!=0) //if a file choosed
    {
        //initial
        gps_num = 0;
        gps_num_cp1 = 0;
        gps_num_cp2 = 0;
        gps_num_cp3 = 0;

        ui->listWidget_GPS_Point->clear();
        fstream gps_f;
        char *path = fileName.toLatin1().data();
        gps_f.open(path,ios::in);

        gps_num = 0; //initialize
        while(!gps_f.eof()){   //while not the end of file
            char str[300];
            gps_f >> str;
            //cout<<endl<<str;

            if(str[0]=='l'&&str[1]=='a'&&str[2]=='t') //lattitude
            {
                double lat=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lat += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lat += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                //cout<<endl<<"lat:"<<lat;
                gps_fence[gps_num][0]=lat;//save
            }

            if(str[0]=='l'&&str[1]=='o'&&str[2]=='n') //longitude
            {
                double lon=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lon += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lon += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                //cout<<endl<<"lon:"<<lon;
                gps_fence[gps_num][1]=lon;//save
                gps_fence[gps_num][2]=gps_num;
                gps_num++;//counter for next point
                if(gps_num > MAX_POINT_NUM-1)
                {
                    gps_fence[0][0] = 0.0;
                    QMessageBox message_box(QMessageBox::Warning,"警告","GPS点过多,无法识别!", QMessageBox::Cancel, NULL);
                    message_box.exec();
                    break;
                }
            }
        }
        gps_num -= 1; //correct num
        gps_f.close(); //reading finished

      //judge if gpx file can be used
        if(gps_num < 3)
        {
            gps_fence[0][0] = 0.0;
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS点少于3个,请重新选择!", QMessageBox::Cancel, NULL);
            message_box.exec();
        }
        else{
            //add items
            for(int i=1;i<=gps_num+1;i++)
            {
                char name[10]="Point";
                char num[4];
                sprintf(num,"%d",i);
                strcat(name,num);
                ui->listWidget_GPS_Point->addItem(new QListWidgetItem(QObject::tr(name)));
            }
            //cout<<"$$"<<gps_fence[4][1];
            draw_gps_fence();
        }
  }
}


void MainWindow::on_pushButton_Open_Diraction_clicked()
{
    //open a file using QFileDialog
    QString fileName = QFileDialog::getOpenFileName(this, tr("open file"), "/home",  tr("track(*.gpx)"));

    if(fileName.length()!=0) //if a file choosed
    {
        fstream gps_d;
        char *path = fileName.toLatin1().data();
        gps_d.open(path,ios::in);

        diraction_p_num = 0; //initialize
        while(!gps_d.eof()){   //while not the end of file
            char str[300];
            gps_d >> str;
            //cout<<endl<<str;

            if(str[0]=='l'&&str[1]=='a'&&str[2]=='t') //lattitude
            {
                double lat=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lat += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lat += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                cout<<endl<<"lat:"<<lat;
                gps_diraction[diraction_p_num][0]=lat;//save
            }

            if(str[0]=='l'&&str[1]=='o'&&str[2]=='n') //longitude
            {
                double lon=0.0;
                int point_p=8;

                for(int i=5;str[i]!='"';i++) //find '.' first
                    if(str[i]=='.') point_p=i;

                for(int m=point_p-1;str[m]!='"';m--) //integer
                    lon += (double)(str[m]-'0')*pow(10,(point_p-1-m));

                for(int n=point_p+1;str[n]!='"';n++) //fractional
                    lon += ((double)(str[n]-'0'))/pow(10,(n-point_p));
                cout<<endl<<"lon:"<<lon;
                gps_diraction[diraction_p_num][1]=lon;//save
                diraction_p_num++;//counter for next point
                if(diraction_p_num > MAX_POINT_NUM-1)
                {
                    gps_diraction[0][0] = 0.0;
                    QMessageBox message_box(QMessageBox::Warning,"警告","GPS点过多,无法识别!", QMessageBox::Cancel, NULL);
                    message_box.exec();
                    break;
                }
            }
        }
        diraction_p_num -= 1; //correct num
        gps_d.close(); //reading finished

        //judge if gpx file can be used
        if(diraction_p_num < 1)
        {
            gps_diraction[0][0] = 0.0;
            QMessageBox message_box(QMessageBox::Warning,"警告","GPS点少于2个,请重新选择!", QMessageBox::Cancel, NULL);
            message_box.exec();
        }


    }
}


void MainWindow::on_pushButton_Delete_Point_clicked()
{
    //store seq and item
    list_seq = ui->listWidget_GPS_Point->currentRow();
    item_cp3=item_cp2;
    item_cp2=item_cp1;
    item_cp1=ui->listWidget_GPS_Point->currentItem();

    delete_point(list_seq);
    ui->listWidget_GPS_Point->takeItem(list_seq);
    ui->pushButton_Restore_Point->setEnabled(true);

    draw_gps_fence();
}


void MainWindow::on_pushButton_Restore_Point_clicked()
{
    if(restore_point())
    {
        ui->listWidget_GPS_Point->insertItem(list_seq_cp1,item_cp1);
        item_cp1=item_cp2;
        item_cp2=item_cp3;

        list_seq_cp1=list_seq_cp2;
        list_seq_cp2=list_seq_cp3;
    }
    draw_gps_fence();
    //cout<<endl;
}

void MainWindow::on_listWidget_GPS_Point_itemClicked()
{
    list_seq = ui->listWidget_GPS_Point->currentRow();
    ui->pushButton_Delete_Point->setEnabled(true);
    //cout<<list_seq<<endl;
}

void MainWindow::on_dial_Offset_Angle_valueChanged(int value)
{
    //cout<<"value="<<value<<endl;

}

int MainWindow::on_pushButton_Save_Config_clicked()
{
    take_off_height = ui->lineEdit_Take_Off_Height->text().toFloat();
    //flying_height = ui->lineEdit_Flying_Height->text().toFloat();
    spray_length = ui->lineEdit_Spray_Length->text().toFloat();
    spray_width = ui->lineEdit_Spray_Width->text().toFloat();

    if(ui->checkBox_Auto_Height->isChecked()) message.extra_function.laser_height_enable = 1;
    else message.extra_function.laser_height_enable = 0;

    if(ui->radioButton_Auto_Avoid->isChecked()) message.extra_function.obs_avoid_enable = 2;
    else if(ui->radioButton_Mannual_Avoid->isChecked()) message.extra_function.obs_avoid_enable = 1;
    else message.extra_function.obs_avoid_enable = 0;


    message.pump.pump_speed_sp = ((float) ui->horizontalSlider_Spray->value())/10.0 - 1.0;
    cout<<ui->horizontalSlider_Spray->value()<<"  "<<message.pump.pump_speed_sp<<endl;

    char name[17] = "/config.txt";
    char path[80]="/home/cc/catkin_ws/src/break_point";

    QDir *temp = new QDir;
    bool exist = temp->exists(QString(path));
    if(!exist)temp->mkdir(QString(path));

    strcat(path,name);

    cout<<"file saved in "<<path<<endl;
    FILE *pTxtFile = NULL;

    pTxtFile = fopen(path, "w+");
    if (pTxtFile == NULL)
    {
        printf("The program exist!\n");
        return 0;
    }

    cout<<"saving...\n";

    fprintf(pTxtFile,"#%.1f#\n",take_off_height);
    fprintf(pTxtFile,"#%.1f#\n",spray_length);
    fprintf(pTxtFile,"#%.1f#\n",spray_width);
    fprintf(pTxtFile,"#%.1f#\n",(float)message.extra_function.laser_height_enable);
    fprintf(pTxtFile,"#%.1f#\n",(float)message.extra_function.obs_avoid_enable);
    fprintf(pTxtFile,"#%.1f#\n",message.pump.pump_speed_sp);

    fprintf(pTxtFile,"end");
    //fprintf(pTxtFile,"take_off_height spray_length spray_width message.extra_function.laser_height_enable message.extra_function.obs_avoid_enable message.pump.pump_speed_sp");

    fclose(pTxtFile);

    QMessageBox message_box(QMessageBox::Warning,"提示","保存成功!", QMessageBox::Ok, NULL);
    message_box.exec();
    return 1;

}

void MainWindow::record_home_gps()
{
    home_lat = message.global_position.gps.x;
    home_lon = message.global_position.gps.y;
}

int MainWindow::record_break_point()
{
    if(!test_mode)
    {
        break_point_lat = message.global_position.gps.x;
        break_point_lon = message.global_position.gps.y;
        break_position_num = message.setpoints_receive.seq;
    }
    if(imitate_mode)
    {
        break_position_num = message.setpoints_receive.seq;
        cout<<"break_position_num"<<break_position_num<<endl;
    }
    char name[17] = "/break_point.txt";
    char path[80]="/home/cc/catkin_ws/src/break_point";

    QDir *temp = new QDir;
    bool exist = temp->exists(QString(path));
    if(!exist)temp->mkdir(QString(path));

    strcat(path,name);
    cout<<"file saved in "<<path<<endl;
    FILE *pTxtFile = NULL;

    pTxtFile = fopen(path, "w+");
    if (pTxtFile == NULL)
    {
        printf("The program exist!\n");
        return 0;
    }

    cout<<"writing...\n";

    float bseq = (float)break_position_num - 2;
    if(bseq < 0) bseq = 0;
    fprintf(pTxtFile,"blat=#%.8lf# blon=#%.8lf#\n",break_point_lat,break_point_lon);
    fprintf(pTxtFile,"bseq=#%f#\n",bseq);
    fprintf(pTxtFile,"heit=#%f#\n",flying_height);
    fprintf(pTxtFile,"angl=#%f#\n",offset_angle_d);
    fprintf(pTxtFile,"dist=#%f#\n",offset_dist_m);
    fprintf(pTxtFile,"yaws=#%f#\n",yaw_set);

    if(common_mode)
    {
        gps_num_last = 0;
        gps_fence_last[0][0] = 99.0; //mark common mode
        gps_fence_last[0][1] = 0.0;
        gps_fence_last[0][2] = 0.0;
    }

    for(int i=0;i<=gps_num_last;i++)
    {
        fprintf(pTxtFile,"flat=#%.8lf# flon=#%.8lf# fseq=#%lf#\n",gps_fence_last[i][0],gps_fence_last[i][1],gps_fence_last[i][2]);
    }

    for(int i=0;i<=intersection_num_last;i++)
    {
        fprintf(pTxtFile,"rlat=#%.8lf# rlon=#%.8lf#\n",route_p_gps_last[i][0],route_p_gps_last[i][1]);
    }

    fprintf(pTxtFile,"end");

    fclose(pTxtFile);
    return 1;
}

int MainWindow::on_pushButton_Open_Break_Point_clicked()
{
    if(home_lat == 0 || (!test_mode && !controller_working))
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未连接到飞机或无GPS信号", QMessageBox::Cancel, NULL);
        message_box.exec();
        return 0;
    }

    for(int i=0;i<MAX_POINT_NUM;i++)
    {
        route_p_local[i][0] = 0;
        route_p_local[i][1] = 0;
    }

    char dir_path[80]="/home/cc/catkin_ws/src/break_point";
    QDir *temp = new QDir;
    bool exist = temp->exists(QString(dir_path));
    if(!exist)
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","尚未有任何断点文件!", QMessageBox::Cancel, NULL);
        message_box.exec();
        return 0;
    }

    QString fileName = "/home/cc/catkin_ws/src/break_point/break_point.txt";

    //initial
    gps_num = 0;
    gps_num_cp1 = 0;
    gps_num_cp2 = 0;
    gps_num_cp3 = 0;

    route_p_num_read = 0;

    ui->listWidget_GPS_Point->clear(); //clear item
    fstream break_f;
    char *path = fileName.toLatin1().data();
    break_f.open(path,ios::in);

    gps_num = 0; //initialize
    double fnum=0.0;
    while(!break_f.eof()){   //while not the end of file
        char str[300];
        break_f >> str;
        //cout<<endl<<str;
        fnum=0.0;
        if(str[0]=='f'&&str[1]=='l'&&str[2]=='a'&&str[3]=='t') //fence lattitude
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            gps_fence[gps_num][0]=fnum;//save

            /*common mode break point situation*/
            if(gps_fence[gps_num][0] > 90) route_plan_mode = 3;
            else route_plan_mode = 2;
        }

        else if(str[0]=='f'&&str[1]=='l'&&str[2]=='o'&&str[3]=='n') //fence longitude
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            gps_fence[gps_num][1]=fnum;//save
            gps_fence[gps_num][2]=gps_num;
            gps_num++;//counter for next point
        }

        else if(str[0]=='r'&&str[1]=='l'&&str[2]=='a'&&str[3]=='t') //route lattitude
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            route_p_gps_read[route_p_num_read][0]=fnum;//save
            //cout<<"rlat "<<fnum<<endl;
        }

        else if(str[0]=='r'&&str[1]=='l'&&str[2]=='o'&&str[3]=='n') //route longitude
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            route_p_gps_read[route_p_num_read][1]=fnum;//save
            route_p_num_read++;//counter for next point
        }

        else if(str[0]=='b'&&str[1]=='l'&&str[2]=='a'&&str[3]=='t') //break point lattitude
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            break_point_lat_read=fnum;//save
        }

        else if(str[0]=='b'&&str[1]=='l'&&str[2]=='o'&&str[3]=='n') //break point longitude
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            break_point_lon_read=fnum;//save
        }

        else if(str[0]=='b'&&str[1]=='s'&&str[2]=='e'&&str[3]=='q') //break point seq
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            break_point_seq_read=(int)fnum;//save
        }

        else if(str[0]=='h'&&str[1]=='e'&&str[2]=='i'&&str[3]=='t') //flying height
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            flying_height=fnum;//save
            ui->lineEdit_Flying_Height_2->setText(QString::number(flying_height));
        }

        else if(str[0]=='a'&&str[1]=='n'&&str[2]=='g'&&str[3]=='l') //wind angle
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            offset_angle_d=fnum;//save
            ui->dial_Offset_Angle_2->setValue((int)(270-offset_angle_d));
        }

        else if(str[0]=='d'&&str[1]=='i'&&str[2]=='s'&&str[3]=='t') //wind dist
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            offset_dist_m=fnum;//save
            ui->lineEdit_Offset_Dist_2->setText(QString::number(offset_dist_m));
        }

        else if(str[0]=='y'&&str[1]=='a'&&str[2]=='w'&&str[3]=='s') //yaw set
        {
            int point_p=8;

            for(int i=6;str[i]!='#';i++) //find '.' first
                if(str[i]=='.') point_p=i;

            for(int m=point_p-1;str[m]!='#';m--) //integer
                fnum += (double)(str[m]-'0')*pow(10,(point_p-1-m));

            for(int n=point_p+1;str[n]!='#';n++) //fractional
                fnum += ((double)(str[n]-'0'))/pow(10,(n-point_p));
            yaw_set=fnum;//save

        }
        else ;

        //cout<<"fnum"<<fnum<<endl;

    }

    gps_num -= 1; //correct num
    route_p_num_read -= 1;
    cout<<"gps_num"<<gps_num<<endl;
    cout<<"route_p_num_read"<<route_p_num_read<<endl;
    break_f.close(); //reading finished

    ui->pushButton_Break_Paras_Update->setEnabled(true);

    break_point_cal();

    return 1;
}

void MainWindow::break_point_cal()
{
    //record home gps position
    if(!test_mode) record_home_gps();
    record_start_p();
    flying_height = ui->lineEdit_Flying_Height_2->text().toFloat();

    /*calculate fence local position*/
    for(int i=0;i<=gps_num;i++)
    {
        gps_to_local(gps_fence[i][0],gps_fence[i][1],&gps_fence_local[i][0],&gps_fence_local[i][1]);     
    }
    //cout<<gps_fence_local[0][0]<<" + "<<gps_fence_local[0][1]<<endl;
    /*calculate route point local position*/
    for(int i=0;i<=route_p_num_read;i++)
    {
        //cout<<"homelat "<<home_lat<<endl;
        gps_to_local(route_p_gps_read[i][0],route_p_gps_read[i][1],&route_p_local_read[i][0],&route_p_local_read[i][1]);
    }
    cout<<"route_p_local_read[0][0]"<<route_p_local_read[0][0]<<endl;


    /*set route_p_local from break point and the first unfinished point*/
    gps_to_local(break_point_lat_read,break_point_lon_read,&route_p_local[0][0],&route_p_local[0][1]);
    for(int i=0;i<=route_p_num_read;i++)
    {
        route_p_local[i+1][0] = route_p_local_read[i+break_point_seq_read][0];
        route_p_local[i+1][1] = route_p_local_read[i+break_point_seq_read][1];
    }
    intersection_num = route_p_num_read + 1 - break_point_seq_read;

    /*translate to global coordnate, for flying back to break point, without wind effect*/
    for(int n=0;n<=intersection_num;n++)
    {
        local_to_gps(route_p_local[n][0],route_p_local[n][1],&route_p_gps[n][0],&route_p_gps[n][1]);
    }

    /*offset to eliminate the effect from wind*/
    float offset_x = offset_dist_m * cos(offset_angle_d*DEG_TO_RAD);
    float offset_y = offset_dist_m * sin(offset_angle_d*DEG_TO_RAD);

    cout<<"break_point_seq_read"<<break_point_seq_read<<endl;
    for(int i=0;i<=intersection_num;i++)
    {
        route_p_local[i][0] += offset_x;
        route_p_local[i][1] += offset_y;
    }

    cout<<"route_p_local[0][0]"<<route_p_local[0][0]<<endl;
    cout<<"route_p_local[0][1]"<<route_p_local[0][1]<<endl;
    cout<<"route_p_local[1][0]"<<route_p_local[1][0]<<endl;
    cout<<"route_p_local[1][1]"<<route_p_local[1][1]<<endl;

    ui->pushButton_Route_Send->setEnabled(true);

    if(route_plan_mode!=3) route_plan_mode = 2;

    draw_route(0);
}

void MainWindow::on_pushButton_Break_Paras_Update_clicked()
{
    offset_dist_m = ui->lineEdit_Offset_Dist_2->text().toFloat();
    int value = ui->dial_Offset_Angle_2->value();
    offset_angle_d = (float)(- value + 270);
    break_point_cal();
}


void MainWindow::record_start_p()
{
    start_x = message.local_position.position.x;
    start_y = message.local_position.position.y;
    draw_start_x = -start_y;
    draw_start_y = start_x;
}

void MainWindow::on_pushButton_Restore_Config__clicked()
{
    take_off_height = 3.5;
    spray_width = 3.0;
    spray_length = 1.6;
    message.extra_function.laser_height_enable = 1;
    message.extra_function.obs_avoid_enable = 1;
    message.pump.pump_speed_sp = 0.8;

    ui->lineEdit_Take_Off_Height->setText(QString::number(take_off_height));
    ui->lineEdit_Spray_Width->setText(QString::number(spray_width));
    ui->lineEdit_Spray_Length->setText(QString::number(spray_length));

    ui->checkBox_Auto_Height->setChecked(message.extra_function.laser_height_enable);
    ui->radioButton_Mannual_Avoid->setChecked(true);
    ui->horizontalSlider_Spray->setValue((int)(message.pump.pump_speed_sp*10));

    on_pushButton_Save_Config_clicked();

}

void MainWindow::on_pushButton_Route_Generate_Common_clicked()
{
    common_length = ui->lineEdit_Fly_Length_Common->text().toFloat();
    common_width = ui->lineEdit_Fly_Width_Common->text().toFloat();
    common_height = ui->lineEdit_Fly_Height_Common->text().toFloat();
    //cout<<"common_height"<<common_height<<endl;
    common_times = ui->lineEdit_Fly_Times_Common->text().toInt() - 1;
    if(ui->radioButton_Left_Common->isChecked()) common_side = true;
    else common_side = false;

    if(common_length < 0.1)
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","请输入合理的飞行长度!", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
    else if(common_width < 0.1)
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","请输入合理的折返宽度!", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
    else if(common_height <  0.1)
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","请输入合理的飞行高度!", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
    else if(common_times < 1)
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","请输入合理的折返次数!", QMessageBox::Cancel, NULL);
        message_box.exec();
    }
    /*else if(home_lat == 0 || (!controller_working && !test_mode))
    {
        QMessageBox message_box(QMessageBox::Warning,"警告","未连接到飞机或无GPS信号", QMessageBox::Cancel, NULL);
        message_box.exec();
    }*/
    else
    {
        common_flight_cal(common_length,common_width,common_height,common_times,common_side);
        ui->pushButton_Route_Send->setEnabled(true);
    }

}

void MainWindow::common_flight_cal(float length, float width, float height, int times, bool left_side)
{
    yaw_set = -message.local_position.orientation.yaw + PI;
    float angle1 = yaw_set + PI_2;
    float angle2 = angle1 + PI_2;
    float x = 0.0;
    float y = 0.0;
    intersection_num = times*2; //abort the last width point

    if(left_side)
    {
        for(int i=0;i<=intersection_num;i+=2)
        {
            if(i%4==0)
            {
                route_p_local[i][0] = x + length * cos(angle1);
                route_p_local[i][1] = y + length * sin(angle1);
                x = route_p_local[i][0];
                y = route_p_local[i][1];
                route_p_local[i+1][0] = x + width * cos(angle2);
                route_p_local[i+1][1] = y + width * sin(angle2);
                x = route_p_local[i+1][0];
                y = route_p_local[i+1][1];
            }
            else
            {
                route_p_local[i][0] = x - length * cos(angle1);
                route_p_local[i][1] = y - length * sin(angle1);
                x = route_p_local[i][0];
                y = route_p_local[i][1];
                route_p_local[i+1][0] = x + width * cos(angle2);
                route_p_local[i+1][1] = y + width * sin(angle2);
                x = route_p_local[i+1][0];
                y = route_p_local[i+1][1];
            }
        }

    }

    else  //right side
    {
        for(int i=0;i<=intersection_num;i+=2)
        {
            if(i%4==0)
            {
                route_p_local[i][0] = x + length * cos(angle1);
                route_p_local[i][1] = y + length * sin(angle1);
                x = route_p_local[i][0];
                y = route_p_local[i][1];
                route_p_local[i+1][0] = x - width * cos(angle2);
                route_p_local[i+1][1] = y - width * sin(angle2);
                x = route_p_local[i+1][0];
                y = route_p_local[i+1][1];
            }
            else
            {
                route_p_local[i][0] = x - length * cos(angle1);
                route_p_local[i][1] = y - length * sin(angle1);
                x = route_p_local[i][0];
                y = route_p_local[i][1];
                route_p_local[i+1][0] = x - width * cos(angle2);
                route_p_local[i+1][1] = y - width * sin(angle2);
                x = route_p_local[i+1][0];
                y = route_p_local[i+1][1];
            }
        }

    }
    flying_height = height;

    route_plan_mode = 1;

    //draw
    draw_route(3);
}
