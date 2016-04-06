//*******painterWiget.cpp********

#include "painterWiget.h"
#include "math.h"
extern int choice;

StatusPainter::StatusPainter()
{
    ground_points[0]=QPointF(0,painting_area_height/2);//初始化设定表示地面的多边形的顶点
    ground_points[1]=QPointF(0,painting_area_height);
    ground_points[2]=QPointF(painting_area_width,painting_area_height);
    ground_points[3]=QPointF(painting_area_width,painting_area_height/2);
    ground_points[4]= ground_points[3];
    arc_center_x=painting_area_width/2;
    arc_center_y=painting_area_height/2+50;
    arc_diameter=300.0;
    arc_rectangle=QRectF(arc_center_x-arc_diameter/2,arc_center_y-arc_diameter/2,arc_diameter,arc_diameter);//圆弧参数初始设置;
    pitchd=0.0;//数据初始化
    pitch=0.0;
    roll=0.0;
    rolld=0.0;
    compassd=0.0;
}

void StatusPainter::paintEvent(QPaintEvent *event)
{
    QPainter painter(this);
    //天空
    painter.setPen(QPen(QColor(135,206,250)));
    painter.setBrush(QBrush(QColor(135,206,250), Qt::SolidPattern));
    painter.drawRect(0,0,painting_area_width,painting_area_height);
    //地面
    painting_Calculate();//计算
    if(choice==1)
    {
        painter.setPen(QPen(QColor(0,238,0)));
        painter.setBrush(QBrush(QColor(0,238,0)));
    }
    else
    {
        painter.setPen(QPen(QColor(205,190,112)));
        painter.setBrush(QBrush(QColor(205,190,112)));
    }
    painter.drawPolygon( ground_points,5);// ground_points更新在mianwindow.cpp中
    //画roll圆弧
    painter.setPen(QPen(Qt::white,2,Qt::SolidLine,Qt::RoundCap,Qt::RoundJoin));
    painter.drawArc(arc_rectangle,480,1920);//参数为角度*16
    //画roll刻度、指针三角形
    painter.save();
    painter.setPen(QPen(Qt::white, 2));
    painter.translate(arc_center_x, arc_center_y);//移动坐标原点
    painter.rotate(-60);//坐标系旋转-60度
    roll_string="6543210123456";
    for(int i=0;i<13;i++)
    {
        painter.drawLine(0, -arc_diameter/2, 0, -arc_diameter/2-3);
        if(i!=6)
        {
            painter.drawText(QPointF(-9.0, -arc_diameter/2-8), QString("%1").arg(roll_string[i]));
            painter.drawText(QPointF(0.0, -arc_diameter/2-8), tr("0"));
        }
        else painter.drawText(QPointF(-5.0, -arc_diameter/2-8), QString("%1").arg(roll_string[i]));
        painter.rotate(10);  //rotate()是顺时针旋转坐标轴
    }

    painter.rotate(-70);
    if(rolld>60)
    {
        painter.setPen(QPen(Qt::red));
        painter.setBrush(QBrush(Qt::red));
        painter.rotate(60);
        arrow_points[0]=QPointF(0.0,-arc_diameter/2+4.0);
        arrow_points[1]=QPointF(-6.0,-arc_diameter/2+16.0);
        arrow_points[2]=QPointF(0.0,-arc_diameter/2+14.0);
        arrow_points[3]=QPointF(6.0,-arc_diameter/2+16.0);
        painter.drawPolygon( arrow_points,4);
    }
    else if(rolld<-60)
    {
        painter.setPen(QPen(Qt::red));
        painter.setBrush(QBrush(Qt::red));
        painter.rotate(-60);
        arrow_points[0]=QPointF(0.0,-arc_diameter/2+4.0);
        arrow_points[1]=QPointF(-6.0,-arc_diameter/2+16.0);
        arrow_points[2]=QPointF(0.0,-arc_diameter/2+14.0);
        arrow_points[3]=QPointF(6.0,-arc_diameter/2+16.0);
        painter.drawPolygon( arrow_points,4);
    }
    else
    {
        painter.setPen(QPen(Qt::white));
        painter.setBrush(QBrush(Qt::white));
        painter.rotate(rolld);
        arrow_points[0]=QPointF(0.0,-arc_diameter/2+4.0);
        arrow_points[1]=QPointF(-6.0,-arc_diameter/2+16.0);
        arrow_points[2]=QPointF(0.0,-arc_diameter/2+14.0);
        arrow_points[3]=QPointF(6.0,-arc_diameter/2+16.0);
        painter.drawPolygon( arrow_points,4);
    }
    painter.restore();
    //画pitch刻度
    painter.save();
    painter.translate(painting_area_width/2,painting_area_height/2);//移动坐标原点
    //double k=tan(roll);//斜率
    painter.rotate(rolld);
    double scale=2.2;//刻线间隔系数
    int short_line_length_half=25;//刻线半长度
    int long_line_length_half=40;
    int pitchd_fives =((int)pitchd)/5*5;//取5的倍数
    int pitchd_tens=pitchd_fives/10*10;//取10位数
    double delt=pitchd-pitchd_fives;//中间刻线位置
    double lines_y[7];//3为中间取10位数后刻线，2~0往下，4～6往上
    for(int i=0;i<7;i++)
        lines_y[i]=(delt-5*(i-3))*scale;
    if(pitchd_fives%10==0)//最中间为10的倍数，要画长线时
        for(int i=0;i<7;i++)
        {
            if(i%2==1)
            {
               painter.drawLine(QPointF(-long_line_length_half,lines_y[i]),QPointF(long_line_length_half,lines_y[i]));
               QString string;
               if(i==3)string=QString::number(pitchd_tens);
               else if(i==1)string=QString::number(pitchd_tens-10);
               else string=QString::number(pitchd_tens+10);
               painter.drawText(QPointF(-long_line_length_half-30,lines_y[i]+5), string);
               painter.drawText(QPointF(long_line_length_half+10,lines_y[i]+5), string);
            }
            else
               painter.drawLine(QPointF(-short_line_length_half,lines_y[i]),QPointF(short_line_length_half,lines_y[i]));
        }
    else//最中间为5的倍数，要画短线时
    {
        for(int i=0;i<7;i++)
        {
            if(i%2==1)
               painter.drawLine(QPointF(-short_line_length_half,lines_y[i]),QPointF(short_line_length_half,lines_y[i]));
            else
            {
               painter.drawLine(QPointF(-long_line_length_half,lines_y[i]),QPointF(long_line_length_half,lines_y[i]));
               QString string;
               if(pitchd>=0)
               {
                   if(i==2)string=QString::number(pitchd_tens);
                   else if(i==0)string=QString::number(pitchd_tens-10);
                   else if(i==4)string=QString::number(pitchd_tens+10);
                   else string=QString::number(pitchd_tens+20);
               }
               else
               {
                   if(i==4)string=QString::number(pitchd_tens);
                   else if(i==2)string=QString::number(pitchd_tens-10);
                   else if(i==0)string=QString::number(pitchd_tens-20);
                   else string=QString::number(pitchd_tens+10);
               }
               painter.drawText(QPointF(-long_line_length_half-30,lines_y[i]+5), string);
               painter.drawText(QPointF(long_line_length_half+10,lines_y[i]+5), string);
            }
        }
    }
    painter.restore();

    //画下方罗盘仪表
    painter.save();
    int compass_panel_radius=painting_area_height/3;
    painter.translate(painting_area_width/2,painting_area_height*0.8+compass_panel_radius);//移动坐标系
    painter.rotate(compassd);
    painter.setFont(QFont("Times",10));//字体
    painter.setPen(QPen(Qt::black,1));//画笔
    painter.setBrush(QBrush(Qt::NoBrush));//画刷不填充
    painter.drawEllipse(-compass_panel_radius,-compass_panel_radius,compass_panel_radius*2,compass_panel_radius*2);
    for(int i=0;i<24;i++)
    {
        painter.drawLine(0, -compass_panel_radius+10, 0, -compass_panel_radius);
        switch(i)
        {
        case 0:{painter.drawText(QPointF(-5.0, -compass_panel_radius+20), tr("N"));break;}
        case 6:{painter.drawText(QPointF(-5.0, -compass_panel_radius+20), tr("E"));break;}
        case 12:{painter.drawText(QPointF(-5.0, -compass_panel_radius+20), tr("S"));break;}
        case 18:{painter.drawText(QPointF(-5.0, -compass_panel_radius+20), tr("W"));break;}
        default:
        {
            if(i%2==0){painter.drawText(QPointF(-5.0, -compass_panel_radius+20), QString::number(i*15));break;}
            else break;
        }
        }
        painter.rotate(15);  //rotate()是顺时针旋转坐标轴
    }

    painter.restore();

    painter.save();
    painter.setPen(QPen(Qt::black,1));//画笔
    painter.translate(painting_area_width/2,painting_area_height*0.8+compass_panel_radius);//移动坐标系
    QPointF compass_arrow_points[3]={
        QPointF(0.0,-compass_panel_radius-3),
        QPointF(-6.0,-compass_panel_radius-15),
        QPointF(6.0,-compass_panel_radius-15)
    };
    painter.drawPolygon(compass_arrow_points,3);//指针三角形
    painter.restore();
    //画下方地磁方向数值
    painter.setFont(QFont("Times",10,QFont::Bold));//字体
    painter.setPen(QPen(Qt::black,1));//画笔
    painter.setBrush(QBrush(Qt::NoBrush));//画刷不填充
    QRectF compass_rect(painting_area_width/2-15, painting_area_height-20,30,12);
    painter.drawRect(compass_rect);
    painter.drawText(compass_rect, Qt::AlignCenter,QString::number(360-(int)compassd));

    //画不动的红色指针
    painter.setPen(QPen(Qt::red,2));
    painter.drawLine(QPointF(80.0,painting_area_height/2),QPointF(130.0,painting_area_height/2));
    painter.drawLine(QPointF(painting_area_width-80.0,painting_area_height/2),QPointF(painting_area_width-130.0,painting_area_height/2));
    painter.drawLine(QPointF(painting_area_width/2,painting_area_height/2),QPointF(painting_area_width/2-10,painting_area_height/2+10));
    painter.drawLine(QPointF(painting_area_width/2,painting_area_height/2),QPointF(painting_area_width/2+10,painting_area_height/2+10));

    //painter.end();
}



/**********飞行界面地面区域绘图计算**********/
void StatusPainter::painting_Calculate()
{
    //******与边框交点计算******//
    double k=tan(roll);//斜率
    double u=painting_area_width/2;
    double v=painting_area_height/2;
    double w=pitchd*painting_area_height/120;//60度满屏
    double lefty=v+w-u*k;
    double righty=lefty+painting_area_width*k;
    double upx=u-(v+w)/k;
    double downx=upx+painting_area_height/k;
    //*********8421分类***********//
    int num=0;
    if(lefty>=0&&lefty<=painting_area_height)num+=1;
    if(downx>0&&downx<painting_area_width)num+=2;
    if(righty>=0&&righty<=painting_area_height)num+=4;
    if(upx>0&&upx<painting_area_width)num+=8;
    //**********判断并设定绘图点***********/
    switch(num)
    {
    case 5:
    {
         ground_points[0]=QPointF(0.0,lefty);
         ground_points[1]=QPointF(0.0,painting_area_height);
         ground_points[2]=QPointF(painting_area_width,painting_area_height);
         ground_points[3]=QPointF(painting_area_width,righty);
         ground_points[4]=  ground_points[3];
        break;
    }
    case 10:
    {
        if(rolld<=0)
        {
             ground_points[0]=QPointF(downx,painting_area_height);
             ground_points[1]=QPointF(painting_area_width,painting_area_height);
             ground_points[2]=QPointF(painting_area_width,0.0);
             ground_points[3]=QPointF(upx,0.0);
             ground_points[4]=  ground_points[3];
            break;
        }
        else
        {
             ground_points[0]=QPointF(downx,painting_area_height);
             ground_points[1]=QPointF(upx,0.0);
             ground_points[2]=QPointF(0.0,0.0);
             ground_points[3]=QPointF(0.0,painting_area_height);
             ground_points[4]=  ground_points[3];
            break;
        }
    }
    case 9:
    {
        if(rolld<=0)
        {
             ground_points[0]=QPointF(0.0,lefty);
             ground_points[1]=QPointF(0.0,painting_area_height);
             ground_points[2]=QPointF(painting_area_width,painting_area_height);
             ground_points[3]=QPointF(painting_area_width,0.0);
             ground_points[4]=QPointF(upx,0.0);
            break;
        }
        else
        {
             ground_points[0]=QPointF(0.0,lefty);
             ground_points[1]=QPointF(upx,0.0);
             ground_points[2]=QPointF(0.0,0.0);
             ground_points[3]=QPointF(0.0,0.0);
             ground_points[4]=  ground_points[3];
            break;
        }
    }
    case 12:
    {
        if(rolld>=0)
        {
             ground_points[0]=QPointF(upx,0.0);
             ground_points[1]=QPointF(0.0,0.0);
             ground_points[2]=QPointF(0.0,painting_area_height);
             ground_points[3]=QPointF(painting_area_width,painting_area_height);
             ground_points[4]=QPointF(painting_area_width,righty);
            break;
        }
        else
        {
             ground_points[0]=QPointF(upx,0.0);
             ground_points[1]=QPointF(painting_area_width,righty);
             ground_points[2]=QPointF(painting_area_width,0.0);
             ground_points[3]=QPointF(painting_area_width,0.0);
             ground_points[4]=  ground_points[3];
            break;
        }
    }
    case 3:
    {
        if(rolld>=0)
        {
             ground_points[0]=QPointF(0.0,lefty);
             ground_points[1]=QPointF(0.0,painting_area_height);
             ground_points[2]=QPointF(downx,painting_area_height);
             ground_points[3]=  ground_points[2];
             ground_points[4]=  ground_points[3];
            break;
        }
        else
        {
             ground_points[0]=QPointF(0.0,lefty);
             ground_points[1]=QPointF(downx,painting_area_height);
             ground_points[2]=QPointF(painting_area_width,painting_area_height);
             ground_points[3]=QPointF(painting_area_width,0.0);
             ground_points[4]=QPointF(0.0,0.0);
            break;
        }
    }
    case 6:
    {
        if(rolld<=0)
        {
             ground_points[0]=QPointF(downx,painting_area_height);
             ground_points[1]=QPointF(painting_area_width,painting_area_height);
             ground_points[2]=QPointF(painting_area_width,righty);
             ground_points[3]=  ground_points[2];
             ground_points[4]=  ground_points[3];
            break;
        }
        else
        {
             ground_points[0]=QPointF(downx,painting_area_height);
             ground_points[1]=QPointF(painting_area_width,righty);
             ground_points[2]=QPointF(painting_area_width,0.0);
             ground_points[3]=QPointF(0.0,0.0);
             ground_points[4]=QPointF(0.0,painting_area_height);
            break;
        }
    }
    }
}
