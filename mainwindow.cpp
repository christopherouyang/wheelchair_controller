#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LTSMC.h"
#include <QDebug>
#include <QMessageBox>
#include <eigen3/Eigen/Dense>
#include "math.h"
#include <unistd.h>

using Eigen::MatrixXd;

static short connection;
const double radius = 0.18;
const double space = 0.555;
const double pi = 3.14159268;
const double coeff = 2*pi*radius/320000;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initDialog();
    connection = smc_board_init(0,2,"192.168.5.11",0);
    if(connection !=0) //检查控制卡是否连接成功
    {
        qDebug("smc_board_init iret = %d\n",connection);
        qDebug("连接失败！请检查控制卡的连接");
        information_connection_fail();
    }
    else
    {
        qDebug("控制卡连接成功！");
        information_connection_success();
    }
    startTimer(200); //定义类QTimerEvent的刷新时间(ms)
}

void MainWindow::information_connection_fail()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "连接失败！请检查控制卡的连接";
    reply= QMessageBox::information(this,tr("Connection Fails"),MESSAGE);
}

void MainWindow::information_connection_success()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "控制卡连接成功";
    reply= QMessageBox::information(this,tr("Connection Success"),MESSAGE);
}

void::MainWindow::information_disable()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "电机未使能,请先将电机使能";
    reply= QMessageBox::information(this,tr("Motor is disabled"),MESSAGE);
}
void::MainWindow::information_disable_0()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "左轮电机未使能,请先将电机使能";
    reply= QMessageBox::information(this,tr("Motor_0 is disabled"),MESSAGE);
}
void::MainWindow::information_disable_1()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "右轮电机未使能,请先将电机使能";
    reply= QMessageBox::information(this,tr("Motor_1 is disabled"),MESSAGE);
}
void::MainWindow::information_emgstop_on()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "急停开关被按下,请先释放急停开关再使能";
    reply= QMessageBox::information(this,tr("EMG stop is on"),MESSAGE);
}
void MainWindow::information_connection_interrupted()
{
    QMessageBox::StandardButton reply;
    QString MESSAGE = "连接中断！请检查控制卡的连接";
    reply= QMessageBox::information(this,tr("Connection is interrupted"),MESSAGE);
}



void MainWindow::initDialog()
{
    ui->textEdit_PortNo->setText("0");//默认port_no为0

    ui->checkBox_axis_0->click();
    ui->checkBox_axis_1->click();//默认两个轮椅的使能状态都被选中

    //设定两个轮子单独运动时，电机速度等参数的默认值
    ui->checkBox_axis_l->click();
    ui->checkBox_axis_r->click(); //默认两个轮子的运动都被选中
    ui->radioButton_fw->click();
    ui->radioButton_fw_2->click();//默认两个轮子的方向均为前进

    ui->radioButton_cs->click();

    ui->textEdit_startvel->setText("100");
    ui->textEdit_runvel->setText("32000");
    ui->textEdit_stopvel->setText("100");
    ui->textEdit_acctime->setText("0.1");
    ui->textEdit_dectime->setText("0.1");
    ui->textEdit_stime->setText("0.05");
    ui->textEdit_destpos->setText("20000");
    ui->textEdit_pulse->setText("320000"); //左轮

    ui->textEdit_startvel_2->setText("100");
    ui->textEdit_runvel_2->setText("32000");
    ui->textEdit_stopvel_2->setText("100");
    ui->textEdit_acctime_2->setText("0.1");
    ui->textEdit_dectime_2->setText("0.1");
    ui->textEdit_stime_2->setText("0.05");
    ui->textEdit_destpos_2->setText("20000");
    ui->textEdit_pulse_2->setText("320000");//右轮


    ui->radioButton_cs_2->click();//默认轮椅运动为匀速运动
    //设定轮椅匀速运动默认参数
    ui->textEdit_linear_vel->setText("0.1"); //默认移动的线速度为0.1m/s
    ui->slider_linear_vel->setValue(10);
    ui->textEdit_angular_vel->setText("0");//默认移动的角速度为0
    ui->slider_angular_vel->setValue(0);

    //轮椅定长运动参数
    ui->textEdit_goal_x->setText("0.2");
    ui->textEdit_goal_y->setText("0");
    ui->textEdit_goal_theta->setText("0"); //默认移动方式：前进0.2m
    ui->textEdit_goal_dv->setText("0.15");//默认直线运动速度为0.2m/s
    ui->textEdit_goal_dtheta->setText("15"); //默认角速度为15°/s

}

MainWindow::~MainWindow()
{
    delete ui;
}

// void MainWindow::RefreshUI()
// {
//     short iret=0;
//     double pos[2];
//     double enc[2];
//     double speed[2];
//     for (int i=0;i<2;i++)
//     {
//         iret = smc_get_position_unit(0,i,&pos[i]);
//         iret = smc_get_encoder_unit(0,i,&enc[i]);
//         iret = smc_read_current_speed_unit(0,i,&speed[i]);
//     }
//     ui->textEdit_prfPosX->setText(QString::number(pos[0],'f',3));
//     ui->textEdit_prfPosY->setText(QString::number(pos[1],'f',3));
//     //
//     ui->textEdit_encPosX->setText(QString::number(enc[0],'f',3));
//     ui->textEdit_encPosY->setText(QString::number(enc[1],'f',3));
//     //
//     ui->textEdit_SpeedX->setText(QString::number(speed[0],'f',3));
//     ui->textEdit_SpeedY->setText(QString::number(speed[1],'f',3));

// }


void MainWindow::timerEvent(QTimerEvent *e)
{
    short iret[2]={0,0};
    double pos[2]={0.0,0.0};
    double enc[2]={0.0,0.0};
    double speed[2]={0.0,0.0};
    double linear_v=0;
    double angular_v=0;
    DWORD status[2]={1,1};
    WORD runmode[2]={0,0};
    for (int i=0;i<2;i++)
    {
        iret[i] = smc_get_position_unit(0,i,&pos[i]);
        iret[i] = smc_get_encoder_unit(0,i,&enc[i]);
        iret[i] = smc_read_current_speed_unit(0,i,&speed[i]);
        status[i] = smc_check_done(0, i );
        iret[i] = smc_get_axis_run_mode(0,i,&runmode[i]);
    }
    //由每个轮子的速度获取轮椅速度，负号是因为电机正方向对应的是轮椅的后退反向
    linear_v=-(speed[0]+speed[1])*coeff/2;
    angular_v=-(speed[1]-speed[0])*coeff/space*180/pi;

    if (status[0]==1)
    {
        ui->label_status_0->setText("Static");
    }
    else
    {
        ui->label_status_0->setText("Running");
    }

    if (status[1]==1)
    {
        ui->label_status_1->setText("Static");
    }
    else
    {
        ui->label_status_1->setText("Running");
    }

    if(runmode[0]==0)
    {
        ui->label_mode_0->setText("Standby");
    }
    else if(runmode[0]==1)
    {
        ui->label_mode_0->setText("Fixed Length");
    }
    else if(runmode[0]==2)
    {
        ui->label_mode_0->setText("Constant Speed");
    }

    if(runmode[1]==0)
    {
        ui->label_mode_1->setText("Standby");
    }
    else if(runmode[1]==1)
    {
        ui->label_mode_1->setText("Fixed Length");
    }
    else if(runmode[1]==2)
    {
        ui->label_mode_1->setText("Constant Speed");
    }

    ui->textEdit_prfPosX->setText(QString::number(pos[0],'f',3));
    ui->textEdit_prfPosY->setText(QString::number(pos[1],'f',3));  
    //
    ui->textEdit_encPosX->setText(QString::number(enc[0],'f',3));
    ui->textEdit_encPosY->setText(QString::number(enc[1],'f',3));
    //
    ui->textEdit_SpeedX->setText(QString::number(speed[0],'f',3));
    ui->textEdit_SpeedY->setText(QString::number(speed[1],'f',3));
    //
    ui->textEdit_linear_current_vel->setText(QString::number(linear_v,'f',3));
    ui->textEdit_angular_current_vel->setText(QString::number(angular_v,'f',3));

    short timeout;
    short status_connect;
    timeout = smc_set_connect_timeout(0);
    status_connect = smc_get_connect_status(0);//读取实时的连接状态


    if(connection==0)
    {
        emg_stop();//与控制器连接成功时调用IO急停信号
        if(status_connect!=1)//如果连接失败,则立刻急停
        {
            for (int i =0;i<2;i++)
            {
                iret[i] =smc_stop(0,i,0);
            }
            on_pushButton_disable_clicked();
            information_connection_interrupted();
        }
    }


    //printf("连接延时%d ms,连接状态%d\n",timeout,status_connect);

    //connect(ui->slider_linear_vel,SIGNAL(valueChanged(int)),ui->textEdit_linear_vel,SLOT(setText(QString::number(int,'f',3))));
    //ui->slider_linear_vel->sliderReleased();
    //connect(ui->slider_linear_vel, SIGNAL(valueChanged(int)), ui->lineEdit, SLOT(setLineEditValue(int)));
}


//IO 急停信号
void MainWindow::emg_stop()
{
    /*********************变量定义****************************/
    WORD MyCardNo = 0; //连接号
    WORD Myaxis[2] = {0,1}; //轴号
    short ret[2] = {0,0}; //错误返回
    WORD Myenable[2] = {1,1}; //急停信号使能
    WORD Mylogic[2] = {1, 1}; //急停信号高电平有效
    /*********************函数调用执行**************************/
    //第一步、设置轴 IO 映射，将通用输入 0 作为各轴的急停信号
    short io_0 = smc_read_inbit(MyCardNo,0); //读取IO口的电平值

    for(int i = 0 ; i<2; i++)
    {
        ret[i]=smc_set_axis_io_map(MyCardNo, Myaxis[i],3, 6, 0, 0);
    }
    //第二步、设置 EMG 使能，高电平有效
    for(int i = 0 ; i<2; i++)
    {
        ret[i]=smc_set_emg_mode(MyCardNo, Myaxis[i], Myenable[i],Mylogic[i]);
    }
    if (io_0==1)
    {
        on_pushButton_disable_clicked();
    }
    //第三步、回读 EMG 使能，高电平有效
    for(int i = 0 ; i<2; i++)
    {
        ret[i]=smc_get_emg_mode(MyCardNo, Myaxis[i],&Myenable[i],&Mylogic[i]);
        //printf("%d 轴急停信号参数,使能,有效电平= %d %d\n ",i,Myenable[i],Mylogic[i]);
    }
}

void MainWindow::on_pushButton_enable_clicked() //轴使能操作函数
{
    // TODO: Add your control notification handler code here
    time_t t1,t2; //设置时间监控变量，在等待轴状态机变化时防止死循环使用
    unsigned long errcode=0; //总线错误代码
    short iret[2] = {0,0};
    short statemachine[2]={1,1};
    int m_nConnectNo=0;
    nmcs_get_errcode(m_nConnectNo,2,&errcode); //获取总线状态

    if(errcode==0) //总线正常才允许使能操作
    {
        short io_0 = smc_read_inbit(0,0);
        if (io_0==1)
        {
            on_pushButton_disable_clicked();
            information_emgstop_on();
            return;
        }
        if (ui->checkBox_axis_0->isChecked() && ui->checkBox_axis_1->isChecked())
        {
            for(int i=0; i<2; i++)
            {
                iret[i] = smc_write_sevon_pin(m_nConnectNo,i,0);//设置0&1轴使能
                statemachine[i]=smc_read_sevon_pin(m_nConnectNo,i);//获取0&1轴状态机
            }
            t1=time(NULL); //设置时间
            while(statemachine[0]==1&& statemachine[1]==1) //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("0、1轴使能超时，请检查设备");
                    return;
                }
                iret[0] = smc_write_sevon_pin(m_nConnectNo,0,0); //设置0轴使能
                statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0); //获取0轴状态机
                iret[1] = smc_write_sevon_pin(m_nConnectNo,1,0); //设置1轴使能
                statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1); //获取1轴状态机
            }
            while(statemachine[0]==1&& statemachine[1]==0) //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("0轴使能超时，请检查设备");
                    return;
                }
                iret[0] = smc_write_sevon_pin(m_nConnectNo,0,0); //设置0轴使能
                statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0); //获取0轴状态机
            }
            while(statemachine[1]==1&& statemachine[0]==0) //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("1轴使能超时，请检查设备");
                    return;
                }
                iret[1] = smc_write_sevon_pin(m_nConnectNo,1,0); //设置1轴使能
                statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1); //获取1轴状态机
            }
            ui->label_error->setText("0、1轴使能成功");
        }
        else if(ui->checkBox_axis_0->isChecked())
        {
            iret[0] = smc_write_sevon_pin(m_nConnectNo,0,0);//设置指定0轴使能
            statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0);//获取0轴状态机
            t1=time(NULL); //设置时间
            while(statemachine[0]==1) //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("0轴使能超时，请检查设备");
                    return;
                }
            iret[0] = smc_write_sevon_pin(m_nConnectNo,0,0); //设置0轴使能
            statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0); //获取0轴状态机
            }
            iret[1] = smc_write_sevon_pin(m_nConnectNo,1,0);//设置指定1轴使能
            statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1);//获取1轴状态机
            while(statemachine[1]==0)//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("1轴去使能失败，请检查设备");
                    return;
                }
                iret[1] = smc_write_sevon_pin(m_nConnectNo,1,1);//设置1轴去使能
                statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1); //获取1轴状态机
            }
            ui->label_error->setText("0轴使能成功");
        }
        else if(ui->checkBox_axis_1->isChecked())
        {
            iret[1] = smc_write_sevon_pin(m_nConnectNo,1,0);//设置指定1轴使能
            statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1);//获取1轴状态机
            t1=time(NULL); //设置时间
            while(statemachine[1]==1) //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("1轴使能超时，请检查设备");
                return;
                }
            iret[1] = smc_write_sevon_pin(m_nConnectNo,1,0); //设置1轴使能
            statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1); //获取1轴状态机
            }

            iret[0] = smc_write_sevon_pin(m_nConnectNo,0,1);//设置0轴去使能
            statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0);//获取0轴状态机
            while(statemachine[0]==0)//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_error->setText("0轴去使能失败，请检查设备");
                    return;
                }
                iret[0] = smc_write_sevon_pin(m_nConnectNo,0,1);//设置0轴去使能
                statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0);//获取0轴状态机
            }

            ui->label_error->setText("1轴使能成功");
        }
    }
    else //总线不正常状态下不响应使能操作
    {
        ui->label_error->setText("总线错误，禁止操作！");
        return;
    }
}

bool MainWindow::on_pushButton_disable_clicked() //轴去使能操作函数
{
    // TODO: Add your control notification handler code here
    time_t t1,t2; //设置时间监控变量，在等待轴状态机变化时防止死循环使用
    unsigned long errcode=0; //总线错误代码
    short iret[2] = {0,0};
    short statemachine[2]={0,0};
    int m_nConnectNo=0;
    nmcs_get_errcode(m_nConnectNo,2,&errcode); //获取总线状态
    if(errcode==0)
    {
        for(int i=0; i<2; i++)
        {
            iret[i] = smc_write_sevon_pin(m_nConnectNo,i,1);//设置0&1轴使能
            statemachine[i]=smc_read_sevon_pin(m_nConnectNo,i);//获取0&1轴状态机
        }
        t1=time(NULL); //设置时间
        while(statemachine[0]==0)//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
        {
            t2=time(NULL);
            if(t2-t1 > 3) //3 秒时间防止死循环
            {
                ui->label_error->setText("0轴去使能失败，请检查设备");
                return false;
            }
            iret[0] = smc_write_sevon_pin(m_nConnectNo,0,1);//设置0轴去使能
            statemachine[0]=smc_read_sevon_pin(m_nConnectNo,0);//获取0轴状态机
        }
        while(statemachine[1]==0)//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
        {
            t2=time(NULL);
            if(t2-t1 > 3) //3 秒时间防止死循环
            {
                ui->label_error->setText("1轴去使能失败，请检查设备");
                return false;
            }
            iret[1] = smc_write_sevon_pin(m_nConnectNo,1,1);//设置1轴去使能
            statemachine[1]=smc_read_sevon_pin(m_nConnectNo,1); //获取1轴状态机
        }
        ui->label_error->setText("0、1轴去使能");
    }
    else //总线不正常状态下不响应去使能操作
    {
        ui->label_error->setText("总线错误，禁止操作！");
        return false;
    }
    return true;
}


//open io
void MainWindow::on_pushButton_openio_clicked()
{
    WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
    short iret = smc_write_outbit(0,ioNo,0);
    qDebug("smc_write_outbit(0,%d,0) iret=%d\n",ioNo,iret);
}
//close io
void MainWindow::on_pushButton_closeio_clicked()
{

    WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
    short iret = smc_write_outbit(0,ioNo,1);
    qDebug("smc_write_outbit(0,%d,1) iret=%d\n",ioNo,iret);
}

//start
void MainWindow::on_pushButton_start_clicked()
{

    double startvel[2] = {ui->textEdit_startvel->toPlainText().toDouble(),ui->textEdit_startvel_2->toPlainText().toDouble()};
    double runvel[2] = {ui->textEdit_runvel->toPlainText().toDouble(),ui->textEdit_runvel_2->toPlainText().toDouble()};
    //限制每个轮子的最大线速度为0.8m/s
    for (int i=0;i<2;i++)
    {
        if(runvel[i]>0.8/coeff)
        {
            runvel[i]=0.8/coeff;
        }
        else if(runvel[i]<(-0.8/coeff))
        {
            runvel[i]=-0.8/coeff;
        }
    }
    ui->textEdit_runvel->setText(QString::number(runvel[0],'f',3));
    ui->textEdit_runvel_2->setText(QString::number(runvel[1],'f',3)); //将限制的速度显示在QT界面上

    double stopvel[2] = {ui->textEdit_stopvel->toPlainText().toDouble(),ui->textEdit_stopvel_2->toPlainText().toDouble()};

    double acctime[2] = {runvel[0]/150000 ,runvel[1]/150000};
    double dectime[2] = {runvel[0]/150000 ,runvel[1]/150000};

    double stime[2] = {ui->textEdit_stime->toPlainText().toDouble(),ui->textEdit_stime_2->toPlainText().toDouble()};
    //double destpos = ui->textEdit_destpos->toPlainText().toDouble();
    double pulse[2] = {ui->textEdit_pulse->toPlainText().toDouble(),ui->textEdit_pulse_2->toPlainText().toDouble()};
    int direction[2];
    if (ui->radioButton_fw->isChecked())
    {
        direction[0]=1;
    }
    else
    {
        direction[0]=0;
    }

    if (ui->radioButton_fw_2->isChecked())
    {
        direction[1]=1;
    }
    else
    {
        direction[1]=0;
    }

    for (int i=0; i<2 ; i++)
    {
        //如果runvel为负数,则将direction反向,runvel改为正数
        if(runvel[i]<0)
        {
            direction[i]=abs(1-direction[i]);
            runvel[i]=-runvel[i];
        }
    }

    short statemachine[2]={1,1};
    WORD axisNo[2] ={0,1};
    short iret[2] = {0,0};
    for(int i = 0; i < 2 ; i++)
    {
        if (smc_check_done( 0, axisNo[i] ) == 0) //该轴已经在运动中
            return;
        if (ui->checkBox_axis_l->isChecked() && ui->checkBox_axis_r->isChecked())
        {
            statemachine[0]=smc_read_sevon_pin(0,0);//获取0轴状态机
            statemachine[1]=smc_read_sevon_pin(0,1);//获取1轴状态机
            if (statemachine[1]==1 || statemachine[0]==1 )//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                information_disable();//返回错误信号,停止该函数的运行
                return;
            }
        }

        if(ui->checkBox_axis_l->isChecked())
        {
            statemachine[0]=smc_read_sevon_pin(0,0);//获取0轴状态机
            if (statemachine[0]==1)//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                information_disable_0();//返回错误信号,停止该函数的运行
                return;
            }
            iret[0] = smc_set_equiv( 0, axisNo[0], 1);//设置脉冲当量
            iret[0] = smc_set_alm_mode(0,axisNo[0],0,0,0); //设置报警使能,关闭报警
            iret[0] = smc_set_pulse_outmode(0 , axisNo[0], 0);//设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
            iret[0] = smc_set_profile_unit(0,axisNo[0],startvel[0],runvel[0],acctime[0],dectime[0],stopvel[0]);//设定单轴运动速度参数
            iret[0] = smc_set_s_profile(0,axisNo[0],0,stime[0]);
            if(ui->radioButton_fl->isChecked())
            {
                iret[0] = smc_pmove_unit(0,axisNo[0],pulse[0]*(2*direction[0]-1),0); //相对定长运动
                //iret[0] = smc_pmove_unit(0,axisNo[0],pulse[0],0); //相对定长运动
            }
            else
            {
                iret[0] = smc_vmove(0,axisNo[0],direction[0]); //恒速运动
            }
        }
        if(ui->checkBox_axis_r->isChecked())
        {
            statemachine[1]=smc_read_sevon_pin(0,1);//获取1轴状态机
            if (statemachine[1]==1)//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
            {
                information_disable_1();//返回错误信号,停止该函数的运行
                return;
            }
            iret[1] = smc_set_equiv( 0, axisNo[1], 1);//设置脉冲当量
            iret[1] = smc_set_alm_mode(0,axisNo[1],0,0,0); //设置报警使能,关闭报警
            iret[1] = smc_set_pulse_outmode(0 , axisNo[1], 0);//设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
            iret[1] = smc_set_profile_unit(0,axisNo[1],startvel[1],runvel[1],acctime[1],dectime[1],stopvel[1]);//设定单轴运动速度参数
            iret[1] = smc_set_s_profile(0,axisNo[1],0,stime[1]);
            if(ui->radioButton_fl->isChecked())
            {
                iret[1] = smc_pmove_unit(0,axisNo[1],pulse[1]*(2*direction[1]-1),0); //相对定长运动
            }
            else
            {
                iret[i] = smc_vmove(0,axisNo[1],direction[1]); //恒速运动
            }
        }
        }
}

//decstop
void MainWindow::on_pushButton_decstop_clicked()
{

    double dectime[2] = {ui->textEdit_dectime->toPlainText().toDouble(),ui->textEdit_dectime_2->toPlainText().toDouble()};
    short iret[2]={0,0};
    double actuvel[2];
    for (int i=0;i<2;i++)
    {
       iret[i] = smc_read_current_speed_unit(0,i,&actuvel[i]);
       double acc = fabs(actuvel[i]/dectime[i]);
       if (acc>20000)
       {
           dectime[i]=fabs(actuvel[i]/100000); //如果减速的加速度>100000,修改减速时间使得加速度为100000
       }
    }
    ui->textEdit_dectime->setText(QString::number(dectime[0],'f',3));
    ui->textEdit_dectime_2->setText(QString::number(dectime[1],'f',3)); //将修改后的减速时间显示在QT界面上

    for (int i =0;i<2;i++)
    {
        smc_set_dec_stop_time(0,i,dectime[i]);//设置减速停止时间

        iret[i] =smc_stop(0,i,0);//减速停止
    }
}
void MainWindow::on_pushButton_decstop_2_clicked()
{
    on_pushButton_decstop_clicked();
}

//emgstop
void MainWindow::on_pushButton_emgstop_clicked()
{
    short iret[2]={0,0};
    for (int i =0;i<2;i++)
    {
        iret[i] =smc_stop(0,i,0);
    }
}
void MainWindow::on_pushButton_emgstop_2_clicked()
{
    short iret[2]={0,0};
    for (int i =0;i<2;i++)
    {
        iret[i] =smc_stop(0,i,0);
    }
}



//zero pos
void MainWindow::on_pushButton_zeropos_clicked()
{
    short axisNo=0;
    short iret=0;
    if(ui->checkBox_axis_l->isChecked())
    {
        axisNo =0;
        iret =smc_set_position_unit(0,axisNo,0);
    }
    if(ui->checkBox_axis_r->isChecked())
    {
        axisNo =1;
        iret =smc_set_position_unit(0,axisNo,0);
    }
}
//enc pos
void MainWindow::on_pushButton_encpos_clicked()
{
    short axisNo=0;
    short iret=0;
    if(ui->checkBox_axis_l->isChecked())
    {
        axisNo =0;
        iret =smc_set_encoder_unit(0,axisNo,0);
    }
    if(ui->checkBox_axis_r->isChecked())
    {
        axisNo =1;
        iret =smc_set_encoder_unit(0,axisNo,0);
    }

}
//stop crd
void MainWindow::on_pushButton_stopcrd_clicked()
{
    short iret =0;
    iret = smc_stop_multicoor(0,0,0);
    iret = smc_stop_multicoor(0,1,0);
}
//change vel
void MainWindow::on_pushButton_changevel_clicked()
{
    WORD axisNo=0;
    short iret=0;
    double runvel[2] = {ui->textEdit_runvel->toPlainText().toDouble(),ui->textEdit_runvel_2->toPlainText().toDouble()};
    //限制每个轮子的最大线速度为0.8m/s
    for (int i=0;i<2;i++)
    {
        if(runvel[i]>0.8/coeff)
        {
            runvel[i]=0.8/coeff;

        }
        else if(runvel[i]<(-0.8/coeff))
        {
            runvel[i]=-0.8/coeff;
        }
    }
    ui->textEdit_runvel->setText(QString::number(runvel[0],'f',3));
    ui->textEdit_runvel_2->setText(QString::number(runvel[1],'f',3)); //将限制的速度显示在QT界面上

    //根据变速前后的速度差值决定变速的时间
    double actuvel[2];
    double time[2];
    for (int i=0;i<2;i++)
    {
       iret = smc_read_current_speed_unit(0,i,&actuvel[i]);
       double diff = fabs(runvel[i]-actuvel[i]);
       if (diff<10000)
       {
           time[i]=0; //如果差值diff<10000,则时间为0
       }
       else
       {
           time[i]=diff/100000; //反之,则时间为diff/100000
       }
    }


    if(ui->checkBox_axis_l->isChecked())
    {
        axisNo =0;
        iret =smc_change_speed_unit(0,axisNo,runvel[0],time[0]);
    }
    if(ui->checkBox_axis_r->isChecked())
    {
        axisNo =1;
        iret =smc_change_speed_unit(0,axisNo,runvel[1],time[1]);
    }
}
//change pos
void MainWindow::on_pushButton_changepos_clicked()
{
    WORD axisNo=0;
    short iret=0;
    double destpos[2] = {ui->textEdit_destpos->toPlainText().toDouble(),ui->textEdit_destpos_2->toPlainText().toDouble()};


    if(ui->checkBox_axis_l->isChecked())
    {
        axisNo =0;
        iret =smc_reset_target_position_unit(0,axisNo,destpos[0]);
    }
    if(ui->checkBox_axis_r->isChecked())
    {
        axisNo =1;
        iret =smc_reset_target_position_unit(0,axisNo,destpos[1]);
    }

}


void MainWindow::on_pushButton_line_clicked()
{

    short iret=0;
    WORD axisList[2] = {0,1};
    double posList[4]={10000,20000,30000,25000};
    for (int i=0;i<2;i++)
    {
        iret = smc_set_equiv(0,i,1);
        iret = smc_set_alm_mode(0,i,0,0,0);
    }
     iret =smc_set_vector_profile_unit(0,0,0,8000,0.1,0.1,0);
     iret = smc_line_unit(0,0,4,axisList,posList,0);
}

//exit board&application
void MainWindow::on_pushButton_exit_clicked()
{

    if (connection ==0)
    {
        bool disable_success = false;
        disable_success = MainWindow::on_pushButton_disable_clicked();
        if(disable_success == true)
        {
            smc_board_close(0);
            qApp->exit(0);
        }
        else
        {
            return;
        }
    }
    else
    {
        smc_board_close(0);
        qApp->exit(0);
    }

}
void MainWindow::on_pushButton_exit_2_clicked()
{
    on_pushButton_exit_clicked(); //该按钮和另一个exit效果相同
}


void MainWindow::on_pushButton_start_wc_clicked()
{

    //定义反速度关系矩阵
    MatrixXd trans_2(2,2);
    trans_2(0,0)=1;
    trans_2(0,1)=space/2;
    trans_2(1,0)=1;
    trans_2(1,1)=-space/2;

    MatrixXd trans(2,2);
    trans = trans_2 / coeff;

    //定义轮椅速度矩阵
    MatrixXd v_wheels(2,1);
    MatrixXd v_chairs(2,1);
    v_chairs(0,0)=ui->textEdit_linear_vel->toPlainText().toDouble();
    v_chairs(1,0)=ui->textEdit_angular_vel->toPlainText().toDouble()*pi/180;

    //v_chairs(0,0)=double(ui->slider_linear_vel->value())/100;
    //v_chairs(1,0)=double(ui->slider_angular_vel->value())*pi/180;

    v_wheels = trans * v_chairs; //由轮椅速度反解出电机速度

    double startvel[2] = {ui->textEdit_startvel->toPlainText().toDouble(),ui->textEdit_startvel_2->toPlainText().toDouble()};
    double runvel[3][2] = {
        {0,0},
        {v_wheels(1,0),v_wheels(0,0)},
        {0,0},
    };
    //限制每个轮子的最大线速度为0.8m/s
    for(int j=0; j<3; j++)
    {
        for (int i=0;i<2;i++)
        {
            if(runvel[j][i]>0.8/coeff)
            {
                runvel[j][i]=0.8/coeff;
            }
            else if(runvel[j][i]<(-0.8/coeff))
            {
                runvel[j][i]=-0.8/coeff;
            }
        }
    }
    double stopvel[2] =
    //{
        //{runvel[0],runvel[1]},
        //{runvel[0],runvel[1]},
        //{100,100},
        //{100,100},
        {ui->textEdit_stopvel->toPlainText().toDouble(),ui->textEdit_stopvel_2->toPlainText().toDouble()}
    //}
    ;

    double acctime[2] = {v_wheels(1,0)/150000 ,v_wheels(0,0)/150000};
    double dectime[2] = {v_wheels(1,0)/150000 ,v_wheels(0,0)/150000};
    double stime[2] = {ui->textEdit_stime->toPlainText().toDouble(),ui->textEdit_stime_2->toPlainText().toDouble()};

    int direction[2]={0,0};//因为电机正方向对应的是轮椅的后退反向，所以电机方向默认为负


    for (int i=0; i<2 ; i++)
    {
        //如果runvel[1]为负数,则将direction反向,runvel[1]改为正数
        if(runvel[1][i]<0)
        {
            direction[i]=abs(1-direction[i]);
            runvel[1][i]=-runvel[1][i];
        }
    }

    WORD axisNo[2] ={0,1};
    short iret[2] = {0,0};
    short statemachine[2]={1,1};
    for(int i = 0; i < 2 ; i++)
    {
        if (smc_check_done( 0, axisNo[i] ) == 0) //该轴已经在运动中
            return;
        statemachine[0]=smc_read_sevon_pin(0,0);//获取0轴状态机
        statemachine[1]=smc_read_sevon_pin(0,1);//获取1轴状态机
        if (statemachine[1]==1 || statemachine[0]==1 )//监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
        {
            information_disable();//返回错误信号,停止该函数的运行
            return;
        }

        //设置两轮运动参数
        iret[i] = smc_set_equiv( 0, axisNo[i], 1);//设置脉冲当量
        iret[i] = smc_set_alm_mode(0,axisNo[i],0,0,0); //设置报警使能,关闭报警
        iret[i] = smc_set_pulse_outmode(0 , axisNo[i], 0);//设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
        iret[i] = smc_set_profile_unit(0,axisNo[i],startvel[i],runvel[1][i],acctime[i],dectime[i],stopvel[i]);//设定单轴运动速度参数
        iret[i] = smc_set_s_profile(0,axisNo[i],0,stime[i]);

//        //左轮运动参数
//        iret[0] = smc_set_equiv( 0, axisNo[0], 1);//设置脉冲当量
//        iret[0] = smc_set_alm_mode(0,axisNo[0],0,0,0); //设置报警使能,关闭报警
//        iret[0] = smc_set_pulse_outmode(0 , axisNo[0], 0);//设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
//        iret[0] = smc_set_profile_unit(0,axisNo[0],startvel[0],runvel[0],acctime[0],dectime[0],stopvel[0]);//设定单轴运动速度参数
//        iret[0] = smc_set_s_profile(0,axisNo[0],0,stime[0]);

//        //右轮运动参数
//        iret[1] = smc_set_equiv( 0, axisNo[1], 1);//设置脉冲当量
//        iret[1] = smc_set_alm_mode(0,axisNo[1],0,0,0); //设置报警使能,关闭报警
//        iret[1] = smc_set_pulse_outmode(0 , axisNo[1], 0);//设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
//        iret[1] = smc_set_profile_unit(0,axisNo[1],startvel[1],runvel[1],acctime[1],dectime[1],stopvel[1]);//设定单轴运动速度参数
//        iret[1] = smc_set_s_profile(0,axisNo[1],0,stime[1]);
    }

    if (ui->radioButton_cs_2->isChecked())
    {
        iret[0] = smc_vmove(0,axisNo[0],direction[0]); //恒速运动
        iret[1] = smc_vmove(0,axisNo[1],direction[1]); //恒速运动
    }
    else if(ui->radioButton_fl_2->isChecked()) //开始定长运动
    {
        //定义轮椅和电机的终点速度
        MatrixXd v_chairs_end(2,1);
        v_chairs_end(0,0)=ui->textEdit_goal_dv->toPlainText().toDouble();
        v_chairs_end(1,0)=ui->textEdit_goal_dtheta->toPlainText().toDouble()*pi/180;
        MatrixXd v_wheels_end(2,1);
        v_wheels_end = trans * v_chairs_end; //由轮椅终点速度反解出电机终点速度
        //stopvel[2][0] = v_wheels_end(0,0);
        //stopvel[2][0] = v_wheels_end(1,0);
        //stopvel[2][0]=100;
        //stopvel[2][1]=100;

        //定义电机在轮椅直线运动时的速度大小
        runvel[1][0] = fabs(ui->textEdit_goal_dv->toPlainText().toDouble()/coeff);
        runvel[1][1] = fabs(ui->textEdit_goal_dv->toPlainText().toDouble()/coeff);
        //定义电机在轮椅旋转时的速度大小
        for(int j=0;j<3;j=j+2)
        {
            for (int i=0; i<2 ; i++)
            {
                runvel[j][i]= fabs(ui->textEdit_goal_dtheta->toPlainText().toDouble()*pi/180 *space / coeff /2) ;
            }
        }

        //定义轮椅的终点位置和角度，其中轮椅前进方向为x轴正方向，轮椅左侧垂直于x轴为y轴正方向，逆时针为theta正方向
        double goal_x=ui->textEdit_goal_x->toPlainText().toDouble();
        double goal_y=ui->textEdit_goal_y->toPlainText().toDouble();
        double goal_theta=ui->textEdit_goal_theta->toPlainText().toDouble()*pi/180;
        double delta_theta;
//        if (goal_x==0)
//        {
//            if(goal_y>=0)
//            {
//                delta_theta = pi/2;
//            }
//            else
//            {
//                delta_theta = -pi/2;
//            }
//        }
//        else
//        {
            delta_theta=atan2(goal_y,goal_x); //计算轮椅终点和起点连线与轮椅当前位置的角度,取值范围(-pi/2,pi/2]
        //}
        double dist=sqrt(pow(goal_x,2)+pow(goal_y,2));
        double pulse[3][2];

        MatrixXd theta_chairs_1(2,1);
        theta_chairs_1(0,0)=0;
        theta_chairs_1(1,0)=delta_theta;

        MatrixXd pulse_wheels_1(2,1);
        pulse_wheels_1=trans*theta_chairs_1;

        MatrixXd theta_chairs_2(2,1);
        theta_chairs_2(0,0)=0;
        theta_chairs_2(1,0)=goal_theta-delta_theta;

        MatrixXd pulse_wheels_2(2,1);
        pulse_wheels_2=trans*theta_chairs_2;

        //step 1:轮椅转到面向终点位置（x,y)的方向
        pulse[0][0]=-pulse_wheels_1(0,0);
        pulse[0][1]=-pulse_wheels_1(1,0);

        //step 2:轮椅沿直线运动到终点位置（x,y)
        pulse[1][0]=-dist/coeff;
        pulse[1][1]=-dist/coeff;

        //step 3:轮椅调整到目标角度goal_theta
        pulse[2][0]=-pulse_wheels_2(0,0);
        pulse[2][1]=-pulse_wheels_2(1,0);


        for(int j=0; j<3; j++)
        {
            for(int i=0; i<2; i++)
            {
                iret[i] = smc_set_equiv( 0, axisNo[i], 1);//设置脉冲当量
                iret[i] = smc_set_alm_mode(0,axisNo[i],0,0,0); //设置报警使能,关闭报警
                iret[i] = smc_set_pulse_outmode(0 , axisNo[i], 0);//设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
                iret[i] = smc_set_profile_unit(0,axisNo[i],startvel[i],runvel[j][i],acctime[i],dectime[i],stopvel[i]);//设定单轴运动速度参数
                iret[i] = smc_set_s_profile(0,axisNo[i],0,stime[i]);

                iret[i] = smc_pmove_unit(0,axisNo[i],pulse[j][i],0);

            }
            while(smc_check_done( 0, 0) == 0 || smc_check_done( 0, 1 ) == 0)
            {
                system("pause");
            }

        }


    }

}

void MainWindow::on_pushButton_changevel_wc_clicked()
{
    //定义反速度关系矩阵
    MatrixXd trans_2(2,2);
    trans_2(0,0)=1;
    trans_2(0,1)=space/2;
    trans_2(1,0)=1;
    trans_2(1,1)=-space/2;

    MatrixXd trans(2,2);
    trans = trans_2 / coeff;

    //定义轮椅速度矩阵
    MatrixXd v_wheels(2,1);
    MatrixXd v_chairs(2,1);
    v_chairs(0,0)=ui->textEdit_linear_vel->toPlainText().toDouble();
    v_chairs(1,0)=ui->textEdit_angular_vel->toPlainText().toDouble()*pi/180;
    //v_chairs(0,0)=double(ui->slider_linear_vel->value())/100;
    //v_chairs(1,0)=double(ui->slider_angular_vel->value())*pi/180;


    v_wheels = trans * v_chairs; //由轮椅速度反解出电机速度

    short iret[2]={0,0};
    double runvel[2] = {-v_wheels(1,0),-v_wheels(0,0)};
    //限制每个轮子的最大线速度为0.8m/s
    for (int i=0;i<2;i++)
    {
        if(runvel[i]>0.8/coeff)
        {
            runvel[i]=0.8/coeff;
        }
        else if(runvel[i]<(-0.8/coeff))
        {
            runvel[i]=-0.8/coeff;
        }
    }
     //根据变速前后的速度差值决定变速的时间
    double actuvel[2];
    double time[2];
    for (int i=0;i<2;i++)
    {
        iret[i] = smc_read_current_speed_unit(0,i,&actuvel[i]);
        double diff = fabs(runvel[i]-actuvel[i]);
        if (diff<20000)
        {
            time[i]=0;
        }
        else
        {
            time[i]=diff/100000;
        }
    }

    for(int i=0; i<2 ; i++)
    {
        iret[i] =smc_change_speed_unit(0,i,runvel[i],time[i]);
    }
}


void MainWindow::on_pushButton_changepos_wc_clicked()
{

}

void MainWindow::on_slider_linear_vel_mouseReleased()
{
    double linear_vel=double(ui->slider_linear_vel->value());
    ui->textEdit_linear_vel->setText(QString::number(linear_vel,'f',3));
}
