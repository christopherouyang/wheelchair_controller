#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LTSMC.h"
#include <QDebug>

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    initDialog();
    short iret = smc_board_init(0,2,"192.168.5.11",0);
    if(iret !=0)
    {
        qDebug("smc_board_init rtn = %d\n",iret);
    }
    startTimer(200);


}
void MainWindow::initDialog()
{
    ui->textEdit_startvel->setText("0");
    ui->textEdit_runvel->setText("5000");
    ui->textEdit_stopvel->setText("0");
    ui->textEdit_acctime->setText("0.1");
    ui->textEdit_dectime->setText("0.1");
    ui->textEdit_stime->setText("0.05");
    ui->textEdit_destpos->setText("20000");
    ui->textEdit_PortNo->setText("0");
    ui->radioButton_X->click();
}

MainWindow::~MainWindow()
{
    delete ui;
}
 void MainWindow::RefreshUI()
 {
     short rtn=0;
     double pos[4];
     double enc[4];
     double speed[4];
     for (int i=0;i<4;i++)
     {
         rtn = smc_get_position_unit(0,i,&pos[i]);
         rtn = smc_get_encoder_unit(0,i,&enc[i]);
         rtn = smc_read_current_speed_unit(0,i,&speed[i]);
     }
     ui->textEdit_prfPosX->setText(QString::number(pos[0],'f',3));
     ui->textEdit_prfPosY->setText(QString::number(pos[1],'f',3));
     //
     ui->textEdit_encPosX->setText(QString::number(enc[0],'f',3));
     ui->textEdit_encPosY->setText(QString::number(enc[1],'f',3));
     //
     ui->textEdit_SpeedX->setText(QString::number(speed[0],'f',3));
     ui->textEdit_SpeedY->setText(QString::number(speed[1],'f',3));
 }
void MainWindow::timerEvent(QTimerEvent *e)
{
    short rtn=0;
    double pos[2];
    double enc[2];
    double speed[2];
    for (int i=0;i<2;i++)
    {
        rtn = smc_get_position_unit(0,i,&pos[i]);
        rtn = smc_get_encoder_unit(0,i,&enc[i]);
        rtn = smc_read_current_speed_unit(0,i,&speed[i]);
    }
    ui->textEdit_prfPosX->setText(QString::number(pos[0],'f',3));
    ui->textEdit_prfPosY->setText(QString::number(pos[1],'f',3));  
    //
    ui->textEdit_encPosX->setText(QString::number(enc[0],'f',3));
    ui->textEdit_encPosY->setText(QString::number(enc[1],'f',3));
    //
    ui->textEdit_SpeedX->setText(QString::number(speed[0],'f',3));
    ui->textEdit_SpeedY->setText(QString::number(speed[1],'f',3));

}

void MainWindow::on_pushButton_12_clicked() //轴使能操作函数
{
    // TODO: Add your control notification handler code here
    time_t t1,t2; //设置时间监控变量，在等待轴状态机变化时防止死循环使用
    unsigned long errcode=0; //总线错误代码
    unsigned short statemachine_0=0; //总线状态机
    unsigned short statemachine_1=0; //总线状态机
    int m_nConnectNo=0;
    nmcs_get_errcode(m_nConnectNo,2,&errcode); //获取总线状态

    if(errcode==0) //总线正常才允许使能操作
    {
        if (ui->checkBox->isChecked() && ui->checkBox_2->isChecked())
        {
            nmcs_set_axis_enable(m_nConnectNo,0);//设置0轴使能
            nmcs_set_axis_enable(m_nConnectNo,1);//设置1轴使能
            nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0);//获取0轴状态机
            nmcs_get_axis_state_machine(m_nConnectNo,1,&statemachine_1);//获取0轴状态机
            t1=time(NULL); //设置时间
            while(statemachine_1!=4 && statemachine_0!=4) //监控轴状态机的值，该值等于 4 表示轴状态机处于准备好状态
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_11->setText("0、1轴使能超时，请检查设备");
                    return;
                }
                nmcs_set_axis_enable(m_nConnectNo,0); //设置0轴使能
                nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0); //获取0轴状态机
                nmcs_set_axis_enable(m_nConnectNo,1); //设置1轴使能
                nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_1); //获取1轴状态机
            }
            while(statemachine_0!=4 && statemachine_1==4) //监控轴状态机的值，该值等于 4 表示轴状态机处于准备好状态
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_11->setText("0轴使能超时，请检查设备");
                    return;
                }
                nmcs_set_axis_enable(m_nConnectNo,0); //设置0轴使能
                nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0); //获取0轴状态机
            }
            while(statemachine_1!=4 && statemachine_0==4) //监控轴状态机的值，该值等于 4 表示轴状态机处于准备好状态
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_11->setText("1轴使能超时，请检查设备");
                    return;
                }
                nmcs_set_axis_enable(m_nConnectNo,1); //设置1轴使能
                nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_1); //获取1轴状态机
            }
            ui->label_11->setText("0、1轴使能成功");
        }
        else if(ui->checkBox->isChecked())
        {
            nmcs_set_axis_enable(m_nConnectNo,0);//设置指定0轴使能
            nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0);//获取0轴状态机
            t1=time(NULL); //设置时间
            while(statemachine_0!=4) //监控轴状态机的值，该值等于 4 表示轴状态机处于准备好状态
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_11->setText("0轴使能超时，请检查设备");
                    return;
                }
            nmcs_set_axis_enable(m_nConnectNo,0); //设置轴使能
            nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0); //获取0轴状态机
            }
        }
        else if(ui->checkBox_2->isChecked())
        {
            nmcs_set_axis_enable(m_nConnectNo,1);//设置指定1轴使能
            nmcs_get_axis_state_machine(m_nConnectNo,1,&statemachine_1);//获取1轴状态机
            t1=time(NULL); //设置时间
            while(statemachine_0!=4) //监控轴状态机的值，该值等于 4 表示轴状态机处于准备好状态
            {
                t2=time(NULL);
                if(t2-t1 > 3) //3 秒时间防止死循环
                {
                    ui->label_11->setText("1轴使能超时，请检查设备");
                return;
                }
            nmcs_set_axis_enable(m_nConnectNo,1); //设置1轴使能
            nmcs_get_axis_state_machine(m_nConnectNo,1,&statemachine_1); //获取1轴状态机
            }
        }
    }
    else //总线不正常状态下不响应使能操作
    {
        ui->label_11->setText("总线错误，禁止操作！");
        return;
    }
}

void MainWindow::on_pushButton_8_clicked() //轴去使能操作函数
{
    // TODO: Add your control notification handler code here
    time_t t1,t2; //设置时间监控变量，在等待轴状态机变化时防止死循环使用
    unsigned long errcode=0; //总线错误代码
    unsigned short statemachine_0=0; //总线状态机
    unsigned short statemachine_1=0; //总线状态机
    int m_nConnectNo=0;
    nmcs_get_errcode(m_nConnectNo,2,&errcode); //获取总线状态
    if(errcode==0)
    {
        nmcs_set_axis_disable(m_nConnectNo,0);//设置0轴去使能
        nmcs_set_axis_disable(m_nConnectNo,1);//设置1轴去使能
        nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0);//获取0轴状态机
        nmcs_get_axis_state_machine(m_nConnectNo,1,&statemachine_1);//获取1轴状态机
        t1=time(NULL); //设置时间
        while(statemachine_0==4)
        {
            t2=time(NULL);
            if(t2-t1 > 3) //3 秒时间防止死循环
            {
                ui->label_11->setText("0、轴去使能失败，请检查设备");
                return;
            }
            nmcs_set_axis_disable(m_nConnectNo,0);//设置0轴去使能
            nmcs_get_axis_state_machine(m_nConnectNo,0,&statemachine_0);//获取0轴状态机
        }
        while(statemachine_1==4)
        {
            t2=time(NULL);
            if(t2-t1 > 3) //3 秒时间防止死循环
            {
                ui->label_11->setText("0、轴去使能失败，请检查设备");
                return;
            }
            nmcs_set_axis_disable(m_nConnectNo,1);//设置1轴去使能
            nmcs_get_axis_state_machine(m_nConnectNo,1,&statemachine_1);//获取1轴状态机
        }
        ui->label_11->setText("0、1轴去使能");
    }
    else //总线不正常状态下不响应去使能操作
    {
        ui->label_11->setText("总线错误，禁止操作！");
        return;
    }
}

//执行定长运动以及连续
void MainWindow::OnButtonDo()
{
    // TODO: Add your control notification handler code here
    UpdateData(true);//刷新参数
    short iret = 0;
    unsigned short statemachine=0;
    unsigned long errcode=0;
    iret = nmcs_get_errcode(m_nConnectNo,2,&errcode);
    if(errcode!=0)
    {
        MessageBox("总线错误","错误");
        return;
    }
    iret = nmcs_get_axis_state_machine(m_nConnectNo,m_nAxis,&statemachine);
    if (statemachine!=4)
    {
        MessageBox("轴状态机错误","错误");
    return;
    }
    //注意：以上部分是总线控制方式，需要检测总线状态和轴状态机，以下部分总线控制方式和脉冲
    控制方式共用
    if (smc_check_done( m_nConnectNo,m_nAxis ) == 0) //已经在运动中
        return;
    iret = smc_set_equiv(m_nConnectNo, m_nAxis, 1);//设置脉冲当量
    //设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
    iret = smc_set_pulse_outmode(m_nConnectNo, m_nAxis, 0);
    //设定单轴运动速度参数
    smc_set_profile_unit(m_nConnectNo,m_nAxis,m_nSpeedMin,m_nSpeed,m_nAcc,m_nDec,m_nSpeedSt
    op);
    //设定 S 段时间
    iret = smc_set_s_profile(m_nConnectNo,m_nAxis,0,m_nSPara);
    if( m_nActionst == 0 )
    {
        iret = smc_pmove_unit(m_nConnectNo, m_nAxis, m_nPulse*(m_bLogic?1:-1), 0);//相对定长运动
    }
    else
    {
        iret = smc_vmove(m_nConnectNo, m_nAxis, m_bLogic?1:0); //恒速运动
    }
    UpdateData(false);
}
//执行清除指令位置，脉冲方式和总线方式一致

//open io
void MainWindow::on_pushButton_clicked()
{
    WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
    short rtn = smc_write_outbit(0,ioNo,0);
    qDebug("smc_write_outbit(0,%d,0) rtn=%d\n",ioNo,rtn);
}
//close io
void MainWindow::on_pushButton_2_clicked()
{

    WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
    short rtn = smc_write_outbit(0,ioNo,1);
    qDebug("smc_write_outbit(0,%d,1) rtn=%d\n",ioNo,rtn);
}
//start
void MainWindow::on_pushButton_3_clicked()
{
    double startvel = ui->textEdit_startvel->toPlainText().toDouble();
    double runvel = ui->textEdit_runvel->toPlainText().toDouble();
    double stopvel = ui->textEdit_stopvel->toPlainText().toDouble();
    double acctime = ui->textEdit_acctime->toPlainText().toDouble();
    double dectime = ui->textEdit_dectime->toPlainText().toDouble();
    double stime = ui->textEdit_stime->toPlainText().toDouble();
    double destpos = ui->textEdit_destpos->toPlainText().toDouble();
    WORD axisNo =0;
    short rtn =0;
    if(ui->radioButton_X->isChecked())
    {
        axisNo=0;
        rtn = smc_set_pulse_outmode(0,axisNo,0);
        rtn = smc_set_equiv(0,axisNo,1);
        rtn = smc_set_alm_mode(0,axisNo,0,0,0);
        rtn = smc_write_sevon_pin(0,axisNo,0);
        rtn = smc_set_s_profile(0,axisNo,0,stime);
        rtn =smc_set_profile_unit(0,axisNo,startvel,runvel,acctime,dectime,stopvel);
        rtn =smc_set_dec_stop_time(0,axisNo,dectime);
        rtn =smc_pmove_unit(0,axisNo,destpos,1);
    }
    if(ui->radioButton_Y->isChecked())
    {
        axisNo=1;
        rtn = smc_set_pulse_outmode(0,axisNo,0);
        rtn = smc_set_equiv(0,axisNo,1);
        rtn = smc_set_alm_mode(0,axisNo,0,0,0);
        rtn = smc_write_sevon_pin(0,axisNo,0);
        rtn = smc_set_s_profile(0,axisNo,0,stime);
        rtn =smc_set_profile_unit(0,axisNo,startvel,runvel,acctime,dectime,stopvel);
        rtn =smc_set_dec_stop_time(0,axisNo,dectime);
        rtn =smc_pmove_unit(0,axisNo,destpos,1);
    }

}
//stop
void MainWindow::on_pushButton_4_clicked()
{
    short axisNo=0;
    short rtn=0;
    if(ui->radioButton_X->isChecked())
    {
        axisNo =0;
        rtn =smc_stop(0,axisNo,0);
    }
    if(ui->radioButton_Y->isChecked())
    {
        axisNo =1;
        rtn =smc_stop(0,axisNo,0);
    }
}
//zero pos
void MainWindow::on_pushButton_5_clicked()
{
    short axisNo=0;
    short rtn=0;
    if(ui->radioButton_X->isChecked())
    {
        axisNo =0;
        rtn =smc_set_position_unit(0,axisNo,0);
    }
    if(ui->radioButton_Y->isChecked())
    {
        axisNo =1;
        rtn =smc_set_position_unit(0,axisNo,0);
    }
}
//enc pos
void MainWindow::on_pushButton_6_clicked()
{
    short axisNo=0;
    short rtn=0;
    if(ui->radioButton_X->isChecked())
    {
        axisNo =0;
        rtn =smc_set_encoder_unit(0,axisNo,0);
    }
    if(ui->radioButton_Y->isChecked())
    {
        axisNo =1;
        rtn =smc_set_encoder_unit(0,axisNo,0);
    }

}
//stop crd
void MainWindow::on_pushButton_7_clicked()
{
    short rtn =0;
    rtn = smc_stop_multicoor(0,0,0);
    rtn = smc_stop_multicoor(0,1,0);
}
//change vel
void MainWindow::on_pushButton_10_clicked()
{
    WORD axisNo=0;
    short rtn=0;
    double runvel = ui->textEdit_runvel->toPlainText().toDouble();
    if(ui->radioButton_X->isChecked())
    {
        axisNo =0;
        rtn =smc_change_speed_unit(0,axisNo,runvel,0);
    }
    if(ui->radioButton_Y->isChecked())
    {
        axisNo =1;
        rtn =smc_change_speed_unit(0,axisNo,runvel,0);
    }
}
//change pos
void MainWindow::on_pushButton_11_clicked()
{
    WORD axisNo=0;
    short rtn=0;
    double destpos = ui->textEdit_destpos->toPlainText().toDouble();
    if(ui->radioButton_X->isChecked())
    {
        axisNo =0;
        rtn =smc_reset_target_position_unit(0,axisNo,destpos);
    }
    if(ui->radioButton_Y->isChecked())
    {
        axisNo =1;
        rtn =smc_reset_target_position_unit(0,axisNo,destpos);
    }

}



void MainWindow::on_pushButton_9_clicked()
{

    short rtn=0;
    WORD axisList[2] = {0,1};
    double posList[4]={10000,20000,30000,25000};
    for (int i=0;i<2;i++)
    {
        rtn = smc_set_equiv(0,i,1);
        rtn = smc_set_alm_mode(0,i,0,0,0);
    }
     rtn =smc_set_vector_profile_unit(0,0,0,8000,0.1,0.1,0);
     rtn = smc_line_unit(0,0,4,axisList,posList,0);
}
