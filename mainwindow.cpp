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
const double RADIUS = 0.164;
const double SPACE = 0.552;
const double PI = 3.1415926358;
const double COEFF = (2 * PI * RADIUS / 320000);
const double MAX_VEL = 0.8;
const double VEL_LIMIT = MAX_VEL / COEFF;

void vel_limit(double *runvel) {  //将两轮的速度限制在0.8m/s之内

  for (int i = 0; i < 2; i++) {
    runvel[i] = runvel[i] > VEL_LIMIT ? VEL_LIMIT : runvel[i];

    runvel[i] = runvel[i] < -VEL_LIMIT ? -VEL_LIMIT : runvel[i];
  }
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  initDialog();
  char pconnectstring[] = "192.168.5.11";
  connection = smc_board_init(0, 2, pconnectstring, 0);
  if (connection != 0)  //检查控制卡是否连接成功
  {
    qDebug("smc_board_init iret = %d\n", connection);
    qDebug("连接失败！请检查控制卡的连接");
    information_connection_fail();
  } else {
    qDebug("控制卡连接成功！");
    information_connection_success();
  }

  MatrixXd trans_2(2, 2);
  trans_2(0, 0) = 1;
  trans_2(0, 1) = SPACE / 2;
  trans_2(1, 0) = 1;
  trans_2(1, 1) = -SPACE / 2;

  trans = trans_2 / COEFF;

  startTimer(200);  //定义类QTimerEvent的刷新时间(ms)
}

void MainWindow::initDialog() {
  ui->textEdit_PortNo->setText("0");  //默认port_no为0

  ui->checkBox_axis_0->click();
  ui->checkBox_axis_1->click();  //默认两个轮椅的使能状态都被选中

  //设定两个轮子单独运动时，电机速度等参数的默认值
  ui->checkBox_axis_l->click();
  ui->checkBox_axis_r->click();  //默认两个轮子的运动都被选中
  ui->radioButton_fw_0->click();
  ui->radioButton_fw_1->click();  //默认两个轮子的方向均为前进

  ui->radioButton_cs->click();

  ui->textEdit_startvel_0->setText("100");
  ui->textEdit_runvel_0->setText("32000");
  ui->textEdit_stopvel_0->setText("100");
  ui->textEdit_acctime_0->setText("0.1");
  ui->textEdit_dectime_0->setText("0.1");
  ui->textEdit_stime_0->setText("0.05");
  ui->textEdit_destpos_0->setText("20000");
  ui->textEdit_pulse_0->setText("320000");  //左轮

  ui->textEdit_startvel_1->setText("100");
  ui->textEdit_runvel_1->setText("32000");
  ui->textEdit_stopvel_1->setText("100");
  ui->textEdit_acctime_1->setText("0.1");
  ui->textEdit_dectime_1->setText("0.1");
  ui->textEdit_stime_1->setText("0.05");
  ui->textEdit_destpos_1->setText("20000");
  ui->textEdit_pulse_1->setText("320000");  //右轮

  //设定轮椅匀速运动默认参数
  ui->textEdit_linear_vel->setText("0.1");  //默认移动的线速度为0.1m/s
  // ui->slider_linear_vel->setValue(10);
  ui->textEdit_angular_vel->setText("0");  //默认移动的角速度为0
  // ui->slider_angular_vel->setValue(0);

  //轮椅定长运动参数
  ui->textEdit_goal_x->setText("0.2");
  ui->textEdit_goal_y->setText("0");
  ui->textEdit_goal_theta->setText("0");    //默认移动方式：前进0.2m
  ui->textEdit_goal_dv->setText("0.15");    //默认直线运动速度为0.2m/s
  ui->textEdit_goal_dtheta->setText("15");  //默认角速度为15°/s
}

MainWindow::~MainWindow() {
  delete ui;
}

void MainWindow::timerEvent(QTimerEvent *e) {
  short iret[2] = {0, 0};
  double pos[2] = {0.0, 0.0};
  double enc[2] = {0.0, 0.0};
  double speed[2] = {0.0, 0.0};
  double linear_v = 0;
  double angular_v = 0;
  DWORD status[2] = {1, 1};
  WORD runmode[2] = {0, 0};
  for (int i = 0; i < 2; i++) {
    iret[i] = smc_get_position_unit(0, i, &pos[i]);
    iret[i] = smc_get_encoder_unit(0, i, &enc[i]);
    iret[i] = smc_read_current_speed_unit(0, i, &speed[i]);
    status[i] = smc_check_done(0, i);
    iret[i] = smc_get_axis_run_mode(0, i, &runmode[i]);
  }
  //由每个轮子的速度获取轮椅速度，负号是因为电机正方向对应的是轮椅的后退反向
  linear_v = -(speed[0] + speed[1]) * COEFF / 2;
  angular_v = -(speed[1] - speed[0]) * COEFF / SPACE * 180 / PI;

  if (status[0] == 1) {
    ui->label_status_0->setText("Static");
  } else {
    ui->label_status_0->setText("Running");
  }

  if (status[1] == 1) {
    ui->label_status_1->setText("Static");
  } else {
    ui->label_status_1->setText("Running");
  }

  if (runmode[0] == 0) {
    ui->label_mode_0->setText("Standby");
  } else if (runmode[0] == 1) {
    ui->label_mode_0->setText("Fixed Length");
  } else if (runmode[0] == 2) {
    ui->label_mode_0->setText("Constant Speed");
  }

  if (runmode[1] == 0) {
    ui->label_mode_1->setText("Standby");
  } else if (runmode[1] == 1) {
    ui->label_mode_1->setText("Fixed Length");
  } else if (runmode[1] == 2) {
    ui->label_mode_1->setText("Constant Speed");
  }

  ui->textEdit_prfPosX->setText(QString::number(pos[0], 'f', 3));
  ui->textEdit_prfPosY->setText(QString::number(pos[1], 'f', 3));
  //
  ui->textEdit_encPosX->setText(QString::number(enc[0], 'f', 3));
  ui->textEdit_encPosY->setText(QString::number(enc[1], 'f', 3));
  //
  ui->textEdit_SpeedX->setText(QString::number(speed[0], 'f', 3));
  ui->textEdit_SpeedY->setText(QString::number(speed[1], 'f', 3));
  //
  ui->textEdit_linear_current_vel->setText(QString::number(linear_v, 'f', 3));
  ui->textEdit_angular_current_vel->setText(QString::number(angular_v, 'f', 3));

  short timeout;
  short status_connect;
  timeout = smc_set_connect_timeout(0);
  status_connect = smc_get_connect_status(0);  //读取实时的连接状态

  if (connection == 0) {
    emg_stop();               //与控制器连接成功时调用IO急停信号
    if (status_connect != 1)  //如果连接失败,则立刻急停
    {
      for (int i = 0; i < 2; i++) {
        iret[i] = smc_stop(0, i, 0);
      }
      on_pushButton_disable_clicked();
      information_connection_interrupted();
    }
  }

  // printf("连接延时%d ms,连接状态%d\n",timeout,status_connect);

  // connect(ui->slider_linear_vel,SIGNAL(valueChanged(int)),ui->textEdit_linear_vel,SLOT(setText(QString::number(int,'f',3))));
  // ui->slider_linear_vel->sliderReleased();
  // connect(ui->slider_linear_vel, SIGNAL(valueChanged(int)), ui->lineEdit, SLOT(setLineEditValue(int)));
}

// IO 急停信号
void MainWindow::emg_stop() {
  /*********************变量定义****************************/
  WORD MyCardNo = 0;          //连接号
  WORD Myaxis[2] = {0, 1};    //轴号
  short ret[2] = {0, 0};      //错误返回
  WORD Myenable[2] = {1, 1};  //急停信号使能
  WORD Mylogic[2] = {1, 1};   //急停信号高电平有效
  /*********************函数调用执行**************************/
  //第一步、设置轴 IO 映射，将通用输入 0 作为各轴的急停信号
  short io_0 = smc_read_inbit(MyCardNo, 0);  //读取IO口的电平值

  for (int i = 0; i < 2; i++) {
    ret[i] = smc_set_axis_io_map(MyCardNo, Myaxis[i], 3, 6, 0, 0);
  }
  //第二步、设置 EMG 使能，高电平有效
  for (int i = 0; i < 2; i++) {
    ret[i] = smc_set_emg_mode(MyCardNo, Myaxis[i], Myenable[i], Mylogic[i]);
  }
  if (io_0 == 1) {
    on_pushButton_disable_clicked();
  }
  //第三步、回读 EMG 使能，高电平有效
  for (int i = 0; i < 2; i++) {
    ret[i] = smc_get_emg_mode(MyCardNo, Myaxis[i], &Myenable[i], &Mylogic[i]);
    // printf("%d 轴急停信号参数,使能,有效电平= %d %d\n ",i,Myenable[i],Mylogic[i]);
  }
}

//轴使能操作函数
void MainWindow::on_pushButton_enable_clicked() {
  unsigned long errcode = 0;  //总线错误代码
  int m_nConnectNo = 0;

  nmcs_get_errcode(m_nConnectNo, 2, &errcode);  //获取总线状态
  if (errcode == 0)                             //总线正常才允许使能操作
  {
    short emgstop_is_on = smc_read_inbit(0, 0);  //检查急停开关的电平
    if (emgstop_is_on == 1) {
      on_pushButton_disable_clicked();
      information_emgstop_on();
    } else if (ui->checkBox_axis_0->isChecked() && ui->checkBox_axis_1->isChecked()) {
      bool enable_success[2] = {false};

      for (int i = 0; i < 2; i++) {
        enable_success[i] = wheelchairstatus->enable_axis(this,i, m_nConnectNo);
      }
      if (enable_success[0] && enable_success[1]) {
        ui->label_error->setText("0、1轴使能成功");
      }
    } else if (ui->checkBox_axis_0->isChecked() && wheelchairstatus->enable_axis(this,0, m_nConnectNo) &&
               wheelchairstatus->disable_axis(this,1, m_nConnectNo)) {
      ui->label_error->setText("0轴使能成功");
    } else if (ui->checkBox_axis_1->isChecked() && wheelchairstatus->enable_axis(this,1, m_nConnectNo) &&
               wheelchairstatus->disable_axis(this,0, m_nConnectNo)) {
      ui->label_error->setText("1轴使能成功");
    }
  } else  //总线不正常状态下不响应使能操作
  {
    ui->label_error->setText("总线错误，禁止操作！");
  }

  return;
}
//轴使能操作函数
bool MainWindow::on_pushButton_disable_clicked() {
  unsigned long errcode = 1;
  int m_nConnectNo = 0;

  nmcs_get_errcode(m_nConnectNo, 2, &errcode);  //获取总线状态
  if (errcode == 0) {
    if (ui->checkBox_axis_0->isChecked() && ui->checkBox_axis_1->isChecked()) {
      bool disable_success[2] = {false};

      for (int i = 0; i < 2; i++) {
        disable_success[i] = wheelchairstatus->disable_axis(this,i, m_nConnectNo);
      }

      if (disable_success[0] && disable_success[1]) {
        ui->label_error->setText("0、1轴去使能成功");
        return true;
      }
    }

  } else  //总线不正常状态下不响应去使能操作
  {
    ui->label_error->setText("总线错误，禁止操作！");
  }
  return false;
}

// open io
void MainWindow::on_pushButton_openio_clicked() {
  WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
  short iret = smc_write_outbit(0, ioNo, 0);
  qDebug("smc_write_outbit(0,%d,0) iret=%d\n", ioNo, iret);
}
// close io
void MainWindow::on_pushButton_closeio_clicked() {
  WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
  short iret = smc_write_outbit(0, ioNo, 1);
  qDebug("smc_write_outbit(0,%d,1) iret=%d\n", ioNo, iret);
}

// start
void MainWindow::on_pushButton_start_0_clicked() {
  double runvel[2] = {ui->textEdit_runvel_0->toPlainText().toDouble(), ui->textEdit_runvel_1->toPlainText().toDouble()};
  double pulse[2] = {ui->textEdit_pulse_0->toPlainText().toDouble(), ui->textEdit_pulse_1->toPlainText().toDouble()};
  wheelchairstatus->get_status(this,0, runvel, pulse);
  wheelchairstatus->get_status(this,1, runvel, pulse);

  assert(wheelchairstatus->is_ready_to_start(this));
  WORD axisNo[2] = {0, 1};
  if (ui->checkBox_axis_l->isChecked()) {
    wheelchairstatus->move_axis(this,axisNo[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    wheelchairstatus->move_axis(this,axisNo[0]);
  }
  return;
}

// decstop
void MainWindow::on_pushButton_decstop_0_clicked() {
  double dectime[2] = {ui->textEdit_dectime_0->toPlainText().toDouble(),
                       ui->textEdit_dectime_1->toPlainText().toDouble()};
  short iret[2] = {0, 0};
  double actuvel[2];
  for (int i = 0; i < 2; i++) {
    iret[i] = smc_read_current_speed_unit(0, i, &actuvel[i]);
    double acc = fabs(actuvel[i] / dectime[i]);
    if (acc > 20000) {
      dectime[i] = fabs(actuvel[i] / 100000);  //如果减速的加速度>100000,修改减速时间使得加速度为100000
    }
  }
  ui->textEdit_dectime_0->setText(QString::number(dectime[0], 'f', 3));
  ui->textEdit_dectime_1->setText(QString::number(dectime[1], 'f', 3));  //将修改后的减速时间显示在QT界面上

  for (int axisNo = 0; axisNo < 2; axisNo++) {
    smc_set_dec_stop_time(0, axisNo, dectime[axisNo]);  //设置减速停止时间

    iret[axisNo] = smc_stop(0, axisNo, 0);  //减速停止
  }
  return;
}
void MainWindow::on_pushButton_decstop_1_clicked() {
  on_pushButton_decstop_0_clicked();
}

// emgstop
void MainWindow::on_pushButton_emgstop_0_clicked() {
  short iret[2] = {0, 0};
  for (int i = 0; i < 2; i++) {
    iret[i] = smc_stop(0, i, 0);
  }
}
void MainWindow::on_pushButton_emgstop_1_clicked() {
  on_pushButton_emgstop_0_clicked();
}

// zero pos
void MainWindow::on_pushButton_zeropos_clicked() {
  short axisNo = 0;
  short iret = 0;
  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iret = smc_set_position_unit(0, axisNo, 0);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iret = smc_set_position_unit(0, axisNo, 0);
  }
}
// enc pos
void MainWindow::on_pushButton_encpos_clicked() {
  short axisNo = 0;
  short iret = 0;
  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iret = smc_set_encoder_unit(0, axisNo, 0);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iret = smc_set_encoder_unit(0, axisNo, 0);
  }
}
// stop crd
void MainWindow::on_pushButton_stopcrd_clicked() {
  short iret = 0;
  iret = smc_stop_multicoor(0, 0, 0);
  iret = smc_stop_multicoor(0, 1, 0);
}
// change vel
void MainWindow::on_pushButton_changevel_clicked() {
  WORD axisNo = 0;
  short iret = 0;
  double runvel[2] = {ui->textEdit_runvel_0->toPlainText().toDouble(), ui->textEdit_runvel_1->toPlainText().toDouble()};
  vel_limit(runvel);  //限制每个轮子的最大线速度为0.8m/s
  ui->textEdit_runvel_0->setText(QString::number(runvel[0], 'f', 3));
  ui->textEdit_runvel_1->setText(QString::number(runvel[1], 'f', 3));  //将限制的速度显示在QT界面上

  //根据变速前后的速度差值决定变速的时间
  double actuvel[2];
  double time[2];
  for (int i = 0; i < 2; i++) {
    iret = smc_read_current_speed_unit(0, i, &actuvel[i]);
    double diff = fabs(runvel[i] - actuvel[i]);
    if (diff < 10000) {
      time[i] = 0;  //如果差值diff<10000,则时间为0
    } else {
      time[i] = diff / 100000;  //反之,则时间为diff/100000
    }
  }

  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iret = smc_change_speed_unit(0, axisNo, runvel[0], time[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iret = smc_change_speed_unit(0, axisNo, runvel[1], time[1]);
  }
}
// change pos
void MainWindow::on_pushButton_changepos_clicked() {
  WORD axisNo = 0;
  short iret = 0;
  double destpos[2] = {ui->textEdit_destpos_0->toPlainText().toDouble(),
                       ui->textEdit_destpos_1->toPlainText().toDouble()};

  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iret = smc_reset_target_position_unit(0, axisNo, destpos[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iret = smc_reset_target_position_unit(0, axisNo, destpos[1]);
  }
}

void MainWindow::on_pushButton_start_wc_clicked() {
  double runvel[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  double pulse[3][2] = {{0, 0}, {0, 0}, {0, 0}};

  ui->radioButton_fw_0->click();
  ui->radioButton_fw_1->click();  //默认前进

  assert(wheelchairstatus->is_ready_to_start(this));

  int begin = 1;
  int end = 2;
  bool isConstantSpeed = true;
  if (ui->radioButton_fl->isChecked())  //定长运动参数设置
  {
    begin = 0;
    end = 3;
    isConstantSpeed = false;
  }

  wheelchairstatus->set_wheelchair_moving_parameter(this, isConstantSpeed, pulse, runvel);
  for (int i = begin; i < end; i++) {
    wheelchairstatus->get_status(this, 0, runvel[i], pulse[i]);
    wheelchairstatus->get_status(this, 1, runvel[i], pulse[i]);

    wheelchairstatus->move_axis(this, 0);
    wheelchairstatus->move_axis(this, 1);

    while (smc_check_done(0, 0) == 0 || smc_check_done(0, 1) == 0) {
      system("pause");
    }
    // sleep(0.5);
  }
}

void MainWindow::on_pushButton_changevel_wc_clicked() {
  //定义轮椅速度矩阵
  MatrixXd v_wheels(2, 1);
  MatrixXd v_chairs(2, 1);
  v_chairs(0, 0) = ui->textEdit_linear_vel->toPlainText().toDouble();
  v_chairs(1, 0) = ui->textEdit_angular_vel->toPlainText().toDouble() / 180 * PI;

  v_wheels = trans * v_chairs;  //由轮椅速度反解出电机速度

  short iret[2] = {0, 0};
  double runvel[2] = {-v_wheels(1, 0), -v_wheels(0, 0)};  //左轮为0号电机,右轮为1号电机
  vel_limit(runvel);                                      //限制每个轮子的最大线速度为0.8m/s

  //根据变速前后的速度差值决定变速的时间
  double actuvel[2];
  double time[2];
  for (int i = 0; i < 2; i++) {
    iret[i] = smc_read_current_speed_unit(0, i, &actuvel[i]);
    double diff = fabs(runvel[i] - actuvel[i]);
    if (diff < 20000) {
      time[i] = 0;
    } else {
      time[i] = diff / 100000;
    }
  }

  for (int i = 0; i < 2; i++) {
    iret[i] = smc_change_speed_unit(0, i, runvel[i], time[i]);
  }
}

void MainWindow::on_pushButton_changepos_wc_clicked() {
}
// exit board&application
void MainWindow::on_pushButton_exit_0_clicked() {
  if (connection == 0) {
    bool disable_success = false;
    disable_success = MainWindow::on_pushButton_disable_clicked();
    if (disable_success == true) {
      smc_board_close(0);
      qApp->exit(0);
    } else {
      return;
    }
  } else {
    smc_board_close(0);
    qApp->exit(0);
  }
}
void MainWindow::on_pushButton_exit_1_clicked() {
  on_pushButton_exit_0_clicked();  //该按钮和另一个exit效果相同
}

void MainWindow::information_connection_fail() {
}
void MainWindow::information_connection_success() {
}
