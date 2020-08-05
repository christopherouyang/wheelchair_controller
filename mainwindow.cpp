#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LTSMC.h"
#include <QDebug>
#include <QMessageBox>
#include <eigen3/Eigen/Dense>
#include "math.h"
#include <unistd.h>
#include <string>

static short borad_init_status = (short)ConnectionStatus::unconnected;
constexpr WORD CARD_NO = 0;
constexpr WORD EMG_STOP_BIT_NO = 0;
constexpr DWORD ERROR_CODE_SUCCESS = 0;

// const parameter for the wheelchair
constexpr double RADIUS = 0.164;
constexpr double SPACE = 0.552;
constexpr double PI = 3.1415926358;
constexpr double COEFF = (2 * PI * RADIUS / 320000);
constexpr double MAX_VEL = 0.8;
constexpr double VEL_LIMIT = MAX_VEL / COEFF;

static void vel_limit(double *runvel) {
  //将两轮的速度限制在0.8m/s之内
  for (int i = 0; i < 2; i++) {
    runvel[i] = runvel[i] > VEL_LIMIT ? VEL_LIMIT : runvel[i];
    runvel[i] = runvel[i] < -VEL_LIMIT ? -VEL_LIMIT : runvel[i];
  }
}

static QString MovingModeInfo(WORD movingmode) {
  QString info;
  switch (movingmode) {
    case (WORD)MovingMode::standby:
      info = "Standby";
      break;
    case (WORD)MovingMode::fixedLength:
      info = "Fixed Length";
      break;
    case (WORD)MovingMode::constantSpeed:
      info = "Constant Speed";
      break;
    default:
      break;
  }
  return info;
}

static QString StatusInfo(DWORD status) {
  QString info;
  if (status == 1) {
    info = "Static";
  } else {
    info = "Running";
  }
  return info;
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  initDialog();
  char pIpAddress[] = "192.168.5.11";

  borad_init_status = smc_board_init(CARD_NO, (WORD)ConnectType::ethercat, pIpAddress, 0);
  if (borad_init_status != (short)ConnectionStatus::connected) {  //检查控制卡是否连接成功
    qDebug("smc_board_init iRet = %d\n", borad_init_status);
    qDebug("连接失败！请检查控制卡的连接");
    information_connection_fail();
  } else {
    qDebug("控制卡连接成功！");
    information_connection_success();
  }

  Eigen::MatrixXd trans_2(2, 2);
  trans_2(0, 0) = 1;
  trans_2(0, 1) = SPACE / 2;
  trans_2(1, 0) = 1;
  trans_2(1, 1) = -SPACE / 2;
  trans = trans_2 / COEFF;

  startTimer(200);  //定义类QTimerEvent的刷新时间(ms)
}

void MainWindow::information_connection_fail() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "连接失败！请检查控制卡的连接";
  reply = QMessageBox::information(this, tr("Connection Fails"), MESSAGE);
}

void MainWindow::information_connection_success() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "控制卡连接成功";
  reply = QMessageBox::information(this, tr("Connection Success"), MESSAGE);
}

void ::MainWindow::information_disable() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "电机未使能,请先将电机使能";
  reply = QMessageBox::information(this, tr("Motor is disabled"), MESSAGE);
}
void ::MainWindow::information_disable_axis(int axisNo) {
  QMessageBox::StandardButton reply;
  QString message[2] = {
      "左轮电机未使能,请先将电机使能"
      "右轮电机未使能,请先将电机使能"};
  reply = QMessageBox::information(this, tr("Motor_0 is disabled"), message[axisNo]);
}

void ::MainWindow::information_emgstop_on() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "急停开关被按下,请先释放急停开关再使能";
  reply = QMessageBox::information(this, tr("EMG stop is on"), MESSAGE);
}
void MainWindow::information_connection_interrupted() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "连接中断！请检查控制卡的连接";
  reply = QMessageBox::information(this, tr("Connection is interrupted"), MESSAGE);
}

void MainWindow::initDialog() {
  ui->textEdit_PortNo->setText("0");  //默认port_no为0

  //设定两个轮子单独运动时，电机速度等参数的默认值
  ui->checkBox_axis_l->click();
  ui->checkBox_axis_r->click();  //默认两个轮子的运动都被选中
  ui->radioButton_fw_0->click();
  ui->radioButton_fw_1->click();  //默认两个轮子的方向均为前进

  ui->radioButton_cs->click();

  ui->textEdit_startvel_0->setText("100");
  ui->textEdit_runvel_0->setText("32000");
  ui->textEdit_stopvel_0->setText("100");
  ui->textEdit_acctime_0->setText("0.2");
  ui->textEdit_dectime_0->setText("0.2");
  ui->textEdit_stime_0->setText("0.05");
  ui->textEdit_destpos_0->setText("20000");
  ui->textEdit_pulse_0->setText("320000");  //左轮

  ui->textEdit_startvel_1->setText("100");
  ui->textEdit_runvel_1->setText("32000");
  ui->textEdit_stopvel_1->setText("100");
  ui->textEdit_acctime_1->setText("0.2");
  ui->textEdit_dectime_1->setText("0.2");
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
  short iRet[2] = {0, 0};
  double pos[2] = {0.0, 0.0};
  double enc[2] = {0.0, 0.0};
  double speed[2] = {0.0, 0.0};
  double linear_v = 0;
  double angular_v = 0;
  DWORD status[2] = {1, 1};
  WORD movingmode[2] = {0, 0};
  QString info;

  for (int i = 0; i < 2; i++) {
    iRet[i] = smc_get_position_unit(CARD_NO, i, &pos[i]);
    iRet[i] = smc_get_encoder_unit(CARD_NO, i, &enc[i]);
    iRet[i] = smc_read_current_speed_unit(CARD_NO, i, &speed[i]);
    status[i] = smc_check_done(CARD_NO, i);
    iRet[i] = smc_get_axis_run_mode(CARD_NO, i, &movingmode[i]);
  }
  //由每个轮子的速度获取轮椅速度，负号是因为电机正方向对应的是轮椅的后退反向
  linear_v = -(speed[0] + speed[1]) * COEFF / 2;
  angular_v = -(speed[1] - speed[0]) * COEFF / SPACE * 180 / PI;

  ui->label_status_0->setText(StatusInfo(status[0]));
  ui->label_status_0->setText(StatusInfo(status[1]));

  ui->label_mode_0->setText(MovingModeInfo(movingmode[0]));
  ui->label_mode_1->setText(MovingModeInfo(movingmode[1]));

  ui->textEdit_prfPosX->setText(QString::number(pos[0], 'f', 3));
  ui->textEdit_prfPosY->setText(QString::number(pos[1], 'f', 3));

  ui->textEdit_encPosX->setText(QString::number(enc[0], 'f', 3));
  ui->textEdit_encPosY->setText(QString::number(enc[1], 'f', 3));

  ui->textEdit_SpeedX->setText(QString::number(speed[0], 'f', 3));
  ui->textEdit_SpeedY->setText(QString::number(speed[1], 'f', 3));

  ui->textEdit_linear_current_vel->setText(QString::number(linear_v, 'f', 3));
  ui->textEdit_angular_current_vel->setText(QString::number(angular_v, 'f', 3));

  short timeout = smc_set_connect_timeout(0);
  short status_connect = smc_get_connect_status(CARD_NO);  //读取实时的连接状态

  if (borad_init_status == 0) {
    emg_stop();               //与控制器连接成功时调用IO急停信号
    if (status_connect != 1)  //如果连接失败,则立刻急停
    {
      for (int i = 0; i < 2; i++) {
        iRet[i] = smc_stop(CARD_NO, i, 0);
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
  WORD axisNo[2] = {(WORD)Wheel::left, (WORD)Wheel::right};   //轴号
  short ret[2] = {0, 0};                                      //错误返回
  WORD enableStatus[2] = {(WORD)SignalEnableStatus::enable};  //急停信号使能
  WORD elecLevel[2] = {(WORD)ElectricalLevel::high};          //急停信号高电平有效
  double filterTime = 0;
  /*********************函数调用执行**************************/
  //第一步、设置轴 IO 映射，将通用输入 0 作为各轴的急停信号
  short io_0 = smc_read_inbit(CARD_NO, EMG_STOP_BIT_NO);  //读取IO口的电平值

  for (int i = 0; i < 2; i++) {
    ret[i] = smc_set_axis_io_map(CARD_NO, axisNo[i], (WORD)IoType::AxisIoInMsg_EMG, (WORD)MapIoType::AxisIoInPort_IO,
                                 EMG_STOP_BIT_NO, filterTime);
  }
  //第二步、设置 EMG 使能，高电平有效
  for (int i = 0; i < 2; i++) {
    ret[i] = smc_set_emg_mode(CARD_NO, axisNo[i], enableStatus[i], elecLevel[i]);
  }
  if (io_0 == 1) {
    on_pushButton_disable_clicked();
  }
  //第三步、回读 EMG 使能，高电平有效
  for (int i = 0; i < 2; i++) {
    ret[i] = smc_get_emg_mode(CARD_NO, axisNo[i], &enableStatus[i], &elecLevel[i]);
    printf("%d 轴急停信号参数,使能,有效电平= %d %d\n ", i, enableStatus[i], elecLevel[i]);
  }
}

bool MainWindow::enable_axis(int axisNo) {
  short statemachine = (short)AxisEnableStatus::off;
  time_t t1, t2;

  t1 = time(NULL);  //设置时间
  while (statemachine == (short)AxisEnableStatus::off) {
    short iRet = smc_write_sevon_pin(CARD_NO, axisNo, (WORD)AxisEnableStatus::on);
    statemachine = smc_read_sevon_pin(CARD_NO, axisNo);
    t2 = time(NULL);
    if (t2 - t1 > 3)  // 3 秒时间防止死循环
    {
      QString error = (QString)axisNo + "轴使能超时，请检查设备";
      ui->label_error->setText(error);
      return false;
    }
  }

  return true;
}

bool MainWindow::disable_axis(int axisNo) {
  short statemachine = (short)AxisEnableStatus::on;
  time_t t1, t2;

  t1 = time(NULL);  //设置时间
  while (statemachine == (short)AxisEnableStatus::on) {
    short iRet = smc_write_sevon_pin(CARD_NO, axisNo, (WORD)AxisEnableStatus::off);
    statemachine = smc_read_sevon_pin(CARD_NO, axisNo);
    t2 = time(NULL);
    if (t2 - t1 > 3)  // 3 秒时间防止死循环
    {
      QString wheel = (axisNo == (int)Wheel::left) ? "左" : "右";
      QString error = wheel + "轮轴去使能超时，请检查设备";
      ui->label_error->setText(error);
      return false;
    }
  }
  return true;
}

void MainWindow::on_pushButton_enable_clicked() {
  DWORD errcode = ERROR_CODE_SUCCESS;  //总线错误代码
  bool bRes = false;
  QString info;

  nmcs_get_errcode(CARD_NO, (WORD)ConnectType::ethercat, &errcode);  //获取总线状态
  if (errcode != ERROR_CODE_SUCCESS) {
    //总线不正常状态下不响应使能操作
    info = "总线错误，禁止操作！";
    ui->label_error->setText(info);
    return;
  }

  //总线正常才允许使能操作
  short emgstopStatus = smc_read_inbit(CARD_NO, EMG_STOP_BIT_NO);  //检查急停开关的电平
  if (emgstopStatus == (short)SignalEnableStatus::enable) {
    on_pushButton_disable_clicked();
    information_emgstop_on();
    return;
  }
  bool bLeftAxisChecked = ui->checkBox_axis_l->isChecked();
  bool bRightAxisChecked = ui->checkBox_axis_r->isChecked();
  if (!bLeftAxisChecked && !bRightAxisChecked) {
    return;
  }
  if (bLeftAxisChecked && bLeftAxisChecked) {
    bRes = enable_axis((WORD)Wheel::left);
    bRes = enable_axis((WORD)Wheel::right) && bRes;
    info = bRes ? "左&右轴使能成功" : "左&右轴使能失败";
  } else {
    Wheel axisToEnable = bLeftAxisChecked ? Wheel::left : Wheel::right;
    Wheel axisToDisable = bLeftAxisChecked ? Wheel::right : Wheel::left;

    bRes = enable_axis((WORD)axisToEnable);
    bRes = disable_axis((WORD)axisToDisable) && bRes;

    info = bLeftAxisChecked ? "左" : "右";
    info += bRes ? "轴使能成功" : "轴使能失败";
  }
  ui->label_error->setText(info);
}

bool MainWindow::on_pushButton_disable_clicked() {
  DWORD errcode = ERROR_CODE_SUCCESS;
  bool bRes = false;
  QString info;

  nmcs_get_errcode(CARD_NO, (WORD)ConnectType::ethercat, &errcode);
  if (errcode != ERROR_CODE_SUCCESS) {
    //总线不正常状态下不响应去使能操作
    info = "总线错误，禁止操作！";
    ui->label_error->setText(info);
    return false;
  }

  bool bLeftAxisChecked = ui->checkBox_axis_l->isChecked();
  bool bRightAxisChecked = ui->checkBox_axis_r->isChecked();
  if (!bLeftAxisChecked && !bRightAxisChecked) {
    return false;
  }
  if (bLeftAxisChecked && bLeftAxisChecked) {
    bRes = disable_axis((WORD)Wheel::left);
    bRes = disable_axis((WORD)Wheel::right) && bRes;
    info = bRes ? "左&右轴去使能成功" : "左&右轴去使能失败";
  } else {
    Wheel axisToDisable = bLeftAxisChecked ? Wheel::left : Wheel::right;
    bRes = disable_axis((WORD)axisToDisable);

    info = bLeftAxisChecked ? "左" : "右";
    info += bRes ? "轴去使能成功" : "轴去使能失败";
  }
  ui->label_error->setText(info);
  return bRes;
}

// open io
void MainWindow::on_pushButton_openio_clicked() {
  WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
  short iRet = smc_write_outbit(CARD_NO, ioNo, (WORD)IoStatus::on);
  qDebug("smc_write_outbit(0,%d,0) iRet=%d\n", ioNo, iRet);
}
// close io
void MainWindow::on_pushButton_closeio_clicked() {
  WORD ioNo = ui->textEdit_PortNo->toPlainText().toShort();
  short iRet = smc_write_outbit(CARD_NO, ioNo, (WORD)IoStatus::on);
  qDebug("smc_write_outbit(0,%d,1) iRet=%d\n", ioNo, iRet);
}

void MainWindow::get_status(Wheel wheelNo, double *runvel, double *pulse) {
  int axisNo = (int)wheelNo;
  vel_limit(runvel);
  if (wheelNo == Wheel::right) {
    status[axisNo].startvel = ui->textEdit_startvel_0->toPlainText().toDouble();
    status[axisNo].runvel = runvel[axisNo];

    status[axisNo].stopvel = ui->textEdit_stopvel_0->toPlainText().toDouble();
    status[axisNo].acctime = ui->textEdit_acctime_0->toPlainText().toDouble();
    status[axisNo].dectime = ui->textEdit_dectime_0->toPlainText().toDouble();
    status[axisNo].stime = ui->textEdit_stime_0->toPlainText().toDouble();
    status[axisNo].pulse = pulse[axisNo];

    status[axisNo].direction = ui->radioButton_fw_0->isChecked() ? (int)Direction::forward : (int)Direction::backward;

  } else if (wheelNo == Wheel::left) {
    status[axisNo].startvel = ui->textEdit_startvel_1->toPlainText().toDouble();
    status[axisNo].runvel = runvel[axisNo];
    status[axisNo].stopvel = ui->textEdit_stopvel_1->toPlainText().toDouble();
    status[axisNo].acctime = ui->textEdit_acctime_1->toPlainText().toDouble();
    status[axisNo].dectime = ui->textEdit_dectime_1->toPlainText().toDouble();
    status[axisNo].stime = ui->textEdit_stime_1->toPlainText().toDouble();
    status[axisNo].pulse = pulse[axisNo];

    status[axisNo].direction = ui->radioButton_fw_1->isChecked() ? (int)Direction::forward : (int)Direction::backward;
  }

  status[axisNo].mode = ui->radioButton_fl->isChecked() ? MovingMode::fixedLength : MovingMode::constantSpeed;

  if (status[axisNo].runvel < 0) {
    status[axisNo].direction = 1 - status[axisNo].direction;
    status[axisNo].runvel = -status[axisNo].runvel;
  }
}

void MainWindow::move_axis(Wheel wheelNo) {
  int axisNo = (int)wheelNo;
  short statemachine = smc_read_sevon_pin(CARD_NO, axisNo);
  if (statemachine == (short)AxisEnableStatus::off) {
    information_disable_axis(axisNo);
    return;
  }
  short iRet = smc_set_equiv(CARD_NO, axisNo, 1);     //设置脉冲当量
  iRet = smc_set_alm_mode(CARD_NO, axisNo, 0, 0, 0);  //设置报警使能,关闭报警
  iRet = smc_set_pulse_outmode(CARD_NO, axisNo, 0);  //设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
  iRet = smc_set_profile_unit(CARD_NO, axisNo, status[axisNo].startvel, status[axisNo].runvel, status[axisNo].acctime,
                              status[axisNo].dectime, status[axisNo].stopvel);  //设定单轴运动速度参数
  iRet = smc_set_s_profile(CARD_NO, axisNo, 0, status[axisNo].stime);
  if (status[axisNo].mode == MovingMode::fixedLength) {
    iRet =
        smc_pmove_unit(CARD_NO, axisNo, status[axisNo].pulse * (2 * status[axisNo].direction - 1), 0);  //相对定长运动
  } else {
    iRet = smc_vmove(CARD_NO, axisNo, status[axisNo].direction);  //恒速运动
  }
}

// start
void MainWindow::on_pushButton_start_0_clicked() {
  double runvel[2] = {ui->textEdit_runvel_0->toPlainText().toDouble(), ui->textEdit_runvel_1->toPlainText().toDouble()};
  double pulse[2] = {ui->textEdit_pulse_0->toPlainText().toDouble(), ui->textEdit_pulse_1->toPlainText().toDouble()};
  get_status(Wheel::left, runvel, pulse);
  get_status(Wheel::right, runvel, pulse);

  short statemachine[2] = {(short)AxisEnableStatus::off, (short)AxisEnableStatus::off};
  if (smc_check_done(CARD_NO, (WORD)Wheel::left) == 0 ||
      smc_check_done(CARD_NO, (WORD)Wheel::right) == 0) {  //该轴已经在运动中
    return;
  }
  if (ui->checkBox_axis_l->isChecked() && ui->checkBox_axis_r->isChecked()) {
    statemachine[0] = smc_read_sevon_pin(CARD_NO, (WORD)Wheel::left);   //获取0轴状态机
    statemachine[1] = smc_read_sevon_pin(CARD_NO, (WORD)Wheel::right);  //获取1轴状态机
    //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
    if (statemachine[1] == (short)AxisEnableStatus::off || statemachine[0] == (short)AxisEnableStatus::off) {
      information_disable();  //返回错误信号,停止该函数的运行
      return;
    }
  }

  if (ui->checkBox_axis_l->isChecked()) {
    move_axis(Wheel::left);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    move_axis(Wheel::right);
  }
}

// decstop
void MainWindow::on_pushButton_decstop_0_clicked() {
  double dectime[2] = {ui->textEdit_dectime_0->toPlainText().toDouble(),
                       ui->textEdit_dectime_1->toPlainText().toDouble()};
  short iRet[2] = {0, 0};
  double actuvel[2];
  for (int i = 0; i < 2; i++) {
    iRet[i] = smc_read_current_speed_unit(CARD_NO, i, &actuvel[i]);
    double acc = fabs(actuvel[i] / dectime[i]);
    if (acc > 20000) {
      dectime[i] = fabs(actuvel[i] / 100000);  //如果减速的加速度>100000,修改减速时间使得加速度为100000
    }
  }
  ui->textEdit_dectime_0->setText(QString::number(dectime[0], 'f', 3));
  ui->textEdit_dectime_1->setText(QString::number(dectime[1], 'f', 3));  //将修改后的减速时间显示在QT界面上

  for (int axisNo = 0; axisNo < 2; axisNo++) {
    smc_set_dec_stop_time(CARD_NO, axisNo, dectime[axisNo]);  //设置减速停止时间
    iRet[axisNo] = smc_stop(CARD_NO, axisNo, 0);              //减速停止
  }
  return;
}

void MainWindow::on_pushButton_decstop_1_clicked() {
  on_pushButton_decstop_0_clicked();
}

// emgstop
void MainWindow::on_pushButton_emgstop_0_clicked() {
  short iRet[2] = {0, 0};
  for (int i = 0; i < 2; i++) {
    iRet[i] = smc_stop(CARD_NO, i, 0);
  }
}
void MainWindow::on_pushButton_emgstop_1_clicked() {
  on_pushButton_emgstop_0_clicked();
}

// zero pos
void MainWindow::on_pushButton_zeropos_clicked() {
  short axisNo = 0;
  short iRet = 0;
  double posToSet = 0;
  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iRet = smc_set_position_unit(CARD_NO, axisNo, posToSet);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iRet = smc_set_position_unit(CARD_NO, axisNo, posToSet);
  }
}
// enc pos
void MainWindow::on_pushButton_encpos_clicked() {
  short axisNo = 0;
  short iRet = 0;
  double posToSet = 0;
  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iRet = smc_set_encoder_unit(CARD_NO, axisNo, posToSet);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iRet = smc_set_encoder_unit(CARD_NO, axisNo, posToSet);
  }
}
// stop crd
void MainWindow::on_pushButton_stopcrd_clicked() {
  short iRet = 0;
  iRet = smc_stop_multicoor(CARD_NO, 0, 0);
  iRet = smc_stop_multicoor(CARD_NO, 1, 0);
}
// change vel
void MainWindow::on_pushButton_changevel_clicked() {
  WORD axisNo = 0;
  short iRet = 0;
  double runvel[2] = {ui->textEdit_runvel_0->toPlainText().toDouble(), ui->textEdit_runvel_1->toPlainText().toDouble()};
  vel_limit(runvel);  //限制每个轮子的最大线速度为0.8m/s
  ui->textEdit_runvel_0->setText(QString::number(runvel[0], 'f', 3));
  ui->textEdit_runvel_1->setText(QString::number(runvel[1], 'f', 3));  //将限制的速度显示在QT界面上

  //根据变速前后的速度差值决定变速的时间
  double actuvel[2];
  double time[2];
  for (int i = 0; i < 2; i++) {
    iRet = smc_read_current_speed_unit(0, i, &actuvel[i]);
    double diff = fabs(runvel[i] - actuvel[i]);
    if (diff < 10000) {
      time[i] = 0;  //如果差值diff<10000,则时间为0
    } else {
      time[i] = diff / 100000;  //反之,则时间为diff/100000
    }
  }

  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iRet = smc_change_speed_unit(CARD_NO, axisNo, runvel[0], time[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iRet = smc_change_speed_unit(CARD_NO, axisNo, runvel[1], time[1]);
  }
}
// change pos
void MainWindow::on_pushButton_changepos_clicked() {
  WORD axisNo = 0;
  short iRet = 0;
  double destpos[2] = {ui->textEdit_destpos_0->toPlainText().toDouble(),
                       ui->textEdit_destpos_1->toPlainText().toDouble()};

  if (ui->checkBox_axis_l->isChecked()) {
    axisNo = 0;
    iRet = smc_reset_target_position_unit(CARD_NO, axisNo, destpos[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    axisNo = 1;
    iRet = smc_reset_target_position_unit(CARD_NO, axisNo, destpos[1]);
  }
}

// exit board&application
void MainWindow::on_pushButton_exit_0_clicked() {
  if (borad_init_status == (short)ConnectionStatus::connected) {
    bool disable_success = MainWindow::on_pushButton_disable_clicked();
    if (disable_success == false) {
      return;
    }
  }
  smc_board_close(0);
  qApp->exit(0);
}

void MainWindow::on_pushButton_exit_1_clicked() {
  on_pushButton_exit_0_clicked();  //该按钮和另一个exit效果相同
}

void MainWindow::on_pushButton_start_wc_clicked() {
  //定义轮椅速度向量
  Eigen::MatrixXd v_wheels(2, 1);
  Eigen::MatrixXd v_chairs(2, 1);

  v_chairs(0, 0) = ui->textEdit_linear_vel->toPlainText().toDouble();
  v_chairs(1, 0) = ui->textEdit_angular_vel->toPlainText().toDouble() / 180 * PI;

  v_wheels = trans * v_chairs;  //由轮椅速度反解出电机速度

  double runvel[3][2] = {
      {0, 0},
      {v_wheels(1, 0), v_wheels(0, 0)},  //左轮为0号电机,右轮为1号电机
      {0, 0},
  };
  //限制每个轮子的最大线速度为0.8m/s

  ui->radioButton_fw_0->click();
  ui->radioButton_fw_1->click();  //默认前进

  WORD axisNo[2] = {0, 1};
  short statemachine[2] = {1, 1};

  if (smc_check_done(CARD_NO, (WORD)Wheel::left) == 0 ||
      smc_check_done(CARD_NO, (WORD)Wheel::right) == 0) {  //该轴已经在运动中
    return;
  }
  statemachine[0] = smc_read_sevon_pin(CARD_NO, (WORD)Wheel::left);   //获取0轴状态机
  statemachine[1] = smc_read_sevon_pin(CARD_NO, (WORD)Wheel::right);  //获取1轴状态机
  if (statemachine[1] == 1 || statemachine[0] == 1) {
    information_disable();  //返回错误信号,停止该函数的运行
    return;
  }

  if (ui->radioButton_cs->isChecked()) {  // constant speed
    double pulse[2] = {0, 0};
    get_status(Wheel::left, runvel[1], pulse);
    get_status(Wheel::right, runvel[1], pulse);

    move_axis(Wheel::left);
    move_axis(Wheel::right);
  } else if (ui->radioButton_fl->isChecked()) {  // fixed length
    //定义电机在轮椅直线运动时的速度大小
    runvel[1][0] = fabs(ui->textEdit_goal_dv->toPlainText().toDouble() / COEFF);
    runvel[1][1] = runvel[1][0];
    //定义电机在轮椅旋转时的速度大小
    for (int i = 0; i < 3; i = i + 2) {
      runvel[i][0] = fabs(ui->textEdit_goal_dtheta->toPlainText().toDouble() * PI / 180 * SPACE / COEFF / 2);
      runvel[1][1] = runvel[i][0];
    }

    //定义轮椅的终点位置和角度，其中轮椅前进方向为x轴正方向，轮椅左侧垂直于x轴为y轴正方向，逆时针为theta正方向
    double goal_x = ui->textEdit_goal_x->toPlainText().toDouble();
    double goal_y = ui->textEdit_goal_y->toPlainText().toDouble();
    //获取目标的轮椅角度,并且保证它的取值范围是(-pi,pi]
    double goal_theta = ui->textEdit_goal_theta->toPlainText().toDouble() / 180 * PI;
    while (goal_theta > PI) {
      goal_theta = goal_theta - 2 * PI;
    }
    while (goal_theta <= -PI) {
      goal_theta = goal_theta + 2 * PI;
    }

    double dist = -sqrt(pow(goal_x, 2) + pow(goal_y, 2));  //计算轮椅起点和终点之间的距离
    double delta_theta = atan2(goal_y, goal_x);  //计算轮椅终点和起点连线与轮椅当前位置的角度,取值范围(-pi,pi]
    //如果目标角度和连线角度之间的差值大于pi/2或者小于-pi/2,则将轮椅在第二部分直线运动的前进改为后退
    if (goal_theta <= PI / 2 && goal_theta >= -PI / 2 && (delta_theta < -PI / 2 || delta_theta > PI / 2)) {
      if (delta_theta < -PI / 2) {
        delta_theta = delta_theta + PI;
      } else {
        delta_theta = delta_theta - PI;
      }
    } else {
      //如果目标角度和连线角度之间的差值在[-pi/2,pi/2]之内,则轮椅第二部分直线运动为前进,对应到电机的运动方向为负
      dist = -dist;
    }
    double pulse[3][2];  //定义轮椅两个电机在三段运动中的脉冲数的多维数组

    Eigen::MatrixXd theta_chairs_1(2, 1);
    theta_chairs_1(0, 0) = 0;
    theta_chairs_1(1, 0) = delta_theta;

    Eigen::MatrixXd pulse_wheels_1(2, 1);
    pulse_wheels_1 = trans * theta_chairs_1;

    //如果目标角度和连线角度之间的差值的绝对值大于pi,则将其补回(-pi,pi]的区间内
    if (goal_theta - delta_theta > PI) {
      delta_theta = delta_theta + 2 * PI;
    } else if (goal_theta - delta_theta < -PI) {
      delta_theta = delta_theta - 2 * PI;
    }
    Eigen::MatrixXd theta_chairs_2(2, 1);
    theta_chairs_2(0, 0) = 0;
    theta_chairs_2(1, 0) = goal_theta - delta_theta;

    Eigen::MatrixXd pulse_wheels_2(2, 1);
    pulse_wheels_2 = trans * theta_chairs_2;

    // step 1:轮椅转到面向终点位置（x,y)的方向
    pulse[0][0] = pulse_wheels_1(1, 0);
    pulse[0][1] = pulse_wheels_1(0, 0);  //左轮为0号电机,右轮为1号电机

    // step 2:轮椅沿直线运动到终点位置（x,y)
    pulse[1][0] = dist / COEFF;
    pulse[1][1] = dist / COEFF;

    // step 3:轮椅调整到目标角度goal_theta
    pulse[2][0] = pulse_wheels_2(1, 0);
    pulse[2][1] = pulse_wheels_2(0, 0);  //左轮为0号电机,右轮为1号电机

    for (int i = 0; i < 3; i++) {
      get_status(Wheel::left, runvel[i], pulse[i]);
      get_status(Wheel::right, runvel[i], pulse[i]);

      move_axis(Wheel::left);
      move_axis(Wheel::right);

      while (smc_check_done(CARD_NO, (WORD)Wheel::left) == 0 || smc_check_done(CARD_NO, (WORD)Wheel::right) == 0) {
        system("pause");
      }
    }
  }
}

void MainWindow::on_pushButton_changevel_wc_clicked() {
  //定义轮椅速度矩阵
  Eigen::MatrixXd v_wheels(2, 1);
  Eigen::MatrixXd v_chairs(2, 1);
  v_chairs(0, 0) = ui->textEdit_linear_vel->toPlainText().toDouble();
  v_chairs(1, 0) = ui->textEdit_angular_vel->toPlainText().toDouble() / 180 * PI;

  v_wheels = trans * v_chairs;  //由轮椅速度反解出电机速度

  short iRet[2] = {0, 0};
  double runvel[2] = {-v_wheels(1, 0), -v_wheels(0, 0)};  //左轮为0号电机,右轮为1号电机
  vel_limit(runvel);                                      //限制每个轮子的最大线速度为0.8m/s

  //根据变速前后的速度差值决定变速的时间
  double actuvel[2];
  double time[2];
  for (int i = 0; i < 2; i++) {
    iRet[i] = smc_read_current_speed_unit(CARD_NO, i, &actuvel[i]);
    double diff = fabs(runvel[i] - actuvel[i]);
    if (diff < 20000) {
      time[i] = 0;
    } else {
      time[i] = diff / 100000;
    }
  }

  for (int i = 0; i < 2; i++) {
    iRet[i] = smc_change_speed_unit(CARD_NO, i, runvel[i], time[i]);
  }
}

void MainWindow::on_pushButton_changepos_wc_clicked() {
}
