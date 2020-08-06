#include "mainwindow.h"
#include "ui_mainwindow.h"
#include "LTSMC.h"
#include <QDebug>
#include <QMessageBox>
#include <eigen3/Eigen/Dense>
#include "math.h"
#include <unistd.h>
#include <string>

// const parameter for the controller board
static short borad_init_status = (short)ConnectionStatus::unconnected;
constexpr WORD CARD_NO = 0;
constexpr WORD EMG_STOP_BIT_NO = 0;
constexpr DWORD ERROR_CODE_SUCCESS = 0;
constexpr WORD S_MODE = 0;
constexpr WORD DEFAULT_PULSE_OUTMODE = 0;
constexpr WORD DEFAULT_ALARM_ACTION = 0;
constexpr double PULSE_PER_UNIT = 1;
constexpr double PULSE_PER_LOOP = 320000;
constexpr double UNIT_PER_LOOP = PULSE_PER_LOOP / PULSE_PER_UNIT;
constexpr int INTERVAL_IN_MILLI_SECOND = 200;

// const paramter in math
constexpr double PI = 3.1415926358;
constexpr double RADIAN_TO_ANGULAR = 180 / PI;
constexpr double ANGULAR_TO_RADIAN = PI / 180;

// const parameter for the wheelchair
constexpr double RADIUS = 0.164;
constexpr double WHEEL_BASE = 0.552;
constexpr double HALF_WHEEL_BASE = WHEEL_BASE / 2;
constexpr double UNIT_PER_METER = UNIT_PER_LOOP / (2 * PI * RADIUS);
constexpr double MAX_VEL = 0.8;
constexpr double VEL_LIMIT = MAX_VEL * UNIT_PER_METER;
constexpr int MAX_ACC = 100000;
static Eigen::MatrixXd TRANS_MATRIX(2, 2);

static void vel_limit(double *runVel) {
  //将两轮的速度限制在0.8m/s之内
  for (int i = 0; i < 2; i++) {
    runVel[i] = runVel[i] > VEL_LIMIT ? VEL_LIMIT : runVel[i];
    runVel[i] = runVel[i] < -VEL_LIMIT ? -VEL_LIMIT : runVel[i];
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
  return (status != 1) ? "Running" : "Static";
}

MainWindow::MainWindow(QWidget *parent) : QMainWindow(parent), ui(new Ui::MainWindow) {
  ui->setupUi(this);
  initDialog();
  char pIpAddress[] = "192.168.5.11";

  borad_init_status = smc_board_init(CARD_NO, (WORD)ConnectType::ethercat, pIpAddress, 0);
  if (borad_init_status != (short)ConnectionStatus::connected) {
    qDebug("smc_board_init iRet = %d\n", borad_init_status);
    qDebug("连接失败！请检查控制卡的连接");
    information_connection_fail();
  } else {
    qDebug("控制卡连接成功！");
    information_connection_success();
  }
  TRANS_MATRIX << UNIT_PER_METER, UNIT_PER_METER * HALF_WHEEL_BASE, UNIT_PER_METER, -UNIT_PER_METER * HALF_WHEEL_BASE;

  startTimer(INTERVAL_IN_MILLI_SECOND);
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

  ui->textEdit_startVel_0->setText("100");
  ui->textEdit_runVel_0->setText("32000");
  ui->textEdit_stopVel_0->setText("100");
  ui->textEdit_accTime_0->setText("0.2");
  ui->textEdit_decTime_0->setText("0.2");
  ui->textEdit_sTime_0->setText("0.05");
  ui->textEdit_destPos_0->setText("20000");
  ui->textEdit_pulse_0->setText("320000");  //左轮

  ui->textEdit_startVel_1->setText("100");
  ui->textEdit_runVel_1->setText("32000");
  ui->textEdit_stopVel_1->setText("100");
  ui->textEdit_accTime_1->setText("0.2");
  ui->textEdit_decTime_1->setText("0.2");
  ui->textEdit_sTime_1->setText("0.05");
  ui->textEdit_destPos_1->setText("20000");
  ui->textEdit_pulse_1->setText("320000");  //右轮

  //设定轮椅匀速运动默认参数
  ui->textEdit_linear_vel->setText("0.1");  //默认移动的线速度为0.1m/s
  ui->textEdit_angular_vel->setText("0");   //默认移动的角速度为0

  //轮椅定长运动参数
  ui->textEdit_destX->setText("0.2");
  ui->textEdit_destY->setText("0");
  ui->textEdit_destAngle->setText("0");            //默认移动方式：前进0.2m
  ui->textEdit_moving_linearVel->setText("0.15");  //默认直线运动速度为0.2m/s
  ui->textEdit_moving_angularVel->setText("15");   //默认角速度为15°/s
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
  linear_v = -(speed[0] + speed[1]) / UNIT_PER_METER / 2;
  angular_v = -(speed[1] - speed[0]) / UNIT_PER_METER / WHEEL_BASE * RADIAN_TO_ANGULAR;

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
        iRet[i] = smc_stop(CARD_NO, i, (WORD)StopMode::emgStop);
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
    if (t2 - t1 > 3) {  // 3 秒时间防止死循环
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
    if (t2 - t1 > 3) {  // 3 秒时间防止死循环
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

void MainWindow::get_moving_status(Wheel wheelNo, double *runVel, double *pulse) {
  int axisNo = (int)wheelNo;
  vel_limit(runVel);
  if (wheelNo == Wheel::left) {
    status[axisNo].startVel = ui->textEdit_startVel_0->toPlainText().toDouble();
    status[axisNo].runVel = runVel[axisNo];
    status[axisNo].stopVel = ui->textEdit_stopVel_0->toPlainText().toDouble();
    status[axisNo].accTime = ui->textEdit_accTime_0->toPlainText().toDouble();
    status[axisNo].decTime = ui->textEdit_decTime_0->toPlainText().toDouble();
    status[axisNo].sTime = ui->textEdit_sTime_0->toPlainText().toDouble();
    status[axisNo].pulse = pulse[axisNo];
    status[axisNo].direction = ui->radioButton_fw_0->isChecked() ? (int)Direction::forward : (int)Direction::backward;

  } else if (wheelNo == Wheel::right) {
    status[axisNo].startVel = ui->textEdit_startVel_1->toPlainText().toDouble();
    status[axisNo].runVel = runVel[axisNo];
    status[axisNo].stopVel = ui->textEdit_stopVel_1->toPlainText().toDouble();
    status[axisNo].accTime = ui->textEdit_accTime_1->toPlainText().toDouble();
    status[axisNo].decTime = ui->textEdit_decTime_1->toPlainText().toDouble();
    status[axisNo].sTime = ui->textEdit_sTime_1->toPlainText().toDouble();
    status[axisNo].pulse = pulse[axisNo];
    status[axisNo].direction = ui->radioButton_fw_1->isChecked() ? (int)Direction::forward : (int)Direction::backward;
  }

  status[axisNo].mode = ui->radioButton_fl->isChecked() ? MovingMode::fixedLength : MovingMode::constantSpeed;

  if (status[axisNo].runVel < 0) {
    status[axisNo].direction = 1 - status[axisNo].direction;
    status[axisNo].runVel = -status[axisNo].runVel;
  }
}

void MainWindow::move_axis(Wheel wheelNo) {
  int axisNo = (int)wheelNo;
  short statemachine = smc_read_sevon_pin(CARD_NO, axisNo);
  if (statemachine == (short)AxisEnableStatus::off) {
    information_disable_axis(axisNo);
    return;
  }
  short iRet = smc_set_equiv(CARD_NO, axisNo, PULSE_PER_UNIT);  //设置脉冲当量
  iRet = smc_set_alm_mode(CARD_NO, axisNo, (WORD)SignalEnableStatus::disable, (WORD)ElectricalLevel::low,
                          DEFAULT_ALARM_ACTION);                         //设置报警使能,关闭报警
  iRet = smc_set_pulse_outmode(CARD_NO, axisNo, DEFAULT_PULSE_OUTMODE);  //此处脉冲模式固定为 P+D 方向：脉冲+方向
  iRet = smc_set_profile_unit(CARD_NO, axisNo, status[axisNo].startVel, status[axisNo].runVel, status[axisNo].accTime,
                              status[axisNo].decTime, status[axisNo].stopVel);  //设定单轴运动速度参数
  iRet = smc_set_s_profile(CARD_NO, axisNo, S_MODE, status[axisNo].sTime);
  if (status[axisNo].mode == MovingMode::fixedLength) {
    iRet = smc_pmove_unit(CARD_NO, axisNo, status[axisNo].pulse * (2 * status[axisNo].direction - 1),
                          (WORD)PulseMoveMOde::relative);  //相对定长运动
  } else {
    iRet = smc_vmove(CARD_NO, axisNo, status[axisNo].direction);  //恒速运动
  }
}

// start
void MainWindow::on_pushButton_start_0_clicked() {
  double runVel[2] = {ui->textEdit_runVel_0->toPlainText().toDouble(), ui->textEdit_runVel_1->toPlainText().toDouble()};
  double pulse[2] = {ui->textEdit_pulse_0->toPlainText().toDouble(), ui->textEdit_pulse_1->toPlainText().toDouble()};
  get_moving_status(Wheel::left, runVel, pulse);
  get_moving_status(Wheel::right, runVel, pulse);

  if (smc_check_done(CARD_NO, (WORD)Wheel::left) == (short)AxisMovingStatus::moving ||
      smc_check_done(CARD_NO, (WORD)Wheel::right) == (short)AxisMovingStatus::moving) {  //该轴已经在运动中
    return;
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
  double decTime[2] = {ui->textEdit_decTime_0->toPlainText().toDouble(),
                       ui->textEdit_decTime_1->toPlainText().toDouble()};
  short iRet = 0;
  double actuvel[2];
  for (int i = 0; i < 2; i++) {
    iRet = smc_read_current_speed_unit(CARD_NO, i, &actuvel[i]);
    decTime[i] = (decTime[i] < actuvel[i] / MAX_ACC) ? actuvel[i] / MAX_ACC : decTime[i];
  }

  ui->textEdit_decTime_0->setText(QString::number(decTime[0], 'f', 3));
  ui->textEdit_decTime_1->setText(QString::number(decTime[1], 'f', 3));  //将修改后的减速时间显示在QT界面上

  for (int axisNo = 0; axisNo < 2; axisNo++) {
    smc_set_dec_stop_time(CARD_NO, axisNo, decTime[axisNo]);    //设置减速停止时间
    iRet = smc_stop(CARD_NO, axisNo, (WORD)StopMode::decStop);  //减速停止
  }

  return;
}

void MainWindow::on_pushButton_decstop_1_clicked() {
  on_pushButton_decstop_0_clicked();
}

// emgstop
void MainWindow::on_pushButton_emgstop_0_clicked() {
  for (int i = 0; i < 2; i++) {
    short iRet = smc_stop(CARD_NO, i, (WORD)StopMode::emgStop);
  }
}
void MainWindow::on_pushButton_emgstop_1_clicked() {
  on_pushButton_emgstop_0_clicked();
}

// zero pos
void MainWindow::on_pushButton_zeropos_clicked() {
  short iRet = 0;
  double posToSet = 0;
  if (ui->checkBox_axis_l->isChecked()) {
    iRet = smc_set_position_unit(CARD_NO, (WORD)Wheel::left, posToSet);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    iRet = smc_set_position_unit(CARD_NO, (WORD)Wheel::right, posToSet);
  }
}
// enc pos
void MainWindow::on_pushButton_encpos_clicked() {
  short iRet = 0;
  double posToSet = 0;
  if (ui->checkBox_axis_l->isChecked()) {
    iRet = smc_set_encoder_unit(CARD_NO, (WORD)Wheel::left, posToSet);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    iRet = smc_set_encoder_unit(CARD_NO, (WORD)Wheel::right, posToSet);
  }
}
// stop crd
void MainWindow::on_pushButton_stopcrd_clicked() {
  short iRet = 0;
  iRet = smc_stop_multicoor(CARD_NO, (WORD)Wheel::left, (WORD)StopMode::decStop);
  iRet = smc_stop_multicoor(CARD_NO, (WORD)Wheel::left, (WORD)StopMode::decStop);
}
// change vel
void MainWindow::on_pushButton_changevel_clicked() {
  short iRet = 0;
  double runVel[2] = {ui->textEdit_runVel_0->toPlainText().toDouble(), ui->textEdit_runVel_1->toPlainText().toDouble()};
  vel_limit(runVel);  //限制每个轮子的最大线速度为0.8m/s
  ui->textEdit_runVel_0->setText(QString::number(runVel[0], 'f', 3));
  ui->textEdit_runVel_1->setText(QString::number(runVel[1], 'f', 3));  //将限制的速度显示在QT界面上

  //根据变速前后的速度差值决定变速的时间
  double actuvel[2];
  double time[2];
  for (int i = 0; i < 2; i++) {
    iRet = smc_read_current_speed_unit(CARD_NO, i, &actuvel[i]);
    time[i] = fabs(runVel[i] - actuvel[i]) / MAX_ACC;
  }

  if (ui->checkBox_axis_l->isChecked()) {
    iRet = smc_change_speed_unit(CARD_NO, (WORD)Wheel::left, runVel[0], time[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    iRet = smc_change_speed_unit(CARD_NO, (WORD)Wheel::right, runVel[1], time[1]);
  }
}
// change pos
void MainWindow::on_pushButton_changepos_clicked() {
  short iRet = 0;
  double destPos[2] = {ui->textEdit_destPos_0->toPlainText().toDouble(),
                       ui->textEdit_destPos_1->toPlainText().toDouble()};

  if (ui->checkBox_axis_l->isChecked()) {
    iRet = smc_reset_target_position_unit(CARD_NO, (WORD)Wheel::left, destPos[0]);
  }
  if (ui->checkBox_axis_r->isChecked()) {
    iRet = smc_reset_target_position_unit(CARD_NO, (WORD)Wheel::right, destPos[1]);
  }
}

// exit board&application
void MainWindow::on_pushButton_exit_0_clicked() {
  if (borad_init_status == (short)ConnectionStatus::connected) {
    if (!MainWindow::on_pushButton_disable_clicked()) {
      printf("warning: disable axis falied");
      return;
    }
  }
  smc_board_close(CARD_NO);
  qApp->exit(0);
}

void MainWindow::on_pushButton_exit_1_clicked() {
  on_pushButton_exit_0_clicked();  //该按钮和另一个exit效果相同
}

void MainWindow::on_pushButton_start_wc_clicked() {
  Eigen::Vector2d motorVel(2, 1);
  calculate_motor_vel(motorVel);

  double runVel[3][2] = {{0, 0}, {motorVel(1, 0), motorVel(0, 0)}, {0, 0}};
  double pulse[3][2] = {{0, 0}, {0, 0}, {0, 0}};
  //限制每个轮子的最大线速度为0.8m/s

  ui->radioButton_fw_0->click();
  ui->radioButton_fw_1->click();  //默认前进

  WORD axisNo[2] = {0, 1};
  short statemachine[2] = {1, 1};

  if (smc_check_done(CARD_NO, (WORD)Wheel::left) == (short)AxisMovingStatus::moving ||
      smc_check_done(CARD_NO, (WORD)Wheel::right) == (short)AxisMovingStatus::moving) {  //该轴已经在运动中
    return;
  }

  if (ui->radioButton_cs->isChecked()) {  // constant speed
    get_moving_status(Wheel::left, runVel[1], pulse[1]);
    get_moving_status(Wheel::right, runVel[1], pulse[1]);

    move_axis(Wheel::left);
    move_axis(Wheel::right);
  } else if (ui->radioButton_fl->isChecked()) {  // fixed length
    get_fixed_length_parameter(runVel, pulse);
    for (int i = 0; i < 3; i++) {
      get_moving_status(Wheel::left, runVel[i], pulse[i]);
      get_moving_status(Wheel::right, runVel[i], pulse[i]);

      move_axis(Wheel::left);
      move_axis(Wheel::right);

      while (smc_check_done(CARD_NO, (WORD)Wheel::left) == (short)AxisMovingStatus::moving ||
             smc_check_done(CARD_NO, (WORD)Wheel::right) == (short)AxisMovingStatus::moving) {
        system("pause");
      }
    }
  }
}

void MainWindow::calculate_motor_vel(Eigen::Vector2d &motorVel) {
  //定义轮椅速度矩阵
  Eigen::Vector2d chairVel(ui->textEdit_linear_vel->toPlainText().toDouble(),
                           ui->textEdit_angular_vel->toPlainText().toDouble() * ANGULAR_TO_RADIAN);
  motorVel = TRANS_MATRIX * chairVel;  //由轮椅速度反解出电机速度
}

void MainWindow::on_pushButton_changevel_wc_clicked() {
  //定义轮椅速度矩阵
  Eigen::Vector2d motorVel(2, 1);
  calculate_motor_vel(motorVel);

  short iRet = 0;
  double runVel[2] = {motorVel(1, 0), motorVel(0, 0)};
  vel_limit(runVel);

  //根据变速前后的速度差值决定变速的时间
  double actuvel[2];
  double time[2];
  for (int i = 0; i < 2; i++) {
    iRet = smc_read_current_speed_unit(CARD_NO, i, &actuvel[i]);
    time[i] = fabs(runVel[i] - actuvel[i]) / MAX_ACC;
  }

  for (int i = 0; i < 2; i++) {
    iRet = smc_change_speed_unit(CARD_NO, i, runVel[i], time[i]);
  }
}

void MainWindow::on_pushButton_changepos_wc_clicked() {
}

void MainWindow::get_fixed_length_parameter(double runVel[][2], double pulse[][2]) {
  //定义电机在轮椅直线运动时的速度大小
  runVel[1][0] = fabs(ui->textEdit_moving_linearVel->toPlainText().toDouble() * UNIT_PER_METER);
  runVel[1][1] = runVel[1][0];
  //定义电机在轮椅旋转时的速度大小
  for (int i = 0; i < 3; i = i + 2) {
    runVel[i][0] = fabs(ui->textEdit_moving_angularVel->toPlainText().toDouble() * ANGULAR_TO_RADIAN * HALF_WHEEL_BASE *
                        UNIT_PER_METER);
    runVel[1][1] = runVel[i][0];
  }

  //定义轮椅的终点位置和角度，其中轮椅前进方向为x轴正方向，轮椅左侧垂直于x轴为y轴正方向，逆时针为theta正方向
  double destX = ui->textEdit_destX->toPlainText().toDouble();
  double destY = ui->textEdit_destY->toPlainText().toDouble();
  //获取目标的轮椅角度,并且保证它的取值范围是(-pi,pi]
  double destAngle = ui->textEdit_destAngle->toPlainText().toDouble() * ANGULAR_TO_RADIAN;
  while (destAngle > PI) {
    destAngle -= 2 * PI;
  }
  while (destAngle <= -PI) {
    destAngle += 2 * PI;
  }

  double dist = -sqrt(pow(destX, 2) + pow(destY, 2));  //计算轮椅起点和终点之间的距离
  double deltaAngle = atan2(destY, destX);  //计算轮椅终点和起点连线与轮椅当前位置的角度,取值范围(-pi,pi]
  //如果目标角度和连线角度之间的差值大于pi/2或者小于-pi/2,则将轮椅在第二部分直线运动的前进改为后退
  if (destAngle <= PI / 2 && destAngle >= -PI / 2 && (deltaAngle < -PI / 2 || deltaAngle > PI / 2)) {
    deltaAngle += (deltaAngle < -PI / 2) ? PI : -PI;
  } else {
    //如果目标角度和连线角度之间的差值在[-pi/2,pi/2]之内,则轮椅第二部分直线运动为前进,对应到电机的运动方向为负
    dist = -dist;
  }

  Eigen::Vector2d angelStep1(0, deltaAngle);
  Eigen::Vector2d pulseStep1 = TRANS_MATRIX * angelStep1;

  //如果目标角度和连线角度之间的差值的绝对值大于pi,则将其补回(-pi,pi]的区间内
  if (destAngle - deltaAngle > PI) {
    deltaAngle += 2 * PI;
  } else if (destAngle - deltaAngle < -PI) {
    deltaAngle -= 2 * PI;
  }
  Eigen::Vector2d angelStep3(0, destAngle - deltaAngle);
  Eigen::Vector2d pulseStep3 = TRANS_MATRIX * angelStep3;

  // step 1:轮椅转到面向终点位置（x,y)的方向
  pulse[0][0] = pulseStep1(1, 0);
  pulse[0][1] = pulseStep1(0, 0);

  // step 2:轮椅沿直线运动到终点位置（x,y)
  pulse[1][0] = dist * UNIT_PER_METER;
  pulse[1][1] = dist * UNIT_PER_METER;

  // step 3:轮椅调整到目标角度destAngle
  pulse[2][0] = pulseStep3(1, 0);
  pulse[2][1] = pulseStep3(0, 0);
}
