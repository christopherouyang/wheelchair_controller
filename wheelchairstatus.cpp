#include "wheelchairstatus.h"
#include "ui_mainwindow.h"

using Eigen::MatrixXd;

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

WheelchairStatus::WheelchairStatus() {
}

bool WheelchairStatus::enable_axis(int axisNo, int m_nConnectNo) {
  // MainWindow *p=(MainWindow*) parentWidget();
  short iret = 0;
  short statemachine = 1;

  time_t t1, t2;

  t1 = time(NULL);           //设置时间
  while (statemachine == 1)  //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
  {
    iret = smc_write_sevon_pin(m_nConnectNo, axisNo, 0);      //设置0轴使能
    statemachine = smc_read_sevon_pin(m_nConnectNo, axisNo);  //获取0轴状态机
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

bool WheelchairStatus::disable_axis(int axisNo, int m_nConnectNo) {
  short iret = 0;
  short statemachine = 0;

  time_t t1, t2;

  t1 = time(NULL);           //设置时间
  while (statemachine == 0)  //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
  {
    iret = smc_write_sevon_pin(m_nConnectNo, axisNo, 1);      //设置0轴使能
    statemachine = smc_read_sevon_pin(m_nConnectNo, axisNo);  //获取0轴状态机
    t2 = time(NULL);
    if (t2 - t1 > 3)  // 3 秒时间防止死循环
    {
      QString error = (QString)axisNo + "轴去使能超时，请检查设备";
      ui->label_error->setText(error);
      return false;
    }
  }
  return true;
}

void WheelchairStatus::get_status(int axisNo, double *runvel, double *pulse) {
  vel_limit(runvel);
  if (axisNo == 0) {
    status[axisNo].startvel = ui->textEdit_startvel_0->toPlainText().toDouble();
    status[axisNo].runvel = runvel[axisNo];
    status[axisNo].stopvel = ui->textEdit_stopvel_0->toPlainText().toDouble();
    status[axisNo].acctime = ui->textEdit_acctime_0->toPlainText().toDouble();
    status[axisNo].dectime = ui->textEdit_dectime_0->toPlainText().toDouble();
    status[axisNo].stime = ui->textEdit_stime_0->toPlainText().toDouble();
    status[axisNo].pulse = pulse[axisNo];

    if (ui->radioButton_fw_0->isChecked()) {
      status[axisNo].direction = (int)Direction::forward;
    } else {
      status[axisNo].direction = (int)Direction::backward;
    }

  } else if (axisNo == 1) {
    status[axisNo].startvel = ui->textEdit_startvel_1->toPlainText().toDouble();
    status[axisNo].runvel = runvel[axisNo];
    status[axisNo].stopvel = ui->textEdit_stopvel_1->toPlainText().toDouble();
    status[axisNo].acctime = ui->textEdit_acctime_1->toPlainText().toDouble();
    status[axisNo].dectime = ui->textEdit_dectime_1->toPlainText().toDouble();
    status[axisNo].stime = ui->textEdit_stime_1->toPlainText().toDouble();
    status[axisNo].pulse = pulse[axisNo];

    if (ui->radioButton_fw_1->isChecked()) {
      status[axisNo].direction = (int)Direction::forward;
    } else {
      status[axisNo].direction = (int)Direction::backward;
    }
  }

  if (ui->radioButton_fl->isChecked()) {
    status[axisNo].mode = MovingMode::fixedLength;
  } else {
    status[axisNo].mode = MovingMode::constantSpeed;
  }

  if (status[axisNo].runvel < 0) {
    status[axisNo].direction = 1 - status[axisNo].direction;
    status[axisNo].runvel = -status[axisNo].runvel;
  }

  return;
}

void WheelchairStatus::set_wheelchair_moving_parameter(bool isConstantSpeed, double pulse[3][2], double runvel[3][2]) {
  if (isConstantSpeed) {
    set_wheelchair_constant_speed_parameter(runvel);
  } else {
    set_wheelchair_fixed_length_parameter(pulse, runvel);
  }
}

void WheelchairStatus::set_wheelchair_constant_speed_parameter(double runvel[3][2]) {
  MatrixXd v_wheels(2, 1);
  MatrixXd v_chairs(2, 1);

  v_chairs(0, 0) = ui->textEdit_linear_vel->toPlainText().toDouble();
  v_chairs(1, 0) = ui->textEdit_angular_vel->toPlainText().toDouble() / 180 * PI;

  v_wheels = trans * v_chairs;  //由轮椅速度反解出电机速度

  runvel[1][0] = v_wheels(1, 0);
  runvel[1][1] = v_wheels(0, 0);  //左轮为0号电机,右轮为1号电机
}

void WheelchairStatus::set_wheelchair_fixed_length_parameter(double pulse[3][2], double runvel[3][2]) {
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

  double dist = sqrt(pow(goal_x, 2) + pow(goal_y, 2));  //计算轮椅起点和终点之间的距离
  double delta_theta = atan2(goal_y, goal_x);  //计算轮椅终点和起点连线与轮椅当前位置的角度,取值范围(-pi,pi]
  //如果目标角度和连线角度之间的差值大于pi/2或者小于-pi/2,则将轮椅在第二部分直线运动的前进改为后退
  if (goal_theta <= PI / 2 && goal_theta >= -PI / 2 && (delta_theta < -PI / 2 || delta_theta > PI / 2)) {
    if (delta_theta < -PI / 2) {
      delta_theta = delta_theta + PI;
    } else {
      delta_theta = delta_theta - PI;
    }

  } else {
    dist =
        -dist;  //如果目标角度和连线角度之间的差值在[-pi/2,pi/2]之内,则轮椅第二部分直线运动为前进,对应到电机的运动方向为负
  }

  MatrixXd theta_chairs_1(2, 1);
  theta_chairs_1(0, 0) = 0;
  theta_chairs_1(1, 0) = delta_theta;

  MatrixXd pulse_wheels_1(2, 1);
  pulse_wheels_1 = trans * theta_chairs_1;

  //如果目标角度和连线角度之间的差值的绝对值大于pi,则将其补回(-pi,pi]的区间内
  if (goal_theta - delta_theta > PI) {
    delta_theta = delta_theta + 2 * PI;
  } else if (goal_theta - delta_theta < -PI) {
    delta_theta = delta_theta - 2 * PI;
  }
  MatrixXd theta_chairs_2(2, 1);
  theta_chairs_2(0, 0) = 0;
  theta_chairs_2(1, 0) = goal_theta - delta_theta;

  MatrixXd pulse_wheels_2(2, 1);
  pulse_wheels_2 = trans * theta_chairs_2;

  // step 1:轮椅转到面向终点位置（x,y)的方向
  pulse[0][0] = -pulse_wheels_1(1, 0);
  pulse[0][1] = -pulse_wheels_1(0, 0);  //左轮为0号电机,右轮为1号电机

  // step 2:轮椅沿直线运动到终点位置（x,y)
  pulse[1][0] = dist / COEFF;
  pulse[1][1] = dist / COEFF;

  // step 3:轮椅调整到目标角度goal_theta
  pulse[2][0] = -pulse_wheels_2(1, 0);
  pulse[2][1] = -pulse_wheels_2(0, 0);  //左轮为0号电机,右轮为1号电机
}

bool WheelchairStatus::is_ready_to_start() {
  short statemachine[2] = {1, 1};
  WORD axisNo[2] = {0, 1};
  for (int i = 0; i < 2; i++) {
    if (smc_check_done(0, axisNo[i]) == 0)  //该轴已经在运动中
      return false;
    if (ui->checkBox_axis_l->isChecked() && ui->checkBox_axis_r->isChecked()) {
      statemachine[0] = smc_read_sevon_pin(0, 0);  //获取0轴状态机
      statemachine[1] = smc_read_sevon_pin(0, 1);  //获取1轴状态机
      if (statemachine[1] == 1 ||
          statemachine[0] == 1)  //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
      {
        information_disable();  //返回错误信号,停止该函数的运行
        return false;
      }
    }
  }

  return true;
}

void WheelchairStatus::move_axis(int axisNo) {
  short statemachine = 1;
  ;

  short iret = 0;

  statemachine = smc_read_sevon_pin(0, axisNo);  //获取状态机
  if (statemachine == 1)  //监控轴状态机的值，该值等于0表示轴已经使能,等于1则表示该轴未使能
  {
    information_disable_axis(axisNo);  //返回错误信号,停止该函数的运行
    return;
  }
  iret = smc_set_equiv(0, axisNo, 1);           //设置脉冲当量
  iret = smc_set_alm_mode(0, axisNo, 0, 0, 0);  //设置报警使能,关闭报警
  iret = smc_set_pulse_outmode(0, axisNo, 0);  //设定脉冲模式（此处脉冲模式固定为 P+D 方向：脉冲+方向）
  iret = smc_set_profile_unit(0, axisNo, status[axisNo].startvel, status[axisNo].runvel, status[axisNo].acctime,
                              status[axisNo].dectime, status[axisNo].stopvel);  //设定单轴运动速度参数
  iret = smc_set_s_profile(0, axisNo, 0, status[axisNo].stime);
  if (status[axisNo].mode == MovingMode::fixedLength) {
    iret = smc_pmove_unit(0, axisNo, status[axisNo].pulse * (2 * status[axisNo].direction - 1), 0);  //相对定长运动
  } else {
    iret = smc_vmove(0, axisNo, status[axisNo].direction);  //恒速运动
  }
  return;
}
