#include "windowinformation.h"
#include <QMessageBox>

WindowInformation::WindowInformation() {
}
void WindowInformation::information_connection_fail() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "连接失败！请检查控制卡的连接";
  reply = QMessageBox::information(this, tr("Connection Fails"), MESSAGE);
}

void WindowInformation::information_connection_success() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "控制卡连接成功";
  reply = QMessageBox::information(this, tr("Connection Success"), MESSAGE);
}

void WindowInformation::information_disable() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "电机未使能,请先将电机使能";
  reply = QMessageBox::information(this, tr("Motor is disabled"), MESSAGE);
}
void WindowInformation::information_disable_axis(int axisNo) {
  QMessageBox::StandardButton reply;
  QString message[2] = {
      "左轮电机未使能,请先将电机使能"
      "右轮电机未使能,请先将电机使能"};
  reply = QMessageBox::information(this, tr("Motor_0 is disabled"), message[axisNo]);
}

void WindowInformation::information_emgstop_on() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "急停开关被按下,请先释放急停开关再使能";
  reply = QMessageBox::information(this, tr("EMG stop is on"), MESSAGE);
}
void WindowInformation::information_connection_interrupted() {
  QMessageBox::StandardButton reply;
  QString MESSAGE = "连接中断！请检查控制卡的连接";
  reply = QMessageBox::information(this, tr("Connection is interrupted"), MESSAGE);
}
