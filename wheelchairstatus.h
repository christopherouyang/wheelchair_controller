#ifndef WHEELCHAIRSTATUS_H
#define WHEELCHAIRSTATUS_H

#include "mainwindow.h"

class WheelchairStatus : public MainWindow {
 public:
  WheelchairStatus();

 private:


  virtual bool enable_axis(int axisNo, int m_nConnectNo);
  virtual bool disable_axis(int axisNo, int m_nConnectNo);

  virtual void get_status(int axisNo, double *runvel, double *pulse);
  virtual void set_wheelchair_moving_parameter(bool isConstantSpeed, double pulse[3][2], double runvel[3][2]);

  virtual void set_wheelchair_constant_speed_parameter(double runvel[3][2]);
  virtual void set_wheelchair_fixed_length_parameter(double pulse[3][2], double runvel[3][2]);

  virtual bool is_ready_to_start();
  virtual void move_axis(int axisNo);
};

#endif  // WHEELCHAIRSTATUS_H
