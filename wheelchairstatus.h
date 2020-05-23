#ifndef WHEELCHAIRSTATUS_H
#define WHEELCHAIRSTATUS_H

#include "mainwindow.h"

class WheelchairStatus {
 public:
  WheelchairStatus();
  bool enable_axis(MainWindow *window, int axisNo, int m_nConnectNo);
  bool disable_axis(MainWindow *window,int axisNo, int m_nConnectNo);

  void get_status(MainWindow *windowint,int axisNo, double *runvel, double *pulse);
  void set_wheelchair_moving_parameter(MainWindow *window,bool isConstantSpeed, double pulse[3][2], double runvel[3][2]);

  void set_wheelchair_constant_speed_parameter(MainWindow *window,double runvel[3][2]);
  void set_wheelchair_fixed_length_parameter(MainWindow *window,double pulse[3][2], double runvel[3][2]);

  bool is_ready_to_start(MainWindow *window);
  void move_axis(MainWindow *window,int axisNo);

};

#endif  // WHEELCHAIRSTATUS_H
