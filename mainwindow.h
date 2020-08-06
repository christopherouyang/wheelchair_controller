#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "mythread.h"
#include "enumeration.h"
#include <eigen3/Eigen/Dense>

// extern short connection;

namespace Ui {
class MainWindow;
}

struct MovingStatus {
  double startVel;
  double runVel;
  double stopVel;
  double accTime;
  double decTime;
  double sTime;
  double pulse;
  int direction;
  MovingMode mode;
};

class MainWindow : public QMainWindow {
  Q_OBJECT

 public:
  explicit MainWindow(QWidget *parent = nullptr);
  ~MainWindow();

 private slots:

  void on_pushButton_openio_clicked();
  void on_pushButton_closeio_clicked();

  void on_pushButton_start_0_clicked();

  void on_pushButton_decstop_0_clicked();
  void on_pushButton_decstop_1_clicked();

  void on_pushButton_emgstop_0_clicked();
  void on_pushButton_emgstop_1_clicked();

  void on_pushButton_zeropos_clicked();
  void on_pushButton_encpos_clicked();

  void on_pushButton_stopcrd_clicked();

  void on_pushButton_enable_clicked();
  bool on_pushButton_disable_clicked();

  void on_pushButton_changevel_clicked();
  void on_pushButton_changepos_clicked();

  void on_pushButton_exit_0_clicked();
  void on_pushButton_exit_1_clicked();

  void on_pushButton_start_wc_clicked();

  void on_pushButton_changevel_wc_clicked();
  void on_pushButton_changepos_wc_clicked();

  //    void on_radioButton_fl_1_clicked();
  //    void on_radioButton_fl_0_clicked();

  //    void on_radioButton_cs_1_clicked();
  //    void on_radioButton_cs_0_clicked();

  // void on_slider_linear_vel_mouseReleased();
 private:
  Ui::MainWindow *ui;
  void timerEvent(QTimerEvent *e);
  void initDialog();

  void information_connection_fail();
  void information_connection_success();
  void information_disable();
  void information_disable_axis(int axisNo);
  void information_emgstop_on();
  void information_connection_interrupted();

  bool enable_axis(int axisNo);
  bool disable_axis(int axisNo);

  void get_moving_status(Wheel axisNo, double *runvel, double *pulse);
  void move_axis(Wheel axisNo);

  void calculate_motor_vel(Eigen::Vector2d &motorVel);
  void get_fixed_length_parameter(double runvel[][2], double pulse[][2]);

  void emg_stop();

  MovingStatus status[2];

 public:
  void RefreshUI();
};

#endif  // MAINWINDOW_H
