#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "LTSMC.h"
#include "mythread.h"
#include "enumeration.h"
#include <eigen3/Eigen/Dense>

// extern short connection;

namespace Ui {
class MainWindow;
}

struct MovingStatus {
  double startvel;
  double runvel;
  double stopvel;
  double acctime;
  double dectime;
  double stime;
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

  bool on_pushButton_disable_clicked();

  void on_pushButton_changevel_clicked();

  void on_pushButton_changepos_clicked();

  void on_pushButton_enable_clicked();

  void on_pushButton_exit_0_clicked();
  void on_pushButton_exit_1_clicked();

  void on_pushButton_start_wc_clicked();

  void on_pushButton_changevel_wc_clicked();
  void on_pushButton_changepos_wc_clicked();

 protected:
  Ui::MainWindow *ui;

  MovingStatus status[2];

  Eigen::MatrixXd trans;

  virtual void information_disable();
  virtual void information_disable_axis(int axisNo);

 private:
  void timerEvent(QTimerEvent *e);
  void initDialog();
  virtual void information_connection_fail();
  virtual void information_connection_success();

  virtual void information_emgstop_on();
  virtual void information_connection_interrupted();

  virtual bool enable_axis(int axisNo, int m_nConnectNo);
  virtual bool disable_axis(int axisNo, int m_nConnectNo);

  virtual void get_status(int axisNo, double *runvel, double *pulse);
  virtual void set_wheelchair_moving_parameter(bool isConstantSpeed, double pulse[3][2], double runvel[3][2]);

  virtual bool is_ready_to_start();
  virtual void move_axis(int axisNo);

  virtual void emg_stop();

 public:
  void RefreshUI();
};

#endif  // MAINWINDOW_H
