#ifndef WINDOWINFORMATION_H
#define WINDOWINFORMATION_H

#include "mainwindow.h"

class WindowInformation : public MainWindow {
  Q_OBJECT
 public:
  WindowInformation();

 private:
  virtual void information_connection_fail();
  virtual void information_connection_success();

  virtual void information_disable();
  virtual void information_disable_axis(int axisNo);

  virtual void information_emgstop_on();
  virtual void information_connection_interrupted();
};

#endif  // WINDOWINFORMATION_H
