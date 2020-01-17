#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "mythread.h"

//extern short connection;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = nullptr);
    ~MainWindow();

private slots:

    void on_pushButton_openio_clicked();

    void on_pushButton_closeio_clicked();

    void on_pushButton_start_clicked();

    void on_pushButton_decstop_clicked();
    void on_pushButton_decstop_2_clicked();

    void on_pushButton_emgstop_clicked();
    void on_pushButton_emgstop_2_clicked();

    void on_pushButton_zeropos_clicked();

    void on_pushButton_encpos_clicked();

    void on_pushButton_stopcrd_clicked();

    bool on_pushButton_disable_clicked();

    void on_pushButton_line_clicked();

    void on_pushButton_changevel_clicked();

    void on_pushButton_changepos_clicked();

    void on_pushButton_enable_clicked();

    void on_pushButton_exit_clicked();
    void on_pushButton_exit_2_clicked();

    void on_pushButton_start_wc_clicked();

    void on_pushButton_changevel_wc_clicked();

    void on_pushButton_changepos_wc_clicked();

    void on_slider_linear_vel_mouseReleased();
private:

    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *e);
    void initDialog();
    void information_connection_fail();
    void information_connection_success();
    void information_disable();
    void information_disable_0();
    void information_disable_1();
    void information_emgstop_on();
    void information_connection_interrupted();

    void emg_stop();

public:
    void RefreshUI();

};

#endif // MAINWINDOW_H
