#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QtCore>
#include "mythread.h"

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

    void on_pushButton_openio();

    void on_pushButton_closeio();

    void on_pushButton_start();

    void on_pushButton_decstop();

    void on_pushButton_emgstop();

    void on_pushButton_zeropos();

    void on_pushButton_encpos();

    void on_pushButton_stopcrd();

    void on_pushButton_disable();

    void on_pushButton_line();

    void on_pushButton_changevel();

    void on_pushButton_changepos();

    void on_pushButton_enable();

private:
    Ui::MainWindow *ui;
    void timerEvent(QTimerEvent *e);
    void initDialog();
    void information_fail();
    void information_success();
    void emg_stop();

public:
    void RefreshUI();
};

#endif // MAINWINDOW_H
