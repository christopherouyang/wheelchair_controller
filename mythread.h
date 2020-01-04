#ifndef MYTHREAD_H
#define MYTHREAD_H

#include <QThread>


class MyThread : public QThread
{
    Q_OBJECT
public:
    explicit MyThread(QObject *parent = 0);

public:
    void run();
    void stop();
private:
    volatile bool stopped;

};

#endif // MYTHREAD_H
