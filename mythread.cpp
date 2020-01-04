#include "mythread.h"

MyThread::MyThread(QObject *parent) : QThread (parent)
{
    stopped = false;

}
void MyThread::run()
{
    while(!stopped)
    {
        msleep(200);

    }
}
void MyThread::stop()
{
    stopped = true;

}
