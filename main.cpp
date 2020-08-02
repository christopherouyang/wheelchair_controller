#include "mainwindow.h"
#include <QApplication>
#include <QTextCodec>

int main(int argc, char *argv[])
{

    QTextCodec *codec = QTextCodec::codecForName("UTF-8");
#ifdef __linux__
    //QTextCodec::setCodecForTr(codec);
#endif
    QTextCodec::setCodecForLocale(codec);
#ifdef __linux__
    //QTextCodec::setCodecForCStrings(codec);
#endif

    QApplication a(argc, argv);
    MainWindow w;
    w.show();


    return a.exec();
}
