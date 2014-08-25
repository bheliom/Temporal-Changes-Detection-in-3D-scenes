#include <QtGui/QApplication>
#include "chngdetect.h"

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    chngDetect w;
    w.show();
    
    return a.exec();
}
