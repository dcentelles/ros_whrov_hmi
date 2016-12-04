#include "whrovmainwindow.h"
#include <QApplication>

using namespace merbots_whrov_hmi;

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    WhrovMainWindow w(argc, argv);
    w.show();

    return a.exec();
}
