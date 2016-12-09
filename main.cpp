#include "whrovmainwindow.h"
#include "imageview.h"
#include <QApplication>


int main(int argc, char *argv[])
{
    QApplication a(argc, argv);
    WhrovMainWindow w(argc, argv);
    ImageView iw;
    w.show();
    iw.show();

    return a.exec();
}
