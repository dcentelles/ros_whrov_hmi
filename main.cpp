#include "whrovmainwindow.h"
#include "imageview.h"
#include <QApplication>
#include <qrosnode.h>

int main(int argc, char *argv[])
{
    QApplication a(argc, argv);

    //ROS interface
    QROSNode rosNode(argc, argv);

    //UI
    WhrovMainWindow w(argc, argv); //For HROV control and position feedback
    ImageView iw; //For image view and ROI selection

    QObject::connect(&iw, SIGNAL(newROI(int,int,int,int)),
                     &w,  SLOT(updateROI(int,int,int,int)));

    QObject::connect(&w, SIGNAL(newROI(int,int,int,int,int)),
                     &iw,  SLOT(updateROI(int,int,int,int, int)));

    QObject::connect(&w, SIGNAL(sendOrder(
                                    bool, int, float,float,float
                                    )),
                     &rosNode, SLOT(sendOrder(
                                    bool, int, float, float, float
                                        )));
    QObject::connect(&w, SIGNAL(newProtocolSettings(
                                    int,int,int,int,
                                    int,
                                    int,
                                    int)),
                     &rosNode, SLOT(updateProtocolSettings(
                                        int,int,int,int,
                                        int,
                                        int,
                                        int
                                        )));

    QObject::connect(&rosNode, SIGNAL(newPosition(int, float, float, float)),
                     &w, SLOT(updatePosition(int, float, float, float)));

    QObject::connect(&rosNode, SIGNAL(newImage(const QImage &)),
                     &iw, SLOT(updateImage(QImage)));

    QObject::connect(&rosNode, SIGNAL(orderPercentCompleteUpdated(int)),
                     &w, SLOT(updatePercentComplete(int)));



    rosNode.init();
    w.show();
    iw.show();


    return a.exec();
}
