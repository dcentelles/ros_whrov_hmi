#include <QApplication>
#include <imageview.h>
#include <qrosnode.h>
#include <whrovmainwindow.h>

int main(int argc, char *argv[]) {
  QApplication a(argc, argv);

  // ROS interface
  QROSNode rosNode(argc, argv);

  // UI
  WhrovMainWindow w(argc, argv); // For HROV control and position feedback
  ImageView iw;                  // For image view and ROI selection

  QObject::connect(&iw, SIGNAL(newROI(int, int, int, int)), &w,
                   SLOT(updateROI(int, int, int, int)));

  QObject::connect(&w, SIGNAL(newROI(int, int, int, int, int)), &iw,
                   SLOT(updateROI(int, int, int, int, int)));

  QObject::connect(&w, SIGNAL(sendOrder(int)), &rosNode, SLOT(sendOrder(int)));

  QObject::connect(
      &w, SIGNAL(newProtocolSettings(int, int, int, int, int, int, bool)),
      &rosNode,
      SLOT(updateProtocolSettings(int, int, int, int, int, int, bool)));

  QObject::connect(&w, SIGNAL(cancelLastOrder()), &rosNode,
                   SLOT(cancelLastOrder()));

  QObject::connect(&rosNode, SIGNAL(orderCancelled()), &w,
                   SLOT(orderCancelled()));

  QObject::connect(&rosNode, SIGNAL(orderActive()), &w, SLOT(orderActive()));

  QObject::connect(&rosNode, SIGNAL(newPosition(int, float, float, float)), &w,
                   SLOT(updatePosition(int, float, float, float)));

  QObject::connect(&rosNode, SIGNAL(newImage(const QImage &)), &iw,
                   SLOT(updateImage(QImage)));

  QObject::connect(&rosNode, SIGNAL(orderFeedback(int, const QString &)), &w,
                   SLOT(handleFeedback(int, const QString &)));

  rosNode.init();
  w.show();
  iw.show();

  return a.exec();
}
