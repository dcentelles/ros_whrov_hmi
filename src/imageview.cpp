#include <QDebug>
#include <QMouseEvent>
#include <ui_imageview.h>
#include <whrov_hmi/constants.h>
#include <whrov_hmi/imageview.h>

ImageView::ImageView(QWidget *parent)
    : QMainWindow(parent), ui(new Ui::ImageView), width(352), height(288),
      pixmap(352, 288), imagePixmap(352, 288)
// painter(&pixmap)
{
  ui->setupUi(this);
  ui->image_label->setPixmap(pixmap);
  // ui->image_label->setFixedHeight(height);
  // ui->image_label->setFixedWidth(width);
  // setMinimumHeight(height);
  // setMinimumWidth(width);

  setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint |
                 Qt::WindowMinMaxButtonsHint);
  ui->image_label->installEventFilter(this);
  roiStarted = false;

  x0 = 0;
  y0 = 0;
  x1 = 1;
  y1 = 1;
  // auxLabel.show();
  // painter.end();
  // imagePixmap.load("/home/centelld/programming/catkin_ws/src/qt_ws/whrov_hmi/Girona-500_1_brit.jpg");
}
void ImageView::mouseReleaseEvent(QMouseEvent *_event) {
  endROI(x1, y1);
  qDebug() << "mouse release";
  QMainWindow::mouseReleaseEvent(_event);
}

bool ImageView::eventFilter(QObject *obj, QEvent *event) {
  if (obj == ui->image_label) {
    QEvent::Type etype = event->type();
    QPoint position;
    QString sposition;
    QMouseEvent *_event;
    if (etype == QEvent::MouseMove) {
      _event = static_cast<QMouseEvent *>(event);
      position = _event->pos();
      sposition = QString("(x: %0 ; y: %1 )")
                      .arg(QString::number(position.x()),
                           QString::number(position.y()));

      // qDebug() << sposition << ": Mouse MOVE event";

      notifyPoint1(position.x(), position.y());
    } else if (etype == QEvent::MouseButtonPress) {
      _event = static_cast<QMouseEvent *>(event);
      position = _event->pos();

      sposition = QString("(x: %0 ; y: %1 )")
                      .arg(QString::number(position.x()),
                           QString::number(position.y()));
      // qDebug() << sposition << ": Mouse button PRESSED";
      startROI(position.x(), position.y());
    } else if (etype == QEvent::MouseButtonRelease) {
      _event = static_cast<QMouseEvent *>(event);
      position = _event->pos();

      sposition = QString("(x: %0 ; y: %1 )")
                      .arg(QString::number(position.x()),
                           QString::number(position.y()));
      // qDebug() << sposition << ": Mouse button RELEASED";
      endROI(position.x(), position.y());
    } else if (etype == QEvent::HoverMove) {
      qDebug() << sposition << "Mouse hover move event";
    } else if (etype == QEvent::MouseTrackingChange) {
      qDebug() << sposition << "Mouse tracking change";
    }
  }

  return QMainWindow::eventFilter(obj, event);
}

ImageView::~ImageView() { delete ui; }
bool ImageView::validPoint0(int x, int y) { return pointIn(x, y); }

bool ImageView::validPoint1(int x, int y) {
  return pointIn(x, y) && x0 < x && y0 < y;
}

bool ImageView::pointIn(int x, int y) {
  return x >= 0 && y >= 0 && x < width && y < height;
}

void ImageView::startROI(int _x0, int _y0) {
  if (validPoint0(_x0, _y0)) {
    x0 = _x0;
    y0 = _y0;
    roiStarted = true;
    QString sposition = QString("(x: %0 ; y: %1 )")
                            .arg(QString::number(x0), QString::number(y0));
    qDebug() << sposition << ": START ROI";
  }
}

void ImageView::endROI(int _x1, int _y1) {
  if (validPoint1(_x1, _y1)) {
    updatePoint1(_x1, _y1);
    roiStarted = false;
    notifyROI();
    QString sposition = QString("(x: %0 ; y: %1 )")
                            .arg(QString::number(x1), QString::number(y1));
    qDebug() << sposition << ": END ROI";
  }
}

void ImageView::notifyPoint1(int x, int y) {
  if (validPoint1(x, y)) {
    updatePoint1(x, y);
    QString sposition = QString("(x: %0 ; y: %1 )")
                            .arg(QString::number(x1), QString::number(y1));
    qDebug() << sposition << ": updating ROI";
  }
}

void ImageView::updatePoint1(int x, int y) {
  x1 = x;
  y1 = y;
  drawCurrentROI();
}

void ImageView::drawCurrentROI() {
  // TODO: draw rectangle on pixmap
  QString sposition = QString("(x0: %0 ; y0: %1 ; x1: %2 ; y1: %3)")
                          .arg(QString::number(x0), QString::number(y0),
                               QString::number(x1), QString::number(y1));
  qDebug() << sposition << ": Drawing rectangle";

  pixmap = imagePixmap.copy(0, 0, imagePixmap.width(), imagePixmap.height());
  painter.begin(&pixmap);

  painter.setBrush(Qt::NoBrush);
  QPen pen(Qt::red, 3, Qt::DashDotLine, Qt::RoundCap, Qt::RoundJoin);
  painter.setPen(pen);

  painter.drawRect(x0, y0, x1 - x0, y1 - y0);
  ui->image_label->setPixmap(pixmap);

  painter.end();
}

void ImageView::updateImage(const QImage &_image) {
  image = _image;
  imagePixmap = QPixmap::fromImage(image);
  drawCurrentROI();
}

void ImageView::notifyROI() { emit newROI(x0, y0, x1, y1); }

void ImageView::updateROI(int _x0, int _y0, int _x1, int _y1, int shift) {
  x0 = _x0;
  y0 = _y0;
  x1 = _x1;
  y1 = _y1;

  drawCurrentROI();
}
