#include "imageview.h"
#include "ui_imageview.h"
#include <QDebug>
#include <QMouseEvent>


ImageView::ImageView(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::ImageView),
    width(352), height(288),
    pixmap(352, 288)
    //painter(&pixmap)

{
    ui->setupUi(this);

    ui->image_label->setPixmap(pixmap);
    //ui->image_label->setFixedHeight(height);
   // ui->image_label->setFixedWidth(width);
    //setMinimumHeight(height);
    // setMinimumWidth(width);

    setWindowFlags(Qt::CustomizeWindowHint | Qt::WindowTitleHint | Qt::WindowMinMaxButtonsHint );
    ui->image_label->installEventFilter(this);
    roiStarted = false;


}

bool ImageView::eventFilter(QObject *obj, QEvent *event)
{
    if(obj == ui->image_label)
    {
        QEvent::Type etype = event->type();
        QPoint position;
        QString sposition;
        QMouseEvent * _event;
        if(etype == QEvent::MouseMove)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();
            sposition = QString("(x: %0 ; y: %1 )").arg(
                        QString::number(position.x()),
                        QString::number(position.y()));

           // qDebug() << sposition << ": Mouse MOVE event";

            notifyPoint1(position.x(), position.y());
        }
        else if (etype == QEvent::MouseButtonPress)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();

            sposition = QString("(x: %0 ; y: %1 )").arg(
                        QString::number(position.x()),
                        QString::number(position.y()));
            //qDebug() << sposition << ": Mouse button PRESSED";
            startROI(position.x(), position.y());
        }
        else if (etype == QEvent::MouseButtonRelease)
        {
            _event = static_cast<QMouseEvent*>(event);
            position = _event->pos();

            sposition = QString("(x: %0 ; y: %1 )").arg(
                        QString::number(position.x()),
                        QString::number(position.y()));
            //qDebug() << sposition << ": Mouse button RELEASED";
            endROI(position.x(), position.y());
        }
        else if(etype == QEvent::HoverMove)
        {
            qDebug() << sposition << "Mouse hover move event";
        }
        else if(etype == QEvent::MouseTrackingChange)
        {
            qDebug() << sposition << "Mouse tracking change";
        }
    }

    return QMainWindow::eventFilter(obj, event);
}

ImageView::~ImageView()
{
    delete ui;
}
bool ImageView::validPoint0(int x, int y)
{
    return pointIn(x, y);
}

bool ImageView::validPoint1(int x, int y)
{
    return pointIn(x,y) &&
            x0 < x &&  y0 < y;
}

bool ImageView::pointIn(int x, int y)
{
    return x >= 0 && y >= 0
            && x < width
            && y < height;
}

void ImageView::startROI(int _x0, int _y0)
{
    if(validPoint0(_x0, _y0))
    {
        x0 = _x0;
        y0 = _y0;
        roiStarted = true;
        QString sposition = QString("(x: %0 ; y: %1 )").arg(
                    QString::number(x0),
                    QString::number(y0));
        qDebug() << sposition << ": START ROI";
    }
}

void ImageView::endROI(int _x1, int _y1)
{
    if(validPoint1(_x1, _y1))
    {
        updatePoint1(_x1, _y1);
        roiStarted = false;
        notifyROI();
        QString sposition = QString("(x: %0 ; y: %1 )").arg(
                    QString::number(x1),
                    QString::number(y1));
        qDebug() << sposition << ": END ROI";

    }
}

void ImageView::notifyPoint1(int x, int y)
{
    if(validPoint1(x, y))
    {
        updatePoint1(x, y);
        QString sposition = QString("(x: %0 ; y: %1 )").arg(
                    QString::number(x1),
                    QString::number(x1));
        qDebug() << sposition << ": updating ROI";
    }
}

void ImageView::updatePoint1(int x, int y)
{
    x1 = x;
    y1 = y;
    drawCurrentROI();
}

void ImageView::drawCurrentROI()
{
    //TODO: draw rectangle on pixmap
    qDebug() << "Drawing rectangle";
    QPainter painter(this);

    painter.setPen(Qt::blue);
    painter.setFont(QFont("Arial", 30));
    painter.drawText(rect(), Qt::AlignCenter, "Qt");

   //QRect rect(x0 , y0 , x1 , y1 );

  //  painter.drawRect(rect);
}

void ImageView::notifyROI()
{
    //TODO: emit a signal with the current ROI
}
