#include "whrovmainwindow.h"
#include "ui_whrovmainwindow.h"
#include <QDebug>
//#include <QPainter>

WhrovMainWindow::WhrovMainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WhrovMainWindow),
    qrosnode(argc, argv)
{
    ui->setupUi(this);

/*
    QPixmap input("/home/centelld/programming/catkin_ws/src/qt_ws/whrov_hmi/Girona-500_1_brit.jpg");
    input = input.scaled(this->size(), Qt::IgnoreAspectRatio);
    QImage image(this->size(), QImage::Format_ARGB32_Premultiplied);

    QPainter p(&image);
    p.drawPixmap(0, 0, input);
    p.end();

    QPixmap bkgnd = QPixmap::fromImage(image);
    QPalette palette;
    palette.setBrush(QPalette::Background, bkgnd);
    this->setPalette(palette);

    this->setFixedSize(this->size());
    */

}

WhrovMainWindow::~WhrovMainWindow()
{
    delete ui;
}

void WhrovMainWindow::on_roi_x0_SpinBox_valueChanged(int arg1)
{
     qDebug() << "ROI X0: " << arg1;
}
void WhrovMainWindow::on_roi_y0_SpinBox_valueChanged(int arg1)
{
     qDebug() << "ROI Y0: " << arg1;
}
void WhrovMainWindow::on_roi_x1_SpinBox_valueChanged(int arg1)
{
     qDebug() << "ROI X1: " << arg1;
}
void WhrovMainWindow::on_roi_y1_SpinBox_valueChanged(int arg1)
{
     qDebug() << "ROI y1: " << arg1;
}

void WhrovMainWindow::on_roi_shift_spinBox_valueChanged(int arg1)
{
    qDebug() << "ROI bit-shift: " << arg1;
}

void WhrovMainWindow::on_packetLength_spinBox_valueChanged(int arg1)
{
    qDebug() << "Packet length: " << arg1;
}

void WhrovMainWindow::on_imageSize_spinBox_valueChanged(int arg1)
{
    qDebug() << "Image size: " << arg1;
}

void WhrovMainWindow::on_updateSettings_toolButton_clicked()
{
    qDebug() << "Update settings button clicked";
}

void WhrovMainWindow::on_relativeOrder_radioButton_clicked()
{
    qDebug() << "relative order";
}

void WhrovMainWindow::on_globalOrder_radioButton_clicked()
{
    qDebug() << "absolute order";
}

void WhrovMainWindow::on_orientation_spinBox_valueChanged(int arg1)
{
    qDebug() << "Desired orientation: " << arg1;
}

void WhrovMainWindow::on_z_doubleSpinBox_valueChanged(double arg1)
{
    qDebug() << "Z: " << arg1;
}

void WhrovMainWindow::on_x_doubleSpinBox_valueChanged(double arg1)
{
    qDebug() << "X: " << arg1;
}

void WhrovMainWindow::on_y_doubleSpinBox_valueChanged(double arg1)
{
    qDebug() << "Y: " << arg1;
}

void WhrovMainWindow::on_sendOrder_pushButton_clicked()
{
    qDebug() << "Send order button clicked";
}

void WhrovMainWindow::on_cancelOrder_pushButton_clicked()
{
    qDebug() << "Cancel order button clicked";
}
