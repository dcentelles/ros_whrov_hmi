#include <QDebug>
#include <QTime>
#include <ui_whrovmainwindow.h>
#include <whrovmainwindow.h>

WhrovMainWindow::WhrovMainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::WhrovMainWindow) {
  ui->setupUi(this);
  ui->notifications_plainTextEdit->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  ui->order_progressBar->setEnabled(false);
}

WhrovMainWindow::~WhrovMainWindow() { delete ui; }

void WhrovMainWindow::printNotif(const QString &notif) {
  QTime time = QTime::currentTime();
  QString msg = QString("<font color=\"%1\">%2:&nbsp;</font>"
                        "<font color=\"%3\">%3 </font>")
                    .arg("blue", time.toString("HH:mm:ss"), notif, "Lime");
  ui->notifications_plainTextEdit->appendHtml(msg);
}

void WhrovMainWindow::on_roi_x0_SpinBox_valueChanged(int arg1) {
  qDebug() << "ROI X0: " << arg1;
  notifyNewROI();
}
void WhrovMainWindow::on_roi_y0_SpinBox_valueChanged(int arg1) {
  qDebug() << "ROI Y0: " << arg1;
  notifyNewROI();
}
void WhrovMainWindow::on_roi_x1_SpinBox_valueChanged(int arg1) {
  qDebug() << "ROI X1: " << arg1;
  notifyNewROI();
}
void WhrovMainWindow::on_roi_y1_SpinBox_valueChanged(int arg1) {
  qDebug() << "ROI y1: " << arg1;
  notifyNewROI();
}

void WhrovMainWindow::on_roi_shift_spinBox_valueChanged(int arg1) {
  qDebug() << "ROI bit-shift: " << arg1;
  notifyNewROI();
}

void WhrovMainWindow::on_imageSize_spinBox_valueChanged(int arg1) {
  qDebug() << "Image size: " << arg1;
}

void WhrovMainWindow::on_updateSettings_toolButton_clicked() {
  qDebug() << "Update settings button clicked";
  emit newProtocolSettings(
      ui->roi_x0_SpinBox->value(), ui->roi_y0_SpinBox->value(),
      ui->roi_x1_SpinBox->value(), ui->roi_y1_SpinBox->value(),
      ui->roi_shift_spinBox->value(), ui->imageSize_spinBox->value());
}

void WhrovMainWindow::on_globalOrder_radioButton_clicked() {
  qDebug() << "absolute order";
}

void WhrovMainWindow::on_orientation_spinBox_valueChanged(int arg1) {
  qDebug() << "Desired orientation: " << arg1;
}

void WhrovMainWindow::on_z_doubleSpinBox_valueChanged(double arg1) {
  qDebug() << "Z: " << arg1;
}

void WhrovMainWindow::on_x_doubleSpinBox_valueChanged(double arg1) {
  qDebug() << "X: " << arg1;
}

void WhrovMainWindow::on_y_doubleSpinBox_valueChanged(double arg1) {
  qDebug() << "Y: " << arg1;
}

void WhrovMainWindow::on_sendOrder_pushButton_clicked() {
  qDebug() << "Send order button clicked";
  int orientation = ui->orientation_spinBox->value();
}

void WhrovMainWindow::orderActive() { ui->order_progressBar->setEnabled(true); }

void WhrovMainWindow::on_cancelOrder_pushButton_clicked() {
  qDebug() << "Cancel order button clicked";
  emit cancelLastOrder();
}

void WhrovMainWindow::orderCancelled() {
  ui->order_progressBar->setValue(0);
  ui->order_progressBar->setEnabled(false);
  printNotif("Last order cancelled");
}

void WhrovMainWindow::updateROI(int x0, int y0, int x1, int y1) {
  ui->roi_x0_SpinBox->setValue(x0);
  ui->roi_y0_SpinBox->setValue(y0);
  ui->roi_x1_SpinBox->setValue(x1);
  ui->roi_y1_SpinBox->setValue(y1);
}

void WhrovMainWindow::updatePosition(int orientation, float altitude,
                                     float roll, float pitch) {
  ui->orientation_lcdNumber->display(orientation);
  ui->altitude_lcdNumber->display(altitude);
  ui->roll_lcdNumber->display(roll);
  ui->pitch_lcdNumber->display(pitch);
}

void WhrovMainWindow::notifyNewROI() {
  emit newROI(ui->roi_x0_SpinBox->value(), ui->roi_y0_SpinBox->value(),
              ui->roi_x1_SpinBox->value(), ui->roi_y1_SpinBox->value(),
              ui->roi_shift_spinBox->value());
}

void WhrovMainWindow::handleFeedback(int percent, const QString &msg) {
  ui->order_progressBar->setValue(percent);
  printNotif(msg);
}

// http://stackoverflow.com/questions/21245121/qt-ui-closing-order
void WhrovMainWindow::closeEvent(QCloseEvent *e) {
  foreach (QWidget *widget, QApplication::topLevelWidgets()) {
    if (widget != this) { // avoid recursion.
      widget->close();
    }
  }
  QMainWindow::closeEvent(e);
}
