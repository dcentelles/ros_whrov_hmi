#include <QDebug>
#include <QTime>
#include <qwt_dial_needle.h>
#include <ui_whrovmainwindow.h>
#include <whrov_hmi/attitude_indicator.h>
#include <whrov_hmi/constants.h>
#include <whrov_hmi/whrovmainwindow.h>

#include <qtimer.h>

using namespace whrov_hmi;

WhrovMainWindow::WhrovMainWindow(int argc, char **argv, QWidget *parent)
    : QMainWindow(parent), ui(new Ui::WhrovMainWindow) {
  ui->setupUi(this);
  ui->notifications_plainTextEdit->setLineWrapMode(QPlainTextEdit::WidgetWidth);
  ui->order_progressBar->setEnabled(false);

  // CONFIGURE COMPASS WIDGET
  QPalette palette0;
  int c;
  for (c = 0; c < QPalette::NColorRoles; c++) {
    const QPalette::ColorRole colorRole = static_cast<QPalette::ColorRole>(c);

    palette0.setColor(colorRole, QColor());
  }
  palette0.setColor(QPalette::Base,
                    palette().color(backgroundRole()).light(120));
  palette0.setColor(QPalette::WindowText, palette0.color(QPalette::Base));

  ui->Compass->setLineWidth(4);
  ui->Compass->setFrameShadow(QwtCompass::Sunken);
  palette0.setColor(QPalette::Base, Qt::darkBlue);
  palette0.setColor(QPalette::WindowText, QColor(Qt::darkBlue).dark(120));
  palette0.setColor(QPalette::Text, Qt::white);

  QwtCompassScaleDraw *scaleDraw = new QwtCompassScaleDraw();
  scaleDraw->enableComponent(QwtAbstractScaleDraw::Ticks, true);
  scaleDraw->enableComponent(QwtAbstractScaleDraw::Labels, true);
  scaleDraw->enableComponent(QwtAbstractScaleDraw::Backbone, false);
  scaleDraw->setTickLength(QwtScaleDiv::MinorTick, 1);
  scaleDraw->setTickLength(QwtScaleDiv::MediumTick, 1);
  scaleDraw->setTickLength(QwtScaleDiv::MajorTick, 3);

  ui->Compass->setScaleDraw(scaleDraw);

  ui->Compass->setScaleMaxMajor(36);
  ui->Compass->setScaleMaxMinor(5);

  ui->Compass->setNeedle(
      new QwtCompassMagnetNeedle(QwtCompassMagnetNeedle::ThinStyle));
  ui->Compass->setValue(220.0);

  auto compass = ui->Compass;
  QPalette newPalette = compass->palette();
  for (c = 0; c < QPalette::NColorRoles; c++) {
    const QPalette::ColorRole colorRole = static_cast<QPalette::ColorRole>(c);

    if (palette0.color(colorRole).isValid())
      newPalette.setColor(colorRole, palette0.color(colorRole));
  }

  for (int i = 0; i < QPalette::NColorGroups; i++) {
    const QPalette::ColorGroup colorGroup =
        static_cast<QPalette::ColorGroup>(i);

    const QColor light =
        newPalette.color(colorGroup, QPalette::Base).light(170);
    const QColor dark = newPalette.color(colorGroup, QPalette::Base).dark(170);
    const QColor mid =
        compass->frameShadow() == QwtDial::Raised
            ? newPalette.color(colorGroup, QPalette::Base).dark(110)
            : newPalette.color(colorGroup, QPalette::Base).light(110);

    newPalette.setColor(colorGroup, QPalette::Dark, dark);
    newPalette.setColor(colorGroup, QPalette::Mid, mid);
    newPalette.setColor(colorGroup, QPalette::Light, light);
  }

  compass->setPalette(newPalette);

  // CREATE AND CONFIGURE ATTITUDE INDICATOR

  QwtDial *dial = NULL;
  d_ai = new AttitudeIndicator(this);
  d_ai->setPalette(customColorTheme(QColor(Qt::darkGray).dark(150)));
  d_ai->scaleDraw()->setPenWidth(3);

  dial = d_ai;
  dial->setReadOnly(true);
  dial->setLineWidth(4);
  dial->setFrameShadow(QwtDial::Sunken);
  d_ai->setSizePolicy(
      QSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding));

  ui->indicatorsHorizontalLayaout->addWidget(d_ai);
}

WhrovMainWindow::~WhrovMainWindow() { delete ui; }

QPalette WhrovMainWindow::customColorTheme(const QColor &base) const {
  QPalette palette;
  palette.setColor(QPalette::Base, base);
  palette.setColor(QPalette::Window, base.dark(150));
  palette.setColor(QPalette::Mid, base.dark(110));
  palette.setColor(QPalette::Light, base.light(170));
  palette.setColor(QPalette::Dark, base.dark(170));
  palette.setColor(QPalette::Text, base.dark(200).light(800));
  palette.setColor(QPalette::WindowText, base.dark(200));

  return palette;
}

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
      ui->roi_shift_spinBox->value(), ui->imageSize_spinBox->value(),
      ui->rgb_radioButton->isChecked());
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

void WhrovMainWindow::on_keepHeading_pushButton_clicked() {
  qDebug() << "Send order button clicked";
  int value = ui->orientation_spinBox->value();
  emit sendOrder(ORDER_TYPE::HEADING, value, 0);
}

void WhrovMainWindow::on_holdImageTime_pushButton_clicked() {
  qDebug() << "Send order button clicked";
  int value = ui->holdImageTime_spinBox->value();
  emit sendOrder(ORDER_TYPE::HOLD, 0, value);
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

void WhrovMainWindow::updateState(int orientation, float altitude, float roll,
                                  float pitch, bool keepingHeading) {
  ui->orientation_lcdNumber->display(orientation);
  ui->Compass->setValue(orientation);
  ui->altitude_lcdNumber->display(altitude);
  ui->roll_lcdNumber->display(roll);
  ui->pitch_lcdNumber->display(pitch);

  ui->stopKeepHeading_pushButton->setEnabled(keepingHeading);
  ui->keepHeading_pushButton->setEnabled(!keepingHeading);

  auto angle = roll > 0 ? roll : 360 + roll;
  d_ai->setAngle(angle);
  auto gradient = pitch > 0 ? pitch : 90 + pitch;
  gradient /= 90;
  if (gradient == 1)
    gradient = 0.005;
  d_ai->setGradient(-gradient);
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

void WhrovMainWindow::on_stopKeepHeading_pushButton_clicked() {
  qDebug() << "Stop keep heading clicked";
  int value = 361; // keep heading disabled if > 360
  emit sendOrder(ORDER_TYPE::HEADING, value, 0);
}
