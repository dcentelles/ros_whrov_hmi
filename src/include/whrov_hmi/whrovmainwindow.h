#ifndef WHROVMAINWINDOW_H
#define WHROVMAINWINDOW_H

#include <QMainWindow>
#include <whrov_hmi/constants.h>
#include <whrov_hmi/qrosnode.h>

class AttitudeIndicator;

namespace Ui {
class WhrovMainWindow;
}

class WhrovMainWindow : public QMainWindow {
  Q_OBJECT

public:
  WhrovMainWindow(int argc, char **argv, QWidget *parent = 0);
  ~WhrovMainWindow();

protected:
  void closeEvent(QCloseEvent *);

private:
  void notifyNewROI();
  QPalette customColorTheme(const QColor &base) const;
  void enableGoToControls(bool v);

signals:
  void newROI(int x0, int y0, int x1, int y1, int shift);
  void newProtocolSettings(int rx0, int ry0, int rx1, int ry1, int shift,
                           int imageSize, bool rgb);
  void sendOrder(ORDER_TYPE type, int orientation, int holdTime, double x, double y, double z);
  void cancelLastOrder();
  void updateDesiredPosition(double x, double y, double z, double yaw);

public slots:
  void updateROI(int x0, int y0, int x1, int y1);
  void updateState(int orientation, float depth, float roll, float pitch,
                   bool keepingHeading, int navmode, bool armed, double x, double y);
  void handleFeedback(int percent, const QString &msg);
  void orderCancelled();
  void orderActive();
  void desiredPositionUpdated(double x, double y, double z, double yaw);

private slots:
  void on_holdPositionStopButton_clicked();

private slots:
  void on_holdPositionStartButton_clicked();

private slots:
  void on_goToStopButton_clicked();

private slots:
  void on_goToStartButton_clicked();

private slots:
  void on_stopKeepHeading_pushButton_clicked();

private slots:
  void on_roi_x0_SpinBox_valueChanged(int arg1);
  void on_roi_y0_SpinBox_valueChanged(int arg1);
  void on_roi_x1_SpinBox_valueChanged(int arg1);
  void on_roi_y1_SpinBox_valueChanged(int arg1);

  void on_roi_shift_spinBox_valueChanged(int arg1);

  void on_imageSize_spinBox_valueChanged(int arg1);

  void on_updateSettings_toolButton_clicked();

  void on_globalOrder_radioButton_clicked();

  void on_orientation_spinBox_valueChanged(int arg1);

  void on_z_doubleSpinBox_valueChanged(double arg1);

  void on_x_doubleSpinBox_valueChanged(double arg1);

  void on_y_doubleSpinBox_valueChanged(double arg1);

  void on_keepHeading_pushButton_clicked();
  void on_holdImageTime_pushButton_clicked();
  void on_cancelOrder_pushButton_clicked();

private:
  Ui::WhrovMainWindow *ui;
  AttitudeIndicator *d_ai;

  void printNotif(const QString &notif);
};

#endif // WHROVMAINWINDOW_H
