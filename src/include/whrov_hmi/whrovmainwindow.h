#ifndef WHROVMAINWINDOW_H
#define WHROVMAINWINDOW_H

#include <QMainWindow>
#include <whrov_hmi/constants.h>
#include <whrov_hmi/qrosnode.h>

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
signals:
  void newROI(int x0, int y0, int x1, int y1, int shift);
  void newProtocolSettings(int rx0, int ry0, int rx1, int ry1, int shift,
                           int imageSize, bool rgb);
  void sendOrder(ORDER_TYPE type, int orientation, int holdTime);
  void cancelLastOrder();

public slots:
  void updateROI(int x0, int y0, int x1, int y1);
  void updatePosition(int orientation, float z, float x, float y);
  void handleFeedback(int percent, const QString &msg);
  void orderCancelled();
  void orderActive();

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

  void printNotif(const QString &notif);
};

#endif // WHROVMAINWINDOW_H
