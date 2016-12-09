#ifndef WHROVMAINWINDOW_H
#define WHROVMAINWINDOW_H

#include <QMainWindow>
#include <qrosnode.h>

namespace Ui {
class WhrovMainWindow;
}

class WhrovMainWindow : public QMainWindow{
Q_OBJECT

public:
    WhrovMainWindow(int argc, char** argv, QWidget *parent = 0);
    ~WhrovMainWindow();

private slots:
    void on_roi_x0_SpinBox_valueChanged(int arg1);
    void on_roi_y0_SpinBox_valueChanged(int arg1);
    void on_roi_x1_SpinBox_valueChanged(int arg1);
    void on_roi_y1_SpinBox_valueChanged(int arg1);

    void on_roi_shift_spinBox_valueChanged(int arg1);

    void on_packetLength_spinBox_valueChanged(int arg1);

    void on_imageSize_spinBox_valueChanged(int arg1);

    void on_updateSettings_toolButton_clicked();

    void on_relativeOrder_radioButton_clicked();

    void on_globalOrder_radioButton_clicked();

    void on_orientation_spinBox_valueChanged(int arg1);

    void on_z_doubleSpinBox_valueChanged(double arg1);

    void on_x_doubleSpinBox_valueChanged(double arg1);

    void on_y_doubleSpinBox_valueChanged(double arg1);

    void on_sendOrder_pushButton_clicked();

    void on_cancelOrder_pushButton_clicked();

private:
    Ui::WhrovMainWindow * ui;
    QROSNode qrosnode;
};

#endif // WHROVMAINWINDOW_H
