#ifndef WHROVMAINWINDOW_H
#define WHROVMAINWINDOW_H

#include <QMainWindow>
#include <qrosnode.h>

namespace Ui {
class WhrovMainWindow;
}

namespace merbots_whrov_hmi {


class WhrovMainWindow : public QMainWindow
{

public:
    WhrovMainWindow(int argc, char** argv, QWidget *parent = 0);
    ~WhrovMainWindow();

private:
    Ui::WhrovMainWindow * ui;
    QROSNode qrosnode;
};
}

#endif // WHROVMAINWINDOW_H
