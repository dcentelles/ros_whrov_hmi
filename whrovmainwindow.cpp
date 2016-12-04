#include "whrovmainwindow.h"
#include "ui_whrovmainwindow.h"

namespace merbots_whrov_hmi {


WhrovMainWindow::WhrovMainWindow(int argc, char** argv, QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::WhrovMainWindow),
    qrosnode(argc, argv)
{
    ui->setupUi(this);
}

WhrovMainWindow::~WhrovMainWindow()
{
    delete ui;
}

}
