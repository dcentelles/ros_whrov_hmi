#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <QMainWindow>
#include <QPainter>

namespace Ui {
class ImageView;
}

class ImageView : public QMainWindow
{
    Q_OBJECT

public:
    explicit ImageView(QWidget *parent = 0);
    ~ImageView();

protected:
    bool eventFilter(QObject *, QEvent *);

private:
    Ui::ImageView *ui;
    QPixmap pixmap;
    int width, height;

    int x0,y0,x1,y1;
    bool roiStarted;

    void startROI(int x0, int y0);
    void endROI(int x1, int y1);
    void drawCurrentROI();
    void notifyROI();
    void updatePoint1(int x, int y);
    bool validPoint0(int x, int y);
    bool validPoint1(int x, int y);
    void notifyPoint1(int x, int y);
    bool pointIn(int x, int y);

    //QPainter painter;
};

#endif // IMAGEVIEW_H
