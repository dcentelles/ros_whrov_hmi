#ifndef IMAGEVIEW_H
#define IMAGEVIEW_H

#include <QMainWindow>
#include <QPainter>
#include <QImage>
//#include <QLabel>
namespace Ui {
class ImageView;
}

class ImageView : public QMainWindow
{
    Q_OBJECT

public:
    explicit ImageView(QWidget *parent = 0);
    ~ImageView();

signals:
    void newROI(int x0, int y0, int x1, int y1);

public slots:
    /*
    Note about signals and slots with parameters as reference:
    Although "const ... &", Qt will (by default) call the copy constructor to
    generate two different objetcs when calling the slot.
    https://www.google.es/search?q=altogh&oq=altogh&aqs=chrome..69i57.1868j0j7&client=ubuntu&sourceid=chrome&ie=UTF-8
    */
    void updateImage(const QImage &);
    void updateROI(int x0, int y0, int x1, int y1, int shift);
protected:
    bool eventFilter(QObject *, QEvent *);
    void updateROI(int x0, int y0, int x1, int y1);

private:
    Ui::ImageView *ui;
    QPixmap pixmap, imagePixmap;
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

    QPainter painter;
    QImage image;
  //  QLabel auxLabel;
};

#endif // IMAGEVIEW_H
