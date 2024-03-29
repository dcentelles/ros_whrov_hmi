#ifndef QROSNODE_H
#define QROSNODE_H
#include "std_msgs/String.h"
#include <QStringListModel>
#include <QThread>
#include <actionlib/client/simple_action_client.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <merbots_whrov_msgs/OrderAction.h>
#include <merbots_whrov_msgs/hrov_settings.h>
#include <merbots_whrov_msgs/state.h>
#include <opencv2/highgui/highgui.hpp>
#include <ros/network.h>
#include <ros/ros.h>
#include <whrov_hmi/constants.h>

using namespace whrov_hmi;

typedef actionlib::SimpleActionClient<merbots_whrov_msgs::OrderAction>
    OrderActionClient;

class QROSNode : public QThread {
  Q_OBJECT
public:
  QROSNode(int argc, char **argv);
  virtual ~QROSNode();
  bool init();
  bool init(const std::string &master_url, const std::string &host_url);
  void run();
  /*********************
  ** Logging
  **********************/
  enum LogLevel { Debug, Info, Warn, Error, Fatal };

  QStringListModel *loggingModel() { return &logging_model; }
  void log(const LogLevel &level, const std::string &msg);

public slots:
  void updateProtocolSettings(int roix0, int roiy0, int roix1, int roiy1,
                              int shift, int imageSize, bool rgb);

  void sendOrder(ORDER_TYPE type, int orientation, int holdTime, double x, double y, double z);
  void cancelLastOrder();
  void updateDesiredPosition(double x, double y, double z, double yaw);

signals:
  void newState(int orientation, float depth, float roll, float pitch, bool keepingHeading,
                int navmode, bool armed, double x, double y);
  void orderActive();
  void orderFeedback(int percent, const QString &msg);
  void orderCancelled();
  void desiredPositionUpdated(double x, double y, double z, double yaw);
  /*
  Note about signals and slots with parameters as reference:
  Although "const ... &", Qt will (by default) call the copy constructor to
  generate two different objetcs when calling the slot.
  https://www.google.es/search?q=altogh&oq=altogh&aqs=chrome..69i57.1868j0j7&client=ubuntu&sourceid=chrome&ie=UTF-8
  */
  void newImage(const QImage &image);
  void rovBusy();
  void rovReady();

  void loggingUpdated();
  void rosShutdown();

private:
  void CreateROSCommunications();
  void HandleNewROVState(const merbots_whrov_msgs::state::ConstPtr &msg);
  void HandleNewImage(const sensor_msgs::ImageConstPtr &msg);
  int init_argc;
  char **init_argv;
  ros::Publisher settings_publisher;
  ros::Publisher pose_pub;
  ros::Subscriber position_subscriber;
  image_transport::Subscriber image_subscriber;
  QStringListModel logging_model;
  OrderActionClient *orderClient;

  void goalActiveCallback();
  void
  goalCompletedCallback(const actionlib::SimpleClientGoalState &state,
                        const merbots_whrov_msgs::OrderResultConstPtr &result);
  void
  feedbackCallback(const merbots_whrov_msgs::OrderFeedbackConstPtr &feedback);

  /*
   //* http://doc.qt.io/qt-4.8/qthread.html#details
  QThread actionWorkerThread;
  class ActionWorker : public QObject
  {
      Q_OBJECT
      QThread workerThread;

  public slots:
      void doWork(QROSNode * node, const merbots_whrov::MoveOrderActionGoal &
  goal) {
          // ...
          emit resultReady(result);
      }

  signals:
      void resultReady(const merbots_whrov::MoveOrderActionResult & result);
      void feedback
  };
  */
};
#endif // QROSNODE_H
