#ifndef QROSNODE_H
#define QROSNODE_H
#include <QThread>
#include <QStringListModel>
#include <ros/ros.h>
#include <ros/network.h>
#include <merbots_whrov/position.h>
#include <merbots_whrov/hrov_settings.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class QROSNode : public QThread{
Q_OBJECT
public:
    QROSNode(int argc, char** argv );
    virtual ~QROSNode();
    bool init();
    bool init(const std::string &master_url, const std::string &host_url);
    void run();
    /*********************
    ** Logging
    **********************/
    enum LogLevel {
             Debug,
             Info,
             Warn,
             Error,
             Fatal
     };

    QStringListModel* loggingModel() { return &logging_model; }
    void log( const LogLevel &level, const std::string &msg);

Q_SIGNALS:
    void loggingUpdated();
    void rosShutdown();

private:
    void CreateROSCommunications(ros::NodeHandle & n);
    void HandleNewROVPosition(const merbots_whrov::position::ConstPtr & msg);
    void HandleNewImage(const sensor_msgs::ImageConstPtr& msg);
    int init_argc;
    char** init_argv;
    ros::Publisher settings_publisher;
    ros::Subscriber position_subscriber;
    image_transport::Subscriber image_subscriber;
    QStringListModel logging_model;
};

#endif // QROSNODE_H

