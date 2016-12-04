#ifndef QROSNODE_H
#define QROSNODE_H
#include <QThread>
#include <QStringListModel>
#include <ros/ros.h>
#include <ros/network.h>

namespace merbots_whrov_hmi {

class QROSNode : public QThread
{
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

    int init_argc;
    char** init_argv;
    ros::Publisher desiredState_publisher;
    ros::Subscriber currentState_subscriber;
    QStringListModel logging_model;
};

#endif // QROSNODE_H

}
