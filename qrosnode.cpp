#include "qrosnode.h"

#include <merbots_whrov/hrov_state.h>

namespace merbots_whrov_hmi {

QROSNode::QROSNode(int argc, char** argv ):
    init_argc(argc),
    init_argv(argv)
{}

QROSNode::~QROSNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
    }
    wait();
}

bool QROSNode::init() {
    ros::init(init_argc,init_argv,"thrusters_controller_ui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;
    // Add your ros communications here.
    CreateROSCommunications(n);

    start();
    return true;
}

bool QROSNode::init(const std::string &master_url, const std::string &host_url) {
    std::map<std::string,std::string> remappings;
    remappings["__master"] = master_url;
    remappings["__hostname"] = host_url;
    ros::init(remappings,"thrusters_controller_ui");
    if ( ! ros::master::check() ) {
        return false;
    }
    ros::start(); // explicitly needed since our nodehandle is going out of scope.
    ros::NodeHandle n;

    // Add your ros communications here.
    CreateROSCommunications(n);

    start();
    return true;
}

void QROSNode::CreateROSCommunications(ros::NodeHandle & nh)
{
    desiredState_publisher = nh.advertise<merbots_whrov::hrov_state>("merbots/whrov/operator/desired_hrov_state", 1);
}

void QROSNode::run() {
    ros::Rate loop_rate(10);
    int count = 0;
        while ( ros::ok() )
    {

        //Do something
        //...

        ros::spinOnce();
        loop_rate.sleep();
        ++count;

    }
    std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
    Q_EMIT rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}


void QROSNode::log( const LogLevel &level, const std::string &_msg) {
    int rleft = logging_model.rowCount() - 9;
    if(rleft > 0)
        logging_model.removeRows(0,rleft);

    logging_model.insertRows(logging_model.rowCount(),1);

    if ( ! ros::master::check() ) {
            std::string msg = "[ERROR] no conectado a ROS master" + _msg;
            QVariant new_row2(QString(msg.c_str()));;
            logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row2);
            Q_EMIT loggingUpdated(); // used to readjust the scrollbar
            return;
    }
    std::stringstream logging_model_msg;
    std::string msg = _msg;
    switch ( level ) {
        case(Debug) : {
                ROS_DEBUG_STREAM(msg);
                logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Info) : {
                                ROS_INFO_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Warn) : {
                ROS_WARN_STREAM(msg);
                logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Error) : {
                ROS_ERROR_STREAM(msg);
                logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
                break;
        }
        case(Fatal) : {
                ROS_FATAL_STREAM(msg);
                logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
                break;
        }
    }
    QVariant new_row(QString(logging_model_msg.str().c_str()));
    logging_model.setData(logging_model.index(logging_model.rowCount()-1),new_row);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}

}
