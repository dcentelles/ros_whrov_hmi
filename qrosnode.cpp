#include "qrosnode.h"
#include <QDebug>
#include <QImage>

QROSNode::QROSNode(int argc, char** argv ):
    init_argc(argc),
    init_argv(argv)
{
    init();
}

QROSNode::~QROSNode() {
    if(ros::isStarted()) {
      ros::shutdown(); // explicitly needed since we use ros::start();
      ros::waitForShutdown();
      delete orderClient;
    }
    wait();
}

bool QROSNode::init() {
    ros::init(init_argc,init_argv,"whrov_hmi");
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

void QROSNode::goalActiveCallback()
{
    qDebug() << "goal active";
    emit orderActive();
}

void QROSNode::goalCompletedCallback(const actionlib::SimpleClientGoalState &state, const merbots_whrov_msgs::MoveOrderResultConstPtr &result)
{
    std::string msg = state.toString();
    qDebug() << "order completed: ";
    if(state.state_ == state.ABORTED || state.state_ == state.PREEMPTED)
    {
        //emit orderCancelled();
    }
    //emit orderCompleted(msg);
}

void QROSNode::feedbackCallback(const merbots_whrov_msgs::MoveOrderFeedbackConstPtr &feedback)
{
    QString _msg = QString("Feedback: (\%%1) %2").arg(QString::number(feedback->percent_complete));
    qDebug() << _msg;
    emit orderFeedback(feedback->percent_complete,
                       QString::fromStdString(feedback->message)
                       );
}

void QROSNode::cancelLastOrder()
{
    orderClient->cancelAllGoals();
}

void QROSNode::sendOrder(bool relative, int orientation, float z, float x, float y)
{
    merbots_whrov_msgs::MoveOrderGoal goal;
    goal.order.relative = relative;
    goal.order.yaw = orientation;
    goal.order.Z = z * 10;
    goal.order.X = x * 10;
    goal.order.Y = y * 10;
    qDebug() << goal.order.Z << " " << goal.order.X << " " << goal.order.Y;
    orderClient->sendGoal(goal,
                          boost::bind(&QROSNode::goalCompletedCallback, this, _1, _2),
                          boost::bind(&QROSNode::goalActiveCallback, this),
                          boost::bind(&QROSNode::feedbackCallback, this, _1)
//OrderActionClient::SimpleActiveCallback(),
//OrderActionClient::SimpleFeedbackCallback()
                          );

}

void QROSNode::updateProtocolSettings(int roix0, int roiy0, int roix1, int roiy1, int shift, int imageSize, int packetLength)
{
    merbots_whrov_msgs::hrov_settings msg;
    msg.image_config.roi_x0 = roix0;
    msg.image_config.roi_y0 = roiy0;
    msg.image_config.roi_x1 = roix1;
    //msg.image_config.roi_x1 = roix1 - roix0;
    msg.image_config.roi_y1 = roiy1;
    //msg.image_config.roi_y1 = roiy1 - roiy0;
    msg.image_config.roi_shift = shift;
    msg.image_config.size = imageSize;
    msg.protocol_config.max_packet_length = packetLength;

    settings_publisher.publish(msg);
}

void QROSNode::HandleNewROVPosition(const merbots_whrov_msgs::position::ConstPtr & msg)
{
    //log(Info, "New ROV position received");
    qDebug() << "New ROV position received";
    emit newPosition(
                msg->yaw,
                msg->Z,// / 10.,
                msg->X,// / 10.,
                msg->Y// / 10.
                );
}

void QROSNode::HandleNewImage(const sensor_msgs::ImageConstPtr &msg)
{
    //log(Info, "New ROV image received");
   // uint8_t * ibuffer = new uint8_t[msg->data.size()];
    qDebug() << "New ROV image received: " << QString::fromStdString(msg->encoding) << " " << msg->data.size()
             << " " << msg->width
             << " " << msg-> height;
    //memcpy(ibuffer, msg->data.data(), msg->data.size());
    QImage image(msg->data.data(), msg->width, msg->height,
                 QImage::Format_RGB888);

    image = image.rgbSwapped();

    //Note: The QImage is responible of deleting the ibuffer

    emit newImage(image);
   // delete ibuffer;

}

void QROSNode::CreateROSCommunications(ros::NodeHandle & nh)
{
    orderClient = new OrderActionClient("/merbots/whrov/operator_control/actions/move_order", true);
    orderClient->waitForServer();
    settings_publisher = nh.advertise<merbots_whrov_msgs::hrov_settings>("/merbots/whrov/operator_control/desired_hrov_settings", 1);
    position_subscriber = nh.subscribe<merbots_whrov_msgs::position>("/merbots/whrov/operator_control/current_hrov_position", 1,
       boost::bind(&QROSNode::HandleNewROVPosition, this, _1));

    image_transport::ImageTransport it(nh);
    image_subscriber = it.subscribe("/merbots/whrov/operator_control/camera", 1,
        boost::bind(&QROSNode::HandleNewImage, this, _1)
                                    );
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

