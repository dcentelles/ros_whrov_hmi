#include <QDebug>
#include <QImage>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <thread>
#include <whrov_hmi/qrosnode.h>

QROSNode::QROSNode(int argc, char **argv) : init_argc(argc), init_argv(argv) {
  init();
}

QROSNode::~QROSNode() {
  if (ros::isStarted()) {
    ros::shutdown(); // explicitly needed since we use ros::start();
    ros::waitForShutdown();
    delete orderClient;
  }
  wait();
}

bool QROSNode::init() {
  ros::init(init_argc, init_argv, "whrov_hmi");
  if (!ros::master::check()) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  // Add your ros communications here.
  CreateROSCommunications();

  start();
  return true;
}

void QROSNode::updateDesiredPosition(double x, double y, double z, double yaw) {

}

bool QROSNode::init(const std::string &master_url,
                    const std::string &host_url) {
  std::map<std::string, std::string> remappings;
  remappings["__master"] = master_url;
  remappings["__hostname"] = host_url;
  ros::init(remappings, "thrusters_controller_ui");
  if (!ros::master::check()) {
    return false;
  }
  ros::start(); // explicitly needed since our nodehandle is going out of scope.

  // Add your ros communications here.
  CreateROSCommunications();

  start();
  return true;
}

void QROSNode::goalActiveCallback() {
  qDebug() << "goal active";
  emit orderActive();
}

void QROSNode::goalCompletedCallback(
    const actionlib::SimpleClientGoalState &state,
    const merbots_whrov_msgs::OrderResultConstPtr &result) {
  std::string msg = state.toString();
  qDebug() << "order completed: ";
  if (state.state_ == state.ABORTED || state.state_ == state.PREEMPTED) {
    // emit orderCancelled();
  }
  // emit orderCompleted(msg);
}

void QROSNode::feedbackCallback(
    const merbots_whrov_msgs::OrderFeedbackConstPtr &feedback) {
  QString _msg = QString("Feedback: (\%%1) %2")
                     .arg(QString::number(feedback->percent_complete));
  qDebug() << _msg;
  emit orderFeedback(feedback->percent_complete,
                     QString::fromStdString(feedback->message));
}

void QROSNode::cancelLastOrder() { orderClient->cancelAllGoals(); }

void QROSNode::sendOrder(ORDER_TYPE type, int orientation, int holdTime,
                         double x, double y, double z) {
  merbots_whrov_msgs::OrderGoal goal;
  goal.type = type;
  goal.keep_heading_degrees = orientation;
  goal.hold_channel_duration = holdTime;
  goal.x = x;
  goal.y = y;
  goal.depth = z;
  orderClient->sendGoal(
      goal, boost::bind(&QROSNode::goalCompletedCallback, this, _1, _2),
      boost::bind(&QROSNode::goalActiveCallback, this),
      boost::bind(&QROSNode::feedbackCallback, this, _1));
}

void QROSNode::updateProtocolSettings(int roix0, int roiy0, int roix1,
                                      int roiy1, int shift, int imageSize,
                                      bool rgb) {
  merbots_whrov_msgs::OrderGoal goal;
  goal.type = UPDATE_IMG_SETTINGS;
  goal.image_config.roi_x0 = roix0;
  goal.image_config.roi_y0 = roiy0;
  goal.image_config.roi_x1 = roix1;
  goal.image_config.roi_y1 = roiy1;
  goal.image_config.roi_shift = shift;
  goal.image_config.size = imageSize;
  goal.image_config.encode_mono = !rgb;
  orderClient->sendGoal(
      goal, boost::bind(&QROSNode::goalCompletedCallback, this, _1, _2),
      boost::bind(&QROSNode::goalActiveCallback, this),
      boost::bind(&QROSNode::feedbackCallback, this, _1));
}

void QROSNode::HandleNewROVState(
    const merbots_whrov_msgs::state::ConstPtr &msg) {
  // log(Info, "New ROV position received");
  qDebug() << "New ROV position received";
  emit newState(msg->heading, msg->altitude, msg->roll, msg->pitch,
                msg->keepingHeading, msg->navMode, msg->armed, msg->x, msg->y);
}

void QROSNode::HandleNewImage(const sensor_msgs::ImageConstPtr &msg) {
  // log(Info, "New ROV image received");
  // uint8_t * ibuffer = new uint8_t[msg->data.size()];
  qDebug() << "New ROV image received: "
           << QString::fromStdString(msg->encoding) << " " << msg->data.size()
           << " " << msg->width << " " << msg->height;
  // memcpy(ibuffer, msg->data.data(), msg->data.size());

  QImage::Format format;
  if (msg->encoding == "mono8")
    format = QImage::Format_Grayscale8;
  else if(msg->encoding == "rgb8")
    format = QImage::Format_RGB888;
  else //if(msg->encoding == "bgr8")
    format = QImage::Format_BGR30;

  QImage image(msg->data.data(), msg->width, msg->height, format);
  //image = image.rgbSwapped();
  // Note: The QImage is responible of deleting the ibuffer

  emit newImage(image);
  // delete ibuffer;
}

void QROSNode::CreateROSCommunications() {
  ros::NodeHandle nh;
  orderClient = new OrderActionClient("order", true);
  orderClient->waitForServer();
  settings_publisher = nh.advertise<merbots_whrov_msgs::hrov_settings>(
      "desired_hrov_settings", 1);
  position_subscriber = nh.subscribe<merbots_whrov_msgs::state>(
      "current_hrov_state", 1,
      boost::bind(&QROSNode::HandleNewROVState, this, _1));

  //pose_pub = _nh.advertise<geometry_msgs::Pose>("bluerov2/pose", 1);

  image_transport::ImageTransport it(nh);
  image_subscriber = it.subscribe(
      "camera", 1, boost::bind(&QROSNode::HandleNewImage, this, _1));

  std::thread ghost([this]() {
    tf::TransformBroadcaster broadcaster;
    tf::TransformListener listener;
    std::string origin = "local_origin_ned", target = "bluerov2_ghost";
    std::string origin_enu = "local_origin", target_enu = "bluerov2_ghost_enu";
    tf::StampedTransform current_origin_target_tf;
    while (1) {
      try {
        listener.waitForTransform(origin, target, ros::Time(0),
                                  ros::Duration(1));
        listener.lookupTransform(origin, target, ros::Time(0),
                                 current_origin_target_tf);

        double x = current_origin_target_tf.getOrigin().x();
        double y = current_origin_target_tf.getOrigin().y();
        double z = current_origin_target_tf.getOrigin().z();
        double yaw;
        yaw = tf::getYaw(current_origin_target_tf.getRotation());

        double degs = yaw * (180 / M_PI);
        if(degs < 0) degs = 360 + degs;

        emit desiredPositionUpdated(x, y, z, degs);

        listener.waitForTransform(origin_enu, target_enu, ros::Time(0),
                                  ros::Duration(1));
        listener.lookupTransform(origin_enu, target_enu, ros::Time(0),
                                 current_origin_target_tf);

        //The following is for debug purposes.
        double enuYaw;
        enuYaw = tf::getYaw(current_origin_target_tf.getRotation());

        double enuDegs = enuYaw * (180 / M_PI);
        if(enuDegs < 0) enuDegs = 360 + enuDegs;


        //qDebug() << "yaw: " << yaw << " degs: " << degs << " enuYaw: " << enuYaw << " enuDegs: " << enuDegs;


        std::this_thread::sleep_for(std::chrono::milliseconds(100));

      } catch (tf::TransformException &e) {
        ROS_ERROR("Not able to lookup transform: %s", e.what());
      }
    }
  });
  ghost.detach();
}

void QROSNode::run() {
  ros::Rate loop_rate(10);
  int count = 0;
  while (ros::ok()) {

    // Do something
    //...

    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }
  std::cout << "Ros shutdown, proceeding to close the gui." << std::endl;
  Q_EMIT
  rosShutdown(); // used to signal the gui for a shutdown (useful to roslaunch)
}

void QROSNode::log(const LogLevel &level, const std::string &_msg) {
  int rleft = logging_model.rowCount() - 9;
  if (rleft > 0)
    logging_model.removeRows(0, rleft);

  logging_model.insertRows(logging_model.rowCount(), 1);

  if (!ros::master::check()) {
    std::string msg = "[ERROR] no conectado a ROS master" + _msg;
    QVariant new_row2(QString(msg.c_str()));
    ;
    logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                          new_row2);
    Q_EMIT loggingUpdated(); // used to readjust the scrollbar
    return;
  }
  std::stringstream logging_model_msg;
  std::string msg = _msg;
  switch (level) {
  case (Debug): {
    ROS_DEBUG_STREAM(msg);
    logging_model_msg << "[DEBUG] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case (Info): {
    ROS_INFO_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case (Warn): {
    ROS_WARN_STREAM(msg);
    logging_model_msg << "[INFO] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case (Error): {
    ROS_ERROR_STREAM(msg);
    logging_model_msg << "[ERROR] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  case (Fatal): {
    ROS_FATAL_STREAM(msg);
    logging_model_msg << "[FATAL] [" << ros::Time::now() << "]: " << msg;
    break;
  }
  }
  QVariant new_row(QString(logging_model_msg.str().c_str()));
  logging_model.setData(logging_model.index(logging_model.rowCount() - 1),
                        new_row);
  Q_EMIT loggingUpdated(); // used to readjust the scrollbar
}
