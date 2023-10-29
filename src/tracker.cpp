#include "tracker.h"

namespace eagle_track
{

void Tracker::onInit() {
  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh, "Tracker");
  param_loader.loadParam("UAV_NAME", _uav_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Tracker]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);

  // | ---------------------- subscribers --------------------- |
  sub_front_ = it.subscribe("tracker_front", 1, &Tracker::callbackImage, this);
  sub_down_ = it.subscribe("tracker_down", 1, &Tracker::callbackImage, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  pub_front_ = it.advertise("tracker_front", 1);
  pub_down_  = it.advertise("tracker_down", 1);

  initialized_ = true;
  ROS_INFO_ONCE("[Tracker]: initialized");
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  const std::string color_encoding = "bgr8";
  const std_msgs::Header msg_header = msg->header;
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
}

void Tracker::callbackDetections(const uav_detect::DetectionsConstPtr& msg) {
  if (!initialized_) {
    return;
  }
}

} // namespace eagle_track

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
