#include "tracker.h"

namespace eagle_track
{

void Tracker::onInit() {
  // obtain node handle
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  // waits for the ROS to publish clock
  ros::Time::waitForValid();

  // | ------------------- ros parameters ------------------ |
  mrs_lib::ParamLoader param_loader(nh, "Tracker");
  param_loader.loadParam("UAV_NAME", _uav_name_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[Tracker]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // initialize the image transport, needs node handle
  image_transport::ImageTransport it(nh);

  // | ---------------------- subscribers --------------------- |
  sub_front_ = it.subscribe("tracker_front", 1, &Tracker::callbackFront, this);
  sub_down_ = it.subscribe("tracker_down", 1, &Tracker::callbackBack, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  pub_front_ = it.advertise("tracker_front", 1);
  pub_down_  = it.advertise("tracker_down", 1);

  initialized_ = true;
  ROS_INFO_ONCE("[Tracker]: initialized");
}

void Tracker::callbackFront(const sensor_msgs::ImageConstPtr& msg) {
  callbackImage(msg, pub_front_);
}

void Tracker::callbackDown(const sensor_msgs::ImageConstPtr& msg) {
  callbackImage(msg, pub_down_);
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher& pub) {
  if (!initialized_) {
    return;
  }

  const std::string color_encoding = "bgr8";
  const std_msgs::Header msg_header = msg->header;
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);
  cv::InputArray image = bridge_image_ptr->image;

  cv::Rect2d bbox;
  bool success = tracker_->update(image, bbox);
  if (success) {
    // cv::InputArray indicates that the variable should not be modified, but we want
    // to draw into the image. Therefore we need to copy it.
    cv::Mat track_image;
    image.copyTo(track_image);

    // Draw the received bounding box onto the copied image
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), 2, 1);

    // Prepare a cv_bridge image to be converted to the ROS message
    cv_bridge::CvImage bridge_image_out;
    bridge_image_out.image = track_image;
    bridge_image_out.encoding = color_encoding;

    // Now convert the cv_bridge image to a ROS message and publish it
    sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
    pub.publish(out_msg);
  }
}

void Tracker::callbackDetections(const uav_detect::DetectionsConstPtr& msg) {
  if (!initialized_) {
    return;
  }
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
