#include "tracker.h"

namespace eagle_track
{

void Tracker::onInit() {
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO("[Tracker]: Waiting for valid time...");
  ros::Time::waitForValid();

  // | ------------------- ros parameters ------------------ |
  mrs_lib::ParamLoader pl(nh, "Tracker");
  NODELET_INFO("[Tracker]: Loading static parameters:");
  pl.loadParam("UAV_NAME", _uav_name_);

  if (!pl.loadedSuccessfully()) {
    NODELET_ERROR("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  image_transport::ImageTransport it(nh);

  // | ---------------------- subscribers --------------------- |
  sub_front_ = it.subscribe("tracker_front", 1, &Tracker::callbackFront, this);
  sub_down_ = it.subscribe("tracker_down", 1, &Tracker::callbackDown, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  pub_front_ = it.advertise("tracker_front", 1);
  pub_down_  = it.advertise("tracker_down", 1);

  initialized_ = true;
  NODELET_INFO("[Tracker]: Initialized");
}

void Tracker::callbackFront(const sensor_msgs::ImageConstPtr& msg) {
  NODELET_INFO_THROTTLE(1.0, "[Tracker]: Processing new image from front camera");
  callbackImage(msg, pub_front_);
}

void Tracker::callbackDown(const sensor_msgs::ImageConstPtr& msg) {
  NODELET_INFO_THROTTLE(1.0, "[Tracker]: Processing new image from down camera");
  callbackImage(msg, pub_down_);
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher& pub) {
  if (!initialized_) {
    return;
  }

  const std::string encoding = "bgr8";
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::InputArray image = bridge_image_ptr->image;

  cv::Rect2d bbox;
  bool success = tracker_->update(image, bbox);
  if (success) {
    // cv::InputArray indicates that the variable should not be modified, but we want
    // to draw into the image. Therefore we need to copy it.
    cv::Mat track_image;
    image.copyTo(track_image);
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), 2, 1);

    publishImage(track_image, msg->header, encoding, pub);
  } else {
    NODELET_ERROR_THROTTLE(1.0, "[Tracker]: update of tracker failed");
  }
}

void Tracker::callbackDetections(const uav_detect::DetectionsConstPtr& msg) {
  if (!initialized_) {
    return;
  }
}

void Tracker::publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub) {
  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.image = image.getMat();
  bridge_image_out.header = header;
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message and publish it
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  pub.publish(out_msg);
  NODELET_INFO_THROTTLE(1.0, "[Tracker]: update of tracker succeeded");
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
