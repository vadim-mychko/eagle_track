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
  sub_front_ = it.subscribe("camera_front", 1, &Tracker::callbackFront, this);
  sub_down_ = it.subscribe("camera_down", 1, &Tracker::callbackDown, this);
  sub_front_info_ = nh.subscribe("camera_front_info", 1, &Tracker::callbackFrontInfo, this);
  sub_down_info_ = nh.subscribe("camera_down_info", 1, &Tracker::callbackDownInfo, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  pub_front_ = it.advertise("tracker_front", 1);
  pub_down_  = it.advertise("tracker_down", 1);

  initialized_ = true;
  NODELET_INFO("[Tracker]: Initialized");
}

void Tracker::callbackFront(const sensor_msgs::ImageConstPtr& msg) {
  NODELET_INFO_THROTTLE(1.0, "[Tracker]: Processing image from front camera");
  callbackImage(msg, front_tracker_, pub_front_);
}

void Tracker::callbackDown(const sensor_msgs::ImageConstPtr& msg) {
  NODELET_INFO_THROTTLE(1.0, "[Tracker]: Processing image from down camera");
  callbackImage(msg, down_tracker_, pub_down_);
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg, cv::Ptr<cv::Tracker> tracker, const image_transport::Publisher& pub) {
  if (!initialized_) {
    return;
  }

  const std::string encoding = "bgr8";
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::InputArray image = bridge_image_ptr->image;

  cv::Rect2d bbox;
  bool success = tracker->update(image, bbox);
  if (success) {
    // cv::InputArray indicates that the variable should not be modified, but we want
    // to draw into the image. Therefore we need to copy it.
    cv::Mat track_image;
    image.copyTo(track_image);
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), 2, 1);

    NODELET_INFO_THROTTLE(1.0, "[Tracker]: Update succeeded");
    publishImage(track_image, msg->header, encoding, pub);
  } else {
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Update failed");
    publishImage(image, msg->header, encoding, pub);
  } 
}

void Tracker::callbackFrontInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  got_front_info_ = true;
  front_model_.fromCameraInfo(*msg);
  NODELET_INFO("[Tracker]: Initialized front camera info");
}

void Tracker::callbackDownInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  got_down_info_ = true;
  down_model_.fromCameraInfo(*msg);
  NODELET_INFO("[Tracker]: Initialized down camera info");
}

void Tracker::callbackDetections(const uav_detect::DetectionsConstPtr& msg) {
  if (!initialized_) {
    return;
  } else if (msg->detections.empty()) {
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Received zero detections");
    return;
  }

  // find the most confident detection
  auto less_confident = [](const uav_detect::Detection& lhs, const uav_detect::Detection& rhs) {
    return lhs.confidence < rhs.confidence;
  };
  const uav_detect::Detection& detection = *std::max_element(msg->detections.begin(), msg->detections.end(), less_confident);
  updateTracker(front_tracker_, got_front_info_, front_model_, detection);
  updateTracker(down_tracker_, got_down_info_, down_model_, detection);
  NODELET_INFO_THROTTLE(1.0, "[Tracker]: Updated tracker with detection");
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
}

void Tracker::updateTracker(cv::Ptr<cv::Tracker> tracker, bool got_camera_info, const image_geometry::PinholeCameraModel& camera_info, const uav_detect::Detection& detection) {
  if (!got_camera_info) {
    return;
  }

  geometry_msgs::Point point = detection.position;
  cv::Rect2d bbox = projectPoint(camera_info, point.x, point.y, point.z);
}

cv::Rect2d Tracker::projectPoint(const image_geometry::PinholeCameraModel& camera_info, double x, double y, double z) {

}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
