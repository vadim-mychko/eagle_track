#include "tracker.h"

namespace eagle_track
{

void Tracker::onInit() {
  /* obtain node handle */
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  /* waits for the ROS to publish clock */
  ros::Time::waitForValid();

  // | ------------------- load ros parameters ------------------ |
  /* (mrs_lib implementation checks whether the parameter was loaded or not) */

  mrs_lib::ParamLoader param_loader(nh, "Tracking");

  param_loader.loadParam("UAV_NAME", _uav_name_);
  param_loader.loadParam("gui", _gui_);

  if (!param_loader.loadedSuccessfully()) {
    ROS_ERROR("[WaypointFlier]: failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | --------------------------- gui -------------------------- |

  if (_gui_) {
    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    cv::namedWindow("tracking", flags);
  }

  /* initialize the image transport, needs node handle */
  image_transport::ImageTransport it(nh);

  // | ----------------- initialize subscribers ----------------- |
  sub_image_ = it.subscribe("image_in", 1, &Tracker::callbackImage, this);

  ROS_INFO_ONCE("[Tracking]: initialized");
  is_initialized_ = true;
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg) {
  if (!is_initialized_) {
    return;
  }

  const std::string color_encoding = "bgr8";
  const std_msgs::Header msg_header = msg->header;
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, color_encoding);

  if (_gui_) {
    showTrackingImage(bridge_image_ptr->image);
    cv::waitKey(1);
  }
}

void Tracker::showTrackingImage(cv::InputArray image) {
  // cv::InputArray indicates that the variable should not be modified, but we want
  // to draw into the image. Therefore we need to copy it.
  cv::Mat tracking_image;
  image.copyTo(tracking_image);

  // Update tracking result
  cv::Rect2d bbox;
  bool success = tracker_->update(image, bbox);

  if (success) {
    // Tracking success : Draw the tracked object
    cv::rectangle(tracking_image, bbox, cv::Scalar(255, 0, 0), 2, 1);
    cv::imshow("tracking", tracking_image);
  } else {
    // Tracking failure detected
    cv::putText(tracking_image, "Tracking failure detected", cv::Point(100, 80), cv::FONT_HERSHEY_SIMPLEX, 0.75, cv::Scalar(0, 0, 255), 2);
    // Pause and allow user to select new bbox
    bbox = cv::selectROI("tracking", tracking_image, false, false);
    // Reinitialize the tracker with the new bbox.
    tracker_->init(image, bbox);
  }
}

} // namespace eagle_track

/* every nodelet must include macros which export the class as a nodelet plugin */
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
