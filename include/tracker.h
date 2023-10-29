#ifndef TRACKER_H
#define TRACKER_H

// each ROS nodelet must have these
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

// some OpenCV includes
#include <opencv2/tracking/tracker.hpp>

// ROS includes for working with OpenCV and images
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <image_geometry/pinhole_camera_model.h>

// custom helper functions from our library
#include <mrs_lib/param_loader.h>

// UAV detection
#include <uav_detect/Detection.h>
#include <uav_detect/Detections.h>

namespace eagle_track
{

class Tracker : public nodelet::Nodelet {

public:
  // onInit() is called when nodelet is launched (similar to main() in regular node)
  virtual void onInit();

private:
  // | ---------------------- flags --------------------- |
  bool initialized_ = false;
  bool got_front_info_ = false;
  bool got_down_info_ = false;

  // | ---------------------- ros parameters --------------------- |
  std::string _uav_name_;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_front_;
  image_transport::Subscriber sub_down_;
  ros::Subscriber sub_front_info_;
  ros::Subscriber sub_down_info_;
  ros::Subscriber sub_detections_;

  void callbackFront(const sensor_msgs::ImageConstPtr& msg);
  void callbackDown(const sensor_msgs::ImageConstPtr& msg);
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, cv::Ptr<cv::Tracker> tracker, const image_transport::Publisher& pub);
  void callbackFrontInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  void callbackDownInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  void callbackDetections(const uav_detect::DetectionsConstPtr& msg);

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_front_;
  image_transport::Publisher pub_down_;

  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const image_transport::Publisher& pub);

  // | -------------------- tracker essentials -------------------- |
  image_geometry::PinholeCameraModel front_model_;
  image_geometry::PinholeCameraModel down_model_;
  cv::Ptr<cv::Tracker> front_tracker_ = cv::TrackerKCF::create();
  cv::Ptr<cv::Tracker> down_tracker_ = cv::TrackerKCF::create();

  // | -------------------- utility functions -------------------- |
  void updateTracker(cv::Ptr<cv::Tracker> tracker, bool got_camera_info, const image_geometry::PinholeCameraModel& camera_info, const uav_detect::Detection& detection);
  cv::Rect2d projectPoint(const image_geometry::PinholeCameraModel& camera_info, double x, double y, double z);
};

} // namespace eagle_track

#endif // TRACKER_H
