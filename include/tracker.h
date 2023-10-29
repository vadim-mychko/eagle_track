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

  // | ---------------------- ros parameters --------------------- |
  std::string _uav_name_;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_front_;
  ros::Subscriber sub_front_info_;
  ros::Subscriber sub_detections_;

  void callbackFront(const sensor_msgs::ImageConstPtr& msg);
  void callbackFrontInfo(const sensor_msgs::CameraInfoConstPtr& msg);
  void callbackDetections(const uav_detect::DetectionsConstPtr& msg);

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_front_;

  void publishFront(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding);

  // | -------------------- tracker essentials -------------------- |
  image_geometry::PinholeCameraModel front_model_;
  cv::Ptr<cv::Tracker> front_tracker_ = cv::TrackerKCF::create();
};

} // namespace eagle_track

#endif // TRACKER_H
