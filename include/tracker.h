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

// custom helper functions from our library
#include <mrs_lib/param_loader.h>

// UAV detection
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

  // | ---------------------- ros parameters --------------------- |
  std::string _uav_name_;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_front_;
  image_transport::Subscriber sub_down_;
  ros::Subscriber sub_detections_;

  void callbackFront(const sensor_msgs::ImageConstPtr& msg);
  void callbackDown(const sensor_msgs::ImageConstPtr& msg);
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, const image_transport::Publisher& pub);
  void callbackDetections(const uav_detect::DetectionsConstPtr& msg);

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_front_;
  image_transport::Publisher pub_down_;

  // | -------------------- tracker essentials -------------------- |
  cv::Ptr<cv::TrackerKCF> tracker_ = cv::TrackerKCF::create();
};

} // namespace eagle_track

#endif // TRACKER_H
