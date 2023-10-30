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
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

// custom helper functions from our library
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

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
  std::string _world_frame_id_;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_front_;
  ros::Subscriber sub_front_info_;

  void callbackFront(const sensor_msgs::ImageConstPtr& msg);
  void callbackFrontInfo(const sensor_msgs::CameraInfoConstPtr& msg);

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_front_;

  void publishFront(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding);

  // | -------------------- tracker essentials -------------------- |
  image_geometry::PinholeCameraModel front_model_;
  cv::Ptr<cv::Tracker> front_tracker_ = cv::TrackerKCF::create();

  // | -------------------- point projection -------------------- |
  std::unique_ptr<mrs_lib::Transformer> transformer_;
  cv::Point2d projectPoint(const cv::Point3d& point);
};

} // namespace eagle_track

#endif // TRACKER_H
