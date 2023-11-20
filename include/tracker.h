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

// UAV detection custom messages
#include <lidar_tracker/Tracks.h>

namespace eagle_track
{

// structure for holding all the information relevant to camera model, projected detection,
// tracker, subscribers and publishers
// callbacks are implemented in the Tracker class with corresponding camera context
struct CameraContext {
  // | ---------------------- flags --------------------- |
  bool got_info = false;

  // | ---------------------- parameters --------------------- |
  std::string name;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_image;
  ros::Subscriber sub_info;

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_image;

  // | -------------------- tracker essentials -------------------- |
  cv::Rect2d detection;
  image_geometry::PinholeCameraModel model;
  cv::Ptr<cv::Tracker> tracker = cv::TrackerCSRT::create();

  CameraContext(const std::string& name);
};

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
  ros::Subscriber sub_detections_;

  void callbackImageFront(const sensor_msgs::ImageConstPtr& msg);
  void callbackImageDown(const sensor_msgs::ImageConstPtr& msg);
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc);
  void callbackCameraInfoFront(const sensor_msgs::CameraInfoConstPtr& msg);
  void callbackCameraInfoDown(const sensor_msgs::CameraInfoConstPtr& msg);
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc);
  void callbackDetections(const lidar_tracker::TracksConstPtr& msg);

  // | ---------------------- publishers --------------------- |
  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, CameraContext& cc);

  // | -------------------- tracker essentials -------------------- |
  CameraContext front = CameraContext("FrontCamera");
  CameraContext down = CameraContext("DownCamera");

  // | -------------------- point projection -------------------- |
  mrs_lib::Transformer transformer_;
  void updateDetection(const sensor_msgs::PointCloud2& points, CameraContext& cc);
};

} // namespace eagle_track

#endif // TRACKER_H
