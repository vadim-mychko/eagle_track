#ifndef TRACKER_H
#define TRACKER_H

// each ROS nodelet must have these
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

// some OpenCV includes
#include <opencv2/opencv.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/video/tracking.hpp>

// ROS includes for working with OpenCV and images
#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <cv_bridge/cv_bridge.h>

// custom helper functions from our library
#include <mrs_lib/param_loader.h>
#include <mrs_lib/transformer.h>

// UAV detection custom messages
#include <lidar_tracker/Tracks.h>

// general boost useful things
#include <boost/circular_buffer.hpp>

namespace eagle_track
{

struct CvImageStamped {
  cv::Mat image;
  ros::Time stamp;
};

// structure for holding all the information relevant to camera model, projected detection,
// tracker, subscribers and publishers
// callbacks are implemented in the Tracker class with corresponding camera context
struct CameraContext {
  // | ---------------------- flags --------------------- |
  bool got_info = false; // whether got camera info for point projection
  bool should_init = false;

  // | ---------------------- struct parameters --------------------- |
  std::string name; // name of the camera context
  std::vector<cv::Point2f> detect_points; // points from the last detection
  std::vector<cv::Point2f> points; // points calculated from previous optical flow
  ros::Time stamp; // timestamp of the bounding box from the latest detection
  image_geometry::PinholeCameraModel model; // camera model for projection of 3d points

  // buffer for storing latest number of images for initializing the tracker with the bounding box
  // assuming bounding boxes are constructed less frequently than images, therefore buffer for images
  boost::circular_buffer<CvImageStamped> buffer;

  // needed for visualization of projections onto the camera frame
  cv::Mat prev_frame;

  // mutex to ensure thread safety
  // also servers as a tool to read "atomically" several variables at once: detection, stamp
  std::mutex mutex;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_image;
  ros::Subscriber sub_info;

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_image;

  CameraContext(const std::string& name);
};

class Tracker : public nodelet::Nodelet {

public:
  // onInit() is called when nodelet is launched (similar to main() in regular node)
  virtual void onInit();

private:
  // | ---------------------- flags --------------------- |
  bool initialized_ = false;

  // | ---------------------- static parameters --------------------- |
  bool _manual_detect_ = false;
  double _throttle_period_;

  // | ---------------------- subscribers --------------------- |
  ros::Subscriber sub_detections_;

  void callbackImageFront(const sensor_msgs::ImageConstPtr& msg);
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc);
  void callbackCameraInfoFront(const sensor_msgs::CameraInfoConstPtr& msg);
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc);
  void callbackDetections(const lidar_tracker::TracksConstPtr& msg);

  // | ---------------------- publishers --------------------- |
  image_transport::Publisher pub_projections_;

  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, image_transport::Publisher& pub);
  void publishProjections(const std::vector<cv::Point2f>& projections, const CameraContext& cc);

  // | -------------------- tracker essentials -------------------- |
  CameraContext front_ = CameraContext("FrontCamera");
  cv::Ptr<cv::SparsePyrLKOpticalFlow> opt_flow_ = cv::SparsePyrLKOpticalFlow::create();

  // | -------------------- point projection -------------------- |
  mrs_lib::Transformer transformer_;
  void updateDetection(const sensor_msgs::PointCloud2& points, CameraContext& cc);
};

} // namespace eagle_track

#endif // TRACKER_H
