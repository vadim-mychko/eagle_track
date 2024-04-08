#ifndef TRACKER_H
#define TRACKER_H

#include "detect_selector.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/video/tracking.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>

#include <boost/circular_buffer.hpp>

#include <eagle_track/TrackParamsConfig.h>
#include <lidar_tracker/Tracks.h>

namespace eagle_track
{

struct CvImageStamped
{
  cv::Mat image;   // image from the camera (grayscale or RGB)
  ros::Time stamp; // exact timestamp related to the image
};

struct CameraContext
{
  // | -------------------------------- flags ------------------------------- |
  bool got_camera_info = false; // whether received camera parameters already

  // | ---------------------------- subscribers ----------------------------- |
  ros::Subscriber sub_info;                    // for receiving camera parameters
  image_transport::SubscriberFilter sub_image; // for receiving images from the camera
  ros::Subscriber sub_detection;               // for receiving incoming detections

  // | ----------------------------- publishers ----------------------------- |
  image_transport::Publisher pub_image;       // for publishing images + tracking result (points, bounding box, etc.)
  image_transport::Publisher pub_projections; // for publishing 2D projections

  // | -------------------------- synchronization --------------------------- |
  bool should_init = false;                      // whether should re-initialize the tracker in the image callback
  std::vector<cv::Point2f> detection_points;     // points from the latest detection
  ros::Time detection_stamp;                     // timestamp of the latest detection
  std::mutex sync_mutex;                         // mutex for synchronization between callbacks
  boost::circular_buffer<CvImageStamped> buffer; // buffer for storing the latest images from the image callback

  // | ------------------------- context essentials ------------------------- |
  std::string name;                         // name of the camera context
  image_geometry::PinholeCameraModel model; // camera model for projection of 3d points
  std::vector<cv::Point2f> prev_points;     // points calculated from previous optical flow

  CameraContext(const std::string& name);   // constructor with the name of the camera context
};

class Tracker : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  // | -------------------------------- flags ------------------------------- |
  bool initialized_ = false; // whether the nodelet is initialized (after calling onInit())

  // | -------------------------- static parameters ------------------------- |
  bool _manual_detect_;     // should use detections from the GUI or not
  double _throttle_period_; // parameter regulating frequency of log messages

  // | -------------------------- dynamic parameters ------------------------ |
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<eagle_track::TrackParamsConfig>;    // short name for the dynamic config
  std::unique_ptr<drmgr_t> drmgr_;                                                   // dynamic config manager
  void callbackConfig(const eagle_track::TrackParamsConfig& config, uint32_t level); // dynamic config callback

  // | ---------------------------- subscribers ----------------------------- |
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc); // callback for the camera info
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc);           // callback for the images from the cameras
  void callbackDetection(const lidar_tracker::TracksConstPtr& msg, CameraContext& cc);    // callback for the detections

  // | ----------------------------- publishers ----------------------------- |
  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, image_transport::Publisher& pub);

  // | ------------------------- tracker essentials ------------------------- |
  CameraContext front_ = CameraContext("FrontCamera");                              // camera context for the front camera
  CameraContext down_ = CameraContext("DownCamera");                                // camera context for the down camera
  cv::Ptr<cv::SparsePyrLKOpticalFlow> flow_ = cv::SparsePyrLKOpticalFlow::create(); // parametrized Lucas-Kanade tracker

  // | ------------------------ coordinate transforms ----------------------- |
  std::unique_ptr<mrs_lib::Transformer> transformer_; // for transforming coordinates between sensors
};

} // namespace eagle_track

#endif // TRACKER_H
