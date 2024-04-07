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
  cv::Mat image;
  ros::Time stamp;
};

struct CameraContext
{
  // | -------------------------------- flags ------------------------------- |
  bool got_camera_info = false;

  // | ---------------------------- subscribers ----------------------------- |
  image_transport::SubscriberFilter sub_image;
  ros::Subscriber sub_info;

  // | ----------------------------- publishers ----------------------------- |
  image_transport::Publisher pub_image;
  image_transport::Publisher pub_projections;

  // | -------------------------- synchronization --------------------------- |
  std::vector<cv::Point2f> detection_points;     // points from the latest detection
  ros::Time detection_stamp;                     // timestamp of the latest detection
  std::mutex sync_mutex;                         // mutex for synchronization between callbacks
  boost::circular_buffer<CvImageStamped> buffer; // buffer for storing the latest images from the image callback

  // | ------------------------- context essentials ------------------------- |
  std::string name;                         // name of the camera context
  image_geometry::PinholeCameraModel model; // camera model for projection of 3d points
  std::vector<cv::Point2f> prev_points;     // points calculated from previous optical flow
  cv::Mat prev_image;                       // previous image for calculating optical flow

  CameraContext(const std::string& name);
};

class Tracker : public nodelet::Nodelet {

public:
  virtual void onInit();

private:
  // | -------------------------------- flags ------------------------------- |
  bool initialized_ = false;

  // | -------------------------- static parameters ------------------------- |
  bool _manual_detect_;
  double _throttle_period_;

  // | -------------------------- dynamic parameters ------------------------ |
  using drmgr_t = mrs_lib::DynamicReconfigureMgr<eagle_track::TrackParamsConfig>;
  std::unique_ptr<drmgr_t> drmgr_;
  void callbackConfig(const eagle_track::TrackParamsConfig& config, uint32_t level);

  // | ---------------------------- subscribers ----------------------------- |
  message_filters::Subscriber<lidar_tracker::Tracks> sub_detection_;
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc);
  void callbackDetection(const lidar_tracker::TracksConstPtr& msg, CameraContext& cc);
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc);

  // | ----------------------------- publishers ----------------------------- |
  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, image_transport::Publisher& pub);

  // | ------------------------- tracker essentials ------------------------- |
  CameraContext front_ = CameraContext("FrontCamera");
  CameraContext down_ = CameraContext("DownCamera");
  cv::Ptr<cv::SparsePyrLKOpticalFlow> flow_ = cv::SparsePyrLKOpticalFlow::create();

  // | ------------------------ coordinate transforms ----------------------- |
  std::unique_ptr<mrs_lib::Transformer> transformer_;
};

} // namespace eagle_track

#endif // TRACKER_H
