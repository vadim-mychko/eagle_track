#ifndef TRACKER_H
#define TRACKER_H

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <opencv2/video/tracking.hpp>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>

#include <eagle_track/TrackParamsConfig.h>

#include <lidar_tracker/Tracks.h>

namespace eagle_track
{

struct CameraContext
{
  // | -------------------------------- flags ------------------------------- |
  bool got_camera_info = false;

  // | ---------------------------- subscribers ----------------------------- |
  image_transport::Subscriber sub_image;
  ros::Subscriber sub_info;

  // | ----------------------------- publishers ----------------------------- |
  image_transport::Publisher pub_image;
  image_transport::Publisher pub_projections;

  // | ------------------------- context essentials ------------------------- |
  std::string name;                         // name of the camera context
  std::vector<cv::Point2f> points;          // points calculated from previous optical flow
  image_geometry::PinholeCameraModel model; // camera model for projection of 3d points

  CameraContext(const std::string& name);
};

class Tracker : public nodelet::Nodelet {

public:
  virtual void onInit();

protected:
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
  ros::Subscriber sub_detection_;
  void callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc);
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc);
  void callbackDetection(const lidar_tracker::TracksConstPtr& msg);

  // | ----------------------------- publishers ----------------------------- |
  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, const CameraContext& cc);
  void publishProjections(const std::vector<cv::Point2f>& projections, const CameraContext& cc);

  // | ------------------------- tracker essentials ------------------------- |
  CameraContext front_ = CameraContext("FrontCamera");
  CameraContext down_ = CameraContext("DownCamera");
  cv::Ptr<cv::SparsePyrLKOpticalFlow> flow_ = cv::SparsePyrLKOpticalFlow::create();

  // | -------------------------- point projection -------------------------- |
  std::unique_ptr<mrs_lib::Transformer> transformer_;
  void updateDetection(const sensor_msgs::PointCloud2& points, CameraContext& cc);
};

} // namespace eagle_track

#endif // TRACKER_H
