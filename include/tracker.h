#ifndef TRACKER_H
#define TRACKER_H

#include "detect_selector.h"

#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking/tracker.hpp>
#include <cv_bridge/cv_bridge.h>

#include <image_transport/image_transport.h>
#include <image_geometry/pinhole_camera_model.h>
#include <image_transport/subscriber_filter.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <mrs_lib/transformer.h>
#include <mrs_lib/dynamic_reconfigure_mgr.h>

#include <boost/circular_buffer.hpp>

#include <eagle_track/TrackParamsConfig.h>
#include <lidar_tracker/Tracks.h>

namespace eagle_track
{

using policy_t = message_filters::sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image>;
using drmgr_t = mrs_lib::DynamicReconfigureMgr<eagle_track::TrackParamsConfig>;

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
  std::unique_ptr<message_filters::Synchronizer<policy_t>> sync; // synchronizer for the image + depth
  ros::Subscriber sub_info;                                      // for receiving camera parameters
  image_transport::SubscriberFilter sub_image;                   // for receiving images from the camera
  image_transport::SubscriberFilter sub_depth;                   // for receiving depth from the camera
  ros::Subscriber sub_detection;                                 // for receiving incoming detections

  // | ----------------------------- publishers ----------------------------- |
  image_transport::Publisher pub_image;       // for publishing images + tracking result (points, bounding box, etc.)
  image_transport::Publisher pub_projections; // for publishing 2D projections

  // | -------------------------- synchronization --------------------------- |
  bool should_init = false;                      // whether should re-initialize the tracker in the image callback
  std::vector<cv::Point2d> detection_points;     // points from the latest detection
  ros::Time detection_stamp;                     // timestamp of the latest detection
  std::mutex sync_mutex;                         // mutex for synchronization between callbacks
  boost::circular_buffer<CvImageStamped> buffer; // buffer for storing the latest images from the image callback

  // | ---------------------- exchange between cameras ---------------------- |
  bool got_exchange = false; // whether got bounding box exchange
  std::mutex exchange_mutex; // mutex for synchronization between exchanges
  cv::Rect2d exchange_bbox;  // bounding box received from the latest exchange
  ros::Time exchange_stamp;  // timestamp of the latest exchange

  // | ------------------------- context essentials ------------------------- |
  std::string name;                         // name of the camera context
  image_geometry::PinholeCameraModel model; // camera model for projection of 3d points
  cv::Ptr<cv::Tracker> tracker;             // tracker used to predict the next bounding box
  cv::Rect2d bbox;                          // bounding box calculated from the previous tracking inference
  bool success = false;                     // whether successfully tracked previously

  CameraContext(const std::string& name); // constructor with the name of the camera context
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
  std::unique_ptr<drmgr_t> drmgr_;                                                   // dynamic config manager
  void callbackConfig(const eagle_track::TrackParamsConfig& config, uint32_t level); // dynamic config callback

  // | ---------------------------- subscribers ----------------------------- |
  void callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc);
  void callbackImage(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg, CameraContext& cc);
  void callbackExchange(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg, CameraContext& self, CameraContext& other);
  void callbackDetection(const lidar_tracker::TracksConstPtr& msg, CameraContext& cc);

  // | ----------------------------- publishers ----------------------------- |
  void publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, image_transport::Publisher& pub);

  // | ------------------------- tracker essentials ------------------------- |
  CameraContext front_ = CameraContext("FrontCamera"); // camera context for the front camera
  CameraContext down_ = CameraContext("DownCamera");   // camera context for the down camera
  int tracker_type_;                                   // type of the tracker to use (chosen by the dynamic config)

  cv::Ptr<cv::Tracker> choose_tracker(const int tracker_type);
  bool processManualDetection(CameraContext& cc, const std_msgs::Header& header);
  bool processDetection(CameraContext& cc, const std_msgs::Header& header);
  bool processExchange(CameraContext& cc);

  // | ------------------------ coordinate transforms ----------------------- |
  std::unique_ptr<mrs_lib::Transformer> transformer_; // for transforming coordinates between sensors
};

} // namespace eagle_track

#endif // TRACKER_H
