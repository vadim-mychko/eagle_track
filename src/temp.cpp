#include "temp.h"

#include <opencv2/highgui/highgui.hpp>

namespace eagle_track
{

CameraContext::CameraContext(const std::string& name) : name(name) {}

void Tracker::onInit() {
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO_ONCE("[Tracker]: Waiting for valid time...");
  ros::Time::waitForValid();

  // | -------------------------- static parameters ------------------------- |
  mrs_lib::ParamLoader pl(nh, "Tracker");
  NODELET_INFO_ONCE("[Tracker]: Loading static parameters:");
  const auto uav_name = pl.loadParam2<std::string>("UAV_NAME");
  const auto image_type = pl.loadParam2<std::string>("image_type");
  pl.loadParam("throttle_period", _throttle_period_);

  // | -------------------------- dynamic parameters ------------------------ |
  drmgr_ = std::make_unique<drmgr_t>(nh, true, "Tracker", boost::bind(&Tracker::callbackConfig, this, _1, _2));

  if (!pl.loadedSuccessfully() || !drmgr_->loaded_successfully()) {
    NODELET_ERROR_ONCE("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ---------------------------- subscribers ----------------------------- |
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(image_type);

  front_.sub_image = it.subscribe("camera_front", 1, boost::bind(&Tracker::callbackImage, this, _1, front_), ros::VoidPtr(), hints);
  front_.sub_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_front_info", 1, boost::bind(&Tracker::callbackCameraInfo, this, _1, front_));
  down_.sub_image = it.subscribe("camera_down", 1, boost::bind(&Tracker::callbackImage, this, _1, down_), ros::VoidPtr(), hints);
  down_.sub_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_down_info", 1, boost::bind(&Tracker::callbackCameraInfo, this, _1, down_));
  sub_detection_ = nh.subscribe("detection", 1, &Tracker::callbackDetection, this);

  // | ----------------------------- publishers ----------------------------- |
  front_.pub_image = it.advertise("tracker_front", 1);
  front_.pub_projections = it.advertise("projections_front", 1);
  down_.pub_image = it.advertise("tracker_down", 1);
  down_.pub_projections = it.advertise("projections_down", 1);

  // | --------------------------- tf transformer --------------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>("Tracker");
  transformer_->setDefaultPrefix(uav_name);
  transformer_->retryLookupNewest(true);

  // | ------------------------ detection through GUI ----------------------- |
  if (_manual_detect_) {
    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    cv::namedWindow("manual_detect", flags);
  }

  initialized_ = true;
  NODELET_INFO_ONCE("[Tracker]: Initialized");
}


} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
