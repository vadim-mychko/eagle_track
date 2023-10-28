#ifndef TRACK_H
#define TRACK_H

/* each ROS nodelet must have these */
#include <ros/ros.h>
#include <ros/package.h>
#include <nodelet/nodelet.h>

/* some OpenCV includes */
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/tracking/tracker.hpp>

/* ROS includes for working with OpenCV and images */
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

/* custom helper functions from our library */
#include <mrs_lib/param_loader.h>

namespace eagle_track
{

class Tracker : public nodelet::Nodelet {

public:
  /* onInit() is called when nodelet is launched (similar to main() in regular node) */
  virtual void onInit();

private:
  /* flags */
  bool is_initialized_ = false;

  /* ros parameters */
  std::string _uav_name_;
  bool _gui_ = true;

  // | ---------------------- subscribers --------------------- |
  image_transport::Subscriber sub_image_;
  void callbackImage(const sensor_msgs::ImageConstPtr& msg);

  // | -------------------- image processing -------------------- |
  cv::Ptr<cv::TrackerKCF> tracker_ = cv::TrackerKCF::create();
  void showTrackingImage(cv::InputArray image);
};

} // namespace eagle_track

#endif // TRACK_H
