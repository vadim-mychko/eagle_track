#include "tracker.h"

namespace eagle_track
{

void Tracker::onInit() {
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO_ONCE("[Tracker]: Waiting for valid time...");
  ros::Time::waitForValid();

  // | ------------------- ros parameters ------------------ |
  mrs_lib::ParamLoader pl(nh, "Tracker");
  NODELET_INFO_ONCE("[Tracker]: Loading static parameters:");
  pl.loadParam("UAV_NAME", _uav_name_);

  if (!pl.loadedSuccessfully()) {
    NODELET_ERROR_ONCE("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  image_transport::ImageTransport it(nh);

  // | ---------------------- subscribers --------------------- |
  sub_front_ = it.subscribe("camera_front", 1, &Tracker::callbackFront, this);
  sub_front_info_ = nh.subscribe("camera_front_info", 1, &Tracker::callbackFrontInfo, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  pub_front_ = it.advertise("tracker_front", 1);

  // | --------------------- tf transformer --------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>("Tracker");
  transformer_->setDefaultPrefix(_uav_name_);
  transformer_->retryLookupNewest(true);

  initialized_ = true;
  NODELET_INFO_ONCE("[Tracker]: Initialized");
}

void Tracker::callbackFront(const sensor_msgs::ImageConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  const std::string encoding = "bgr8";
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::InputArray image = bridge_image_ptr->image;

  cv::Rect2d bbox;
  bool success = front_tracker_->update(image, bbox);

  // if could not update tracker, try re-initialize it with the latest detection
  if (!success && got_detection_) {
    bbox = projectPoints(last_detection_.points);
    success = front_tracker_->init(image, bbox);
  }

  if (success) {
    // cv::InputArray indicates that the variable should not be modified, but we want
    // to draw into the image. Therefore we need to copy it.
    cv::Mat track_image;
    image.copyTo(track_image);
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), 2, 1);

    publishFront(track_image, msg->header, encoding);
    NODELET_INFO_THROTTLE(1.0, "[Tracker]: Front tracker update succeeded");
  } else {
    publishFront(image, msg->header, encoding);
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Front tracker update failed");
  }
}

void Tracker::callbackFrontInfo(const sensor_msgs::CameraInfoConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  got_front_info_ = true;
  front_model_.fromCameraInfo(*msg);
  NODELET_INFO_ONCE("[Tracker]: Initialized front camera info");
}

void Tracker::callbackDetections(const lidar_tracker::TracksConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  if (msg->tracks.empty()) {
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Received zero detections");
    return;
  }

  for (auto track : msg->tracks) {
    if (track.selected) {
      last_detection_ = track;
      got_detection_ = true;
      break;
    }
  }
}

void Tracker::publishFront(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding) {
  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.image = image.getMat();
  bridge_image_out.header = header;
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message and publish it
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  pub_front_.publish(out_msg);
}

cv::Rect2d Tracker::projectPoints(const sensor_msgs::PointCloud2& points) {
  if (!got_front_info_) {
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Failed to transform pointcloud to the camera frame");
    return cv::Rect2d();
  }

  // | --------- transform the point to the camera frame -------- |
  auto ret = transformer_->transformSingle(points, front_model_.tfFrame());
  if (!ret.has_value()) {
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Failed to transform pointcloud to the camera frame");
    return cv::Rect2d();
  }

  // Convert PointCloud2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(points, cloud);

  // variables for creating bounding box to return
  double min_x = std::numeric_limits<double>::max();
  double min_y = std::numeric_limits<double>::max();
  double max_x = std::numeric_limits<double>::min();
  double max_y = std::numeric_limits<double>::min();

  for (const auto& point : cloud.points) {
    // | ----------- backproject the point from 3D to 2D ---------- |
    cv::Point2d pt2d = front_model_.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
    // | ----------- unrectify the 2D point coordinates ----------- |
    // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
    // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
    cv::Point2d pt2d_unrec = front_model_.unrectifyPoint(pt2d);

    // Check if the point projection is within image bounds
    if (pt2d_unrec.x >= 0 && pt2d_unrec.x < front_model_.fullResolution().width
          && pt2d_unrec.y >= 0 && pt2d_unrec.y < front_model_.fullResolution().height) {
      // Update bounding box coordinates
      min_x = std::min(min_x, pt2d_unrec.x);
      min_y = std::min(min_y, pt2d_unrec.y);
      max_x = std::max(max_x, pt2d_unrec.x);
      max_y = std::max(max_y, pt2d_unrec.y);
    }
  }

  return min_x < max_x && min_y < max_y ? cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y) : cv::Rect2d();
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
