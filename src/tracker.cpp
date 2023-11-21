#include "tracker.h"

namespace eagle_track
{

CameraContext::CameraContext(const std::string& name) : name(name) {}

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
  front_.sub_image = it.subscribe("camera_front", 1, &Tracker::callbackImageFront, this);
  front_.sub_info = nh.subscribe("camera_front_info", 1, &Tracker::callbackCameraInfoFront, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  front_.pub_image = it.advertise("tracker_front", 1);
  pub_projections_ = it.advertise("projections", 1);

  // | --------------------- tf transformer --------------------- |
  transformer_ = mrs_lib::Transformer("Tracker");
  transformer_.setDefaultPrefix(_uav_name_);
  transformer_.retryLookupNewest(true);

  initialized_ = true;
  NODELET_INFO_ONCE("[Tracker]: Initialized");
}

void Tracker::callbackImageFront(const sensor_msgs::ImageConstPtr& msg) {
  callbackImage(msg, front_);
}

void Tracker::callbackImageDown(const sensor_msgs::ImageConstPtr& msg) {
  callbackImage(msg, down_);
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc) {
  if (!initialized_) {
    return;
  }

  cc.buffer.push_back(msg);

  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::InputArray image = bridge_image_ptr->image;

  cv::Rect2d bbox;
  bool success = true;
  bool shouldInit = !cc.tracker->update(image, bbox);
  if (shouldInit) {
    ros::Time stamp;
    {
      std::lock_guard<std::mutex> lock(cc.mutex);
      bbox = cc.detection;
      stamp = cc.stamp;
    }
    success = bbox != cv::Rect2d() && initContext(cc, bbox, stamp);
  }

  if (shouldInit && !success) {
    NODELET_WARN_STREAM_THROTTLE(1.0, "[" << cc.name << "]: Reinitialization failed with " << bbox);
  }

  if (success) {
    // cv::InputArray indicates that the variable should not be modified, but we want
    // to draw into the image. Therefore we need to copy it.
    cv::Mat track_image;
    image.copyTo(track_image);
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), 2);
    publishImage(track_image, msg->header, encoding, cc);
  } else {
    publishImage(image, msg->header, encoding, cc);
  }
}

void Tracker::callbackCameraInfoFront(const sensor_msgs::CameraInfoConstPtr& msg) {
  callbackCameraInfo(msg, front_);
}

void Tracker::callbackCameraInfoDown(const sensor_msgs::CameraInfoConstPtr& msg) {
  callbackCameraInfo(msg, down_);
}

void Tracker::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc) {
  if (!initialized_ || cc.got_info) {
    return;
  }

  cc.got_info = true;
  cc.model.fromCameraInfo(*msg);
  NODELET_INFO_STREAM_ONCE("[" << cc.name << "]: Initialized camera info");
}

void Tracker::callbackDetections(const lidar_tracker::TracksConstPtr& msg) {
  if (!initialized_) {
    return;
  }

  for (auto track : msg->tracks) {
    if (track.selected) {
      updateDetection(track.points, front_);
      break;
    }
  }
}

void Tracker::publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, CameraContext& cc) {
  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.image = image.getMat();
  bridge_image_out.header = header;
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message and publish it
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  cc.pub_image.publish(out_msg);
}

void Tracker::publishProjections(const std::vector<cv::Point2d>& projections, const CameraContext& cc) {
  if (cc.buffer.empty()) {
    return;
  }

  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(cc.buffer.back(), encoding);
  cv::Mat project_image;
  bridge_image_ptr->image.copyTo(project_image);

  for (const cv::Point2d& point : projections) {
    cv::circle(project_image, point, 5, cv::Scalar(0, 0, 255), -1);
  }

  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.image = project_image;
  bridge_image_out.encoding = encoding;
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  pub_projections_.publish(out_msg);
}

bool Tracker::initContext(CameraContext& cc, cv::Rect2d& bbox, const ros::Time& stamp) {
  if (cc.buffer.empty()) {
    return false;
  }

  // find the closest image in terms of timestamps
  double target = stamp.toSec();
  auto closest = std::min_element(cc.buffer.begin(), cc.buffer.end(),
    [&target](const sensor_msgs::ImageConstPtr& a, const sensor_msgs::ImageConstPtr& b) {
      return std::abs(a->header.stamp.toSec() - target) < std::abs(b->header.stamp.toSec() - target);
  });

  // reinitialize our tracker with the found image frame with the closest timestamp and bounding box
  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(*closest, encoding);
  bool success = cc.tracker->init(bridge_image_ptr->image, bbox);

  // iterate over all other images and update our tracker
  for (auto it = closest + 1; it != cc.buffer.end(); ++it) {
    if (!success) { 
      break;
    }

    bridge_image_ptr = cv_bridge::toCvShare(*it, encoding);
    success = cc.tracker->update(bridge_image_ptr->image, bbox);
  }

  return success;
}

void Tracker::updateDetection(const sensor_msgs::PointCloud2& points, CameraContext& cc) {
  if (!cc.got_info) {
    NODELET_WARN_STREAM_THROTTLE(1.0, "[" << cc.name << "]: Failed to transform the pointcloud to the camera frame");
    return;
  }

  // | --------- get the transformation from to the camera frame -------- |
  auto ret = transformer_.getTransform(points.header.frame_id, cc.model.tfFrame(), points.header.stamp);
  if (!ret.has_value()) {
    NODELET_WARN_STREAM_THROTTLE(1.0, "[" << cc.name << "]: Failed to transform the pointcloud to the camera frame");
    return;
  }

  // Convert PointCloud2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(points, cloud);

  // | --------- transform the pointcloud to the camera frame -------- |
  pcl_ros::transformPointCloud(cloud, cloud, ret.value().transform);

  double cam_width = cc.model.fullResolution().width;
  double cam_height = cc.model.fullResolution().height;

  // variables for creating bounding box to return
  double min_x = cam_width;
  double min_y = cam_height;
  double max_x = 0.0;
  double max_y = 0.0;

  std::vector<cv::Point2d> projections;
  for (const pcl::PointXYZ& point : cloud.points) {
    // | ----------- backproject the point from 3D to 2D ---------- |
    cv::Point2d pt2d = cc.model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
    // | ----------- unrectify the 2D point coordinates ----------- |
    // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
    // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
    cv::Point2d pt2d_unrec = cc.model.unrectifyPoint(pt2d);

    // check if the projected point is inside the camera frame
    if (pt2d_unrec.x >= 0 && pt2d_unrec.x < cam_width
          && pt2d_unrec.y >= 0 && pt2d_unrec.y < cam_height) {
      // Update bounding box coordinates
      min_x = std::min(min_x, pt2d_unrec.x);
      min_y = std::min(min_y, pt2d_unrec.y);
      max_x = std::max(max_x, pt2d_unrec.x);
      max_y = std::max(max_y, pt2d_unrec.y);

      projections.push_back(pt2d_unrec);
    }
  }

  double width = max_x - min_x;
  double height = max_y - min_y;

  if (width > 0 && height > 0) {
    std::lock_guard<std::mutex> lock(cc.mutex);
    cc.detection = cv::Rect2d(min_x, min_y, width, height);
    cc.stamp = points.header.stamp;
  }

  publishProjections(projections, cc);
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
