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
  pl.loadParam("world_frame_id", _world_frame_id_);

  if (!pl.loadedSuccessfully()) {
    NODELET_ERROR_ONCE("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  image_transport::ImageTransport it(nh);

  // | ---------------------- subscribers --------------------- |
  sub_front_ = it.subscribe("camera_front", 1, &Tracker::callbackFront, this);
  sub_front_info_ = nh.subscribe("camera_front_info", 1, &Tracker::callbackFrontInfo, this);

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

  NODELET_INFO_THROTTLE(1.0, "[Tracker]: Processing image from front camera");

  const std::string encoding = "bgr8";
  const cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::InputArray image = bridge_image_ptr->image;

  cv::Rect2d bbox;
  bool success = front_tracker_->update(image, bbox);
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

cv::Point2d Tracker::projectPoint(const cv::Point3d& point) {
  // | --------- transform the point to the camera frame -------- |
  geometry_msgs::PoseStamped pt3d_world;
  pt3d_world.header.frame_id = _world_frame_id_;
  pt3d_world.header.stamp = ros::Time::now();
  pt3d_world.pose.position.x = point.x;
  pt3d_world.pose.position.y = point.y;
  pt3d_world.pose.position.z = point.z;

  std::string camera_frame = front_model_.tfFrame();
  auto ret = transformer_->transformSingle(pt3d_world, camera_frame);
  geometry_msgs::PoseStamped pt3d_cam;
  if (ret) {
    pt3d_cam = ret.value();
  } else {
    NODELET_WARN_THROTTLE(1.0, "[Tracker]: Failed to transform point from world to camera frame, cannot project point");
    return cv::Point2d();
  }

  // | ----------- backproject the point from 3D to 2D ---------- |
  cv::Point3d pt3d(pt3d_cam.pose.position.x, pt3d_cam.pose.position.y, pt3d_cam.pose.position.z);
  cv::Point2d pt2d = front_model_.project3dToPixel(pt3d);

  // | ----------- unrectify the 2D point coordinates ----------- |
  // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
  // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
  cv::Point2d pt2d_unrec = front_model_.unrectifyPoint(pt2d);

  return pt2d_unrec;
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
