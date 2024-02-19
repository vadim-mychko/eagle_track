#include "tracker.h"

namespace eagle_track
{

CameraContext::CameraContext(const std::string& name) : name(name) {}

void Tracker::onInit() {
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  ROS_INFO_ONCE("[Tracker]: Waiting for valid time...");
  ros::Time::waitForValid();

  // | ------------------- static parameters ------------------ |
  mrs_lib::ParamLoader pl(nh, "Tracker");
  NODELET_INFO_ONCE("[Tracker]: Loading static parameters:");
  const auto uav_name = pl.loadParam2<std::string>("UAV_NAME");
  const auto image_type = pl.loadParam2<std::string>("image_type");
  pl.loadParam("manual_detect", _manual_detect_);
  pl.loadParam("throttle_period", _throttle_period_);
  const auto image_buffer_size = pl.loadParam2<int>("image_buffer_size");

  // | ------------------- dynamic parameters ------------------ |
  drmgr_ = std::make_unique<drmgr_t>(nh, true, "Tracker", boost::bind(&Tracker::callbackConfig, this, _1, _2));

  if (!pl.loadedSuccessfully() || !drmgr_->loaded_successfully()) {
    NODELET_ERROR_ONCE("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ---------------------- camera contexts --------------------- |
  // resize buffers of camera contexts based on the provided parameter
  front_.buffer.resize(image_buffer_size);

  // | ---------------------- subscribers --------------------- |
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(image_type);

  front_.sub_image = it.subscribe("camera_front", 1, &Tracker::callbackImageFront, this, hints);
  front_.sub_info = nh.subscribe("camera_front_info", 1, &Tracker::callbackCameraInfoFront, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  front_.pub_image = it.advertise("tracker_front", 1);
  pub_projections_ = it.advertise("projections", 1);

  // | --------------------- tf transformer --------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>("Tracker");
  transformer_->setDefaultPrefix(uav_name);
  transformer_->retryLookupNewest(true);

  // | --------------------- detection through GUI --------------------- |
  if (_manual_detect_) {
    int flags = cv::WINDOW_NORMAL | cv::WINDOW_FREERATIO | cv::WINDOW_GUI_EXPANDED;
    cv::namedWindow("manual_detect", flags);
  }

  initialized_ = true;
  NODELET_INFO_ONCE("[Tracker]: Initialized");
}

void Tracker::callbackConfig([[maybe_unused]] const eagle_track::TrackParamsConfig& config, [[maybe_unused]] uint32_t level) {

}

void Tracker::callbackImageFront(const sensor_msgs::ImageConstPtr& msg) {
  callbackImage(msg, front_);
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc) {
  if (!initialized_) {
    return;
  }

  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::Mat image = bridge_image_ptr->image;

  cc.buffer.push_back({image, msg->header.stamp});

  cv::Rect2d bbox;
  bool success = cc.tracker->update(image, bbox);
  if (_manual_detect_ && !success) {
    cc.tracker = cv::TrackerCSRT::create();
    auto points = selectPoints("manual_detect", image);
    if (points.size() >= 2) {
      double min_x = cc.model.fullResolution().width;
      double min_y = cc.model.fullResolution().height;
      double max_x = 0.0;
      double max_y = 0.0;
      for (auto& point : points) {
        min_x = std::min(min_x, point.x);
        min_y = std::min(min_y, point.y);
        max_x = std::max(max_x, point.x);
        max_y = std::max(max_y, point.y);
      }

      bbox = cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
      success = cc.tracker->init(image, bbox);
    }
  } else if (cc.should_init && cc.detect_bbox.width > 1 && cc.detect_bbox.height > 1) {
    ros::Time stamp;
    {
      std::lock_guard lock(cc.mutex);
      cc.should_init = false;
      bbox = std::move(cc.detect_bbox);
      stamp = cc.stamp;
    }

    // find the closest image in terms of timestamps
    double target = stamp.toSec();
    auto closest = std::min_element(cc.buffer.begin(), cc.buffer.end(),
      [&target](const CvImageStamped& a, const CvImageStamped& b) {
        return std::abs(a.stamp.toSec() - target) < std::abs(b.stamp.toSec() - target);
    });

    cc.tracker = cv::TrackerCSRT::create();
    success = cc.tracker->init(closest->image, bbox);
    for (auto it = closest + 1; it < cc.buffer.end(); ++it) {
      success = cc.tracker->update(it->image, bbox);
    }
  }

  if (success) {
    // we don't want to modify the original image, therefore we need to copy it
    cv::Mat track_image = image.clone();
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), 2);
    publishImage(track_image, msg->header, encoding, cc.pub_image);
  } else {
    publishImage(image, msg->header, encoding, cc.pub_image);
  }
}

void Tracker::callbackCameraInfoFront(const sensor_msgs::CameraInfoConstPtr& msg) {
  callbackCameraInfo(msg, front_);
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

void Tracker::publishImage(cv::InputArray image, const std_msgs::Header& header, const std::string& encoding, image_transport::Publisher& pub) {
  // Prepare a cv_bridge image to be converted to the ROS message
  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.image = image.getMat();
  bridge_image_out.header = header;
  bridge_image_out.encoding = encoding;

  // Now convert the cv_bridge image to a ROS message and publish it
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  pub.publish(out_msg);
}

void Tracker::publishProjections(const std::vector<cv::Point2f>& projections, const CameraContext& cc) {
  if (cc.buffer.empty()) {
    return;
  }

  cv::Mat project_image = cc.buffer.back().image.clone();
  for (const cv::Point2d& point : projections) {
    cv::circle(project_image, point, 5, cv::Scalar(0, 0, 255), -1);
  }

  cv_bridge::CvImage bridge_image_out;
  bridge_image_out.image = project_image;
  bridge_image_out.encoding = "bgr8";
  sensor_msgs::ImageConstPtr out_msg = bridge_image_out.toImageMsg();
  pub_projections_.publish(out_msg);
}

void Tracker::updateDetection(const sensor_msgs::PointCloud2& points, CameraContext& cc) {
  if (!cc.got_info) {
    NODELET_WARN_STREAM_THROTTLE(_throttle_period_, "[" << cc.name << "]: Failed to transform the pointcloud to the camera frame");
    return;
  }

  // | --------- get the transformation from to the camera frame -------- |
  auto ret = transformer_->getTransform(points.header.frame_id, cc.model.tfFrame(), points.header.stamp);
  if (!ret.has_value()) {
    NODELET_WARN_STREAM_THROTTLE(_throttle_period_, "[" << cc.name << "]: Failed to transform the pointcloud to the camera frame");
    return;
  }

  // Convert PointCloud2 to pcl::PointCloud
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(points, cloud);

  // | --------- transform the pointcloud to the camera frame -------- |
  pcl_ros::transformPointCloud(cloud, cloud, ret.value().transform);

  double cam_width = cc.model.fullResolution().width;
  double cam_height = cc.model.fullResolution().height;

  double min_x = cam_width;
  double min_y = cam_height;
  double max_x = 0.0;
  double max_y = 0.0;

  std::vector<cv::Point2f> projections;
  projections.reserve(cloud.points.size());
  for (const pcl::PointXYZ& point : cloud.points) {
    if (point.z < 0) {
      continue;
    }

    // | ----------- backproject the point from 3D to 2D ---------- |
    cv::Point2d pt2d = cc.model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
    // | ----------- unrectify the 2D point coordinates ----------- |
    // This is done to correct for camera distortion. IT has no effect in simulation, where the camera model
    // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
    cv::Point2d pt2d_unrec = cc.model.unrectifyPoint(pt2d);

    // check if the projected point is inside the camera frame
    if (pt2d_unrec.x >= 0 && pt2d_unrec.x < cam_width
          && pt2d_unrec.y >= 0 && pt2d_unrec.y < cam_height) {
      min_x = std::min(min_x, pt2d_unrec.x);
      min_y = std::min(min_y, pt2d_unrec.y);
      max_x = std::max(max_x, pt2d_unrec.x);
      max_y = std::max(max_y, pt2d_unrec.y);
      projections.push_back(pt2d_unrec);
    }
  }

  {
    std::lock_guard lock(cc.mutex);
    cc.should_init = true;
    cc.detect_bbox = cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
    cc.stamp = points.header.stamp;
  }

  publishProjections(projections, cc);
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
