#include "tracker.h"

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
  const auto image_buffer_size = pl.loadParam2<int>("image_buffer_size");

  // | -------------------------- dynamic parameters ------------------------ |
  drmgr_ = std::make_unique<drmgr_t>(nh, true, "Tracker", boost::bind(&Tracker::callbackConfig, this, _1, _2));

  if (!pl.loadedSuccessfully() || !drmgr_->loaded_successfully()) {
    NODELET_ERROR_ONCE("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ---------------------------- subscribers ----------------------------- |
  image_transport::ImageTransport it(nh);
  image_transport::TransportHints hints(image_type);

  front_.sub_image.subscribe(it, "camera_front", 1, hints);
  front_.sub_image.registerCallback(boost::bind(&Tracker::callbackImage, this, _1, std::ref(front_)));
  front_.sub_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_front_info", 1, boost::bind(&Tracker::callbackCameraInfo, this, _1, std::ref(front_)));

  down_.sub_image.subscribe(it, "camera_down", 1, hints);
  down_.sub_image.registerCallback(boost::bind(&Tracker::callbackImage, this, _1, std::ref(down_)));
  down_.sub_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_down_info", 1, boost::bind(&Tracker::callbackCameraInfo, this, _1, std::ref(down_)));

  front_.sub_detection = nh.subscribe<lidar_tracker::Tracks>("detection", 1, boost::bind(&Tracker::callbackDetection, this, _1, std::ref(front_)));
  down_.sub_detection = nh.subscribe<lidar_tracker::Tracks>("detection", 1, boost::bind(&Tracker::callbackDetection, this, _1, std::ref(down_)));

  // | ----------------------------- publishers ----------------------------- |
  front_.pub_image = it.advertise("tracker_front", 1);
  front_.pub_projections = it.advertise("projections_front", 1);

  down_.pub_image = it.advertise("tracker_down", 1);
  down_.pub_projections = it.advertise("projections_down", 1);

  // | -------------------------- camera contexts --------------------------- |
  front_.buffer.set_capacity(image_buffer_size);
  down_.buffer.set_capacity(image_buffer_size);
  

  // | ------------------------ coordinate transforms ----------------------- |
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

void Tracker::callbackConfig(const eagle_track::TrackParamsConfig& config, uint32_t level) {
  if (level == 1) {
    tracker_type_ = config.tracker_type;
    front_.tracker = choose_tracker(tracker_type_);
    down_.tracker = choose_tracker(tracker_type_);
  }
}

void Tracker::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc) {
  if (!initialized_ || cc.got_camera_info) {
    return;
  }

  cc.got_camera_info = true;
  cc.model.fromCameraInfo(*msg);
  NODELET_INFO_STREAM("[" << cc.name << "]: Initialized camera info");
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& msg, CameraContext& cc) {
  if (!initialized_) {
    return;
  }

  // | -------------------- convert the image to BGR ------------------------ |
  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::Mat image = bridge_image_ptr->image;
  cc.buffer.push_back({image, msg->header.stamp});

  // | ---------------- prepare for processing the detection ---------------- |
  cv::Rect2d bbox;
  bool success = cc.tracker->update(image, bbox);
  bool got_detection = false;
  std::vector<cv::Point2d> points;
  auto from = cc.buffer.size() == 1 ? cc.buffer.begin() : cc.buffer.end() - 2;

  // | ----------------- get the detection points if needed ----------------- |
  if (_manual_detect_ && !success) {
    got_detection = true;
    points = selectPoints("manual_detect", image);
  } else if (cc.should_init && !cc.detection_points.empty()) {
    got_detection = true;
    // | ------- obtain the latest detection in a thread-safe manner -------- |
    ros::Time stamp;
    {
      std::lock_guard lock(cc.sync_mutex);
      cc.should_init = false;
      points = std::move(cc.detection_points);
      stamp = cc.detection_stamp;
    }

    // | ---------- find the closest image in terms of timestamps ----------- |
    double target = stamp.toSec();
    from = std::min_element(cc.buffer.begin(), cc.buffer.end(),
      [&target](const CvImageStamped& lhs, const CvImageStamped& rhs) {
        return std::abs(lhs.stamp.toSec() - target) < std::abs(rhs.stamp.toSec() - target);
    });
  }

  if (got_detection && !points.empty()) {
    // | --------------------- projections visualization -------------------- |
    cv::Mat projection_image = image.clone();
    for (const auto& point : points) {
      cv::circle(projection_image, point, 3, cv::Scalar(0, 0, 255), -1);
    }
    publishImage(projection_image, msg->header, encoding, cc.pub_projections);

    // | ---------- transform the detection into the bounding box ----------- |
    double min_x = cc.model.fullResolution().width;
    double min_y = cc.model.fullResolution().height;
    double max_x = 0.0;
    double max_y = 0.0;
    for (const auto& point : points) {
      min_x = std::min(min_x, point.x);
      min_y = std::min(min_y, point.y);
      max_x = std::max(max_x, point.x);
      max_y = std::max(max_y, point.y);
    }

    bbox = cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
    cc.tracker = choose_tracker(tracker_type_);
    success = cc.tracker->init(from->image, bbox);
  }

  // | -------- perform tracking for all images left in the buffer ---------- |
  for (auto it = from + 1; it < cc.buffer.end() && success; ++it) {
    success = cc.tracker->update(it->image, bbox);
  }

  // | ----------------------- tracking visualization ----------------------- |
  if (!success) {
    publishImage(image, msg->header, encoding, cc.pub_image);
  } else {
    cv::Mat track_image = image.clone();
    cv::rectangle(track_image, bbox, cv::Scalar(255, 0, 0), -1);
    publishImage(track_image, msg->header, encoding, cc.pub_image);
  }
}

void Tracker::callbackDetection(const lidar_tracker::TracksConstPtr& msg, CameraContext& cc) {
  if (!initialized_ || !cc.got_camera_info || msg->tracks.empty()) {
    return;
  }

  // | -------------------------- get the pointcloud ------------------------ |
  auto it = std::find_if(msg->tracks.cbegin(), msg->tracks.cend(), [](const lidar_tracker::Track& track) {
    return track.selected;
  });
  const auto& points = it->points;

  // | ------------- get the transformation to the camera frame ------------- |
  auto ret = transformer_->getTransform(points.header.frame_id, cc.model.tfFrame(), points.header.stamp);
  if (!ret.has_value()) {
    NODELET_WARN_STREAM("[" << cc.name << "]: Failed to transform the pointcloud to the camera frame");
    return;
  }

  // | --------------- convert PointCloud2 to pcl::PointCloud --------------- |
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(points, cloud);

  // | ------------- transform the pointcloud to the camera frame ----------- |
  pcl_ros::transformPointCloud(cloud, cloud, ret.value().transform);

  double cam_width = cc.model.fullResolution().width;
  double cam_height = cc.model.fullResolution().height;

  // | ------------- project the poincloud onto the camera plane ------------ |
  std::vector<cv::Point2d> projections;
  projections.reserve(cloud.points.size());
  for (const auto& point : cloud.points) {
    if (point.z < 0) {
      continue;
    }

    // | ---------------- backproject the point from 3D to 2D --------------- |
    cv::Point2d pt2d = cc.model.project3dToPixel(cv::Point3d(point.x, point.y, point.z));
    // | ---------------- unrectify the 2D point coordinates ---------------- |
    // This is done to correct for camera distortion. It has no effect in simulation, where the camera model
    // is an ideal pinhole camera, but usually has a BIG effect when using real cameras, so don't forget this part!
    cv::Point2d pt2d_unrec = cc.model.unrectifyPoint(pt2d);

    // check if the projected point is inside the camera frame
    if (pt2d_unrec.x >= 0 && pt2d_unrec.x < cam_width && pt2d_unrec.y >= 0 && pt2d_unrec.y < cam_height) {
      projections.push_back(pt2d_unrec);
    }
  }

  // | ---------------------- update the camera context --------------------- |
  std::lock_guard lock(cc.sync_mutex);
  cc.should_init = true;
  cc.detection_points = std::move(projections);
  cc.detection_stamp = points.header.stamp;
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

cv::Ptr<cv::Tracker> Tracker::choose_tracker(const std::string& tracker_type) {
  if (tracker_type == "Boosting") {
    return cv::TrackerBoosting::create();
  } else if (tracker_type == "MIL") {
    return cv::TrackerMIL::create();
  } else if (tracker_type == "KCF") {
    return cv::TrackerKCF::create();
  } else if (tracker_type == "TLD") {
    return cv::TrackerTLD::create();
  } else if (tracker_type == "MedianFlow") {
    return cv::TrackerMedianFlow::create();
  } else if (tracker_type == "GOTURN") {
    return cv::TrackerGOTURN::create();
  } else if (tracker_type == "MOSSE") {
    return cv::TrackerMOSSE::create();
  } else if (tracker_type == "CSRT") {
    return cv::TrackerCSRT::create();
  }

  return nullptr;
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
