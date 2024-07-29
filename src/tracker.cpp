#include "tracker.h"

namespace eagle_track
{

CameraContext::CameraContext(const std::string& name) : name(name) {}

void Tracker::onInit() {
  ros::NodeHandle nh = nodelet::Nodelet::getMTPrivateNodeHandle();

  NODELET_INFO_ONCE("[Tracker]: Waiting for valid time...");
  ros::Time::waitForValid();

  // | -------------------------- static parameters ------------------------- |
  mrs_lib::ParamLoader pl(nh, "Tracker");
  NODELET_INFO_ONCE("[Tracker]: Loading static parameters:");
  const auto uav_name = pl.loadParam2<std::string>("UAV_NAME");
  const auto image_type = pl.loadParam2<std::string>("image_type");
  pl.loadParam("throttle_period", throttle_period_);
  const auto image_buffer_size = pl.loadParam2<int>("image_buffer_size");
  const auto rgbd_queue_size = pl.loadParam2<int>("rgbd_queue_size");

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
  front_.sub_depth.subscribe(it, "camera_front_depth", 1);
  front_.sync = std::make_unique<message_filters::Synchronizer<policy_t>>(policy_t(rgbd_queue_size), front_.sub_image, front_.sub_depth);
  front_.sync->registerCallback(boost::bind(&Tracker::callbackExchange, this, _1, _2, std::ref(front_), std::ref(down_)));
  front_.sub_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_front_info", 1, boost::bind(&Tracker::callbackCameraInfo, this, _1, std::ref(front_)));
  front_.sub_detection = nh.subscribe<eagle_msgs::Tracks>("detection", 1, boost::bind(&Tracker::callbackDetection, this, _1, std::ref(front_)));

  down_.sub_image.subscribe(it, "camera_down", 1, hints);
  down_.sub_depth.subscribe(it, "camera_down_depth", 1);
  down_.sync = std::make_unique<message_filters::Synchronizer<policy_t>>(policy_t(rgbd_queue_size), down_.sub_image, down_.sub_depth);
  down_.sync->registerCallback(boost::bind(&Tracker::callbackImage, this, _1, _2, std::ref(down_)));
  down_.sub_info = nh.subscribe<sensor_msgs::CameraInfo>("camera_down_info", 1, boost::bind(&Tracker::callbackCameraInfo, this, _1, std::ref(down_)));
  down_.sub_detection = nh.subscribe<eagle_msgs::Tracks>("detection", 1, boost::bind(&Tracker::callbackDetection, this, _1, std::ref(down_)));

  // | ----------------------------- publishers ----------------------------- |
  front_.pub_image = it.advertise("tracker_front", 1);
  front_.pub_projections = it.advertise("projections_front", 1);

  down_.pub_image = it.advertise("tracker_down", 1);
  down_.pub_projections = it.advertise("projections_down", 1);

  // | -------------------------- camera contexts --------------------------- |
  front_.tracker = choose_tracker(tracker_type_);
  front_.buffer.set_capacity(image_buffer_size);

  down_.tracker = choose_tracker(tracker_type_);
  down_.buffer.set_capacity(image_buffer_size);

  // | ------------------------ coordinate transforms ----------------------- |
  transformer_ = std::make_unique<mrs_lib::Transformer>("Tracker");
  transformer_->setDefaultPrefix(uav_name);
  transformer_->retryLookupNewest(true);

  initialized_ = true;
  NODELET_INFO_ONCE("[Tracker]: Initialized");
}

void Tracker::callbackConfig(const eagle_track::TrackParamsConfig& config, [[maybe_unused]] uint32_t level) {
  tracker_type_ = config.tracker_type;
  detection_points_threshold_ = config.detection_points_threshold;
}

void Tracker::callbackCameraInfo(const sensor_msgs::CameraInfoConstPtr& msg, CameraContext& cc) {
  if (!initialized_ || cc.got_camera_info) {
    return;
  }

  cc.got_camera_info = true;
  cc.model.fromCameraInfo(*msg);
  NODELET_INFO_STREAM("[" << cc.name << "]: Initialized camera info");
}

void Tracker::callbackImage(const sensor_msgs::ImageConstPtr& img_msg, [[maybe_unused]] const sensor_msgs::ImageConstPtr& depth_msg, CameraContext& cc) {
  if (!initialized_) {
    return;
  }

  constexpr double s2ms = 1000;
  const double sync_error = std::abs(img_msg->header.stamp.toSec() - depth_msg->header.stamp.toSec()) * s2ms;
  NODELET_INFO_STREAM_THROTTLE(throttle_period_, "[" << cc.name << "]: rgbd: sync_error=" << sync_error << "ms");

  cv_bridge::CvImageConstPtr img_bridge = cv_bridge::toCvShare(img_msg, "bgr8");
  cv::Mat image = img_bridge->image;
  const auto& header = img_msg->header;
  cc.buffer.push_back({image, header.stamp});

  // this if statement is doing a lot:
  // 1. checks if got a detection, and processes if needed
  // 2. if didn't get any detection, checks if got any exchanges from any other camera, and processes if needed
  // 3. if didn't get any detection nor exchange from any other camera, tries to perform the update with the current state
  //    of the tracker and the newly got image
  if (!processDetection(cc, header) && !processExchange(cc)) {
    cc.success = cc.tracker->update(image, cc.bbox);
  }

  // | ----------------------- tracking visualization ----------------------- |
  if (!cc.success) {
    publishImage(image, header, "bgr8", cc.pub_image);
  } else {
    cv::Mat track_image = image.clone();
    cv::rectangle(track_image, cc.bbox, {255, 0, 0}, 3);
    publishImage(track_image, header, "bgr8", cc.pub_image);
  }
}

void Tracker::callbackExchange(const sensor_msgs::ImageConstPtr& img_msg, const sensor_msgs::ImageConstPtr& depth_msg, CameraContext& self, CameraContext& other) {
  if (!initialized_) {
    return;
  }

  // | ------------------ perform the tracking on one camera ---------------- |
  callbackImage(img_msg, depth_msg, self);

  // | ------------- exchange the information to the second camera ---------- |
  if (self.success) {
    std::lock_guard lock(other.exchange_mutex);
    other.got_exchange = true;
    other.exchange_image = cv_bridge::toCvShare(img_msg, "bgr8")->image;
    other.exchange_depth = cv_bridge::toCvShare(depth_msg)->image;
    other.exchange_bbox = self.bbox;
    other.exchange_stamp = img_msg->header.stamp;
  }
}

void Tracker::callbackDetection(const eagle_msgs::TracksConstPtr& msg, CameraContext& cc) {
  if (!initialized_ || !cc.got_camera_info || msg->tracks.empty()) {
    return;
  }

  // | -------------------------- get the pointcloud ------------------------ |
  auto it = std::find_if(msg->tracks.cbegin(), msg->tracks.cend(), [](const eagle_msgs::Track& track) {
    return track.selected;
  });
  const auto& points = it->points;

  // i don't know why, but the same detection might be published multiple times
  // same detection == detection with the same timestamp
  // however, this can be beneficial if the newly got image is closer to the detection in terms of the timestamps
  // therefore the tracker is going to be re-initialized with the same detection and the new image, which leads to better initialization overall
  // right now, do not perform the same work twice
  {
    std::lock_guard lock(cc.detection_mutex);
    const double stamp_diff = std::abs(points.header.stamp.toSec() - cc.detection_stamp.toSec());
    if (stamp_diff < 1e-9) {
      return;
    }
  }

  // | ------------- get the transformation to the camera frame ------------- |
  auto ret = transformer_->getTransform(points.header.frame_id, cc.model.tfFrame(), points.header.stamp);
  if (!ret.has_value()) {
    NODELET_WARN_STREAM("[" << cc.name << "]: Failed to transform the pointcloud to the camera frame");
    return;
  }

  // | --------------- convert PointCloud2 to pcl::PointCloud --------------- |
  pcl::PointCloud<pcl::PointXYZ> cloud;
  pcl::fromROSMsg(points, cloud);
  if (cloud.points.empty()) {
    return;
  }

  // | ------------- transform the pointcloud to the camera frame ----------- |
  pcl_ros::transformPointCloud(cloud, cloud, ret.value().transform);

  const double cam_width = cc.model.fullResolution().width;
  const double cam_height = cc.model.fullResolution().height;

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
  std::lock_guard lock(cc.detection_mutex);
  cc.got_detection = true;
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

cv::Ptr<cv::Tracker> Tracker::choose_tracker(const int tracker_type) {
  switch (tracker_type) {
    case eagle_track::TrackParams_Boosting: return cv::TrackerBoosting::create();
    case eagle_track::TrackParams_MIL: return cv::TrackerMIL::create();
    case eagle_track::TrackParams_KCF: return cv::TrackerKCF::create();
    case eagle_track::TrackParams_TLD: return cv::TrackerTLD::create();
    case eagle_track::TrackParams_MedianFlow: {
      auto params = cv::TrackerMedianFlow::Params();
      params.maxLevel = 4;
      params.winSize = {21, 21};
      params.termCriteria = cv::TermCriteria(cv::TermCriteria::EPS, 0, 0.01);
      return cv::TrackerMedianFlow::create(params);
    }
    case eagle_track::TrackParams_GOTURN: return cv::TrackerGOTURN::create();
    case eagle_track::TrackParams_MOSSE: return cv::TrackerMOSSE::create();
    case eagle_track::TrackParams_CSRT: return cv::TrackerCSRT::create();
  }

  return nullptr;
}

bool Tracker::processDetection(CameraContext& cc, const std_msgs::Header& header) {
  // first unsafe check, later make additional check with the safely obtained variables
  if (!cc.got_detection || cc.detection_points.empty()) {
    return false;
  }

  // | -------- obtain the latest detection in a thread-safe manner --------- |
  bool got_detection;
  std::vector<cv::Point2d> points;
  ros::Time stamp;
  {
    std::lock_guard lock(cc.detection_mutex);
    got_detection = cc.got_detection;
    cc.got_detection = false;
    points = std::move(cc.detection_points);
    stamp = cc.detection_stamp;
  }

  // second SAFE check if nothing changed during allocating the needed variables!
  if (!got_detection || points.empty()
      || (cc.success && points.size() < detection_points_threshold_)) {
    return false;
  }

  // | ----------- find the closest image in terms of timestamps ------------ |
  const double target = stamp.toSec();
  auto from = std::min_element(cc.buffer.begin(), cc.buffer.end(),
    [&target](const CvImageStamped& lhs, const CvImageStamped& rhs) {
      return std::abs(lhs.stamp.toSec() - target) < std::abs(rhs.stamp.toSec() - target);
  });

  constexpr double s2ms = 1000;
  const double sync_error = std::abs(target - from->stamp.toSec()) * s2ms;
  NODELET_INFO_STREAM("[" << cc.name << "]: det: size=" << points.size() << " sync_error=" << sync_error << "ms");

  // | ---------------------- projections visualization --------------------- |
  cv::Mat projection_image = from->image.clone();
  for (const auto& point : points) {
    cv::circle(projection_image, point, 3, {0, 0, 255}, -1);
  }
  publishImage(projection_image, header, "bgr8", cc.pub_projections);

  // | ----------- transform the detection into the bounding box ------------ |
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

  // | ----------------------- initialize the tracker ----------------------- |
  cc.bbox = cv::Rect2d(min_x, min_y, max_x - min_x, max_y - min_y);
  cc.tracker = choose_tracker(tracker_type_);
  cc.success = cc.tracker->init(from->image, cc.bbox);

  // | -------- perform tracking for all images left in the buffer ---------- |
  for (auto it = from + 1; it < cc.buffer.end() && cc.success; ++it) {
    cc.success = cc.tracker->update(it->image, cc.bbox);
  }

  return true;
}

bool Tracker::processExchange(CameraContext& cc) {
  // first unsafe check, later make additional check with the safely obtained variables
  if (cc.success || !cc.got_exchange) {
    return false;
  }

  // | -------- obtain the latest exchange in a thread-safe manner ---------- |
  bool got_exchange;
  cv::Mat image;
  cv::Mat depth;
  cv::Rect2d bbox;
  ros::Time stamp;
  {
    std::lock_guard lock(cc.exchange_mutex);
    got_exchange = cc.got_exchange;
    cc.got_exchange = false;
    image = cc.exchange_image;
    depth = cc.exchange_depth;
    bbox = cc.exchange_bbox;
    stamp = cc.exchange_stamp;
  }

  // second SAFE check if nothing changed during allocating the needed variables!
  // also check if the exchanged bounding box is not too small
  constexpr double min_width = 20.0;
  constexpr double min_height = 20.0;
  if (!got_exchange || bbox.width < min_width || bbox.height < min_height) {
    return false;
  }

  // backproject the center of the bounding box in the front camera
  const cv::Point2d center(bbox.x + bbox.width / 2, bbox.y + bbox.height / 2);
  const auto ray = front_.model.projectPixelTo3dRay(center);

  // calculate the distance between the front camera's coordinate system and the target
  const cv::Point2i topleft_corner(bbox.x, bbox.y);
  const cv::Point2i botright_corner(bbox.x + bbox.width, bbox.y + bbox.height);
  std::vector<double> depths_bbox;
  for (int y = topleft_corner.y; y <= botright_corner.y; ++y) {
    for (int x = topleft_corner.x; x <= botright_corner.x; ++x) {
      constexpr double mm2m = 1e-3;
      const double depth_num = depth.at<uint16_t>({x, y}) * mm2m;
      if (depth_num > 1e-6) {
        depths_bbox.push_back(depth_num);
      }
    }
  }

  // estimate the depth at the center of the bounding box
  std::sort(depths_bbox.begin(), depths_bbox.end());
  const size_t lowerIndex = depths_bbox.size() / 4;
  const size_t upperIndex = depths_bbox.size() * 3 / 4;
  const double sum = std::accumulate(depths_bbox.begin() + lowerIndex, depths_bbox.begin() + upperIndex, 0.0);
  const double estimated_depth = sum / (upperIndex - lowerIndex);

  // complement the 3d ray with the depth information and convert to the stamped pose
  geometry_msgs::PointStamped inferred_pos;
  inferred_pos.header.frame_id = front_.model.tfFrame();
  inferred_pos.header.stamp = stamp;
  inferred_pos.point.x = ray.x;
  inferred_pos.point.y = ray.y;
  inferred_pos.point.z = estimated_depth;

  // transform the 3d point from the front camera's coordinate system into the down camera's coordinate system
  const auto ret = transformer_->transformSingle(inferred_pos, down_.model.tfFrame());
  if (!ret.has_value()) {
    NODELET_WARN_STREAM("[" << cc.name << "]: exchange: failed to transform the 3d point to the camera frame");
    return false;
  }
  const auto val = ret.value();

  // project the transformed 3d point onto the image plane of the down camera
  const auto proj = down_.model.project3dToPixel({val.point.x, val.point.y, val.point.z});
  // check if the projected point is in the bounds of the image
  const double cam_width = cc.model.fullResolution().width;
  const double cam_height = cc.model.fullResolution().height;
  if (proj.x < 0 || proj.x >= cam_width || proj.y < 0 || proj.y >= cam_height) {
    return false;
  }

  NODELET_INFO_STREAM("[" << cc.name << "]: exchange: projected 3d center point " << proj);

  // find the closest image in terms of timestamps
  const double target = stamp.toSec();
  const auto from = std::min_element(cc.buffer.begin(), cc.buffer.end(),
    [&target](const CvImageStamped& lhs, const CvImageStamped& rhs) {
      return std::abs(lhs.stamp.toSec() - target) < std::abs(rhs.stamp.toSec() - target);
  });

  // | ---------------------- projections visualization --------------------- |
  cv::Mat projection_image = from->image.clone();
  cv::circle(projection_image, proj, 6, {0, 0, 255}, -1);
  std_msgs::Header header;
  header.frame_id = cc.model.tfFrame();
  header.stamp = from->stamp;
  publishImage(projection_image, header, "bgr8", cc.pub_projections);

  // initialize the down camera tracker with the projected 3d center point and dimensions of the original bounding box
  cc.bbox = cv::Rect2d(proj.x - bbox.width / 2, proj.y - bbox.height / 2,
                       proj.x + bbox.width / 2, proj.y + bbox.height / 2);
  cc.tracker = choose_tracker(tracker_type_);
  cc.success = cc.tracker->init(from->image, cc.bbox);

  // perform tracking for all images left in the buffer
  for (auto it = from + 1; it < cc.buffer.end() && cc.success; ++it) {
    cc.success = cc.tracker->update(it->image, cc.bbox);
  }

  NODELET_INFO_STREAM("[" << cc.name << "]: exchange: initial bbox " << cc.bbox);

  return true;
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
