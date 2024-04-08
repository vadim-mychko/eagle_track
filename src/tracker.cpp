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

  // | -------------------------- synchronization --------------------------- |
  front_.buffer.resize(image_buffer_size);
  down_.buffer.resize(image_buffer_size);

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
    flow_->setWinSize({config.winSizeWidth, config.winSizeHeight});
    flow_->setMaxLevel(config.maxLevel);
    cv::TermCriteria criteria(cv::TermCriteria::COUNT | cv::TermCriteria::EPS, config.maxCount, config.epsilon);
    flow_->setTermCriteria(criteria);
    flow_->setFlags((config.useInitialFlow ? cv::OPTFLOW_USE_INITIAL_FLOW : 0)
                      | (config.getMinEigenvals ? cv::OPTFLOW_LK_GET_MIN_EIGENVALS : 0));
    flow_->setMinEigThreshold(config.minEigThreshold);
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

  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::Mat image = bridge_image_ptr->image;

  cv::Mat grayscale;
  cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);

  if (cc.prev_image.empty()) {
    cc.prev_image = grayscale;
    return;
  }

  if (_manual_detect_ && cc.prev_points.empty()) {
    cc.prev_points = selectPoints("manual_detect", cc.prev_image);
  }

  if (cc.prev_points.empty()) {
    publishImage(image, msg->header, encoding, cc.pub_image);
    return;
  }

  // | --------------- perform optical flow for the two images -------------- |
  std::vector<cv::Point2f> next_points;
  std::vector<uchar> status;
  flow_->calc(cc.prev_image, grayscale, cc.prev_points, next_points, status);

  // | --------- filter points by their status after the optical flow ------- |
  std::vector<cv::Point2f> filtered_points;
  for (std::size_t i = 0; i < next_points.size(); ++i) {
    if (status[i] == 1) {
      filtered_points.push_back(next_points[i]);
    }
  }

  cc.prev_points = std::move(filtered_points);
  cc.prev_image = grayscale;

  // | ---------------------------- visualization --------------------------- |
  if (cc.prev_points.empty()) {
    publishImage(image, msg->header, encoding, cc.pub_image);
  } else {
    cv::Mat track_image = image.clone();
    for (const auto& point : cc.prev_points) {
      cv::circle(track_image, point, 3, cv::Scalar(255, 0, 0), -1);
    }

    publishImage(track_image, msg->header, encoding, cc.pub_image);
  }
}

void Tracker::callbackDetection(const lidar_tracker::TracksConstPtr& msg, CameraContext& cc) {
  if (!initialized_ || !cc.got_camera_info) {
    return;
  }

  // | -------------------------- get the pointcloud ------------------------ |
  auto it = std::find_if(msg->tracks.cbegin(), msg->tracks.cend(), [](const lidar_tracker::Track& track) {
    return track.selected;
  });
  const sensor_msgs::PointCloud2& points = it->points;

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
  std::vector<cv::Point2f> projections;
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

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
