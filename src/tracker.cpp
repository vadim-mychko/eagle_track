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
  pl.loadParam("throttle_period", _throttle_period_);
  const auto image_buffer_size = pl.loadParam2<int>("image_buffer_size");

  if (!pl.loadedSuccessfully()) {
    NODELET_ERROR_ONCE("[Tracker]: Failed to load non-optional parameters!");
    ros::shutdown();
  }

  // | ---------------------- camera contexts --------------------- |
  // resize buffers of camera contexts based on the provided parameter
  front_.buffer.resize(image_buffer_size);

  // | ---------------------- subscribers --------------------- |
  image_transport::ImageTransport it(nh);

  front_.sub_image = it.subscribe("camera_front", 1, &Tracker::callbackImageFront, this);
  front_.sub_info = nh.subscribe("camera_front_info", 1, &Tracker::callbackCameraInfoFront, this);
  sub_detections_ = nh.subscribe("detections", 1, &Tracker::callbackDetections, this);

  // | ---------------------- publishers --------------------- |
  front_.pub_image = it.advertise("tracker_front", 1);
  pub_projections_ = it.advertise("projections", 1);

  // | --------------------- tf transformer --------------------- |
  transformer_ = mrs_lib::Transformer("Tracker");
  transformer_.setDefaultPrefix(uav_name);
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

  const std::string encoding = "bgr8";
  cv_bridge::CvImageConstPtr bridge_image_ptr = cv_bridge::toCvShare(msg, encoding);
  cv::Mat image = bridge_image_ptr->image;

  // store the last bgr image for visualization of projections onto the camera frame
  cc.prev_frame = image;

  // push grayscale image into the buffer
  cv::Mat grayscale;
  cv::cvtColor(image, grayscale, cv::COLOR_BGR2GRAY);
  cc.buffer.push_back({grayscale, msg->header.stamp});

  // if there is only one image inside buffer (including current image pushed), then
  // do not perform any tracking nor visualization
  if (cc.buffer.size() <= 1) {
    return;
  }

  auto from = cc.buffer.end() - 2;
  if (cc.should_init && !cc.points.empty()) {
    ros::Time stamp;
    {
      std::lock_guard lock(cc.mutex);
      cc.should_init = false;
      cc.points = std::move(cc.detect_points);
      stamp = cc.stamp;
    }

    // find the closest image in terms of timestamps
    double target = stamp.toSec();
    auto closest = std::min_element(cc.buffer.begin(), cc.buffer.end(),
      [&target](const CvImageStamped& a, const CvImageStamped& b) {
        return std::abs(a.stamp.toSec() - target) < std::abs(b.stamp.toSec() - target);
    });

    // starting image has to be before the last image in the buffer
    from = closest == cc.buffer.end() - 1 ? closest - 1 : closest;
  }

  // initialize data structure here for further visualization (updated at the last iteration)
  cv::Mat matched_image;

  // iterate over remaining images in the buffer
  for (auto it = from; !cc.points.empty() && it < cc.buffer.end() - 1; ++it) {
    auto prev = it;
    auto curr = it + 1;

    // | --------- perform optical flow for the two images -------- |
    std::vector<cv::Point2f> next_points;
    std::vector<uchar> status;
    std::vector<float> err;
    cv::calcOpticalFlowPyrLK(prev->image, curr->image, cc.points, next_points, status, err);

    // | --------- filter points by their status after the optical flow -------- |
    std::vector<cv::Point2f> filtered_points;
    for (std::size_t i = 0; i < next_points.size(); ++i) {
      if (status[i] == 1) {
        filtered_points.push_back(next_points[i]);
      }
    }

    cc.points = std::move(filtered_points);

    // // | --------- convert points from to keypoints -------- |
    // std::vector<cv::KeyPoint> keypoints;
    // std::vector<cv::KeyPoint> next_keypoints;
    // cv::KeyPoint::convert(cc.points, keypoints);
    // cv::KeyPoint::convert(filtered_points, next_keypoints);

    // // | --------- compute descriptors for each vector of keypoints -------- |
    // cv::Mat descriptors;
    // cv::Mat next_descriptors;
    // orb_->compute(prev->image, keypoints, descriptors);
    // orb_->compute(curr->image, next_keypoints, next_descriptors);

    // // | --------- match descriptors between the previous and next image -------- |
    // std::vector<std::vector<cv::DMatch>> knn_matches;
    // matcher_->knnMatch(descriptors, next_descriptors, knn_matches, 2);

    // // | --------- filter matches based on Lowe's ratio test -------- |
    // constexpr float ratio_thresh = 0.7f;
    // std::vector<cv::DMatch> good_matches;
    // for (size_t i = 0; i < knn_matches.size(); i++) {
    //   if (knn_matches[i][0].distance < ratio_thresh * knn_matches[i][1].distance) {
    //       good_matches.push_back(knn_matches[i][0]);
    //   }
    // }

    // // if the last iteration, update visualization data structure
    // if (curr == cc.buffer.end() - 1) {
    //   cv::drawMatches(prev->image, keypoints, curr->image, next_keypoints, good_matches, matched_image);
    // }

    // // | --------- extract points from matched points for the next iteration -------- |
    // cc.points.clear();
    // for (const auto& match : good_matches) {
    //   cc.points.push_back(next_keypoints[match.trainIdx].pt);
    // }
  }

  if (cc.points.empty()) {
    publishImage(image, msg->header, encoding, cc);
  } else {
    // we don't want to modify the original image, therefore we need to copy it
    cv::Mat track_image = image.clone();
    for (const auto& point : cc.points) {
      cv::circle(track_image, point, 5, cv::Scalar(255, 0, 0), -1);
    }

    publishImage(track_image, msg->header, encoding, cc);
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

void Tracker::publishProjections(const std::vector<cv::Point2f>& projections, const CameraContext& cc) {
  if (cc.prev_frame.empty()) {
    return;
  }

  cv::Mat project_image = cc.prev_frame.clone();
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
  auto ret = transformer_.getTransform(points.header.frame_id, cc.model.tfFrame(), points.header.stamp);
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
      projections.push_back(pt2d_unrec);
    }
  }

  {
    std::lock_guard lock(cc.mutex);
    cc.should_init = true;
    cc.detect_points = projections;
    cc.stamp = points.header.stamp;
  }

  publishProjections(projections, cc);
}

} // namespace eagle_track

// every nodelet must include macros which export the class as a nodelet plugin
#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(eagle_track::Tracker, nodelet::Nodelet);
