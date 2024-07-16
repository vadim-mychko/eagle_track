# (Multi)-Camera Tracking

Package for extending [VoFOD](https://github.com/ctu-mrs/vofod) with visual input from multiple cameras.

## Installation

1. Install [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).
2. Create a catkin workspace with the following packages with the specified commits known to work:
   * [vofod](https://github.com/ctu-mrs/vofod) @46abe2fbe2d6b8fc027127d7a621bbf2c0040036
   * [lidar_tracker](https://github.com/ctu-mrs/lidar_tracker) @7c7454e986f3ad0ca84d11df447a531cfc7732fd
   **Note:** here you might want to set the *downsample_leaf_size* to 0.0 meters in the config/tracking.yaml for proper tracking initialization
   * [eagle_gazebo_resources](https://mrs.fel.cvut.cz/gitlab/eagle_one/eagle_gazebo_resources) @00915f919ee6d84080a160cd3ae29669d9b6da97
   * and eagle_track (any branch **without rosbag** in the name should work)
3. Start the simulation script ./tmux/simulation/start.sh.

## Branches

| Branch                  | Description |
|:------------------------|:------------|
| **flow**                | Main branch for integrating the LK tracker (refactored flow_old), no multi-camera exchanges (yet)
| flow_old                | Initial work on integrating the LK tracker, no multi-camera exchanges, mainly used as a reference point for newer branches, if after refactoring something does not work            
| flow_rosbag             | Flow branch for working with rosbags, stale, no multi-camera exchanges, mainly changes the camera subscribers' topics
| bbox_old                | Initial work on integraing bounding box-based trackers, no multi-camera exchanges, mainly used a reference point if something doesn't work after refactoring
| bbox                    | Refactored bbox_old branch, but **not** the main branch used for bounding box-based trackers, no multi-camera exchanges
| bbox_revised            | (Initial) multi-camera exchange logic, adding information to the bbox branch
| **bbox_improved**       | **Latest** work on the bounding box trackers, based on the bbox_revised work. Look here for gaining an idea how things work
| **bbox_improved_depth** | Another latest branch based off bbox_improved, but experiments with depth realsense cameras, implements synchronization logic
| bbox_rosbag_analysis    | Branch for working with rosbags, contains hard-coded paths for saving pictures from cameras, nothing interesting here
| bbox_rosbag_geom        | Branch for trying out the geometry of the pinhole camera model for exchanging information (unsuccessful), this branch is a joke
| bbox_rosbag_nano        | Branch for trying to use Nano tracker from OpenCV 4.9.0 (unsuccessful). It requires building dependencies that depend on the OpenCV with the OpenCV 4.9.0 + compiling the eagle_track package with OpenCV 4.9.0. Compiled successfully, however crashes during runtime

## Interception Trajectory Generation

Might be outdated information, but the following packages are needed to start the interception algorithm in the simulation:
**Also need to add *points* to the eagle_msgs Track.msg message like this:**
sensor_msgs/PointCloud2   points
Also set downsample_leaf_size in the lidar_tracker to 0.0

* [eagle_gazebo_resources](https://mrs.fel.cvut.cz/gitlab/eagle_one/eagle_gazebo_resources) @00915f919ee6d84080a160cd3ae29669d9b6da97
* [eagle_msgs](https://mrs.fel.cvut.cz/gitlab/eagle_one/eagle_msgs) @55ae047c015c217ba4556189bcb6af4000b22f53
* [vofod](https://mrs.fel.cvut.cz/gitlab/eagle_one/vofod) @cfe3a8037950898c600bae7ed23d76f701e10563
* [lidar_tracker](https://mrs.fel.cvut.cz/gitlab/kratot13/my-awesome-lidar) @be4e29877d3370fed91f2643a6e663bdc6b42c02
* [interceptor](https://mrs.fel.cvut.cz/gitlab/vrbamato/interceptor) @3a27a7865fe6cad799315a0fa9d77a6c226ffd51
* [mrs_uav_controllers](https://github.com/ctu-mrs/mrs_uav_controllers) @0113264f8ccb9239a8050d702588693abc1aad72
