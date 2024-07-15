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

| Branch               | Description |
|:---------------------|:------------|
| **flow**             | Main branch for integrating the LK tracker (refactored flow_old), no multi-camera exchanges (yet)
| flow_old             | Initial work on integrating the LK tracker, no multi-camera exchanges, mainly used as a reference point for newer branches, if after refactoring something does not work            
| flow_rosbag          | Flow branch for working with rosbags, stale, no multi-camera exchanges, mainly changes the camera subscribers' topics
| bbox_old             | Initial work on integraing bounding box-based trackers, no multi-camera exchanges, mainly used a reference point if something doesn't work after refactoring
| bbox                 | Refactored bbox_old branch, but **not** the main branch used for bounding box-based trackers, no multi-camera exchanges
| bbox_revised         | (Initial) multi-camera exchange logic, adding information to the bbox branch
| **bbox_improved**    | **Latest** work on the bounding box trackers, based on the bbox_revised work. Look here for gaining an idea how things work
| bbox_rosbag_analysis | Branch for working with rosbags, contains hard-coded paths for saving pictures from cameras, nothing interesting here
| bbox_rosbag_geom     | Branch for trying out the geometry of the pinhole camera model for exchanging information (unsuccessful), this branch is a joke
| bbox_rosbag_nano     | Branch for trying to use Nano tracker from OpenCV 4.9.0 (unsuccessful). It requires building dependencies that depend on the OpenCV with the OpenCV 4.9.0 + compiling the eagle_track package with OpenCV 4.9.0. Compiled successfully, however crashes during runtime
