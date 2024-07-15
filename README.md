# (Multi)-Camera Tracking

Package for extending [VoFOD](https://github.com/ctu-mrs/vofod) with visual input from multiple cameras.

## Installation

1. Install [MRS UAV System](https://github.com/ctu-mrs/mrs_uav_system).
2. Create a catkin workspace with the following packages with the specified commits known to work:
   * [vofod](https://github.com/ctu-mrs/vofod) @46abe2fbe2d6b8fc027127d7a621bbf2c0040036
   * [lidar_tracker](https://github.com/ctu-mrs/lidar_tracker) @7c7454e986f3ad0ca84d11df447a531cfc7732fd
   * [eagle_gazebo_resources](https://mrs.fel.cvut.cz/gitlab/eagle_one/eagle_gazebo_resources) @00915f919ee6d84080a160cd3ae29669d9b6da97
   * and eagle_track (any branch should work)
3. Start the simulation script [./tmux/simulation/start.sh](tmux/simulation/start.sh).

## Branches

| Branch               | Description |
|:---------------------|:------------|
| flow                 | 
| flow_old             | Initial work on integrating the LK tracker             
| flow_rosbag          |
| bbox_old             |
| bbox                 |
| bbox_revised         |
| bbox_improved        |
| bbox_rosbag_analysis |
| bbox_rosbag_geom     |
| bbox_rosbag_nano     |
