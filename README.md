# 

## Environmental Dependence

- octomap 
- nav_msgs
- Eigen3
### Install octomap from source
[octomap-1.9.7](https://github.com/OctoMap/octomap/releases/tag/v1.9.7)

Build Only octomap (Core Library)

```bash
cd octomap
mkdir build
cd build
cmake ..
make
```

### rviz2 octomap visualization

```bash
sudo apt-get install ros-${ROS_DISTRO}-octomap-rviz-plugins
```

### Topic
Sub:  **/odometry_** 

Pub: **/localmap**