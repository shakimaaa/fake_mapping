#include "local_btmap/localmap_from_bt.hpp"

LocalmapFromBt::LocalmapFromBt() : Node("localmap_from_bt") {
    map_bt_path_ = this->declare_parameter("bt.map_bt_path", "/home/beauhowe/my_project/ENV/octomap-1.9.7/octomap/share/data/geb079.bt");
    fov_horizontal_ = this->declare_parameter("bt.fov_horizontal", 60.0);
    fov_vertical_ = this->declare_parameter("bt.fov_vertical", 45.0);
    max_range_ = this->declare_parameter("bt.max_range", 10.0);
    resolution_ = this->declare_parameter("bt.resolution", 0.05);
    m_Expansion_range_x = this->declare_parameter("bt.m_Expansion_range_x", 0.5);
    m_Expansion_range_y = this->declare_parameter("bt.m_Expamsion_range_y", 0.5);
    m_Expansion_range_z = this->declare_parameter("bt.m_Expansion_range_z", 0.5);
    m_isoccupiedThresh = this->declare_parameter("bt.m_isoccupiedThresh", 0.7);

    RCLCPP_INFO(this->get_logger(), "map_bt_path: %s", map_bt_path_.c_str());
    RCLCPP_INFO(this->get_logger(), "fov_horizontal: %f", fov_horizontal_);
    RCLCPP_INFO(this->get_logger(), "fov_vertical: %f", fov_vertical_);
    RCLCPP_INFO(this->get_logger(), "max_range: %f", max_range_);
    RCLCPP_INFO(this->get_logger(), "resolution: %f", resolution_);
    RCLCPP_INFO(this->get_logger(), "m_Expansion_range_x: %f", m_Expansion_range_x);
    RCLCPP_INFO(this->get_logger(), "m_Expamsion_range_y: %f", m_Expansion_range_y);
    RCLCPP_INFO(this->get_logger(), "m_Expansion_range_z: %f", m_Expansion_range_z);
    RCLCPP_INFO(this->get_logger(), "m_isoccupiedThresh: %f", m_isoccupiedThresh);

    if (!loadMap()) {
        RCLCPP_ERROR(this->get_logger(), "Failed to load map");
        return;
    }

    odomtery_sub_ = this->create_subscription<nav_msgs::msg::Odometry>("/odometry_", 10, 
        std::bind(&LocalmapFromBt::odometryCallback, this, std::placeholders::_1));
    
    map_pub_ = this->create_publisher<octomap_msgs::msg::Octomap>("/localmap", 10);
}

bool LocalmapFromBt::loadMap() {
    std::unique_ptr<octomap::AbstractOcTree> abs(
        octomap::AbstractOcTree::read(map_bt_path_));
    if (!abs) {
        return false;
    }
    // make sure it is OcTree
    if (auto* tree = dynamic_cast<octomap::OcTree*>(abs.get())) {
        global_tree_.reset(new octomap::OcTree(*tree));  // copy
        resolution_ = global_tree_->getResolution();
        local_tree_ = std::make_shared<octomap::OcTree>(resolution_);
        inflated_octree_ = std::make_shared<octomap::OcTree>(*local_tree_);
        return true;
    }
    return false;
}

std::vector<octomap::point3d> LocalmapFromBt::calculateFOVRays(const octomap::point3d& sensor_origin, 
                                                                const nav_msgs::msg::Odometry& msg) {
    std::vector<octomap::point3d> rays;

    double x = msg.pose.pose.orientation.x, y = msg.pose.pose.orientation.y, z = msg.pose.pose.orientation.z, w = msg.pose.pose.orientation.w;

    double siny_cosp = 2 * (w * z + x * y);
    double cosy_cosp = 1 - 2 * (y * y + z * z);
    double yaw = std::atan2(siny_cosp, cosy_cosp);

    double sinp = 2 * (w * y - z * x);
    double pitch = 0.0;
    if (std::abs(sinp) >= 1) {
        pitch = std::copysign(M_PI / 2, sinp);
    } else {
        pitch = std::asin(sinp);
    }
    int horizontal_steps = static_cast<int>(fov_horizontal_ / 10.0);  // 10度步长
    int vertical_steps = static_cast<int>(fov_vertical_ / 10.0);

    for (int i = -horizontal_steps/2; i <= horizontal_steps/2; ++i) {
        for (int j = -vertical_steps/2; j <= vertical_steps/2; ++j) {
            double yaw_offset = i * 10.0 * M_PI / 180.0;    // 转换为弧度
            double pitch_offset = j * 10.0 * M_PI / 180.0;  // 转换为弧度
            
            // 计算全局坐标系中的方向
            double current_yaw = yaw + yaw_offset;
            double current_pitch = pitch + pitch_offset;
            
            // 方向向量
            double x_dir = cos(current_pitch) * cos(current_yaw);
            double y_dir = cos(current_pitch) * sin(current_yaw);
            double z_dir = sin(current_pitch);
            
            // 计算射线终点
            octomap::point3d ray_end = sensor_origin + octomap::point3d(
                x_dir * max_range_,
                y_dir * max_range_,
                z_dir * max_range_
            );
            
            rays.push_back(ray_end);
        }
    }
    return rays;
}

void LocalmapFromBt::updateLocalMap(const nav_msgs::msg::Odometry& msg) {
    octomap::point3d sensor_origin(
        msg.pose.pose.position.x,
        msg.pose.pose.position.y,
        msg.pose.pose.position.z
    );

    std::vector<octomap::point3d> fov_rays = calculateFOVRays(sensor_origin, msg);

    for (const auto& ray_end : fov_rays) {
        octomap::point3d direction = ray_end - sensor_origin;
        if (direction.norm() <= 1e-9) continue;
        direction.normalize();

        octomap::point3d occupied_point;
        const bool hit = global_tree_->castRay(sensor_origin, direction, occupied_point,/*ignoreUnknownCells=*/true, max_range_);
        const double total_dist = (ray_end - sensor_origin).norm();
        for (double d = resolution_; d < total_dist; d += resolution_) {
            octomap::point3d ray_point = sensor_origin +  direction * d;
            local_tree_->updateNode(ray_point, false, /*lazy_eval=*/true);
        }

        if (hit) {
            local_tree_->updateNode(occupied_point, true, /*lazy_eval=*/true);
        }
    }

    local_tree_->updateInnerOccupancy();

}

void LocalmapFromBt::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    // if (!loadMap()) return;
    updateLocalMap(*msg);
    Inflated_octree();
    publishMap(global_tree_);
}

void LocalmapFromBt::Inflated_octree() {
    // inflated_octree_->setResolution(resolution_);
    // inflated_octree_->updateInnerOccupancy();
    inflated_octree_ = std::make_shared<octomap::OcTree>(*local_tree_);
    for (auto it = local_tree_->begin_leafs(); it != local_tree_->end_leafs(); ++it)
    {
        if (local_tree_->isNodeOccupied(*it))
        {
            octomap::point3d occupied_point = it.getCoordinate();
            for ( double dx = -m_Expansion_range_x; dx <= m_Expansion_range_x; dx += resolution_ )
            {
                for ( double dy = -m_Expansion_range_y; dy <= m_Expansion_range_y; dy += resolution_ )
                {
                    for ( double dz = -m_Expansion_range_z; dz <= m_Expansion_range_z; dz += resolution_ )
                    {
                        octomap::point3d inflated_point(occupied_point.x() + dx, 
                                                        occupied_point.y() + dy, 
                                                        occupied_point.z() + dz);
                        inflated_octree_->updateNode(inflated_point, true);
                    }
                }
            }
        }
    }
}

void LocalmapFromBt::publishMap(const std::shared_ptr<octomap::OcTree>& octree)
{
    if (!octree || octree->size() == 0)
    {
        RCLCPP_WARN(this->get_logger(), "Octree is null");
        return;
    }
    
    octomap_msgs::msg::Octomap map;
    map.header.stamp = this->get_clock()->now();
    map.header.frame_id = "world"; 
    const auto& tree_ref = *octree; 
    if (octomap_msgs::binaryMapToMsg(tree_ref, map)) {
        // Publishing OctoMap messages
        // RCLCPP_INFO(node_->get_logger(), "publish octobinaryMap!");
        map_pub_->publish(map);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Error serializing OctoMap");
    }
}

bool LocalmapFromBt::getInflateOccupancy(const Eigen::Vector3d& pos) {
  // 0) sanity check: do we even have an inflated tree?
  if (!inflated_octree_) {
    RCLCPP_ERROR(this->get_logger(),
      "getInflateOccupancy(): inflated octree not set");
    return false;  
  }

  // 1) convert to OctoMap point
  float x = static_cast<float>(pos.x());
  float y = static_cast<float>(pos.y());
  float z = static_cast<float>(pos.z());
  octomap::point3d p{x, y, z};

  // 2) search for the leaf
  auto* node = inflated_octree_->search(p);
  if (!node) {
    // no leaf → treat as “free” (or adjust as your logic needs)
    return false;
  }

  // 3) compare against your threshold
  return (node->getOccupancy() >= m_isoccupiedThresh);
}

double LocalmapFromBt::getResolution() const {
  return local_tree_->getResolution();
}