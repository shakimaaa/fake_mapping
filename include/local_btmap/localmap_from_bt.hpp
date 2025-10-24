#pragma once

#define OCTOMAP_NODEBUGOUT

#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>
#include <octomap/octomap.h>
#include <octomap/OcTree.h>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <nav_msgs/msg/odometry.hpp>

#include <memory>
#include <string>
#include <vector>



class LocalmapFromBt : public rclcpp::Node
{
public:
    LocalmapFromBt();
private:
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr msg);
    bool loadMap();
    std::vector<octomap::point3d> calculateFOVRays(const octomap::point3d& sensor_origin, 
                                                    const nav_msgs::msg::Odometry& msg);
    void updateLocalMap(const nav_msgs::msg::Odometry& msg);

    void Inflated_octree();
    void publishMap(const std::shared_ptr<octomap::OcTree>& octree);
    bool getInflateOccupancy(const Eigen::Vector3d& pos);
    double getResolution() const;

    
    std::string map_bt_path_;
    double fov_horizontal_;
    double fov_vertical_;
    double max_range_;
    double resolution_;
    double m_Expansion_range_x;
    double m_Expansion_range_y;
    double m_Expansion_range_z;
    double m_isoccupiedThresh;

    // octomap
    std::shared_ptr<octomap::OcTree>  local_tree_;
    std::shared_ptr<octomap::OcTree>  global_tree_;
    std::shared_ptr<octomap::OcTree>  inflated_octree_;
    
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomtery_sub_;
    rclcpp::Publisher<octomap_msgs::msg::Octomap>::SharedPtr map_pub_;
};