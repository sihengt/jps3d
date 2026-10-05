#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util_octo.h>
#include <jps3d/jps3d_frontend.hpp>

class Jps3dNode : public rclcpp::Node
{
public:
    Jps3dNode();

private:
    std::shared_ptr<JPS::OctomapMapUtil<3>> octo_map_util_;
    std::shared_ptr<Jps3dFrontend> frontend_;

    // Protects octo_map_util_/frontend_ across octomap_callback/plan_callback.
    std::mutex map_mutex_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        voxel_pub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_srv_;

    visualization_msgs::msg::Marker occ_marker_template_;
    int inflate_cell_size_ = 2;

    /**
     * @brief Bootstraps octo_map_util_ (bounds/origin/resolution/ceiling,
     * all-unknown until the first octomap message) and frontend_.
     */
    void init_map();
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void plan_callback(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);
};
