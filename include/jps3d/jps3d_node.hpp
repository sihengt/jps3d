#include <cmath>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>
#include <jps_planner/jps_planner/jps_planner.h>

class Jps3dNode : public rclcpp::Node
{
public:
    Jps3dNode();

private:
    // Map and planner objects
    std::shared_ptr<JPS::VoxelMapUtil> map_util_;
    std::shared_ptr<JPSPlanner3D>      planner_;

    // To protect shared map/planner state
    std::mutex map_mutex_;

    // ROS interfaces
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr              path_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr voxel_sub_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr             plan_srv_;

    void init_map();
    void voxel_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void plan_callback(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request>  request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response>       response);
};