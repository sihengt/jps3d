#include <cmath>
#include <limits>
#include <memory>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

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
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr                   path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr  voxel_pub_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr      voxel_sub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr         octomap_sub_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr                  plan_srv_;

    visualization_msgs::msg::Marker occ_marker_template;

    // Three-state occupancy snapshot kept alongside the planner's (optimistic) map.
    // val_unknown_ (-1) = never observed, val_free_ (0) = observed free, val_occ_ (100) = occupied.
    // Used to truncate plans at the first unknown/occupied cell so the planner only ever
    // commits to observed-free space (frontier-toward-goal). Guarded by map_mutex_.
    JPS::Tmap true_map_;
    static constexpr int8_t val_free_    = 0;
    static constexpr int8_t val_occ_     = 100;
    static constexpr int8_t val_unknown_ = -1;
    bool   block_unknown_      = true;
    double frontier_seed_radius_ = 0.6;
    int inflate_cell_size_;

    /**
     * @brief initializes map, planner, and sets map within planner class.
     * 
     * Initializes map data (JPS::Tmap), map_util_ (JPS::VoxelMapUtil) as well as a 
     * planner (JPSPlanner3D). Sets the map util within the planner and update the map 
     * within the planner.
     */
    void init_map();
    void voxel_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void plan_callback(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request>  request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response>       response);
};