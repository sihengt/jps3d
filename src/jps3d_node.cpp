#include "jps3d/jps3d_node.hpp"

Jps3dNode::Jps3dNode() : Node("jps3d_node")
{
    this->declare_parameter<double>("map_origin_x", -20.0);
    this->declare_parameter<double>("map_origin_y", -20.0);
    this->declare_parameter<double>("map_origin_z", 0.0);
    this->declare_parameter<double>("map_size_x", 40.0);
    this->declare_parameter<double>("map_size_y", 40.0);
    this->declare_parameter<double>("map_size_z", 10.0);
    this->declare_parameter<double>("map_resolution", 0.1);
    this->declare_parameter<double>(
        "ceiling_height_z", std::numeric_limits<double>::infinity());
    this->declare_parameter<double>("eps", 1.0);
    this->declare_parameter<bool>("use_jps", true);
    this->declare_parameter<std::string>("path_pub_topic", "/global_plan");
    this->declare_parameter<std::string>("plan_srv_topic", "/plan");
    this->declare_parameter<std::string>("octomap_sub_topic", "/octomap_full");
    this->declare_parameter<bool>("block_unknown", true);
    this->declare_parameter<double>("frontier_seed_radius", 0.3);
    this->declare_parameter<int>("inflate_cell_size", 2);

    init_map();

    std::string octomap_sub_topic =
        this->get_parameter("octomap_sub_topic").as_string();
    std::string path_pub_topic = this->get_parameter("path_pub_topic").as_string();
    std::string plan_srv_topic = this->get_parameter("plan_srv_topic").as_string();
    inflate_cell_size_ = this->get_parameter("inflate_cell_size").as_int();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic,
                                                             rclcpp::QoS(10));
    voxel_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "jps3d_voxels", rclcpp::QoS(10));

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        octomap_sub_topic, 10,
        std::bind(&Jps3dNode::octomap_callback, this, std::placeholders::_1));

    plan_srv_ = this->create_service<nav_msgs::srv::GetPlan>(
        plan_srv_topic,
        std::bind(&Jps3dNode::plan_callback, this, std::placeholders::_1,
                  std::placeholders::_2));

    auto dim = octo_map_util_->getDim();
    RCLCPP_INFO(this->get_logger(),
                "JPS3D node ready. Map dim: [%d, %d, %d], resolution: %.3f m, "
                "ceiling: %.2f m",
                dim(0), dim(1), dim(2), octo_map_util_->getRes(),
                this->get_parameter("ceiling_height_z").as_double());
}

void Jps3dNode::init_map()
{
    double ox = this->get_parameter("map_origin_x").as_double();
    double oy = this->get_parameter("map_origin_y").as_double();
    double oz = this->get_parameter("map_origin_z").as_double();
    double sx = this->get_parameter("map_size_x").as_double();
    double sy = this->get_parameter("map_size_y").as_double();
    double sz = this->get_parameter("map_size_z").as_double();
    double res = this->get_parameter("map_resolution").as_double();

    if (res <= 0.0)
    {
        RCLCPP_FATAL(this->get_logger(), "map_resolution must be > 0, got %f",
                    res);
        throw std::invalid_argument("map_resolution must be positive");
    }

    Vec3f origin(ox, oy, oz);
    Vec3i dim(static_cast<int>(std::ceil(sx / res)),
             static_cast<int>(std::ceil(sy / res)),
             static_cast<int>(std::ceil(sz / res)));

    // All-unknown until the first octomap message arrives.
    std::vector<signed char> data(
        static_cast<size_t>(dim(0)) * dim(1) * dim(2), -1);

    octo_map_util_ = std::make_shared<JPS::OctomapMapUtil<3>>();
    octo_map_util_->setMap(origin, dim, data, res);

    double ceiling_z = this->get_parameter("ceiling_height_z").as_double();
    if (std::isfinite(ceiling_z) && ceiling_z <= oz)
        RCLCPP_WARN(this->get_logger(),
                    "ceiling_height_z (%.2f) is at or below map_origin_z "
                    "(%.2f); the entire map will be blocked",
                    ceiling_z, oz);
    octo_map_util_->setCeiling(ceiling_z);

    bool block_unknown = this->get_parameter("block_unknown").as_bool();
    double frontier_seed_radius =
        this->get_parameter("frontier_seed_radius").as_double();
    frontend_ = std::make_shared<Jps3dFrontend>(
        octo_map_util_, /*verbose=*/true, block_unknown, frontier_seed_radius);
    frontend_->updateMap();

    std_msgs::msg::ColorRGBA occupied_color;
    occupied_color.r = 1.0;
    occupied_color.g = 0.0;
    occupied_color.b = 0.0;
    occupied_color.a = 0.5;

    geometry_msgs::msg::Vector3 scale;
    scale.x = res;
    scale.y = res;
    scale.z = res;

    occ_marker_template_.pose.orientation.w = 1;
    occ_marker_template_.type = visualization_msgs::msg::Marker::CUBE;
    occ_marker_template_.action = visualization_msgs::msg::Marker::ADD;
    occ_marker_template_.scale = scale;
    occ_marker_template_.color = occupied_color;
}

void Jps3dNode::octomap_callback(
    const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    std::unique_ptr<octomap::AbstractOcTree> abstract(
        octomap_msgs::fullMsgToMap(*msg));
    if (!abstract)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "octomap_callback: failed to deserialize %s",
                            this->get_parameter("octomap_sub_topic")
                                .as_string()
                                .c_str());
        return;
    }
    auto *tree = dynamic_cast<octomap::OcTree *>(abstract.get());
    if (!tree)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "octomap_callback: octree is not an OcTree");
        return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    octo_map_util_->updateFromOctree(tree);
    octo_map_util_->dilateByRadius(inflate_cell_size_);
    frontend_->updateMap();

    visualization_msgs::msg::Marker cube_list;
    cube_list.header = msg->header;
    cube_list.ns = "jps3d_voxels";
    cube_list.id = 0;
    cube_list.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cube_list.action = visualization_msgs::msg::Marker::ADD;
    cube_list.scale = occ_marker_template_.scale;
    cube_list.color = occ_marker_template_.color;
    cube_list.pose.orientation.w = 1.0;

    auto occ = octo_map_util_->getCloud();
    cube_list.points.reserve(occ.size());
    for (const auto &p : occ)
    {
        geometry_msgs::msg::Point gp;
        gp.x = p(0);
        gp.y = p(1);
        gp.z = p(2);
        cube_list.points.push_back(gp);
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(cube_list);
    voxel_pub_->publish(markers);

    RCLCPP_DEBUG(this->get_logger(), "Octomap map updated: %zu occupied cells",
                occ.size());
}

void Jps3dNode::plan_callback(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    Vec3f start(request->start.pose.position.x, request->start.pose.position.y,
               request->start.pose.position.z);
    Vec3f goal(request->goal.pose.position.x, request->goal.pose.position.y,
              request->goal.pose.position.z);

    std::string frame_id = request->start.header.frame_id.empty()
                              ? request->goal.header.frame_id
                              : request->start.header.frame_id;
    if (frame_id.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "frame_id not set on start or goal");
        return;
    }

    double eps = this->get_parameter("eps").as_double();
    bool use_jps = this->get_parameter("use_jps").as_bool();

    vec_Vec3f path_pts;
    if (!frontend_->planPath(start, goal, eps, use_jps, path_pts))
    {
        RCLCPP_WARN(this->get_logger(), "Planning failed (status %d)",
                   frontend_->status());
        return;
    }

    nav_msgs::msg::Path path_msg;
    rclcpp::Time stamp = this->now();
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = stamp;

    for (const auto &pt : path_pts)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = stamp;
        pose.pose.position.x = pt(0);
        pose.pose.position.y = pt(1);
        pose.pose.position.z = pt(2);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    response->plan = path_msg;
    path_pub_->publish(path_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Jps3dNode>());
    rclcpp::shutdown();
    return 0;
}
