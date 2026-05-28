#include "jps3d/jps3d_node.hpp"

Jps3dNode::Jps3dNode() : Node("jps3d_node")
{
    // Declare all parameters
    this->declare_parameter<double>("map_origin_x", -30.0);
    this->declare_parameter<double>("map_origin_y", -30.0);
    this->declare_parameter<double>("map_origin_z", 0.0);
    this->declare_parameter<double>("map_size_x", 60.0);
    this->declare_parameter<double>("map_size_y", 60.0);
    this->declare_parameter<double>("map_size_z", 20.0);
    this->declare_parameter<double>("map_resolution", 0.1); // .05 for super
    this->declare_parameter<double>("eps", 1.0);
    this->declare_parameter<bool>("use_jps", true);
    this->declare_parameter<std::string>("voxel_sub_topic", "/octomap_point_cloud_centers");
    this->declare_parameter<std::string>("path_pub_topic", "/global_plan");
    this->declare_parameter<std::string>("plan_srv_topic", "/plan");

// Initialize the map and planner
    init_map();

    // Get relevant topics
    std::string voxel_sub_topic = this->get_parameter("voxel_sub_topic").as_string();
    std::string path_pub_topic  = this->get_parameter("path_pub_topic").as_string();
    std::string plan_srv_topic  = this->get_parameter("plan_srv_topic").as_string();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic, rclcpp::QoS(10));
    voxel_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("jps3d_voxels" , rclcpp::QoS(10));
    // Subscription: /occupied_voxels, depth 10
    voxel_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
        voxel_sub_topic, 10,
        std::bind(&Jps3dNode::voxel_callback, this, std::placeholders::_1));

    // Service: /plan
    plan_srv_ = this->create_service<nav_msgs::srv::GetPlan>(
        plan_srv_topic,
        std::bind(&Jps3dNode::plan_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

    auto dim = map_util_->getDim();
    RCLCPP_INFO(this->get_logger(),
                "JPS3D node ready. Map dim: [%d, %d, %d], resolution: %.3f m",
                dim(0), dim(1), dim(2), map_util_->getRes());
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

    if (res <= 0.0) {
        RCLCPP_FATAL(this->get_logger(), "map_resolution must be > 0, got %f", res);
        throw std::invalid_argument("map_resolution must be positive");
    }

    Vec3f origin(ox, oy, oz);
    Vec3i dim(
        static_cast<int>(std::ceil(sx / res)),
        static_cast<int>(std::ceil(sy / res)),
        static_cast<int>(std::ceil(sz / res)));
    
    // Initialize data with zeros (Tmap = std::vector<signed char>)
    JPS::Tmap data(static_cast<size_t>(dim(0)) * dim(1) * dim(2), 0);

    map_util_ = std::make_shared<JPS::VoxelMapUtil>();
    map_util_->setMap(origin, dim, data, res);

    // constructor takes in verbosity boolean
    planner_ = std::make_shared<JPSPlanner3D>(true);
    planner_->setMapUtil(map_util_);
    planner_->updateMap();

    // Hard-coding occupied voxels to always be red.
    std_msgs::msg::ColorRGBA occupied_color;
    occupied_color.r = 1.0;
    occupied_color.g = 0.0;
    occupied_color.b = 0.0;
    occupied_color.a = 0.5;

    // Initializing occupied marker scale
    geometry_msgs::msg::Vector3 scale;
    scale.x = res;
    scale.y = res;
    scale.z = res;

    // Setting up marker templates for occupied cells.
    occ_marker_template.pose.orientation.w = 1;
    occ_marker_template.type = visualization_msgs::msg::Marker::CUBE;
    occ_marker_template.action = visualization_msgs::msg::Marker::ADD;
    occ_marker_template.scale = scale;
    occ_marker_template.color = occupied_color;
}

void Jps3dNode::voxel_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    auto origin = map_util_->getOrigin();
    auto dim    = map_util_->getDim();
    double res  = map_util_->getRes();

    // Reset map to all-free
    JPS::Tmap data(static_cast<size_t>(dim(0)) * dim(1) * dim(2), 0);

    sensor_msgs::PointCloud2ConstIterator<float> iter_x(*msg, "x");
    sensor_msgs::PointCloud2ConstIterator<float> iter_y(*msg, "y");
    sensor_msgs::PointCloud2ConstIterator<float> iter_z(*msg, "z");

    int marked = 0;
    for (; iter_x != iter_x.end(); ++iter_x, ++iter_y, ++iter_z) {
        Vec3f pt(*iter_x, *iter_y, *iter_z);
        Vec3i pn = map_util_->floatToInt(pt);
        if (map_util_->isOutside(pn)) continue;
        data[map_util_->getIndex(pn)] = 100;
        ++marked;
    }

    map_util_->setMap(origin, dim, data, res);

    // Inflate obstacles by drone radius so the planner treats the drone as a point.
    // inflate_cells = ceil(drone_radius / res). At 0.1 m/cell and ~0.2 m radius use 2.
    constexpr int inflate_cells = 2;
    vec_Veci<3> neighbors;
    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx)
        for (int dy = -inflate_cells; dy <= inflate_cells; ++dy)
            for (int dz = -inflate_cells; dz <= inflate_cells; ++dz)
                if (dx || dy || dz)
                    neighbors.push_back(Vec3i(dx, dy, dz));
    map_util_->dilate(neighbors);

    planner_->updateMap();

    // Publish a single CUBE_LIST marker instead of one marker per voxel.
    // This is orders of magnitude faster to render in RViz.
    visualization_msgs::msg::Marker cube_list;
    cube_list.header      = msg->header;
    cube_list.ns          = "jps3d_voxels";
    cube_list.id          = 0;
    cube_list.type        = visualization_msgs::msg::Marker::CUBE_LIST;
    cube_list.action      = visualization_msgs::msg::Marker::ADD;
    cube_list.scale       = occ_marker_template.scale;
    cube_list.color       = occ_marker_template.color;
    cube_list.pose.orientation.w = 1.0;

    auto occ = map_util_->getCloud();
    cube_list.points.reserve(occ.size());
    for (const auto & p : occ) {
        geometry_msgs::msg::Point gp;
        gp.x = p(0); gp.y = p(1); gp.z = p(2);
        cube_list.points.push_back(gp);
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(cube_list);
    voxel_pub_->publish(markers);

    RCLCPP_DEBUG(this->get_logger(),
        "Voxel map updated: %d raw, %zu after inflation", marked, occ.size());
}

void Jps3dNode::plan_callback(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request>  request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response>       response)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    Vec3f start(
        request->start.pose.position.x,
        request->start.pose.position.y,
        request->start.pose.position.z);
    Vec3f goal(
        request->goal.pose.position.x,
        request->goal.pose.position.y,
        request->goal.pose.position.z);

    // Quick check to see if frame_ids have been set on start / goal. No point doing the rest otherwise.
    std::string frame_id = request->start.header.frame_id.empty()
        ? request->goal.header.frame_id
        : request->start.header.frame_id;
    
    if (frame_id.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "frame_id not set on start or goal");
        return;
    }

    double eps     = this->get_parameter("eps").as_double();
    bool use_jps   = this->get_parameter("use_jps").as_bool();

    // Plan
    bool success = planner_->plan(start, goal, eps, use_jps);
    
    // Set to fail if planner was unable to obtain a plan
    if (!success) {
        RCLCPP_WARN(this->get_logger(),
                    "Planning failed. Planner status: %d", static_cast<int>(planner_->status()));
        return;
    }

    // If plan succeeds extract plan, populate response, and publish
    auto path_pts = planner_->getPath();
    nav_msgs::msg::Path path_msg;

    rclcpp::Time stamp = this->now();
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp    = stamp;

    for (const auto & pt : path_pts) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id    = frame_id;
        pose.header.stamp       = stamp;
        pose.pose.position.x    = pt(0);
        pose.pose.position.y    = pt(1);
        pose.pose.position.z    = pt(2);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    response->plan = path_msg;
    path_pub_->publish(path_msg);
}

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Jps3dNode>());
    rclcpp::shutdown();
    return 0;
}
