#include "jps3d/jps3d_node.hpp"

Jps3dNode::Jps3dNode() : Node("jps3d_node")
{
    // Declare all parameters
    this->declare_parameter<double>("map_origin_x", -20.0);
    this->declare_parameter<double>("map_origin_y", -20.0);
    this->declare_parameter<double>("map_origin_z", 0.0);
    this->declare_parameter<double>("map_size_x", 40.0);
    this->declare_parameter<double>("map_size_y", 40.0);
    this->declare_parameter<double>("map_size_z", 10.0);
    this->declare_parameter<double>("map_resolution", 0.1); // .05 for super
    this->declare_parameter<double>(
        "ceiling_height_z", std::numeric_limits<double>::infinity());
    this->declare_parameter<double>("eps", 1.0);
    this->declare_parameter<bool>("use_jps", true);
    this->declare_parameter<std::string>("voxel_sub_topic", "/octomap_point_cloud_centers");
    this->declare_parameter<std::string>("path_pub_topic", "/global_plan");
    this->declare_parameter<std::string>("plan_srv_topic", "/plan");
    // Frontier-toward-goal: source the full octree (occupied + free + unknown) so we can refuse to
    // commit to paths through never-observed space. octomap_sub_topic="" keeps the old occupied-only
    // /octomap_point_cloud_centers behavior (block_unknown has no effect then).
    this->declare_parameter<std::string>("octomap_sub_topic", "/octomap_full");
    this->declare_parameter<bool>("block_unknown", true);
    this->declare_parameter<double>("frontier_seed_radius", 0.3);
    this->declare_parameter<int>("inflate_cell_size", 2);

// Initialize the map and planner
    init_map();

    // Get relevant topics
    std::string voxel_sub_topic   = this->get_parameter("voxel_sub_topic").as_string();
    std::string octomap_sub_topic = this->get_parameter("octomap_sub_topic").as_string();
    std::string path_pub_topic    = this->get_parameter("path_pub_topic").as_string();
    std::string plan_srv_topic    = this->get_parameter("plan_srv_topic").as_string();
    inflate_cell_size_ = this->get_parameter("inflate_cell_size").as_int();
    block_unknown_        = this->get_parameter("block_unknown").as_bool();
    frontier_seed_radius_ = this->get_parameter("frontier_seed_radius").as_double();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic, rclcpp::QoS(10));
    voxel_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("jps3d_voxels" , rclcpp::QoS(10));

    // Prefer the full octree (3-state: occupied/free/unknown) so the planner can refuse to route
    // through never-observed space. Fall back to the occupied-only point cloud if no octomap topic.
    if (!octomap_sub_topic.empty()) {
        octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
            octomap_sub_topic, 10,
            std::bind(&Jps3dNode::octomap_callback, this, std::placeholders::_1));
        RCLCPP_INFO(this->get_logger(),
                    "Using octomap '%s' (block_unknown=%d, seed_radius=%.2f m)",
                    octomap_sub_topic.c_str(), (int)block_unknown_, frontier_seed_radius_);
    } else {
        voxel_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            voxel_sub_topic, 10,
            std::bind(&Jps3dNode::voxel_callback, this, std::placeholders::_1));
    }

    // Service: /plan
    plan_srv_ = this->create_service<nav_msgs::srv::GetPlan>(
        plan_srv_topic,
        std::bind(&Jps3dNode::plan_callback, this,
                    std::placeholders::_1, std::placeholders::_2));

    auto dim = map_util_->getDim();
    RCLCPP_INFO(this->get_logger(),
                "JPS3D node ready. Map dim: [%d, %d, %d], resolution: %.3f m, "
                "ceiling: %.2f m",
                dim(0), dim(1), dim(2), map_util_->getRes(),
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

    double ceiling_z = this->get_parameter("ceiling_height_z").as_double();
    if (std::isfinite(ceiling_z) && ceiling_z <= oz)
        RCLCPP_WARN(this->get_logger(),
                    "ceiling_height_z (%.2f) is at or below map_origin_z "
                    "(%.2f); the entire map will be blocked",
                    ceiling_z, oz);
    map_util_->setCeiling(ceiling_z);

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
    vec_Veci<3> neighbors;
    for (int dx = -inflate_cell_size_; dx <= inflate_cell_size_; ++dx)
        for (int dy = -inflate_cell_size_; dy <= inflate_cell_size_; ++dy)
            for (int dz = -inflate_cell_size_; dz <= inflate_cell_size_; ++dz)
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

void Jps3dNode::octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    // Deserialize the full octree (carries occupied / free / unknown, unlike the occupied-only cloud).
    std::unique_ptr<octomap::AbstractOcTree> abstract(octomap_msgs::fullMsgToMap(*msg));
    if (!abstract) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "octomap_callback: failed to deserialize /octomap_full");
        return;
    }
    auto* tree = dynamic_cast<octomap::OcTree*>(abstract.get());
    if (!tree) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                             "octomap_callback: octree is not an OcTree");
        return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);
    auto   origin = map_util_->getOrigin();
    auto   dim    = map_util_->getDim();
    double res    = map_util_->getRes();
    const size_t n = static_cast<size_t>(dim(0)) * dim(1) * dim(2);

    // optimistic = what the planner searches on: unknown treated as free (lets JPS find a goal-ward
    // path even through unseen space). true_map_ = unknown(-1)/free(0)/occ(100), used afterwards to
    // truncate that path at the first unobserved cell.
    JPS::Tmap optimistic(n, val_free_);
    true_map_.assign(n, val_unknown_);

    int n_occ = 0, n_free = 0;
    for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end; ++it) {
        const bool occ = tree->isNodeOccupied(*it);
        const double s = it.getSize();                 // leaves can be coarser than our grid
        const octomap::point3d c = it.getCoordinate();
        const Vec3i lo = map_util_->floatToInt(Vec3f(c.x() - s / 2, c.y() - s / 2, c.z() - s / 2));
        const Vec3i hi = map_util_->floatToInt(Vec3f(c.x() + s / 2, c.y() + s / 2, c.z() + s / 2));
        for (int x = lo(0); x <= hi(0); ++x)
            for (int y = lo(1); y <= hi(1); ++y)
                for (int z = lo(2); z <= hi(2); ++z) {
                    Vec3i pn(x, y, z);
                    if (map_util_->isOutside(pn)) continue;
                    const int idx = map_util_->getIndex(pn);
                    if (occ) {
                        optimistic[idx] = val_occ_;
                        true_map_[idx]  = val_occ_;
                        ++n_occ;
                    } else if (true_map_[idx] != val_occ_) {
                        true_map_[idx] = val_free_;     // observed free (optimistic already free)
                        ++n_free;
                    }
                }
    }

    map_util_->setMap(origin, dim, optimistic, res);

    // Inflate obstacles by drone radius so the planner can treat the drone as a point (same as before).
    constexpr int inflate_cells = 2;
    vec_Veci<3> neighbors;
    for (int dx = -inflate_cells; dx <= inflate_cells; ++dx)
        for (int dy = -inflate_cells; dy <= inflate_cells; ++dy)
            for (int dz = -inflate_cells; dz <= inflate_cells; ++dz)
                if (dx || dy || dz)
                    neighbors.push_back(Vec3i(dx, dy, dz));
    map_util_->dilate(neighbors);

    planner_->updateMap();

    // Visualize occupied cells (single CUBE_LIST), same as voxel_callback.
    visualization_msgs::msg::Marker cube_list;
    cube_list.header              = msg->header;
    cube_list.ns                  = "jps3d_voxels";
    cube_list.id                  = 0;
    cube_list.type                = visualization_msgs::msg::Marker::CUBE_LIST;
    cube_list.action              = visualization_msgs::msg::Marker::ADD;
    cube_list.scale               = occ_marker_template.scale;
    cube_list.color               = occ_marker_template.color;
    cube_list.pose.orientation.w  = 1.0;
    auto occ = map_util_->getCloud();
    cube_list.points.reserve(occ.size());
    for (const auto& p : occ) {
        geometry_msgs::msg::Point gp;
        gp.x = p(0); gp.y = p(1); gp.z = p(2);
        cube_list.points.push_back(gp);
    }
    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(cube_list);
    voxel_pub_->publish(markers);

    RCLCPP_DEBUG(this->get_logger(),
        "Octomap map updated: occ_cells=%d free_cells=%d (rest unknown)", n_occ, n_free);
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

    // Plan on the (optimistic) map.
    bool success = planner_->plan(start, goal, eps, use_jps);

    vec_Vec3f path_pts;
    if (success) {
        path_pts = planner_->getPath();
    } else {
        // Goal not free / unreachable (e.g. it sits in unobserved or occupied space). Rather than give
        // up, head straight at the goal and let the frontier truncation below cut the line at the first
        // unknown/occupied cell -- i.e. get as close to the goal as observed-free space allows.
        RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
            "plan() failed (status %d); greedy straight-line toward goal, truncated at frontier.",
            static_cast<int>(planner_->status()));
        path_pts.push_back(start);
        path_pts.push_back(goal);
    }

    // Frontier-toward-goal: the path above was found on the optimistic map (unknown=free), so it may
    // run through never-observed space. Walk it from the start and cut it at the first cell that is
    // unknown or occupied in the true 3-state map -> the drone only ever commits to observed-free
    // space, advances to the frontier, observes more, and replans. A small seed radius around the
    // start is always treated as free so the drone's own (possibly unobserved) cell can't stall it.
    if (block_unknown_ && !true_map_.empty() && path_pts.size() >= 2) {
        const double step = std::max(0.5 * map_util_->getRes(), 1e-3);
        auto cell_state = [&](const Vec3f & p) -> int8_t {
            Vec3i pn = map_util_->floatToInt(p);
            if (map_util_->isOutside(pn)) return val_unknown_;   // off-map == never observed
            return true_map_[map_util_->getIndex(pn)];
        };
        decltype(path_pts) truncated;
        truncated.push_back(path_pts.front());
        bool cut = false;
        for (size_t i = 0; i + 1 < path_pts.size() && !cut; ++i) {
            const Vec3f a = path_pts[i], b = path_pts[i + 1];
            const double L = (b - a).norm();
            const int steps = std::max(1, static_cast<int>(std::ceil(L / step)));
            for (int s = 1; s <= steps; ++s) {
                const Vec3f p = a + (b - a) * (static_cast<double>(s) / steps);
                const bool seeded = (p - start).norm() <= frontier_seed_radius_;
                const int8_t st = cell_state(p);
                if (!seeded && (st == val_unknown_ || st >= val_occ_)) { cut = true; break; }
                truncated.push_back(p);
            }
        }
        if (cut) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                "Plan truncated at frontier: %zu->%zu pts, frontier=(%.2f,%.2f,%.2f)",
                path_pts.size(), truncated.size(),
                truncated.back()(0), truncated.back()(1), truncated.back()(2));
        }
        path_pts = truncated;
    }

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
