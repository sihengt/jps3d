#include "read_map.hpp"
#include "timer.hpp"
#include <chrono>
#include <jps_basis/data_utils.h>
#include <jps_collision/map_util_super.h>
#include <jps_planner/jps_planner/jps_planner.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <rog_map/rog_map.h>
#include <vector>

using namespace JPS;

// Create a minimal non-ROS implementation of ROGMap
class NonRosROGMap : public rog_map::ROGMap
{
public:
    explicit NonRosROGMap(const std::string &cfg_path)
    {
        cfg_ = rog_map::Config(cfg_path);
        init();
        // ROGMap::init() only slides the prob/inf maps to fix_map_origin when
        // map sliding is disabled; the ESDF map's bounds are left at zero, so
        // updateESDF3D() would compute an empty box. Slide it here.
        if (cfg_.esdf_en)
            esdf_map_->mapSliding(cfg_.fix_map_origin);
    }
    ~NonRosROGMap() override = default;

    void rebuildESDF()
    {
        if (cfg_.esdf_en)
            esdf_map_->updateESDF3D(cfg_.fix_map_origin);
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr saveMap()
    {
        // Get local map bounds
        auto bounds = getLocalMapBounds();
        rog_map::vec_E<rog_map::Vec3f> occ_pts;

        // Search for occupied points in the map
        boxSearch(bounds.first, bounds.second, super_utils::OCCUPIED, occ_pts);

        // Convert to PCL point cloud and save to PCD file
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud =
            std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
        cloud->reserve(occ_pts.size());
        for (const auto &pt : occ_pts)
        {
            cloud->push_back(
                pcl::PointXYZRGB(pt.x(), pt.y(), pt.z(), 255, 255, 255));
        }
        pcl::io::savePCDFileBinary("rog_map_occ.pcd", *cloud);
        printf("Saved %zu occupied points to rog_map_occ.pcd\n",
               occ_pts.size());
        return cloud;
    }

private:
    const double getSystemWalltimeNow() override
    {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now.time_since_epoch()).count();
    }
};

void inflateSinglePoint(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud,
                        const pcl::PointXYZ &point, double radius,
                        uint8_t r_color, uint8_t g_color, uint8_t b_color)
{
    // Fix the number of points for a simple, quick shell
    const int numberOfPoints = 120;
    const double goldenRatio = (1.0 + std::sqrt(5.0)) / 2.0;

    for (int i = 0; i < numberOfPoints; ++i)
    {
        // Uniformly distribute points over the sphere surface
        double t = static_cast<double>(i) / numberOfPoints;
        double phi = std::acos(1.0 - 2.0 * t);
        double theta = 2.0 * M_PI * i / goldenRatio;

        // Create the new surface point
        pcl::PointXYZRGB newPoint;
        newPoint.x = point.x + radius * std::sin(phi) * std::cos(theta);
        newPoint.y = point.y + radius * std::sin(phi) * std::sin(theta);
        newPoint.z = point.z + radius * std::cos(phi);

        // Apply your color
        newPoint.r = r_color;
        newPoint.g = g_color;
        newPoint.b = b_color;

        // Add to cloud
        cloud->push_back(newPoint);
    }

    // Quick metadata update
    cloud->width = cloud->size();
    cloud->height = 1;
}

/**
 * @brief Templated function to add any vector of Eigen points (float or double)
 * to an XYZRGB cloud.
 * * @tparam DerivedEigenVector The Eigen vector type (e.g., Eigen::Vector3f or
 * Eigen::Vector3d)
 * @tparam Allocator The vector's memory allocator type
 */
template <typename DerivedEigenVector, typename Allocator>
void addPathToCloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
    const std::vector<DerivedEigenVector, Allocator> &eigen_points,
    uint8_t r = 255, uint8_t g = 255, uint8_t b = 255)
{
    if (!cloud)
    {
        cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>());
    }
    cloud->reserve(cloud->size() + eigen_points.size());

    for (const auto &eigen_pt : eigen_points)
    {
        pcl::PointXYZRGB pcl_pt;

        // .template cast<float>() safely handles the double-to-float conversion
        // if your input uses doubles, and does nothing if it's already floats.
        pcl_pt.x = static_cast<float>(eigen_pt.x());
        pcl_pt.y = static_cast<float>(eigen_pt.y());
        pcl_pt.z = static_cast<float>(eigen_pt.z());

        pcl_pt.r = r;
        pcl_pt.g = g;
        pcl_pt.b = b;

        cloud->push_back(pcl_pt);
    }

    cloud->width = cloud->size();
    cloud->height = 1;
    cloud->is_dense = true;
}

/**
 * @brief Walk a 3D path and report ESDF-based obstacle clearance statistics.
 *
 * Consecutive waypoints farther apart than `max_seg_len` are ray-cast and
 * interpolated so that every ESDF cell the path crosses is sampled, not just
 * the cells nearest to the original waypoints.
 */
void analyzePathClearance(const vec_Vecf<3> &path, const std::string &path_name,
                          const std::shared_ptr<ROGMapUtil<3>> &map_util,
                          double max_seg_len = -1.0)
{
    if (path.empty())
    {
        printf("[%s] empty path, nothing to analyze\n", path_name.c_str());
        return;
    }

    if (max_seg_len <= 0.0)
    {
        max_seg_len = map_util->getRes();
    }

    double sum_dist = 0.0;
    double min_dist = std::numeric_limits<double>::max();
    Vec3f min_dist_pos = Vec3f::Zero();
    int num_samples = 0;
    int num_outside = 0;
    int num_unknown = 0;

    const Tmap &esdf = map_util->map_;

    auto sampleAt = [&](const Vec3f &pt)
    {
        Vec3i idx = map_util->floatToInt(pt);
        if (map_util->isOutside(idx))
        {
            num_outside++;
            return;
        }
        int hash_idx = map_util->getIndex(idx);
        if (map_util->isUnknown(hash_idx))
        {
            num_unknown++;
            return;
        }
        double d = esdf[hash_idx];
        sum_dist += d;
        num_samples++;
        if (d < min_dist)
        {
            min_dist = d;
            min_dist_pos = pt;
        }
    };

    sampleAt(path.front());
    for (size_t i = 1; i < path.size(); ++i)
    {
        const Vec3f &p0 = path[i - 1];
        const Vec3f &p1 = path[i];
        double seg_len = (p1 - p0).norm();

        if (seg_len > max_seg_len)
        {
            // Interpolate so every cell the segment crosses gets sampled.
            int num_steps = static_cast<int>(std::ceil(seg_len / max_seg_len));
            for (int s = 1; s <= num_steps; ++s)
            {
                double t = static_cast<double>(s) / num_steps;
                sampleAt(p0 + t * (p1 - p0));
            }
        }
        else
        {
            sampleAt(p1);
        }
    }

    if (num_samples == 0)
    {
        printf("[%s] no valid in-map samples found along path\n",
               path_name.c_str());
        return;
    }

    double avg_dist = sum_dist / num_samples;
    printf("[%s] waypoints=%zu samples=%d (outside_map=%d, unknown=%d) "
           "avg_dist= %.4f min_dist= %.4f @ (%.3f, %.3f, %.3f)\n",
           path_name.c_str(), path.size(), num_samples, num_outside,
           num_unknown, avg_dist, min_dist, min_dist_pos.x(), min_dist_pos.y(),
           min_dist_pos.z());
}

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
        return -1;
    }

    // Construct the ROGMap instance
    auto rog_map = std::make_shared<NonRosROGMap>(argv[1]);
    rog_map->rebuildESDF();

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud = rog_map->saveMap();
    auto map_bounds = rog_map->getLocalMapBounds();
    std::cout << "Local map bounds: " << map_bounds.first.transpose() << " to "
              << map_bounds.second.transpose() << std::endl;
    std::cout << "Local map origin: "
              << rog_map->getLocalMapOrigin().transpose() << std::endl;
    std::cout << "Local map size: " << rog_map->getLocalMapSize().transpose()
              << std::endl;
    std::cout << "Local map resolution: " << rog_map->getResolution()
              << std::endl;

    const std::vector<float> &data = rog_map->getOccupancyBuffer();

    const std::vector<double> &esdf = rog_map->getESDFBuffer();

    // Convert to PCL point cloud and save to PCD file
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr esdf_cloud =
        std::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    esdf_cloud->reserve(esdf.size());
    for (int i = 0; i < esdf.size(); i++)
    {
        if (esdf[i] >= 0.2)
            continue;
        Vec3f pos;
        rog_map->getESDFPosFromHashIndex(i, pos);
        esdf_cloud->push_back(
            pcl::PointXYZRGB(pos.x(), pos.y(), pos.z(), 255, 255, 255));
    }
    esdf_cloud->width = esdf_cloud->size();
    esdf_cloud->height = 1;
    esdf_cloud->is_dense = true;
    pcl::io::savePCDFileBinary("esdf_cloud.pcd", *esdf_cloud);
    printf("Saved %zu occupied points to esdf_cloud.pcd\n", esdf.size());

    // store map in map_util
    std::shared_ptr<ROGMapUtil<3>> map_util =
        std::make_shared<ROGMapUtil<3>>(rog_map);
    std::cout << "Map dimensions: " << map_util->getDim().transpose()
              << std::endl;

    // Hard coded waypoint values: (0, -5, 1.5), (9, -10, 2.0), (0, -16, 1.3)
    const Vec3f start(0.0, -5.0, 1.5);
    const Vec3f goal1(9.0, -10.0, 2.0);
    const Vec3f goal2(0.0, -16.0, 1.3);

    std::unique_ptr<JPSPlanner3D> planner_ptr(
        new JPSPlanner3D(false)); // Declare a planner

    Timer time_update(true);
    planner_ptr->setMapUtil(map_util); // Set collision checking function
    planner_ptr->setThreshVal(0.4);
    map_util->updateVirtualCeilingFloor();
    planner_ptr->updateMap();
    double dt_update = time_update.Elapsed().count();
    printf("updateMap() takes: %f ms\n", dt_update);

    // Add in start / goal points into cloud
    inflateSinglePoint(cloud, pcl::PointXYZ(start.x(), start.y(), start.z()),
                       0.2, 255, 0, 0);
    inflateSinglePoint(cloud, pcl::PointXYZ(goal1.x(), goal1.y(), goal1.z()),
                       0.2, 0, 255, 0);
    inflateSinglePoint(cloud, pcl::PointXYZ(goal2.x(), goal2.y(), goal2.z()),
                       0.2, 0, 0, 255);

    // pcl::io::savePCDFileBinary("grid_with_goals.pcd", *cloud);

    double weight = 1.5;

    Timer time_astar(true);
    bool valid_astar = planner_ptr->plan(
        start, goal1, weight, false); // Plan from start to goal using A*
    double dt_astar = time_astar.Elapsed().count();
    float astar_dist_1 = total_distance3f(planner_ptr->getRawPath());
    printf("AStar Planner takes (start to G1): %f ms\n", dt_astar);
    printf("AStar Path Distance (start to G1): %f\n", astar_dist_1);
    printf("AStar Path (start to G1): \n");
    auto path_astar_1 = planner_ptr->getRawPath();

    addPathToCloud(cloud, path_astar_1, 0, 0, 255);

    time_astar.Reset();
    valid_astar = planner_ptr->plan(goal1, goal2, weight, false);
    double dt_astar2 = time_astar.Elapsed().count();
    float astar_dist_2 = total_distance3f(planner_ptr->getRawPath());
    printf("AStar Planner takes (G1 to G2): %f ms\n", dt_astar2);
    printf("AStar Path Distance (G1 to G2): %f\n", astar_dist_2);
    printf("AStar Path (G1 to G2): \n");
    auto path_astar_2 = planner_ptr->getRawPath();
    // for (const auto& it : path_astar_2)
    //     std::cout << it.transpose() << std::endl;

    addPathToCloud(cloud, path_astar_2, 0, 0, 255);

    printf("AStar Planner takes (Start to G2): %f ms\n", dt_astar2 + dt_astar);
    printf("AStar Path Distance (Start to G2): %f\n",
           astar_dist_1 + astar_dist_2);

    Timer time_jps(true);
    bool valid_jps = planner_ptr->plan(
        start, goal1, weight, true); // Plan from start to goal using JPS
    double dt_jps = time_jps.Elapsed().count();
    float jps_dist_1 = total_distance3f(planner_ptr->getRawPath());
    printf("JPS Planner takes (start to G1): %f ms\n", dt_jps);
    printf("JPS Path Distance (start to G1): %f\n", jps_dist_1);
    printf("JPS Path (start to G1): \n");
    auto path_jps_1 = planner_ptr->getRawPath();
    // for (const auto& it : path_jps_1)
    //     std::cout << it.transpose() << std::endl;

    addPathToCloud(cloud, path_jps_1, 255, 255, 0);

    time_jps.Reset();
    valid_jps = planner_ptr->plan(goal1, goal2, weight, true);
    double dt_jps2 = time_jps.Elapsed().count();
    float jps_dist_2 = total_distance3f(planner_ptr->getRawPath());
    printf("JPS Planner takes (G1 to G2): %f ms\n", dt_jps2);
    printf("JPS Path Distance (G1 to G2): %f\n", jps_dist_2);
    printf("JPS Path (G1 to G2): \n");
    auto path_jps_2 = planner_ptr->getRawPath();
    // for (const auto& it : path_jps_2)
    //     std::cout << it.transpose() << std::endl;

    addPathToCloud(cloud, path_jps_2, 255, 255, 0);

    printf("JPS Planner takes (start to G2): %f ms\n", dt_jps2 + dt_jps);
    printf("JPS Path Distance (start to G2): %f\n", jps_dist_1 + jps_dist_2);

    // --- JPS with fast mode enabled (flat occupancy snapshot instead of
    //     per-cell MapUtil calls during search) as a speed comparison. Fast
    //     mode is latched by updateMap(), so this needs its own planner
    //     instance sharing the same map_util. ---
    std::unique_ptr<JPSPlanner3D> fast_planner_ptr(new JPSPlanner3D(false));
    fast_planner_ptr->setMapUtil(map_util);
    fast_planner_ptr->setThreshVal(0.4);
    fast_planner_ptr->setFastMode(true);

    Timer time_fast_update(true);
    fast_planner_ptr->updateMap();
    double dt_fast_update = time_fast_update.Elapsed().count();
    printf("Fast-mode updateMap() (snapshot build) takes: %f ms\n",
           dt_fast_update);

    Timer time_fast_jps(true);
    bool valid_fast_jps = fast_planner_ptr->plan(start, goal1, weight, true);
    double dt_fast_jps = time_fast_jps.Elapsed().count();
    float fast_jps_dist_1 = total_distance3f(fast_planner_ptr->getRawPath());
    printf("Fast JPS Planner takes (start to G1): %f ms\n", dt_fast_jps);
    printf("Fast JPS Path Distance (start to G1): %f\n", fast_jps_dist_1);
    auto path_fast_jps_1 = fast_planner_ptr->getRawPath();

    addPathToCloud(cloud, path_fast_jps_1, 255, 0, 255);

    time_fast_jps.Reset();
    valid_fast_jps = fast_planner_ptr->plan(goal1, goal2, weight, true);
    double dt_fast_jps2 = time_fast_jps.Elapsed().count();
    float fast_jps_dist_2 = total_distance3f(fast_planner_ptr->getRawPath());
    printf("Fast JPS Planner takes (G1 to G2): %f ms\n", dt_fast_jps2);
    printf("Fast JPS Path Distance (G1 to G2): %f\n", fast_jps_dist_2);
    auto path_fast_jps_2 = fast_planner_ptr->getRawPath();

    addPathToCloud(cloud, path_fast_jps_2, 255, 0, 255);

    printf("Fast JPS Planner takes (start to G2): %f ms\n",
           dt_fast_jps2 + dt_fast_jps);
    printf("Fast JPS Path Distance (start to G2): %f\n",
           fast_jps_dist_1 + fast_jps_dist_2);

    pcl::io::savePCDFileBinary("grid_with_path.pcd", *cloud);

    // Analyze ESDF clearance statistics for every computed path
    vec_Vecf<3> path_astar_full, path_jps_full, path_fast_jps_full;
    path_astar_full.insert(path_astar_full.end(), path_astar_1.begin(),
                           path_astar_1.end());
    path_astar_full.insert(path_astar_full.end(), path_astar_2.begin(),
                           path_astar_2.end());

    path_jps_full.insert(path_jps_full.end(), path_jps_1.begin(),
                         path_jps_1.end());
    path_jps_full.insert(path_jps_full.end(), path_jps_2.begin(),
                         path_jps_2.end());

    path_fast_jps_full.insert(path_fast_jps_full.end(), path_fast_jps_1.begin(),
                              path_fast_jps_1.end());
    path_fast_jps_full.insert(path_fast_jps_full.end(), path_fast_jps_2.begin(),
                              path_fast_jps_2.end());

    printf("\n");
    analyzePathClearance(path_astar_full, "AStar (start->G2)", map_util);
    printf("AStar Planner takes (Start to G2): %f ms\n", dt_astar2 + dt_astar);
    printf("AStar Path Distance (Start to G2): %f\n",
           astar_dist_1 + astar_dist_2);

    printf("\n");
    analyzePathClearance(path_jps_full, "JPS (start->G2)", map_util);
    printf("JPS Planner takes (start to G2): %f ms\n", dt_jps2 + dt_jps);
    printf("JPS Path Distance (start to G2): %f\n", jps_dist_1 + jps_dist_2);

    printf("\n");
    analyzePathClearance(path_fast_jps_full, "Fast JPS (start->G2)", map_util);
    printf(
        "Fast JPS Planner takes (start to G2): %f ms (snapshot build: %f ms)\n",
        dt_fast_jps2 + dt_fast_jps, dt_fast_update);
    printf("Fast JPS Path Distance (start to G2): %f\n",
           fast_jps_dist_1 + fast_jps_dist_2);

    // --- Timing comparison summary across all three planning modes ---
    // "Total" is end-to-end: setup (updateMap(), shared by AStar/JPS since
    // they reuse the same planner_ptr) plus both plan() calls (start->G1,
    // G1->G2) to produce the final path.
    const double total_astar = dt_update + dt_astar + dt_astar2;
    const double total_jps = dt_update + dt_jps + dt_jps2;
    const double total_fast_jps = dt_fast_update + dt_fast_jps + dt_fast_jps2;

    printf("\n=== Planner timing comparison (start -> G1 -> G2) ===\n");
    printf("%-12s %12s %14s %12s %10s\n", "Mode", "Plan (ms)", "Setup (ms)",
           "Total (ms)", "Dist (m)");
    printf("%-12s %12.3f %14.3f %12.3f %10.3f\n", "AStar", dt_astar + dt_astar2,
           dt_update, total_astar, astar_dist_1 + astar_dist_2);
    printf("%-12s %12.3f %14.3f %12.3f %10.3f\n", "JPS", dt_jps + dt_jps2,
           dt_update, total_jps, jps_dist_1 + jps_dist_2);
    printf("%-12s %12.3f %14.3f %12.3f %10.3f\n", "JPS (fast)",
           dt_fast_jps + dt_fast_jps2, dt_fast_update, total_fast_jps,
           fast_jps_dist_1 + fast_jps_dist_2);
    printf("Fast JPS speedup vs. regular JPS (plan time only): %.2fx\n",
           (dt_jps + dt_jps2) / (dt_fast_jps + dt_fast_jps2));
    printf(
        "Fast JPS speedup vs. regular JPS (end-to-end, incl. setup): %.2fx\n",
        total_jps / total_fast_jps);

    return 0;
}
