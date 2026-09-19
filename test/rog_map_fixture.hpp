/**
 * @file rog_map_fixture.hpp
 * @brief Non-ROS rog_map::ROGMap subclass + synthetic scenes for tests and
 * benchmarks of JPS::ROGMapUtil (map_util_super.h).
 */
#pragma once

#include <chrono>
#include <cmath>
#include <random>
#include <string>

#include <jps_collision/map_util_super.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <rog_map/rog_map.h>

namespace jps_test
{

/// ROGMap driven entirely from code: config from a yaml path, obstacles
/// inserted as point clouds, ESDF rebuilt on demand. No ROS.
class TestROGMap : public rog_map::ROGMap
{
public:
    explicit TestROGMap(const std::string &cfg_path)
    {
        cfg_ = rog_map::Config(cfg_path);
        init();
        // ROGMap::init() only slides the prob/inf maps to fix_map_origin when
        // map sliding is disabled; the ESDF map's bounds are left at zero, so
        // updateESDF3D() would compute an empty box. Slide it here.
        if (cfg_.esdf_en)
            esdf_map_->mapSliding(cfg_.fix_map_origin);
    }
    ~TestROGMap() override = default;

    /// Mark the cloud's points occupied and rebuild the ESDF.
    void insert(const rog_map::PointCloud &cloud)
    {
        updateOccPointCloud(cloud);
        rebuildESDF();
    }

    void rebuildESDF()
    {
        if (cfg_.esdf_en)
            esdf_map_->updateESDF3D(cfg_.fix_map_origin);
    }

    const rog_map::Config &config() const { return cfg_; }

    /// ROG-Map's own ESDF lookup, for cross-checking ROGMapUtil indexing.
    double esdfDistance(const Vec3f &p) const
    {
        return esdf_map_->getDistance(p);
    }

private:
    const double getSystemWalltimeNow() override
    {
        auto now = std::chrono::steady_clock::now();
        return std::chrono::duration<double>(now.time_since_epoch()).count();
    }
};

// ---------------------------------------------------------------------------
// Synthetic scenes. All points sampled on a `step` lattice (default 0.05 m,
// the prob-map resolution). World: x,y in [-20,20], z in [0, 20], with the
// planner's virtual ceiling at 3.0 m.
// ---------------------------------------------------------------------------

struct SceneBuilder
{
    rog_map::PointCloud cloud;
    double step = 0.05;
    double wall_h = 3.0; // planner ceiling is 2.7 m (3.0 - robot_r)

    void point(double x, double y, double z)
    {
        pcl::PointXYZI p;
        p.x = x;
        p.y = y;
        p.z = z;
        p.intensity = 100;
        cloud.push_back(p);
    }

    /// Axis-aligned box [x0,x1]x[y0,y1]x[z0,z1], filled (solid).
    void box(double x0, double x1, double y0, double y1, double z0, double z1)
    {
        for (double x = x0; x <= x1 + 1e-9; x += step)
            for (double y = y0; y <= y1 + 1e-9; y += step)
                for (double z = z0; z <= z1 + 1e-9; z += step)
                    point(x, y, z);
    }

    /// Thin wall (0.1 m thick) from (xa,ya) to (xb,yb), axis-aligned.
    void wall(double xa, double ya, double xb, double yb)
    {
        const double t = 0.05;
        box(std::min(xa, xb) - t, std::max(xa, xb) + t, std::min(ya, yb) - t,
            std::max(ya, yb) + t, step / 2, wall_h);
    }

    /// Ground plane at the bottom of the first cell layer.
    void ground(double half = 20.0)
    {
        for (double x = -half; x <= half; x += step)
            for (double y = -half; y <= half; y += step)
                point(x, y, step / 2);
    }

    void finalize()
    {
        cloud.width = cloud.size();
        cloud.height = 1;
        cloud.is_dense = true;
    }
};

/// Open space with `n` random vertical pillars (radius 0.3 m).
inline rog_map::PointCloud openPillars(int n, unsigned seed = 7)
{
    SceneBuilder b;
    std::mt19937 rng(seed);
    std::uniform_real_distribution<double> u(-18.0, 18.0);
    for (int i = 0; i < n; ++i)
    {
        double cx = u(rng), cy = u(rng);
        // Keep the diagonal start/goal spots used by the tests clear.
        if (std::abs(cx - cy) < 1.5)
            continue;
        b.box(cx - 0.3, cx + 0.3, cy - 0.3, cy + 0.3, b.step / 2, b.wall_h);
    }
    b.finalize();
    return b.cloud;
}

/// 4x4 grid of 10 m rooms. Every interior wall has one 1 m doorway per
/// room edge, offset from the room centre so paths must zig-zag.
inline rog_map::PointCloud roomsAndDoors()
{
    SceneBuilder b;
    const double L = 20.0;
    for (double w : {-10.0, 0.0, 10.0})
    {
        // walls parallel to y (at x = w) and to x (at y = w), each split into
        // four 10 m segments with a 1 m door centred at seg_mid + 2.5.
        for (int s = 0; s < 4; ++s)
        {
            double a = -L + 10.0 * s;
            double door = a + 7.5;
            b.wall(w, a, w, door - 0.5);
            b.wall(w, door + 0.5, w, a + 10.0);
            b.wall(a, w, door - 0.5, w);
            b.wall(door + 0.5, w, a + 10.0, w);
        }
    }
    b.finalize();
    return b.cloud;
}

/// A U-shaped pocket (dead end) of depth `depth` and width `width`, open
/// toward -y, closed side on y = 0. The start sits inside the pocket near
/// the closed side, the goal just beyond it, so straight-line distance is
/// ~2 m but the path must leave the pocket: length ~ 2*depth + width.
inline rog_map::PointCloud deadEndPocket(double depth, double width)
{
    SceneBuilder b;
    const double hw = width / 2.0;
    b.wall(-hw, 0.0, hw, 0.0);        // closed side
    b.wall(-hw, -depth, -hw, 0.0);    // left side
    b.wall(hw, -depth, hw, 0.0);      // right side
    // A sealed 3x3 m room at (10,10) for "no path" cases.
    b.wall(8.5, 8.5, 11.5, 8.5);
    b.wall(8.5, 11.5, 11.5, 11.5);
    b.wall(8.5, 8.5, 8.5, 11.5);
    b.wall(11.5, 8.5, 11.5, 11.5);
    b.finalize();
    return b.cloud;
}

/// Find the nearest cell to `p` (within `radius` m) that the map util reports
/// free, ignoring unknown status (with raycasting disabled every non-occupied
/// cell is unknown, so getNearestKnownFreePos() would never succeed).
template <typename MapUtilT>
inline bool snapToFree(MapUtilT &mu, const Vec3f &p, Vec3f &out,
                       double radius = 1.0)
{
    const double res = mu.getRes();
    const int r = static_cast<int>(std::ceil(radius / res));
    const Vec3i c = mu.floatToInt(p);
    if (mu.isFree(c, mu.getThreshDist()))
    {
        out = mu.intToFloat(c);
        return true;
    }
    int best = std::numeric_limits<int>::max();
    for (int dx = -r; dx <= r; ++dx)
        for (int dy = -r; dy <= r; ++dy)
            for (int dz = -r; dz <= r; ++dz)
            {
                int d2 = dx * dx + dy * dy + dz * dz;
                if (d2 >= best)
                    continue;
                Vec3i q = c + Vec3i(dx, dy, dz);
                if (mu.isFree(q, mu.getThreshDist()))
                {
                    best = d2;
                    out = mu.intToFloat(q);
                }
            }
    return best != std::numeric_limits<int>::max();
}

} // namespace jps_test
