#pragma once

#include <memory>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>
#include <jps_planner/jps_planner/jps_planner.h>

/**
 * @brief Thin planning facade over an abstract JPS::MapUtil<3> + JPSPlanner3D.
 *
 * Holds map_util_ ONLY as the abstract JPS::MapUtil<3> interface -- never
 * the concrete backend type (OctomapMapUtil, SimpleMapUtil, or any future
 * one) -- so every method here, including frontier truncation, works
 * identically for any backend without a single backend-specific branch.
 */
class Jps3dFrontend
{
public:
    Jps3dFrontend(std::shared_ptr<JPS::MapUtil<3>> map_util, bool verbose,
                  bool block_unknown, double frontier_seed_radius);

    /// Forwards to map_util_->setThreshVal(); affects both the planner's
    /// obstacle margin and (indirectly, via getThreshDist()) frontier
    /// truncation's occupied check.
    void setThreshVal(double thresh_val);

    /// Refresh the planner's internal snapshot after the map changed. Must
    /// be called after every map_util_ mutation before the next planPath().
    void updateMap();

    /// Plans start->goal. On success, fills out_path (world-space waypoints,
    /// start to goal) and returns true. If block_unknown_ is set, the
    /// returned path is truncated at the first point beyond
    /// frontier_seed_radius_ of start that is unknown or occupied. Returns
    /// false (out_path untouched) if no path exists.
    bool planPath(const Vec3f &start, const Vec3f &goal, double eps,
                  bool use_jps, vec_Vec3f &out_path);

    /// Last plan()'s status code (0=ok, -1=no path, 1=start blocked,
    /// 2=goal blocked), for the caller's logging.
    int status() const;

private:
    std::shared_ptr<JPS::MapUtil<3>> map_util_;
    std::shared_ptr<JPSPlanner3D> planner_;
    bool block_unknown_;
    double frontier_seed_radius_;
};
