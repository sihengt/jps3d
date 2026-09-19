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
    /// 2=goal blocked, 3=expansion budget hit, partial path returned), for
    /// the caller's logging.
    int status() const;

    /// Fast mode (flat occupancy snapshot), see JPSPlanner::setFastMode.
    void setFastMode(bool on);
    /// Expansion budget per search, see JPSPlanner::setMaxExpand.
    void setMaxExpand(int n);
    /// Iterative search-box widening (fast mode only): the first search is
    /// confined to the start/goal bounding box padded by margin_m metres;
    /// every exhaustive failure doubles the margin until the box covers the
    /// whole map (that last attempt equals an unrestricted search, so
    /// completeness is unchanged). 0 disables.
    void setSearchBoxWidening(double margin_m);
    /// Number of searches the last planPath() ran (>1 only with widening).
    int lastAttempts() const { return last_attempts_; }
    /// Wall time of the last planPath() summed over all attempts (ms).
    double lastPlanMs() const { return last_plan_ms_; }
    /// Time the last updateMap() spent building the fast-mode snapshot.
    double lastSnapshotMs() const { return planner_->lastSnapshotMs(); }

    /// Phase timings / search counters of the last planPath() call.
    const JPSPlanner3D::Timings &lastTimings() const
    {
        return planner_->lastTimings();
    }

private:
    std::shared_ptr<JPS::MapUtil<3>> map_util_;
    std::shared_ptr<JPSPlanner3D> planner_;
    bool block_unknown_;
    double frontier_seed_radius_;
    double widen_margin_m_ = 0.0;
    int last_attempts_ = 0;
    double last_plan_ms_ = 0.0;
};
