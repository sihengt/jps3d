/**
 * @file jps_planner.h
 * @brief JPSPlanner
 */
#ifndef JPS_PLANNER_BASE_H
#define JPS_PLANNER_BASE_H

#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>
#include <jps_planner/jps_planner/graph_search.h>

class GraphSearch;
/**
 * @brief Abstract base for planning
 *
 * @param Dim is the dimension of the workspace
 * @param ValueT is the map cell value type, forwarded to JPS::MapUtil<Dim,
 * ValueT> and JPS::GraphSearch<Dim, ValueT>. Defaults to double so existing
 * JPSPlanner<Dim> callers keep compiling unchanged.
 */
template <int Dim, typename ValueT = double> class JPSPlanner
{
public:
    using TmapValue = ValueT;
    using Tmap = std::vector<ValueT>;

    /**
     * @brief Simple constructor
     * @param verbose enable debug mode
     */
    JPSPlanner(bool verbose = false);

    /**
     * @brief map_util_ setter
     */
    inline void
    setMapUtil(const std::shared_ptr<JPS::MapUtil<Dim, ValueT>> &map_util)
    {
        map_util_ = map_util;
    }

    /**
     * @brief Status of the planner
     *
     * 0 --- exit normally;
     * -1 --- no path found;
     * 1, 2 --- start or goal is not free.
     */
    int status();
    /// Get the modified path
    vec_Vecf<Dim> getPath();
    /// Get the raw path
    vec_Vecf<Dim> getRawPath();
    /// remove redundant points on the same line
    vec_Vecf<Dim> removeLinePts(const vec_Vecf<Dim> &path);
    /// Remove some corner waypoints
    vec_Vecf<Dim> removeCornerPts(const vec_Vecf<Dim> &path);
    /// Planning function
    bool plan(const Vecf<Dim> &start, const Vecf<Dim> &goal, decimal_t eps = 1,
              bool use_jps = true);
    /// Refresh the planner's view of the map. Must be called after
    /// setMapUtil(), setThreshVal() and whenever the map data/dims change.
    /// A no-op unless fast mode is on (then it rebuilds the snapshot).
    void updateMap();
    /// Get the nodes in open set
    vec_Vecf<Dim> getOpenSet() const;
    /// Get the nodes in close set
    vec_Vecf<Dim> getCloseSet() const;
    /// Get all the nodes
    vec_Vecf<Dim> getAllSet() const;
    /// Phase timings (ms) and search size of the last plan() call.
    struct Timings
    {
        double check_ms = 0;   // start/goal validation
        double build_ms = 0;   // GraphSearch reconstruction (fastmode)
        double search_ms = 0;  ///< graph search
        double convert_ms = 0; ///< StatePtr path -> world coordinates
        double corner_ms = 0;  ///< removeCornerPts (both passes)
        double line_ms = 0;    ///< removeLinePts
        double total_ms = 0;
        long long expand = 0, succ = 0, jump_steps = 0, cell_queries = 0,
                  heap_push = 0; ///< from GraphSearch::Stats (JPS_PROFILE)
    };
    const Timings &lastTimings() const { return timings_; }

    /// Cap on graph-search expansions per plan(); <= 0 means unlimited.
    /// When the cap is hit, plan() returns true with status() == 3 and
    /// getPath()/getRawPath() hold a partial path toward the goal.
    void setMaxExpand(int n) { max_expand_ = n; }

    /// Fast mode: updateMap() takes a flat occupancy snapshot of the map's
    /// live region (MapUtil::snapshotOccupancy) and plan() searches that
    /// array directly instead of calling MapUtil per cell. updateMap() must
    /// be called after every map change and after setThreshVal().
    void setFastMode(bool on) { fast_mode_ = on; }
    bool fastMode() const { return fast_mode_; }
    /// Time spent building the last snapshot (ms).
    double lastSnapshotMs() const { return snapshot_ms_; }
    /// Diagnostic accessors for the last fast-mode snapshot (temporary --
    /// used to A/B the live query path against the snapshot it was built
    /// from, see docs/perf investigation).
    const std::vector<uint8_t> &debugSnapshot() const { return occ_; }
    Veci<Dim> debugSnapLo() const { return snap_lo_; }
    Veci<Dim> debugSnapDim() const { return snap_dim_; }
    /// Diagnostic: the persistent graph search (null before first plan()).
    const std::shared_ptr<JPS::GraphSearch<Dim, ValueT>> &debugGraphSearch() const
    {
        return graph_search_;
    }

    /// Restrict plan() to the inclusive cell box [lo, hi] (global cell
    /// indices, as returned by MapUtil::floatToInt). Fast mode only.
    /// Returns true if the box covers the whole snapshot, i.e. widening it
    /// further cannot change the result.
    bool setSearchBox(const Veci<Dim> &lo, const Veci<Dim> &hi);
    void clearSearchBox() { box_en_ = false; }

    /// Set thresh_val_
    void setThreshVal(TmapValue val)
    {
        thresh_val_ = val;
        if (graph_search_)
            graph_search_->setThreshVal(val);
    }

protected:
    /// Assume using 3D voxel map for all 2d and 3d planning
    std::shared_ptr<JPS::MapUtil<Dim, ValueT>> map_util_;
    /// The planner -- persists across plan() calls; only rebuilt if the map
    /// dimensions change. See plan() in jps_planner.cpp.
    std::shared_ptr<JPS::GraphSearch<Dim, ValueT>> graph_search_;
    /// Dimensions graph_search_ was last built for (-1 = not built yet)
    int graph_search_dim_x_ = -1;
    int graph_search_dim_y_ = -1;
    int graph_search_dim_z_ = -1;
    /// Raw path from planner
    vec_Vecf<Dim> raw_path_;
    /// Modified path for future usage
    vec_Vecf<Dim> path_;
    /// Flag indicating the success of planning
    int status_ = 0;
    /// Enabled for printing info
    bool planner_verbose_;
    /// Distance >= thresh_val_ are considered free
    TmapValue thresh_val_ = 0;
    Timings timings_;
    int max_expand_ = -1;

    bool fast_mode_ = false;
    std::vector<uint8_t> occ_;
    Veci<Dim> snap_lo_ = Veci<Dim>::Zero();
    Veci<Dim> snap_dim_ = Veci<Dim>::Zero();
    double snapshot_ms_ = 0;
    bool box_en_ = false;
    Veci<Dim> box_lo_ = Veci<Dim>::Zero(), box_hi_ = Veci<Dim>::Zero();

    /// Grid cell -> world, adding the snapshot offset in fast mode
    Vecf<Dim> cellToWorld(const JPS::StatePtr &it) const;
};

/// Planner for 2D OccMap
typedef JPSPlanner<2> JPSPlanner2D;

/// Planner for 3D VoxelMap
typedef JPSPlanner<3> JPSPlanner3D;

#endif
