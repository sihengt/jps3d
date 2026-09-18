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
    /// Get the nodes in open set
    vec_Vecf<Dim> getOpenSet() const;
    /// Get the nodes in close set
    vec_Vecf<Dim> getCloseSet() const;
    /// Get all the nodes
    vec_Vecf<Dim> getAllSet() const;
    /// Set thresh_dist_
    void setThreshDist(TmapValue val)
    {
        thresh_dist_ = val;
        if (graph_search_)
            graph_search_->setThreshDist(val);
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
    /// 1-D map array
    Tmap cmap_;
    /// TODO: to refactor
    /// Distance >= thresh_dist_ are considered free
    TmapValue thresh_dist_ = 0;
};

/// Planner for 2D OccMap
typedef JPSPlanner<2> JPSPlanner2D;

/// Planner for 3D VoxelMap
typedef JPSPlanner<3> JPSPlanner3D;

#endif
