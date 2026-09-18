/**
 * @file graph_search.h
 * @brief backend of graph search for distance map
 */

#ifndef DMP_GRAPH_SEARCH_H
#define DMP_GRAPH_SEARCH_H

#include <boost/heap/d_ary_heap.hpp> // boost::heap::d_ary_heap
#include <jps_collision/map_util.h>  // For JPS::TmapValue
#include <limits>                    // std::numeric_limits
#include <memory>                    // std::shared_ptr
#include <unordered_map>             // std::unordered_map
#include <vector>                    // std::vector

namespace DMP
{
/// Heap element comparison
template <class T> struct compare_state
{
    bool operator()(T a1, T a2) const
    {
        double f1 = a1->g + a1->h;
        double f2 = a2->g + a2->h;
        if ((f1 >= f2 - 0.000001) && (f1 <= f2 + 0.000001))
            return a1->g < a2->g; // if equal compare gvals
        return f1 > f2;
    }
};

/// Define priority queue
struct State; // forward declaration
/// State pointer
using StatePtr = std::shared_ptr<State>;
using priorityQueue =
    boost::heap::d_ary_heap<StatePtr, boost::heap::mutable_<true>,
                            boost::heap::arity<2>,
                            boost::heap::compare<compare_state<StatePtr>>>;

/// Node of the graph in graph search
struct State
{
    /// ID
    int id;
    /// Coord
    int x, y, z = 0;
    /// id of predecessors
    int parentId = -1;

    /// pointer to heap location
    priorityQueue::handle_type heapkey;

    /// g cost
    double g = std::numeric_limits<double>::infinity();
    /// heuristic cost
    double h;
    /// if has been opened
    bool opened = false;
    /// if has been closed
    bool closed = false;

    /// 2D constructor
    State(int id, int x, int y) : id(id), x(x), y(y) {}

    /// 3D constructor
    State(int id, int x, int y, int z) : id(id), x(x), y(y), z(z) {}
};

/**
 * @brief GraphSearch class
 *
 * Implement A* and Jump Point Search
 *
 * @param Dim is the dimension of the workspace
 * @param ValueT is the map cell value type, forwarded to JPS::MapUtil<Dim,
 * ValueT>. Defaults to double so existing GraphSearch<Dim> callers keep
 * compiling unchanged.
 */
template <int Dim, typename ValueT = double> class GraphSearch
{
public:
    using TmapValue = ValueT;
    using Tmap = std::vector<ValueT>;

    /**
     * @brief graph search constructor
     *
     * @param map_util map util for collision checking
     * @param eps weight of heuristic, optional, default as 1
     * @param cweight weight of distance cost, optional, default as 0.1
     * @param verbose flag for printing debug info, optional, default as False
     * @param thresh_dist cells with value >= this are considered free,
     * optional, default as 0
     * @param h_max max potential value, optional, default as 100
     * @param potential_radius maximum radius of concern for potential
     * calculation, optional, default as 0.5
     * @param pow power index for potential calculation, optional, default as 1
     */
    GraphSearch(const std::shared_ptr<JPS::MapUtil<Dim, ValueT>> &map_util,
                double eps = 1, double cweight = 0.1, bool verbose = false,
                TmapValue thresh_dist = 0, TmapValue h_max = 100,
                TmapValue potential_radius = 0.5, int pow = 1);

    /// Set thresh_dist_
    void setThreshDist(TmapValue thresh_dist) { thresh_dist_ = thresh_dist; }

    /**
     * @brief start 2D planning thread
     *
     * @param xStart start x coordinate
     * @param yStart start y coordinate
     * @param xGoal goal x coordinate
     * @param yGoal goal y coordinate
     * @param in_region a region that is valid for searching, empty means no
     * boundary
     *
     * return the total path cost, which is infinity if failed
     */
    double plan(int xStart, int yStart, int xGoal, int yGoal,
                std::vector<bool> in_region = std::vector<bool>());
    /**
     * @brief start 3D planning thread
     *
     * @param xStart start x coordinate
     * @param yStart start y coordinate
     * @param zStart start z coordinate
     * @param xGoal goal x coordinate
     * @param yGoal goal y coordinate
     * @param zGoal goal z coordinate
     * @param in_region a region that is valid for searching, empty means no
     * boundary
     *
     * return the total path cost, which is infinity if failed
     */
    double plan(int xStart, int yStart, int zStart, int xGoal, int yGoal,
                int zGoal, std::vector<bool> in_region = std::vector<bool>());

    /// Get the optimal path
    std::vector<StatePtr> getPath() const;

    /// Get the states in open set
    std::vector<StatePtr> getOpenSet() const;

    /// Get the states in close set
    std::vector<StatePtr> getCloseSet() const;

    /// Get the states in hash map
    std::vector<StatePtr> getAllSet() const;

private:
    /// Main planning loop
    double plan(StatePtr &currNode_ptr, int start_id, int goal_id);
    /// Get successor function for A*
    void getSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                 std::vector<double> &succ_costs);
    /// Recover the optimal path
    std::vector<StatePtr> recoverPath(StatePtr node, int id);

    /// Get subscript
    int coordToId(int x, int y) const;
    /// Get subscript
    int coordToId(int x, int y, int z) const;

    /// Check if (x, y) is free
    bool isFree(int x, int y) const;
    /// Check if (x, y, z) is free
    bool isFree(int x, int y, int z) const;

    /// Calculate heuristic
    double getHeur(int x, int y) const;
    /// Calculate heuristic
    double getHeur(int x, int y, int z) const;

    std::shared_ptr<JPS::MapUtil<Dim, ValueT>> map_util_;
    // Raw cost map, aliased to map_util_->map_ (shared, never modified)
    std::shared_ptr<const Tmap> cMap_;
    int xDim_, yDim_, zDim_;
    // TODO: to refactor variable name
    TmapValue thresh_dist_ = 0;
    /// weight of heuristic
    double eps_;
    /// weight of distance map
    double cweight_;
    /// max potential value
    TmapValue H_MAX;
    /// maximum radius of concern, any further not important
    TmapValue potential_radius_;
    /// power index for potential calculation
    int pow_;
    bool verbose_;

    int xGoal_, yGoal_, zGoal_;
    bool use_2d_;
    bool global_;

    priorityQueue pq_;
    std::vector<StatePtr> hm_;
    std::vector<uint16_t> visited_;
    uint16_t current_planning_token_ = 0;
    std::vector<bool> in_region_;

    std::vector<StatePtr> path_;

    /// Precomputed neighbor offset + its Euclidean grid-step distance
    /// (only ever 1, sqrt(2), or sqrt(3) for a 3x3(x3) stencil), so
    /// getSucc() doesn't recompute sqrt() per successor per expansion.
    struct Neighbor
    {
        int dx, dy, dz;
        double dist;
    };
    std::vector<Neighbor> ns_;
};
} // namespace DMP
#endif
