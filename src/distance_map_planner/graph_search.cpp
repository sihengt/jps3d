#include <cmath>
#include <jps_planner/distance_map_planner/graph_search.h>

using namespace DMP;

// Single template constructor handling both 2D and 3D structurally
template <int Dim>
GraphSearch<Dim>::GraphSearch(
    const std::shared_ptr<JPS::MapUtil<Dim>> &map_util, double eps,
    double cweight, bool verbose, JPS::TmapValue thresh_dist,
    JPS::TmapValue h_max, JPS::TmapValue potential_radius, int pow)
    : map_util_(map_util),
      // aliasing ctor: cMap_ shares map_util_'s refcount but points at its raw
      // map_ data. The map is used directly as the cost map and is never
      // modified.
      cMap_(map_util_, &map_util_->map_), thresh_dist_(thresh_dist), eps_(eps),
      cweight_(cweight), H_MAX(h_max), potential_radius_(potential_radius),
      pow_(pow), verbose_(verbose)
{
    xDim_ = map_util_->getDim()(0);
    yDim_ = map_util_->getDim()(1);
    zDim_ = (Dim == 3) ? map_util_->getDim()(2) : 1;

    int total_cells = xDim_ * yDim_ * zDim_;
    hm_.resize(total_cells, nullptr);
    visited_.resize(total_cells, 0);

    // Dynamic generation of neighbors based on dimension, with the
    // Euclidean grid-step distance precomputed once (only 1, sqrt(2), or
    // sqrt(3) ever occur), instead of recomputing sqrt() per successor
    // in the hot getSucc() loop.
    if (Dim == 2)
    {
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                if (x == 0 && y == 0)
                    continue;
                ns_.push_back({x, y, 0, std::sqrt(double(x * x + y * y))});
            }
        }
    }
    else if (Dim == 3)
    {
        for (int x = -1; x <= 1; x++)
        {
            for (int y = -1; y <= 1; y++)
            {
                for (int z = -1; z <= 1; z++)
                {
                    if (x == 0 && y == 0 && z == 0)
                        continue;
                    ns_.push_back(
                        {x, y, z, std::sqrt(double(x * x + y * y + z * z))});
                }
            }
        }
    }
}

template <int Dim> inline int GraphSearch<Dim>::coordToId(int x, int y) const
{
    if constexpr (Dim == 2)
    {
        return map_util_->getIndex(Vec2i(x, y));
    }
    else
    {
        // Not implemented
        return -1;
    }
}

template <int Dim>
inline int GraphSearch<Dim>::coordToId(int x, int y, int z) const
{
    if constexpr (Dim == 3)
    {
        return map_util_->getIndex(Vec3i(x, y, z));
    }
    else
    {
        // Not implemented
        return -1;
    }
}

template <int Dim> inline bool GraphSearch<Dim>::isFree(int x, int y) const
{
    if constexpr (Dim == 2)
    {
        return map_util_->isFree(Vec2i(x, y), thresh_dist_);
    }
    else
    {
        // Not implemented
        return false;
    }
}

template <int Dim>
inline bool GraphSearch<Dim>::isFree(int x, int y, int z) const
{
    if constexpr (Dim == 3)
    {
        return map_util_->isFree(Vec3i(x, y, z), thresh_dist_);
    }
    else
    {
        // Not implemented
        return false;
    }
}

template <int Dim> inline double GraphSearch<Dim>::getHeur(int x, int y) const
{
    return eps_ *
           std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_));
}

template <int Dim>
inline double GraphSearch<Dim>::getHeur(int x, int y, int z) const
{
    return eps_ *
           std::sqrt((x - xGoal_) * (x - xGoal_) + (y - yGoal_) * (y - yGoal_) +
                     (z - zGoal_) * (z - zGoal_));
}

template <int Dim>
double GraphSearch<Dim>::plan(int xStart, int yStart, int xGoal, int yGoal,
                              std::vector<bool> in_region)
{
    use_2d_ = true;
    pq_.clear();
    path_.clear();
    hm_.assign(xDim_ * yDim_, nullptr);

    current_planning_token_++;
    if (current_planning_token_ == 0)
    {
        visited_.assign(xDim_ * yDim_, 0);
        current_planning_token_ = 1;
    }
    in_region_ = in_region;

    global_ = in_region.empty();
    if (verbose_)
        printf(global_ ? "global planning!\n" : "local planning!\n");

    // Set goal
    int goal_id = coordToId(xGoal, yGoal);
    xGoal_ = xGoal;
    yGoal_ = yGoal;

    // Set start node
    int start_id = coordToId(xStart, yStart);
    StatePtr currNode_ptr = std::make_shared<State>(start_id, xStart, yStart);
    currNode_ptr->g = (*cMap_)[start_id];
    currNode_ptr->h = getHeur(xStart, yStart);

    return plan(currNode_ptr, start_id, goal_id);
}

template <int Dim>
double GraphSearch<Dim>::plan(int xStart, int yStart, int zStart, int xGoal,
                              int yGoal, int zGoal, std::vector<bool> in_region)
{
    use_2d_ = false;
    pq_.clear();
    path_.clear();
    hm_.assign(xDim_ * yDim_ * zDim_, nullptr);

    current_planning_token_++;
    if (current_planning_token_ == 0)
    {
        visited_.assign(xDim_ * yDim_ * zDim_, 0);
        current_planning_token_ = 1;
    }
    in_region_ = in_region;

    global_ = in_region.empty();
    if (verbose_)
        printf(global_ ? "global planning!\n" : "local planning!\n");

    int goal_id = coordToId(xGoal, yGoal, zGoal);
    xGoal_ = xGoal;
    yGoal_ = yGoal;
    zGoal_ = zGoal;

    // Set start node
    int start_id = coordToId(xStart, yStart, zStart);
    StatePtr currNode_ptr =
        std::make_shared<State>(start_id, xStart, yStart, zStart);
    currNode_ptr->g = (*cMap_)[start_id];
    currNode_ptr->h = getHeur(xStart, yStart, zStart);

    return plan(currNode_ptr, start_id, goal_id);
}

template <int Dim>
double GraphSearch<Dim>::plan(StatePtr &currNode_ptr, int start_id, int goal_id)
{
    // Insert start node
    currNode_ptr->heapkey = pq_.push(currNode_ptr);
    currNode_ptr->opened = true;
    hm_[currNode_ptr->id] = currNode_ptr;
    visited_[currNode_ptr->id] = current_planning_token_;

    int expand_iteration = 0;

    std::vector<int> succ_ids;
    std::vector<double> succ_costs;

    while (true)
    {
        expand_iteration++;
        // get element with smallest cost
        currNode_ptr = pq_.top();
        pq_.pop();
        currNode_ptr->closed = true; // Add to closed list

        if (currNode_ptr->id == goal_id)
        {
            if (verbose_)
                printf("Goal Reached!!!!!!\n\n");
            break;
        }

        succ_ids.clear();
        succ_costs.clear();
        // Get successors
        getSucc(currNode_ptr, succ_ids, succ_costs);

        // Process successors
        for (int s = 0; s < (int)succ_ids.size(); s++)
        {
            // see if we can improve the value of succstate
            StatePtr &child_ptr = hm_[succ_ids[s]];
            double tentative_gval = currNode_ptr->g + succ_costs[s];
            // Add a small epsilon for robust comparison
            if (tentative_gval < child_ptr->g - 1e-9)
            {
                child_ptr->parentId = currNode_ptr->id; // Assign new parent
                child_ptr->g = tentative_gval;          // Update gval

                // double fval = child_ptr->g + child_ptr->h;

                // if currently in OPEN, update
                if (child_ptr->opened && !child_ptr->closed)
                    pq_.increase(child_ptr->heapkey); // update heap
                // if currently in CLOSED
                else if (child_ptr->opened && child_ptr->closed)
                {
                    if (verbose_)
                        printf(
                            "ASTAR ERROR: re-opened a CLOSED node (id=%d)!\n",
                            child_ptr->id);
                }
                else // new node, add to heap
                {
                    // printf("add to open set: %d, %d\n", child_ptr->x,
                    // child_ptr->y);
                    child_ptr->heapkey = pq_.push(child_ptr);
                    child_ptr->opened = true;
                }
            } //
        } // Process successors

        if (pq_.empty())
        {
            if (verbose_)
                printf("Priority queue is empty!!!!!!\n\n");
            return std::numeric_limits<double>::infinity();
        }
    }

    if (verbose_)
    {
        printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
        printf("Expand [%d] nodes!\n", expand_iteration);
    }

    path_ = recoverPath(currNode_ptr, start_id);

    return currNode_ptr->g;
}

template <int Dim>
std::vector<StatePtr> GraphSearch<Dim>::recoverPath(StatePtr node, int start_id)
{
    std::vector<StatePtr> path;
    path.push_back(node);
    while (node && node->id != start_id)
    {
        node = hm_[node->parentId];
        // printf("waypoint g: %f, h: %f!\n", node->g, node->h);
        path.push_back(node);
    }

    return path;
}

template <int Dim>
void GraphSearch<Dim>::getSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                               std::vector<double> &succ_costs)
{

    succ_ids.reserve(ns_.size());
    succ_costs.reserve(ns_.size());

    if (use_2d_)
    {
        for (const auto &d : ns_)
        {
            int new_x = curr->x + d.dx;
            int new_y = curr->y + d.dy;
            if (!isFree(new_x, new_y))
                continue;

            int new_id = coordToId(new_x, new_y);
            if (!global_ && !in_region_[new_id])
                continue;

            if (visited_[new_id] != current_planning_token_)
            {
                visited_[new_id] = current_planning_token_;
                hm_[new_id] = std::make_shared<State>(new_id, new_x, new_y);
                hm_[new_id]->h = getHeur(new_x, new_y);
            }
            // TODO: Defer weights calculation to here if using a ESDF map
            // h = H_MAX × (1 - distance/radius)^pow, distance: ESDF value
            float potential =
                H_MAX * std::pow(1 - std::max(std::min((*cMap_)[new_id],
                                                       potential_radius_),
                                              0.0) /
                                         potential_radius_,
                                 pow_);
            succ_ids.push_back(new_id);
            succ_costs.push_back(d.dist + cweight_ * potential);
            // succ_costs.push_back(std::sqrt(d[0] * d[0] + d[1] * d[1]) +
            //                      cweight_ * (cMap_[new_id]));
        }
    }
    else
    {
        for (const auto &d : ns_)
        {
            int new_x = curr->x + d.dx;
            int new_y = curr->y + d.dy;
            int new_z = curr->z + d.dz;
            if (!isFree(new_x, new_y, new_z))
                continue;

            int new_id = coordToId(new_x, new_y, new_z);
            if (!global_ && !in_region_[new_id])
                continue;

            if (visited_[new_id] != current_planning_token_)
            {
                visited_[new_id] = current_planning_token_;
                hm_[new_id] =
                    std::make_shared<State>(new_id, new_x, new_y, new_z);
                hm_[new_id]->h = getHeur(new_x, new_y, new_z);
            }
            // TODO: Defer weights calculation to here for ESDF map
            // h = H_MAX × (1 - distance/radius)^pow, distance: ESDF value
            float potential =
                H_MAX * std::pow(1 - std::max(std::min((*cMap_)[new_id],
                                                       potential_radius_),
                                              0.0) /
                                         potential_radius_,
                                 pow_);
            succ_ids.push_back(new_id);
            succ_costs.push_back(d.dist + cweight_ * potential);
            // succ_costs.push_back(
            //     std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]) +
            //     cweight_ * cMap_[new_id]);
        }
    }
}

template <int Dim> std::vector<StatePtr> GraphSearch<Dim>::getPath() const
{
    return path_;
}

template <int Dim> std::vector<StatePtr> GraphSearch<Dim>::getOpenSet() const
{
    std::vector<StatePtr> ss;
    for (const auto &it : hm_)
    {
        if (it && it->opened && !it->closed)
            ss.push_back(it);
    }
    return ss;
}

template <int Dim> std::vector<StatePtr> GraphSearch<Dim>::getCloseSet() const
{
    std::vector<StatePtr> ss;
    for (const auto &it : hm_)
    {
        if (it && it->closed)
            ss.push_back(it);
    }
    return ss;
}

template <int Dim> std::vector<StatePtr> GraphSearch<Dim>::getAllSet() const
{
    std::vector<StatePtr> ss;
    for (const auto &it : hm_)
    {
        if (it)
            ss.push_back(it);
    }
    return ss;
}

template class DMP::GraphSearch<2>;
template class DMP::GraphSearch<3>;