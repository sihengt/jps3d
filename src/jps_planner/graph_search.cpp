#include <cmath>
#include <jps_planner/jps_planner/graph_search.h>

using namespace JPS;

GraphSearch::GraphSearch(const char *cMap, int xDim, int yDim, double eps,
                         bool verbose)
    : cMap_(cMap), xDim_(xDim), yDim_(yDim), eps_(eps), verbose_(verbose)
{
    hm_.resize(xDim_ * yDim_);
    visited_.resize(xDim_ * yDim_, 0);

    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            if (x == 0 && y == 0)
                continue;
            ns_.push_back(std::vector<int>{x, y});
        }
    }

    jn2d_ = std::make_shared<JPS2DNeib>();
}

GraphSearch::GraphSearch(const char *cMap, int xDim, int yDim, int zDim,
                         double eps, bool verbose)
    : cMap_(cMap), xDim_(xDim), yDim_(yDim), zDim_(zDim), eps_(eps),
      verbose_(verbose)
{
    hm_.resize(xDim_ * yDim_ * zDim_);
    visited_.resize(xDim_ * yDim_ * zDim_, 0);

    // Set 3D neighbors
    for (int x = -1; x <= 1; x++)
    {
        for (int y = -1; y <= 1; y++)
        {
            for (int z = -1; z <= 1; z++)
            {
                if (x == 0 && y == 0 && z == 0)
                    continue;
                ns_.push_back(std::vector<int>{x, y, z});
            }
        }
    }
    jn3d_ = std::make_shared<JPS3DNeib>();
}

inline int GraphSearch::coordToId(int x, int y) const { return x + y * xDim_; }

inline int GraphSearch::coordToId(int x, int y, int z) const
{
    return x + y * xDim_ + z * xDim_ * yDim_;
}

inline bool GraphSearch::isFree(int x, int y) const
{
    return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ &&
           cMap_[coordToId(x, y)] == val_free_;
}

inline bool GraphSearch::isFree(int x, int y, int z) const
{
    return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
           cMap_[coordToId(x, y, z)] == val_free_;
}

inline bool GraphSearch::isOccupied(int x, int y) const
{
    return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ &&
           cMap_[coordToId(x, y)] > val_free_;
}

inline bool GraphSearch::isOccupied(int x, int y, int z) const
{
    return x >= 0 && x < xDim_ && y >= 0 && y < yDim_ && z >= 0 && z < zDim_ &&
           cMap_[coordToId(x, y, z)] > val_free_;
}

inline double GraphSearch::getHeur(int x, int y) const
{
    int dx = std::abs(x - xGoal_);
    int dy = std::abs(y - yGoal_);
    return eps_ * (dx + dy + (SQRT2 - 2.0) * std::min(dx, dy));
}

inline double GraphSearch::getHeur(int x, int y, int z) const
{
    int dx = std::abs(x - xGoal_);
    int dy = std::abs(y - yGoal_);
    int dz = std::abs(z - zGoal_);
    int dmin = std::min({dx, dy, dz});
    int dmax = std::max({dx, dy, dz});
    int dmid = dx + dy + dz - dmin - dmax;
    return eps_ * (SQRT3 * dmin + SQRT2 * (dmid - dmin) + 1.0 * (dmax - dmid));
}

bool GraphSearch::plan(int xStart, int yStart, int xGoal, int yGoal,
                       bool useJps, int maxExpand)
{
    use_2d_ = true;
    pq_.clear();
    path_.clear();

    current_planning_token_++;
    if (current_planning_token_ == 0) // wrapped after 65535 plans
    {
        std::fill(visited_.begin(), visited_.end(), 0);
        current_planning_token_ = 1;
    }
    current_block_idx_ = 0;
    current_slot_idx_ = 0;

    // Set jps
    use_jps_ = useJps;

    // Set goal
    int goal_id = coordToId(xGoal, yGoal);
    xGoal_ = xGoal;
    yGoal_ = yGoal;

    // Set start node
    int start_id = coordToId(xStart, yStart);
    StatePtr currNode_ptr = allocateState(start_id, xStart, yStart, 0, 0);
    currNode_ptr->g = 0;
    currNode_ptr->h = getHeur(xStart, yStart);

    return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

bool GraphSearch::plan(int xStart, int yStart, int zStart, int xGoal, int yGoal,
                       int zGoal, bool useJps, int maxExpand)
{
    use_2d_ = false;
    pq_.clear();
    path_.clear();

    current_planning_token_++;
    if (current_planning_token_ == 0) // wrapped after 65535 plans
    {
        std::fill(visited_.begin(), visited_.end(), 0);
        current_planning_token_ = 1;
    }
    current_block_idx_ = 0;
    current_slot_idx_ = 0;

    // Set jps
    use_jps_ = useJps;

    // Set goal
    int goal_id = coordToId(xGoal, yGoal, zGoal);
    xGoal_ = xGoal;
    yGoal_ = yGoal;
    zGoal_ = zGoal;
    // Set start node
    int start_id = coordToId(xStart, yStart, zStart);
    StatePtr currNode_ptr =
        allocateState(start_id, xStart, yStart, zStart, 0, 0, 0);
    currNode_ptr->g = 0;
    currNode_ptr->h = getHeur(xStart, yStart, zStart);

    return plan(currNode_ptr, maxExpand, start_id, goal_id);
}

bool GraphSearch::plan(StatePtr &currNode_ptr, int maxExpand, int start_id,
                       int goal_id)
{
    // Insert start node
    currNode_ptr->heapkey = pq_.push(currNode_ptr);
    currNode_ptr->opened = true;
    hm_[currNode_ptr->id] = currNode_ptr;
    visited_[currNode_ptr->id] = current_planning_token_;

    int expand_iteration = 0;

    // Hoisted out of the loop — these used to be constructed/destroyed on
    // every single expansion, forcing a heap allocation per iteration. Now
    // they're cleared and reused; their capacity stabilizes after the first
    // few iterations.
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
        if (!use_jps_)
            getSucc(currNode_ptr, succ_ids, succ_costs);
        else
            getJpsSucc(currNode_ptr, succ_ids, succ_costs);

        // if(verbose_)
        // printf("size of succs: %zu\n", succ_ids.size());
        // Process successors
        for (int s = 0; s < (int)succ_ids.size(); s++)
        {
            // see if we can improve the value of succstate
            StatePtr &child_ptr = hm_[succ_ids[s]];
            double tentative_gval = currNode_ptr->g + succ_costs[s];

            // Epsilon added for floating-point robustness now that edge costs
            // can be irrational (sqrt(2)/sqrt(3) sums).
            if (tentative_gval < child_ptr->g - 1e-9)
            {
                child_ptr->parentId = currNode_ptr->id; // Assign new parent
                child_ptr->g = tentative_gval;          // Update gval

                // double fval = child_ptr->g + child_ptr->h;

                // if currently in OPEN, update
                if (child_ptr->opened && !child_ptr->closed)
                {
                    pq_.increase(child_ptr->heapkey); // update heap

                    // BUG FIX: this block used to unconditionally recompute
                    // child_ptr->dx/dy/dz as sign(child - curr), which is only
                    // valid for plain A* unit-step grid moves. For JPS, a step
                    // between parent and child is a multi-cell jump (e.g.
                    // dx=5), and collapsing that to its sign here corrupts the
                    // travel direction stored on the state. getJpsSucc() uses
                    // curr->dx/dy/dz to index into the jn2d_/jn3d_ pruning
                    // tables on the node's *next* expansion, so a corrupted
                    // direction silently produces wrong successors/pruning,
                    // including re-discovering already-CLOSED nodes with a
                    // lower g -- exactly what triggers "ASTAR ERROR!" below.
                    // For JPS, dx/dy/dz was already set correctly when the
                    // child was first allocated (in getJpsSucc, from the
                    // actual jump direction) and must be left untouched.
                    if (!use_jps_)
                    {
                        child_ptr->dx = (child_ptr->x - currNode_ptr->x);
                        child_ptr->dy = (child_ptr->y - currNode_ptr->y);
                        if (!use_2d_)
                            child_ptr->dz = (child_ptr->z - currNode_ptr->z);
                        if (child_ptr->dx != 0)
                            child_ptr->dx /= std::abs(child_ptr->dx);
                        if (child_ptr->dy != 0)
                            child_ptr->dy /= std::abs(child_ptr->dy);
                        if (!use_2d_ && child_ptr->dz != 0)
                            child_ptr->dz /= std::abs(child_ptr->dz);
                    }
                }
                // if currently in CLOSED
                else if (child_ptr->opened && child_ptr->closed)
                {
                    if (verbose_)
                        printf("ASTAR ERROR! Reopened closed node id=%d\n",
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

        if (maxExpand > 0 && expand_iteration >= maxExpand)
        {
            if (verbose_)
                printf("MaxExpandStep [%d] Reached!!!!!!\n\n", maxExpand);
            return false;
        }

        if (pq_.empty())
        {
            if (verbose_)
                printf("Priority queue is empty!!!!!!\n\n");
            return false;
        }
    }

    if (verbose_)
    {
        printf("goal g: %f, h: %f!\n", currNode_ptr->g, currNode_ptr->h);
        printf("Expand [%d] nodes!\n", expand_iteration);
    }

    path_ = recoverPath(currNode_ptr, start_id);

    return true;
}

std::vector<StatePtr> GraphSearch::recoverPath(StatePtr node, int start_id)
{
    std::vector<StatePtr> path;
    path.push_back(node);
    while (node && node->id != start_id)
    {
        node = hm_[node->parentId];
        path.push_back(node);
    }

    return path;
}

void GraphSearch::getSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                          std::vector<double> &succ_costs)
{
    succ_ids.reserve(ns_.size());
    succ_costs.reserve(ns_.size());

    if (use_2d_)
    {
        for (const auto &d : ns_)
        {
            int new_x = curr->x + d[0];
            int new_y = curr->y + d[1];
            if (!isFree(new_x, new_y))
                continue;

            int new_id = coordToId(new_x, new_y);
            if (visited_[new_id] != current_planning_token_)
            {
                visited_[new_id] = current_planning_token_;
                hm_[new_id] = allocateState(new_id, new_x, new_y, d[0], d[1]);
                hm_[new_id]->h = getHeur(new_x, new_y);
            }

            succ_ids.push_back(new_id);
            succ_costs.push_back(std::sqrt(d[0] * d[0] + d[1] * d[1]));
        }
    }
    else
    {
        for (const auto &d : ns_)
        {
            int new_x = curr->x + d[0];
            int new_y = curr->y + d[1];
            int new_z = curr->z + d[2];
            if (!isFree(new_x, new_y, new_z))
                continue;

            int new_id = coordToId(new_x, new_y, new_z);
            if (visited_[new_id] != current_planning_token_)
            {
                visited_[new_id] = current_planning_token_;
                hm_[new_id] = allocateState(new_id, new_x, new_y, new_z,
                                            d[0], d[1], d[2]);
                hm_[new_id]->h = getHeur(new_x, new_y, new_z);
            }

            succ_ids.push_back(new_id);
            succ_costs.push_back(
                std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]));
        }
    }
}

void GraphSearch::getJpsSucc(const StatePtr &curr, std::vector<int> &succ_ids,
                             std::vector<double> &succ_costs)
{
    if (use_2d_)
    {
        const int norm1 = std::abs(curr->dx) + std::abs(curr->dy);
        int num_neib = jn2d_->nsz[norm1][0];
        int num_fneib = jn2d_->nsz[norm1][1];
        int id = (curr->dx + 1) + 3 * (curr->dy + 1);

        succ_ids.reserve(num_neib + num_fneib);
        succ_costs.reserve(num_neib + num_fneib);

        for (int dev = 0; dev < num_neib + num_fneib; ++dev)
        {
            int new_x, new_y;
            int dx, dy;
            if (dev < num_neib)
            {
                dx = jn2d_->ns[id][0][dev];
                dy = jn2d_->ns[id][1][dev];
                if (!jump(curr->x, curr->y, dx, dy, new_x, new_y))
                    continue;
            }
            else
            {
                int nx = curr->x + jn2d_->f1[id][0][dev - num_neib];
                int ny = curr->y + jn2d_->f1[id][1][dev - num_neib];
                if (isOccupied(nx, ny))
                {
                    dx = jn2d_->f2[id][0][dev - num_neib];
                    dy = jn2d_->f2[id][1][dev - num_neib];
                    if (!jump(curr->x, curr->y, dx, dy, new_x, new_y))
                        continue;
                }
                else
                    continue;
            }

            int new_id = coordToId(new_x, new_y);
            if (visited_[new_id] != current_planning_token_)
            {
                visited_[new_id] = current_planning_token_;
                hm_[new_id] = allocateState(new_id, new_x, new_y, dx, dy);
                hm_[new_id]->h = getHeur(new_x, new_y);
            }

            succ_ids.push_back(new_id);

            // JPS jumps are always axis-aligned or diagonal, so cost is always
            // (steps * unit_cost) where unit_cost is 1.0 or SQRT2 — exact for
            // cardinal and diagonal moves, avoids a sqrt() call per successor.
            const int adx = std::abs(new_x - curr->x);
            const int ady = std::abs(new_y - curr->y);
            const double unit = (adx == ady) ? GraphSearch::SQRT2 : 1.0;
            succ_costs.push_back(unit * std::max(adx, ady));
        }
    }
    else
    {
        const int norm1 =
            std::abs(curr->dx) + std::abs(curr->dy) + std::abs(curr->dz);
        int num_neib = jn3d_->nsz[norm1][0];
        int num_fneib = jn3d_->nsz[norm1][1];
        int id = (curr->dx + 1) + 3 * (curr->dy + 1) + 9 * (curr->dz + 1);

        succ_ids.reserve(num_neib + num_fneib);
        succ_costs.reserve(num_neib + num_fneib);

        for (int dev = 0; dev < num_neib + num_fneib; ++dev)
        {
            int new_x, new_y, new_z;
            int dx, dy, dz;
            if (dev < num_neib)
            {
                dx = jn3d_->ns[id][0][dev];
                dy = jn3d_->ns[id][1][dev];
                dz = jn3d_->ns[id][2][dev];
                if (!jump(curr->x, curr->y, curr->z, dx, dy, dz, new_x, new_y,
                          new_z))
                    continue;
            }
            else
            {
                int nx = curr->x + jn3d_->f1[id][0][dev - num_neib];
                int ny = curr->y + jn3d_->f1[id][1][dev - num_neib];
                int nz = curr->z + jn3d_->f1[id][2][dev - num_neib];
                if (isOccupied(nx, ny, nz))
                {
                    dx = jn3d_->f2[id][0][dev - num_neib];
                    dy = jn3d_->f2[id][1][dev - num_neib];
                    dz = jn3d_->f2[id][2][dev - num_neib];
                    if (!jump(curr->x, curr->y, curr->z, dx, dy, dz, new_x,
                              new_y, new_z))
                        continue;
                }
                else
                    continue;
            }

            int new_id = coordToId(new_x, new_y, new_z);
            if (visited_[new_id] != current_planning_token_)
            {
                visited_[new_id] = current_planning_token_;
                hm_[new_id] = allocateState(new_id, new_x, new_y, new_z,
                                            dx, dy, dz);
                hm_[new_id]->h = getHeur(new_x, new_y, new_z);
            }

            succ_ids.push_back(new_id);

            // Same reasoning as the 2D branch: direction is cardinal,
            // face-diagonal, or body-diagonal, so cost is steps * unit_cost.
            const int adx = std::abs(new_x - curr->x);
            const int ady = std::abs(new_y - curr->y);
            const int adz = std::abs(new_z - curr->z);
            const int steps = std::max({adx, ady, adz});
            const int axes = (adx > 0) + (ady > 0) + (adz > 0);
            const double unit = (axes == 1) ? 1.0 : (axes == 2) ? GraphSearch::SQRT2 : GraphSearch::SQRT3;
            succ_costs.push_back(unit * steps);
        }
    }
}

bool GraphSearch::jump(int x, int y, int dx, int dy, int &new_x, int &new_y)
{
    // Constant for the entire corridor walk below — compute once.
    const int id = (dx + 1) + 3 * (dy + 1);
    const int norm1 = std::abs(dx) + std::abs(dy);
    const int num_neib = jn2d_->nsz[norm1][0];
    const int *sub_dx = jn2d_->ns[id][0];
    const int *sub_dy = jn2d_->ns[id][1];

    // Iterative corridor walk — replaces tail recursion, avoiding stack
    // growth on long straight corridors. Sub-direction probes still recurse
    // below, bounded to 2 live frames (diagonal -> cardinal -> no sub-calls).
    while (true)
    {
        new_x = x + dx;
        new_y = y + dy;

        if (!isFree(new_x, new_y))
            return false;

        // No corner cutting: a diagonal step may only be taken if it does
        // not squeeze diagonally between two blocked cells.
        if (cutsCorner(x, y, dx, dy))
            return false;

        if (new_x == xGoal_ && new_y == yGoal_)
            return true;

        if (hasForcedWithId(new_x, new_y, id, norm1))
            return true;

        for (int k = 0; k < num_neib - 1; ++k)
        {
            int sub_new_x, sub_new_y;
            if (jump(new_x, new_y, sub_dx[k], sub_dy[k], sub_new_x, sub_new_y))
                return true;
        }

        x = new_x;
        y = new_y;
    }
}

bool GraphSearch::jump(int x, int y, int z, int dx, int dy, int dz, int &new_x,
                       int &new_y, int &new_z)
{
    const int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
    const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
    const int num_neib = jn3d_->nsz[norm1][0];
    const int *sub_dx = jn3d_->ns[id][0];
    const int *sub_dy = jn3d_->ns[id][1];
    const int *sub_dz = jn3d_->ns[id][2];

    // Iterative corridor walk. Sub-direction probes still recurse below;
    // recursion depth is bounded because sub-directions always have
    // strictly lower norm1 (body-diag -> face-diag -> cardinal -> none),
    // so max live stack depth is 3 frames regardless of grid size.
    while (true)
    {
        new_x = x + dx;
        new_y = y + dy;
        new_z = z + dz;

        if (!isFree(new_x, new_y, new_z))
            return false;

        // No corner cutting: a diagonal/body-diagonal step may only be taken
        // if it does not squeeze through blocked bypass cells.
        if (cutsCorner(x, y, z, dx, dy, dz))
            return false;

        if (new_x == xGoal_ && new_y == yGoal_ && new_z == zGoal_)
            return true;

        if (hasForcedWithId(new_x, new_y, new_z, id, norm1))
            return true;

        for (int k = 0; k < num_neib - 1; ++k)
        {
            int sub_new_x, sub_new_y, sub_new_z;
            if (jump(new_x, new_y, new_z, sub_dx[k], sub_dy[k], sub_dz[k],
                     sub_new_x, sub_new_y, sub_new_z))
                return true;
        }

        x = new_x;
        y = new_y;
        z = new_z;
    }
}

inline bool GraphSearch::hasForced(int x, int y, int dx, int dy)
{
    const int id = (dx + 1) + 3 * (dy + 1);
    for (int fn = 0; fn < 2; ++fn)
    {
        int nx = x + jn2d_->f1[id][0][fn];
        int ny = y + jn2d_->f1[id][1][fn];
        if (isOccupied(nx, ny))
            return true;
    }
    return false;
}

inline bool GraphSearch::hasForced(int x, int y, int z, int dx, int dy, int dz)
{
    int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
    int id = (dx + 1) + 3 * (dy + 1) + 9 * (dz + 1);
    switch (norm1)
    {
    case 1:
        // 1-d move, check 8 neighbors
        for (int fn = 0; fn < 8; ++fn)
        {
            int nx = x + jn3d_->f1[id][0][fn];
            int ny = y + jn3d_->f1[id][1][fn];
            int nz = z + jn3d_->f1[id][2][fn];
            if (isOccupied(nx, ny, nz))
                return true;
        }
        return false;
    case 2:
        // 2-d move, check 8 neighbors
        for (int fn = 0; fn < 8; ++fn)
        {
            int nx = x + jn3d_->f1[id][0][fn];
            int ny = y + jn3d_->f1[id][1][fn];
            int nz = z + jn3d_->f1[id][2][fn];
            if (isOccupied(nx, ny, nz))
                return true;
        }
        return false;
    case 3:
        // 3-d move, check 6 neighbors
        for (int fn = 0; fn < 6; ++fn)
        {
            int nx = x + jn3d_->f1[id][0][fn];
            int ny = y + jn3d_->f1[id][1][fn];
            int nz = z + jn3d_->f1[id][2][fn];
            if (isOccupied(nx, ny, nz))
                return true;
        }
        return false;
    default:
        return false;
    }
}

inline bool GraphSearch::hasForcedWithId(int x, int y, int id, int norm1)
{
    const int num_forced = jn2d_->nsz[norm1][1];
    const int *f1x = jn2d_->f1[id][0];
    const int *f1y = jn2d_->f1[id][1];
    for (int fn = 0; fn < num_forced; ++fn)
    {
        if (isOccupied(x + f1x[fn], y + f1y[fn]))
            return true;
    }
    return false;
}

inline bool GraphSearch::hasForcedWithId(int x, int y, int z, int id, int norm1)
{
    const int num_forced = jn3d_->nsz[norm1][1];
    const int *f1x = jn3d_->f1[id][0];
    const int *f1y = jn3d_->f1[id][1];
    const int *f1z = jn3d_->f1[id][2];
    for (int fn = 0; fn < num_forced; ++fn)
    {
        if (isOccupied(x + f1x[fn], y + f1y[fn], z + f1z[fn]))
            return true;
    }
    return false;
}

// 2D no-corner-cut test. For a diagonal step (dx,dy both nonzero) from (x,y),
// the move squeezes through a diagonal gap only if BOTH orthogonal bypass
// cells -- (x+dx, y) and (x, y+dy) -- are blocked. If either is free the
// diagonal is a legitimate move (wall-hugging is allowed). Cardinal moves
// (one of dx,dy is zero) never cut a corner.
inline bool GraphSearch::cutsCorner(int x, int y, int dx, int dy)
{
    if (dx == 0 || dy == 0)
        return false;
    return isOccupied(x + dx, y) && isOccupied(x, y + dy);
}

// 3D no-corner-cut test.
//   - cardinal (norm1==1): never cuts a corner.
//   - face-diagonal (norm1==2): the two nonzero axes form a 2D diagonal in a
//     plane; same rule as 2D applied to those two axes (the third axis is 0).
//   - body-diagonal (norm1==3): blocked only if all three single-axis
//     -reverted bypass cells are occupied (no free route around the squeeze).
inline bool GraphSearch::cutsCorner(int x, int y, int z, int dx, int dy, int dz)
{
    const int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
    if (norm1 <= 1)
        return false;

    if (norm1 == 2)
    {
        if (dx == 0)
            return isOccupied(x, y + dy, z) && isOccupied(x, y, z + dz);
        if (dy == 0)
            return isOccupied(x + dx, y, z) && isOccupied(x, y, z + dz);
        return isOccupied(x + dx, y, z) && isOccupied(x, y + dy, z);
    }

    return isOccupied(x, y + dy, z + dz) &&
           isOccupied(x + dx, y, z + dz) &&
           isOccupied(x + dx, y + dy, z);
}

std::vector<StatePtr> GraphSearch::getPath() const { return path_; }

std::vector<StatePtr> GraphSearch::getOpenSet() const
{
    std::vector<StatePtr> ss;
    for (size_t i = 0; i < hm_.size(); ++i)
    {
        if (visited_[i] == current_planning_token_ && hm_[i] && hm_[i]->opened &&
            !hm_[i]->closed)
            ss.push_back(hm_[i]);
    }
    return ss;
}

std::vector<StatePtr> GraphSearch::getCloseSet() const
{
    std::vector<StatePtr> ss;
    for (size_t i = 0; i < hm_.size(); ++i)
    {
        if (visited_[i] == current_planning_token_ && hm_[i] && hm_[i]->closed)
            ss.push_back(hm_[i]);
    }
    return ss;
}

std::vector<StatePtr> GraphSearch::getAllSet() const
{
    std::vector<StatePtr> ss;
    for (size_t i = 0; i < hm_.size(); ++i)
    {
        if (visited_[i] == current_planning_token_ && hm_[i])
            ss.push_back(hm_[i]);
    }
    return ss;
}

constexpr int JPS2DNeib::nsz[3][2];

JPS2DNeib::JPS2DNeib()
{
    int id = 0;
    for (int dy = -1; dy <= 1; ++dy)
    {
        for (int dx = -1; dx <= 1; ++dx)
        {
            int norm1 = std::abs(dx) + std::abs(dy);
            for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                Neib(dx, dy, norm1, dev, ns[id][0][dev], ns[id][1][dev]);
            for (int dev = 0; dev < nsz[norm1][1]; ++dev)
            {
                FNeib(dx, dy, norm1, dev, f1[id][0][dev], f1[id][1][dev],
                      f2[id][0][dev], f2[id][1][dev]);
            }
            id++;
        }
    }
}

void JPS2DNeib::print()
{
    for (int dx = -1; dx <= 1; dx++)
    {
        for (int dy = -1; dy <= 1; dy++)
        {
            int id = (dx + 1) + 3 * (dy + 1);
            printf("[dx: %d, dy: %d]-->id: %d:\n", dx, dy, id);
            for (unsigned int i = 0;
                 i < sizeof(f1[id][0]) / sizeof(f1[id][0][0]); i++)
                printf("                f1: [%d, %d]\n", f1[id][0][i],
                       f1[id][1][i]);
        }
    }
}

void JPS2DNeib::Neib(int dx, int dy, int norm1, int dev, int &tx, int &ty)
{
    switch (norm1)
    {
    case 0:
        switch (dev)
        {
        case 0:
            tx = 1;
            ty = 0;
            return;
        case 1:
            tx = -1;
            ty = 0;
            return;
        case 2:
            tx = 0;
            ty = 1;
            return;
        case 3:
            tx = 1;
            ty = 1;
            return;
        case 4:
            tx = -1;
            ty = 1;
            return;
        case 5:
            tx = 0;
            ty = -1;
            return;
        case 6:
            tx = 1;
            ty = -1;
            return;
        case 7:
            tx = -1;
            ty = -1;
            return;
        }
    case 1:
        tx = dx;
        ty = dy;
        return;
    case 2:
        switch (dev)
        {
        case 0:
            tx = dx;
            ty = 0;
            return;
        case 1:
            tx = 0;
            ty = dy;
            return;
        case 2:
            tx = dx;
            ty = dy;
            return;
        }
    }
}

void JPS2DNeib::FNeib(int dx, int dy, int norm1, int dev, int &fx, int &fy,
                      int &nx, int &ny)
{
    switch (norm1)
    {
    case 1:
        switch (dev)
        {
        case 0:
            fx = 0;
            fy = 1;
            break;
        case 1:
            fx = 0;
            fy = -1;
            break;
        }

        // switch order if different direction
        if (dx == 0)
            fx = fy, fy = 0;

        nx = dx + fx;
        ny = dy + fy;
        return;
    case 2:
        switch (dev)
        {
        case 0:
            fx = -dx;
            fy = 0;
            nx = -dx;
            ny = dy;
            return;
        case 1:
            fx = 0;
            fy = -dy;
            nx = dx;
            ny = -dy;
            return;
        }
    }
}

constexpr int JPS3DNeib::nsz[4][2];

JPS3DNeib::JPS3DNeib()
{
    int id = 0;
    for (int dz = -1; dz <= 1; ++dz)
    {
        for (int dy = -1; dy <= 1; ++dy)
        {
            for (int dx = -1; dx <= 1; ++dx)
            {
                int norm1 = std::abs(dx) + std::abs(dy) + std::abs(dz);
                for (int dev = 0; dev < nsz[norm1][0]; ++dev)
                    Neib(dx, dy, dz, norm1, dev, ns[id][0][dev], ns[id][1][dev],
                         ns[id][2][dev]);
                for (int dev = 0; dev < nsz[norm1][1]; ++dev)
                {
                    FNeib(dx, dy, dz, norm1, dev, f1[id][0][dev],
                          f1[id][1][dev], f1[id][2][dev], f2[id][0][dev],
                          f2[id][1][dev], f2[id][2][dev]);
                }
                id++;
            }
        }
    }
}

void JPS3DNeib::Neib(int dx, int dy, int dz, int norm1, int dev, int &tx,
                     int &ty, int &tz)
{
    switch (norm1)
    {
    case 0:
        switch (dev)
        {
        case 0:
            tx = 1;
            ty = 0;
            tz = 0;
            return;
        case 1:
            tx = -1;
            ty = 0;
            tz = 0;
            return;
        case 2:
            tx = 0;
            ty = 1;
            tz = 0;
            return;
        case 3:
            tx = 1;
            ty = 1;
            tz = 0;
            return;
        case 4:
            tx = -1;
            ty = 1;
            tz = 0;
            return;
        case 5:
            tx = 0;
            ty = -1;
            tz = 0;
            return;
        case 6:
            tx = 1;
            ty = -1;
            tz = 0;
            return;
        case 7:
            tx = -1;
            ty = -1;
            tz = 0;
            return;
        case 8:
            tx = 0;
            ty = 0;
            tz = 1;
            return;
        case 9:
            tx = 1;
            ty = 0;
            tz = 1;
            return;
        case 10:
            tx = -1;
            ty = 0;
            tz = 1;
            return;
        case 11:
            tx = 0;
            ty = 1;
            tz = 1;
            return;
        case 12:
            tx = 1;
            ty = 1;
            tz = 1;
            return;
        case 13:
            tx = -1;
            ty = 1;
            tz = 1;
            return;
        case 14:
            tx = 0;
            ty = -1;
            tz = 1;
            return;
        case 15:
            tx = 1;
            ty = -1;
            tz = 1;
            return;
        case 16:
            tx = -1;
            ty = -1;
            tz = 1;
            return;
        case 17:
            tx = 0;
            ty = 0;
            tz = -1;
            return;
        case 18:
            tx = 1;
            ty = 0;
            tz = -1;
            return;
        case 19:
            tx = -1;
            ty = 0;
            tz = -1;
            return;
        case 20:
            tx = 0;
            ty = 1;
            tz = -1;
            return;
        case 21:
            tx = 1;
            ty = 1;
            tz = -1;
            return;
        case 22:
            tx = -1;
            ty = 1;
            tz = -1;
            return;
        case 23:
            tx = 0;
            ty = -1;
            tz = -1;
            return;
        case 24:
            tx = 1;
            ty = -1;
            tz = -1;
            return;
        case 25:
            tx = -1;
            ty = -1;
            tz = -1;
            return;
        }
    case 1:
        tx = dx;
        ty = dy;
        tz = dz;
        return;
    case 2:
        switch (dev)
        {
        case 0:
            if (dz == 0)
            {
                tx = 0;
                ty = dy;
                tz = 0;
                return;
            }
            else
            {
                tx = 0;
                ty = 0;
                tz = dz;
                return;
            }
        case 1:
            if (dx == 0)
            {
                tx = 0;
                ty = dy;
                tz = 0;
                return;
            }
            else
            {
                tx = dx;
                ty = 0;
                tz = 0;
                return;
            }
        case 2:
            tx = dx;
            ty = dy;
            tz = dz;
            return;
        }
    case 3:
        switch (dev)
        {
        case 0:
            tx = dx;
            ty = 0;
            tz = 0;
            return;
        case 1:
            tx = 0;
            ty = dy;
            tz = 0;
            return;
        case 2:
            tx = 0;
            ty = 0;
            tz = dz;
            return;
        case 3:
            tx = dx;
            ty = dy;
            tz = 0;
            return;
        case 4:
            tx = dx;
            ty = 0;
            tz = dz;
            return;
        case 5:
            tx = 0;
            ty = dy;
            tz = dz;
            return;
        case 6:
            tx = dx;
            ty = dy;
            tz = dz;
            return;
        }
    }
}

void JPS3DNeib::FNeib(int dx, int dy, int dz, int norm1, int dev, int &fx,
                      int &fy, int &fz, int &nx, int &ny, int &nz)
{
    switch (norm1)
    {
    case 1:
        switch (dev)
        {
        case 0:
            fx = 0;
            fy = 1;
            fz = 0;
            break;
        case 1:
            fx = 0;
            fy = -1;
            fz = 0;
            break;
        case 2:
            fx = 1;
            fy = 0;
            fz = 0;
            break;
        case 3:
            fx = 1;
            fy = 1;
            fz = 0;
            break;
        case 4:
            fx = 1;
            fy = -1;
            fz = 0;
            break;
        case 5:
            fx = -1;
            fy = 0;
            fz = 0;
            break;
        case 6:
            fx = -1;
            fy = 1;
            fz = 0;
            break;
        case 7:
            fx = -1;
            fy = -1;
            fz = 0;
            break;
        }
        nx = fx;
        ny = fy;
        nz = dz;
        // switch order if different direction
        if (dx != 0)
        {
            fz = fx;
            fx = 0;
            nz = fz;
            nx = dx;
        }
        if (dy != 0)
        {
            fz = fy;
            fy = 0;
            nz = fz;
            ny = dy;
        }
        return;
    case 2:
        if (dx == 0)
        {
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = 0;
                fz = -dz;
                nx = 0;
                ny = dy;
                nz = -dz;
                return;
            case 1:
                fx = 0;
                fy = -dy;
                fz = 0;
                nx = 0;
                ny = -dy;
                nz = dz;
                return;
            case 2:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = dy;
                nz = dz;
                return;
            case 3:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = dy;
                nz = dz;
                return;
            case 4:
                fx = 1;
                fy = 0;
                fz = -dz;
                nx = 1;
                ny = dy;
                nz = -dz;
                return;
            case 5:
                fx = 1;
                fy = -dy;
                fz = 0;
                nx = 1;
                ny = -dy;
                nz = dz;
                return;
            case 6:
                fx = -1;
                fy = 0;
                fz = -dz;
                nx = -1;
                ny = dy;
                nz = -dz;
                return;
            case 7:
                fx = -1;
                fy = -dy;
                fz = 0;
                nx = -1;
                ny = -dy;
                nz = dz;
                return;
            // Extras
            case 8:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = dy;
                nz = 0;
                return;
            case 9:
                fx = 1;
                fy = 0;
                fz = 0;
                nx = 1;
                ny = 0;
                nz = dz;
                return;
            case 10:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = dy;
                nz = 0;
                return;
            case 11:
                fx = -1;
                fy = 0;
                fz = 0;
                nx = -1;
                ny = 0;
                nz = dz;
                return;
            }
        }
        else if (dy == 0)
        {
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = 0;
                fz = -dz;
                nx = dx;
                ny = 0;
                nz = -dz;
                return;
            case 1:
                fx = -dx;
                fy = 0;
                fz = 0;
                nx = -dx;
                ny = 0;
                nz = dz;
                return;
            case 2:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = dx;
                ny = 1;
                nz = dz;
                return;
            case 3:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = dx;
                ny = -1;
                nz = dz;
                return;
            case 4:
                fx = 0;
                fy = 1;
                fz = -dz;
                nx = dx;
                ny = 1;
                nz = -dz;
                return;
            case 5:
                fx = -dx;
                fy = 1;
                fz = 0;
                nx = -dx;
                ny = 1;
                nz = dz;
                return;
            case 6:
                fx = 0;
                fy = -1;
                fz = -dz;
                nx = dx;
                ny = -1;
                nz = -dz;
                return;
            case 7:
                fx = -dx;
                fy = -1;
                fz = 0;
                nx = -dx;
                ny = -1;
                nz = dz;
                return;
            // Extras
            case 8:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = dx;
                ny = 1;
                nz = 0;
                return;
            case 9:
                fx = 0;
                fy = 1;
                fz = 0;
                nx = 0;
                ny = 1;
                nz = dz;
                return;
            case 10:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = dx;
                ny = -1;
                nz = 0;
                return;
            case 11:
                fx = 0;
                fy = -1;
                fz = 0;
                nx = 0;
                ny = -1;
                nz = dz;
                return;
            }
        }
        else
        { // dz==0
            switch (dev)
            {
            case 0:
                fx = 0;
                fy = -dy;
                fz = 0;
                nx = dx;
                ny = -dy;
                nz = 0;
                return;
            case 1:
                fx = -dx;
                fy = 0;
                fz = 0;
                nx = -dx;
                ny = dy;
                nz = 0;
                return;
            case 2:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = dx;
                ny = dy;
                nz = 1;
                return;
            case 3:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = dx;
                ny = dy;
                nz = -1;
                return;
            case 4:
                fx = 0;
                fy = -dy;
                fz = 1;
                nx = dx;
                ny = -dy;
                nz = 1;
                return;
            case 5:
                fx = -dx;
                fy = 0;
                fz = 1;
                nx = -dx;
                ny = dy;
                nz = 1;
                return;
            case 6:
                fx = 0;
                fy = -dy;
                fz = -1;
                nx = dx;
                ny = -dy;
                nz = -1;
                return;
            case 7:
                fx = -dx;
                fy = 0;
                fz = -1;
                nx = -dx;
                ny = dy;
                nz = -1;
                return;
            // Extras
            case 8:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = dx;
                ny = 0;
                nz = 1;
                return;
            case 9:
                fx = 0;
                fy = 0;
                fz = 1;
                nx = 0;
                ny = dy;
                nz = 1;
                return;
            case 10:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = dx;
                ny = 0;
                nz = -1;
                return;
            case 11:
                fx = 0;
                fy = 0;
                fz = -1;
                nx = 0;
                ny = dy;
                nz = -1;
                return;
            }
        }
    case 3:
        switch (dev)
        {
        case 0:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = dy;
            nz = dz;
            return;
        case 1:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = dx;
            ny = -dy;
            nz = dz;
            return;
        case 2:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = dx;
            ny = dy;
            nz = -dz;
            return;
        // Need to check up to here for forced!
        case 3:
            fx = 0;
            fy = -dy;
            fz = -dz;
            nx = dx;
            ny = -dy;
            nz = -dz;
            return;
        case 4:
            fx = -dx;
            fy = 0;
            fz = -dz;
            nx = -dx;
            ny = dy;
            nz = -dz;
            return;
        case 5:
            fx = -dx;
            fy = -dy;
            fz = 0;
            nx = -dx;
            ny = -dy;
            nz = dz;
            return;
        // Extras
        case 6:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = 0;
            nz = dz;
            return;
        case 7:
            fx = -dx;
            fy = 0;
            fz = 0;
            nx = -dx;
            ny = dy;
            nz = 0;
            return;
        case 8:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = 0;
            ny = -dy;
            nz = dz;
            return;
        case 9:
            fx = 0;
            fy = -dy;
            fz = 0;
            nx = dx;
            ny = -dy;
            nz = 0;
            return;
        case 10:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = 0;
            ny = dy;
            nz = -dz;
            return;
        case 11:
            fx = 0;
            fy = 0;
            fz = -dz;
            nx = dx;
            ny = 0;
            nz = -dz;
            return;
        }
    }
}
