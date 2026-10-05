#include <jps_basis/timer.hpp>
#include <cstdlib>
#include <iostream>
#include <string>
#include <jps_planner/jps_planner/jps_planner.h>

template <int Dim, typename ValueT>
JPSPlanner<Dim, ValueT>::JPSPlanner(bool verbose) : planner_verbose_(verbose)
{
    planner_verbose_ = verbose;
    if (planner_verbose_)
        printf(ANSI_COLOR_CYAN "JPS PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template <int Dim, typename ValueT> int JPSPlanner<Dim, ValueT>::status()
{
    return status_;
}

template <int Dim, typename ValueT>
vec_Vecf<Dim> JPSPlanner<Dim, ValueT>::getPath()
{
    return path_;
}

template <int Dim, typename ValueT>
vec_Vecf<Dim> JPSPlanner<Dim, ValueT>::getRawPath()
{
    return raw_path_;
}

template <int Dim, typename ValueT> void JPSPlanner<Dim, ValueT>::updateMap()
{
    // lo / hi -= map_util_-> lo_d / hi_d

    // (The former full-map copy into cmap_ was never read; removed.)
    if (!fast_mode_ || !map_util_)
        return;
    JPS::Timer t(true);
    map_util_->updateVirtualCeilingFloor();
    
    // get map bounds and convert into voxel coords
    Vecf<Dim> lo_d, hi_d;
    map_util_->getLocalMapBound(lo_d, hi_d);
    Veci<Dim> lo = map_util_->floatToInt(lo_d);
    Veci<Dim> hi = map_util_->floatToInt(hi_d);
    

    // increases low / decreases hi until the integer values fail "isOutside()" check.
    auto trim = [&](int i)
    {
        const Veci<Dim> mid = (lo + hi) / 2;
        Veci<Dim> probe = mid;
        probe(i) = hi(i);
        while (hi(i) > lo(i) && map_util_->isOutside(probe))
            probe(i) = --hi(i);
        probe = mid;
        probe(i) = lo(i);
        while (lo(i) < hi(i) && map_util_->isOutside(probe))
            probe(i) = ++lo(i);
    };
    // deals with virtual ceiling (z) first
    trim(Dim - 1);
    // deals with other axes
    for (int i = 0; i < Dim - 1; ++i)
        trim(i);
    if (planner_verbose_)
        std::cout << "snapshot box lo " << lo.transpose() << " hi "
                  << hi.transpose() << std::endl;
    snap_lo_ = lo;
    snap_dim_ = hi - lo + Veci<Dim>::Ones();
    // create snapshotOccupancy
    map_util_->snapshotOccupancy(lo, hi, thresh_val_, occ_);
    snapshot_ms_ = t.ElapsedMs();
}

template <int Dim, typename ValueT>
bool JPSPlanner<Dim, ValueT>::setSearchBox(const Veci<Dim> &lo,
                                           const Veci<Dim> &hi)
{
    box_en_ = true;
    box_lo_ = lo;
    box_hi_ = hi;
    const Veci<Dim> snap_hi = snap_lo_ + snap_dim_ - Veci<Dim>::Ones();
    return (lo.array() <= snap_lo_.array()).all() &&
           (hi.array() >= snap_hi.array()).all();
}

template <int Dim, typename ValueT>
Vecf<Dim> JPSPlanner<Dim, ValueT>::cellToWorld(const JPS::StatePtr &it) const
{
    Veci<Dim> pn;
    if constexpr (Dim == 3)
        pn << it->x, it->y, it->z;
    else
        pn << it->x, it->y;
    if (fast_mode_)
        pn += snap_lo_;
    return map_util_->intToFloat(pn);
}

template <int Dim, typename ValueT>
vec_Vecf<Dim>
JPSPlanner<Dim, ValueT>::removeCornerPts(const vec_Vecf<Dim> &path)
{
    if (path.size() < 2)
        return path;

    // cut zigzag segment
    vec_Vecf<Dim> optimized_path;
    Vecf<Dim> pose1 = path[0];
    Vecf<Dim> pose2 = path[1];
    Vecf<Dim> prev_pose = pose1;
    optimized_path.push_back(pose1);
    decimal_t cost1, cost2, cost3;

    if (!map_util_->isBlocked(pose1, pose2))
        cost1 = (pose1 - pose2).norm();
    else
        cost1 = std::numeric_limits<decimal_t>::infinity();

    for (unsigned int i = 1; i < path.size() - 1; i++)
    {
        pose1 = path[i];
        pose2 = path[i + 1];
        if (!map_util_->isBlocked(pose1, pose2))
            cost2 = (pose1 - pose2).norm();
        else
            cost2 = std::numeric_limits<decimal_t>::infinity();

        if (!map_util_->isBlocked(prev_pose, pose2))
            cost3 = (prev_pose - pose2).norm();
        else
            cost3 = std::numeric_limits<decimal_t>::infinity();

        if (cost3 < cost1 + cost2)
            cost1 = cost3;
        else
        {
            optimized_path.push_back(path[i]);
            cost1 = (pose1 - pose2).norm();
            prev_pose = pose1;
        }
    }

    optimized_path.push_back(path.back());
    return optimized_path;
}

template <int Dim, typename ValueT>
vec_Vecf<Dim> JPSPlanner<Dim, ValueT>::removeLinePts(const vec_Vecf<Dim> &path)
{
    if (path.size() < 3)
        return path;

    vec_Vecf<Dim> new_path;
    new_path.push_back(path.front());
    for (unsigned int i = 1; i < path.size() - 1; i++)
    {
        Vecf<Dim> p = (path[i + 1] - path[i]) - (path[i] - path[i - 1]);
        if (Dim == 3)
        {
            if (fabs(p(0)) + fabs(p(1)) + fabs(p(2)) > 1e-2)
                new_path.push_back(path[i]);
        }
        else
        {
            if (fabs(p(0)) + fabs(p(1)) > 1e-2)
                new_path.push_back(path[i]);
        }
    }
    new_path.push_back(path.back());
    return new_path;
}

template <int Dim, typename ValueT>
vec_Vecf<Dim> JPSPlanner<Dim, ValueT>::getOpenSet() const
{
    vec_Vecf<Dim> ps;
    const auto ss = graph_search_->getOpenSet();
    for (const auto &it : ss)
    {
        ps.push_back(cellToWorld(it));
    }
    return ps;
}

template <int Dim, typename ValueT>
vec_Vecf<Dim> JPSPlanner<Dim, ValueT>::getCloseSet() const
{
    vec_Vecf<Dim> ps;
    const auto ss = graph_search_->getCloseSet();
    for (const auto &it : ss)
    {
        ps.push_back(cellToWorld(it));
    }
    return ps;
}

template <int Dim, typename ValueT>
vec_Vecf<Dim> JPSPlanner<Dim, ValueT>::getAllSet() const
{
    vec_Vecf<Dim> ps;
    const auto ss = graph_search_->getAllSet();
    for (const auto &it : ss)
    {
        ps.push_back(cellToWorld(it));
    }
    return ps;
}

template <int Dim, typename ValueT>
bool JPSPlanner<Dim, ValueT>::plan(const Vecf<Dim> &start,
                                   const Vecf<Dim> &goal, decimal_t eps,
                                   bool use_jps)
{
    if (!map_util_)
    {
        if (planner_verbose_)
            printf(
                ANSI_COLOR_RED
                "need to set map_util, call setMapUtil()!\n" ANSI_COLOR_RESET);
        return false;
    }

    if (planner_verbose_)
    {
        std::cout << "Start: " << start.transpose() << std::endl;
        std::cout << "Goal:  " << goal.transpose() << std::endl;
        std::cout << "Epsilon:  " << eps << std::endl;
    }

    path_.clear();
    raw_path_.clear();
    status_ = 0;
    timings_ = Timings();
    JPS::Timer t_total(true), t_phase(true);

    if (fast_mode_ && occ_.empty())
    {
        if (planner_verbose_)
            printf(ANSI_COLOR_RED "fast mode: call updateMap() first!\n" ANSI_COLOR_RESET);
        status_ = -1;
        return false;
    }

    // In fast mode the search runs on the snapshot, so start and goal are
    // checked against it below instead of the live map.
    const Veci<Dim> start_int = map_util_->floatToInt(start);
    if (!fast_mode_ && !map_util_->isFree(start_int, thresh_val_))
    {
        if (planner_verbose_)
        {
            if (map_util_->isOccupied(start_int, thresh_val_))
                printf(ANSI_COLOR_RED "start is occupied!\n" ANSI_COLOR_RESET);
            else if (map_util_->isUnknown(start_int))
                printf(ANSI_COLOR_RED "start is unknown!\n" ANSI_COLOR_RESET);
            else
            {
                printf(ANSI_COLOR_RED "start is outside!\n" ANSI_COLOR_RESET);
                std::cout << "startI: " << start_int.transpose() << std::endl;
                std::cout << "Map origin: "
                          << map_util_->getOrigin().transpose() << std::endl;
                std::cout << "Map dim: " << map_util_->getDim().transpose()
                          << std::endl;
            }
        }
        status_ = 1;
        return false;
    }

    // Early exits if the goal index is not free
    const Veci<Dim> goal_int = map_util_->floatToInt(goal);
    if (!fast_mode_ && !map_util_->isFree(goal_int, thresh_val_))
    {
        if (planner_verbose_)
            printf(ANSI_COLOR_RED "goal is not free!\n" ANSI_COLOR_RESET);
        status_ = 2;
        return false;
    }

    // timing to check if start/goal is free, marked as phase
    timings_.check_ms = t_phase.ElapsedMs();
    t_phase.Reset();

    // Reuse the persistent graph_search_ across plan() calls, only rebuilding
    // it if the map dimensions changed since it was last built.
    JPS::Timer time_search(true);

    // TODO: validate bounds check for fast_mode_
    const Veci<Dim> dim = fast_mode_ ? snap_dim_ : map_util_->getDim();
    Veci<Dim> start_l = start_int, goal_l = goal_int;
    if (fast_mode_)
    {
        start_l -= snap_lo_;
        goal_l -= snap_lo_;
        if ((start_l.array() < 0).any() || (start_l.array() >= dim.array()).any())
        {
            status_ = 1;
            return false;
        }
        if ((goal_l.array() < 0).any() || (goal_l.array() >= dim.array()).any())
        {
            status_ = 2;
            return false;
        }
        // Snapshot layout: 3D z-fastest, 2D x-fastest (see
        // MapUtil::snapshotOccupancy).
        const auto snapId = [&](const Veci<Dim> &p)
        {
            if constexpr (Dim == 3)
                return (static_cast<size_t>(p(0)) * dim(1) + p(1)) * dim(2) + p(2);
            else
                return static_cast<size_t>(p(0)) + static_cast<size_t>(dim(0)) * p(1);
        };
        if (occ_[snapId(start_l)])
        {
            if (planner_verbose_)
                printf(ANSI_COLOR_RED "start is blocked in the snapshot!\n" ANSI_COLOR_RESET);
            status_ = 1;
            return false;
        }
        if (occ_[snapId(goal_l)])
        {
            if (planner_verbose_)
                printf(ANSI_COLOR_RED "goal is blocked in the snapshot!\n" ANSI_COLOR_RESET);
            status_ = 2;
            return false;
        }
    }

    // Check if dimensions have changed - if so, create a new graph_search_ with new dimensions and map_util_
    bool dims_changed;
    if (Dim == 3)
        dims_changed = dim(0) != graph_search_dim_x_ ||
                       dim(1) != graph_search_dim_y_ ||
                       dim(2) != graph_search_dim_z_;
    else
        dims_changed =
            dim(0) != graph_search_dim_x_ || dim(1) != graph_search_dim_y_;

    if (!graph_search_ || dims_changed)
    {
        if (Dim == 3)
            graph_search_ = std::make_shared<JPS::GraphSearch<Dim, ValueT>>(
                map_util_, dim(0), dim(1), dim(2), eps, planner_verbose_);
        else
            graph_search_ = std::make_shared<JPS::GraphSearch<Dim, ValueT>>(
                map_util_, dim(0), dim(1), eps, planner_verbose_);
        graph_search_dim_x_ = dim(0);
        graph_search_dim_y_ = dim(1);
        if (Dim == 3)
            graph_search_dim_z_ = dim(2);
    }

    graph_search_->setEps(eps);
    graph_search_->setThreshVal(thresh_val_);
    graph_search_->setFlatSnapshot(fast_mode_ ? occ_.data() : nullptr);
    if (fast_mode_ && box_en_)
        graph_search_->setSearchBox(box_lo_ - snap_lo_, box_hi_ - snap_lo_,
                                    true);
    else
        graph_search_->setSearchBox(Veci<Dim>(), Veci<Dim>(), false);
    timings_.build_ms = t_phase.ElapsedMs();
    t_phase.Reset();

    if (Dim == 3)
        graph_search_->plan(start_l(0), start_l(1), start_l(2), goal_l(0),
                            goal_l(1), goal_l(2), use_jps, max_expand_);
    else
        graph_search_->plan(start_l(0), start_l(1), goal_l(0), goal_l(1),
                            use_jps, max_expand_);
    if (graph_search_->partial())
        status_ = 3;

    timings_.search_ms = t_phase.ElapsedMs();
    t_phase.Reset();
    {
        const auto &st = graph_search_->stats();
        timings_.expand = st.expand;
        timings_.succ = st.succ;
        timings_.jump_steps = st.jump_steps;
        timings_.cell_queries = st.cell_queries;
        timings_.heap_push = st.heap_push;
    }

    double dt_search = time_search.ElapsedMs();
    if (planner_verbose_)
    {
        printf("Search takes: %f ms\n", dt_search);
        fflush(stdout);
    }

    const auto path = graph_search_->getPath();
    if (path.size() < 1)
    {
        if (planner_verbose_)
            std::cout << ANSI_COLOR_RED "Cannot find a path from "
                      << start.transpose() << " to " << goal.transpose()
                      << " Abort!" ANSI_COLOR_RESET << std::endl;
        status_ = -1;
        timings_.total_ms = t_total.ElapsedMs();
        return false;
    }

    //**** raw path, s --> g
    vec_Vecf<Dim> ps;
    for (const auto &it : path)
        ps.push_back(cellToWorld(it));

    raw_path_ = ps;
    std::reverse(std::begin(raw_path_), std::end(raw_path_));
    timings_.convert_ms = t_phase.ElapsedMs();
    t_phase.Reset();

    // Simplify the raw path
    path_ = removeCornerPts(raw_path_);
    std::reverse(std::begin(path_), std::end(path_));
    path_ = removeCornerPts(path_);
    std::reverse(std::begin(path_), std::end(path_));
    timings_.corner_ms = t_phase.ElapsedMs();
    t_phase.Reset();
    path_ = removeLinePts(path_);
    timings_.line_ms = t_phase.ElapsedMs();
    timings_.total_ms = t_total.ElapsedMs();

    return true;
}

template class JPSPlanner<2, double>;

template class JPSPlanner<3, double>;
