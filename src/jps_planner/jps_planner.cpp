#include "../../test/timer.hpp"
#include <jps_planner/jps_planner/jps_planner.h>

template <int Dim>
JPSPlanner<Dim>::JPSPlanner(bool verbose) : planner_verbose_(verbose)
{
    planner_verbose_ = verbose;
    if (planner_verbose_)
        printf(ANSI_COLOR_CYAN "JPS PLANNER VERBOSE ON\n" ANSI_COLOR_RESET);
}

template <int Dim>
void JPSPlanner<Dim>::setMapUtil(
    const std::shared_ptr<JPS::MapUtil<Dim>> &map_util)
{
    map_util_ = map_util;
    // aliasing ctor: cmap_ shares map_util_'s refcount but points at its map_
    // data
    cmap_ = std::shared_ptr<const JPS::Tmap>(map_util_, &map_util_->map_);

    const Veci<Dim> dim = map_util_->getDim();
    if (Dim == 3)
    {
        graph_search_ = std::make_shared<JPS::GraphSearch<Dim>>(
            map_util_, dim(0), dim(1), dim(2), 1.0, planner_verbose_);
    }
    else
    {
        graph_search_ = std::make_shared<JPS::GraphSearch<Dim>>(
            map_util_, dim(0), dim(1), 1.0, planner_verbose_);
    }
    graph_search_->setThreshDist(thresh_dist_);
}

template <int Dim> int JPSPlanner<Dim>::status() { return status_; }

template <int Dim> vec_Vecf<Dim> JPSPlanner<Dim>::getPath() { return path_; }

template <int Dim> vec_Vecf<Dim> JPSPlanner<Dim>::getRawPath()
{
    return raw_path_;
}

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::removeCornerPts(const vec_Vecf<Dim> &path)
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

template <int Dim>
vec_Vecf<Dim> JPSPlanner<Dim>::removeLinePts(const vec_Vecf<Dim> &path)
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

template <int Dim> vec_Vecf<Dim> JPSPlanner<Dim>::getOpenSet() const
{
    vec_Vecf<Dim> ps;
    const auto ss = graph_search_->getOpenSet();
    for (const auto &it : ss)
    {
        if (Dim == 3)
        {
            Veci<Dim> pn;
            pn << it->x, it->y, it->z;
            ps.push_back(map_util_->intToFloat(pn));
        }
        else
            ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
    }
    return ps;
}

template <int Dim> vec_Vecf<Dim> JPSPlanner<Dim>::getCloseSet() const
{
    vec_Vecf<Dim> ps;
    const auto ss = graph_search_->getCloseSet();
    for (const auto &it : ss)
    {
        if (Dim == 3)
        {
            Veci<Dim> pn;
            pn << it->x, it->y, it->z;
            ps.push_back(map_util_->intToFloat(pn));
        }
        else
            ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
    }
    return ps;
}

template <int Dim> vec_Vecf<Dim> JPSPlanner<Dim>::getAllSet() const
{
    vec_Vecf<Dim> ps;
    const auto ss = graph_search_->getAllSet();
    for (const auto &it : ss)
    {
        if (Dim == 3)
        {
            Veci<Dim> pn;
            pn << it->x, it->y, it->z;
            ps.push_back(map_util_->intToFloat(pn));
        }
        else
            ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
    }
    return ps;
}

template <int Dim>
bool JPSPlanner<Dim>::plan(const Vecf<Dim> &start, const Vecf<Dim> &goal,
                           decimal_t eps, bool use_jps)
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

    // Refresh the virtual ceiling/floor z indices from the current map. The map
    // may have slid since setMapUtil()/the last plan(), so these bounds cannot
    // be cached in the ctor — recompute them here before any isOutside() check.
    // TODO: the virtual ceiling/floor might be removed
    map_util_->updateVirtualCeilingFloor();

    // If start/goal fall outside the local map, project them onto the map
    // edge along the line toward a seed point, then snap to nearest free cell
    Vecf<Dim> local_start = start;
    Vecf<Dim> local_goal = goal;
    const bool start_out_of_map =
        map_util_->isOutside(map_util_->floatToInt(start));

    if (start_out_of_map)
    {
        if (planner_verbose_)
            printf(ANSI_COLOR_RED "Start point is out of map, find a waypoint "
                                  "to the map edge.\n" ANSI_COLOR_RESET);
        Vecf<Dim> map_min, map_max;
        map_util_->getLocalMapBound(map_min, map_max);
        Vecf<Dim> map_center = (map_min + map_max) / 2;
        Vecf<Dim> hit;
        if (map_util_->lineIntersectMapBound(start, map_center, hit))
        {
            Vecf<Dim> dir = (hit - start).normalized();
            decimal_t dis = (hit - start).norm();
            Vecf<Dim> shifted = start + dir * (dis + map_util_->getRes() * 2);
            if (!map_util_->getNearestKnownFreePos(shifted, local_start))
            {
                if (planner_verbose_)
                {
                    printf(ANSI_COLOR_RED
                           "Start point deeply occupied, cannot find feasible "
                           "path.\n" ANSI_COLOR_RESET);
                }
                status_ = 1;
                return false;
            }
        }
    }

    const bool goal_out_of_map =
        map_util_->isOutside(map_util_->floatToInt(goal));
    if (goal_out_of_map)
    {
        Vecf<Dim> map_min, map_max;
        map_util_->getLocalMapBound(map_min, map_max);
        Vecf<Dim> seed_pt = start_out_of_map ? (map_min + map_max) / 2 : start;
        Vecf<Dim> hit;
        if (map_util_->lineIntersectMapBound(goal, seed_pt, hit))
        {
            Vecf<Dim> dir = (hit - goal).normalized();
            decimal_t dis = (hit - goal).norm();
            Vecf<Dim> shifted = goal + dir * (dis + 2.5 * map_util_->getRes());
            if (!map_util_->getNearestKnownFreePos(shifted, local_goal))
            {
                if (planner_verbose_)
                {
                    printf(ANSI_COLOR_RED
                           "Goal point deeply occupied, cannot find feasible "
                           "path.\n" ANSI_COLOR_RESET);
                }
                status_ = 2;
                return false;
            }
        }
        if (planner_verbose_)
        {
            std::cout << "Shifted goal point is " << local_goal.transpose()
                      << std::endl;
        }
    }

    const Veci<Dim> start_int = map_util_->floatToInt(local_start);
    if (!map_util_->isFree(start_int, thresh_dist_))
    {
        if (planner_verbose_)
        {
            if (map_util_->isOccupied(start_int, thresh_dist_))
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

    const Veci<Dim> goal_int = map_util_->floatToInt(local_goal);
    if (!map_util_->isFree(goal_int, thresh_dist_))
    {
        if (planner_verbose_)
            printf(ANSI_COLOR_RED "goal is not free!\n" ANSI_COLOR_RESET);
        status_ = 2;
        return false;
    }

    // Reuse the persistent graph_search_ built once in setMapUtil(). It resets
    // its own per-search state (generation token, pq_, path_, pool indices) at
    // the top of plan(), so no reallocation is needed here. eps and thresh_dist
    // are (re)applied per-plan since the object outlives a single search.
    JPS::Timer time_search(true);
    graph_search_->setEps(eps);
    graph_search_->setThreshDist(thresh_dist_);
    if (Dim == 3)
    {
        graph_search_->plan(start_int(0), start_int(1), start_int(2),
                            goal_int(0), goal_int(1), goal_int(2), use_jps);
    }
    else
    {
        graph_search_->plan(start_int(0), start_int(1), goal_int(0),
                            goal_int(1), use_jps);
    }
    double dt_search = time_search.Elapsed().count();
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
        return false;
    }

    //**** raw path, s --> g
    vec_Vecf<Dim> ps;
    for (const auto &it : path)
    {
        if (Dim == 3)
        {
            Veci<Dim> pn;
            pn << it->x, it->y, it->z;
            ps.push_back(map_util_->intToFloat(pn));
        }
        else
            ps.push_back(map_util_->intToFloat(Veci<Dim>(it->x, it->y)));
    }

    raw_path_ = ps;
    std::reverse(std::begin(raw_path_), std::end(raw_path_));

    // Simplify the raw path
    // path_ = removeLinePts(raw_path_);
    // path_ = removeCornerPts(path_);
    path_ = removeCornerPts(raw_path_);
    std::reverse(std::begin(path_), std::end(path_));
    path_ = removeCornerPts(path_);
    std::reverse(std::begin(path_), std::end(path_));
    path_ = removeLinePts(path_);

    return true;
}

template class JPSPlanner<2>;

template class JPSPlanner<3>;
