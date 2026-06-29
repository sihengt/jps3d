#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include <algorithm>
#include <limits>

#include "read_map.hpp"

using namespace JPS;

// Verifies the virtual ceiling actually constrains the JPS/A* search itself
// (via MapUtil::isOccupied -> JPSPlanner::updateMap -> GraphSearch's cmap_),
// not just the start/goal validity check -- the gap in the original
// "just check isFree" suggestion (see design doc).
int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf(ANSI_COLOR_RED "Input yaml required!\n" ANSI_COLOR_RESET);
        return -1;
    }

    MapReader<Vec3i, Vec3f> reader(argv[1], true);
    if (!reader.exist())
    {
        printf(ANSI_COLOR_RED "Cannot read input file [%s]!\n" ANSI_COLOR_RESET,
               argv[1]);
        return -1;
    }

    std::shared_ptr<VoxelMapUtil> map_util = std::make_shared<VoxelMapUtil>();
    map_util->setMap(reader.origin(), reader.dim(), reader.data(),
                     reader.resolution());

    // Ceiling at world Z = 4.0m. On this 5x5x5 @ 1m map (origin z=0), that
    // blocks exactly the top layer (cell z=4) -- which is where this map's
    // own goal cell (4,4,4) lives -- while leaving z=0..3 free.
    map_util->setCeiling(4.0);

    std::unique_ptr<JPSPlanner3D> planner_ptr =
        std::make_unique<JPSPlanner3D>(false);
    planner_ptr->setMapUtil(map_util);
    planner_ptr->updateMap();

    bool all_ok = true;

    // Case 1: the map's own goal sits in the ceiling-blocked top layer. The
    // search must not be able to reach it.
    const Vec3f start(0.5, 0.5, 0.5);
    const Vec3f blocked_goal(4.5, 4.5, 4.5);
    bool valid1 = planner_ptr->plan(start, blocked_goal, 1, true);
    if (valid1 || planner_ptr->status() != 2)
    {
        printf(ANSI_COLOR_RED
               "FAILED case 1: expected goal-not-free (status 2), got "
               "valid=%d status=%d\n" ANSI_COLOR_RESET,
               valid1, planner_ptr->status());
        all_ok = false;
    }
    else
    {
        printf("Case 1 OK: ceiling-blocked goal correctly rejected "
               "(status=%d)\n",
               planner_ptr->status());
    }

    // Case 2: a goal that's reachable without ever crossing the ceiling.
    // This map only connects start to anything beyond z=0 by winding
    // through every layer in turn (see data/simple3d.yaml's layout: a
    // single-cell-wide opening per layer, each at a different (x,y)), so
    // reaching (0,0,3) requires actually climbing through z=0,1,2,3 -- a
    // real exercise of the search, not a trivial same-cell check.
    const Vec3f reachable_goal(0.5, 0.5, 3.5);
    bool valid2 = planner_ptr->plan(start, reachable_goal, 1, true);
    if (!valid2)
    {
        printf(ANSI_COLOR_RED
               "FAILED case 2: expected a path below the ceiling, found "
               "none (status=%d)\n" ANSI_COLOR_RESET,
               planner_ptr->status());
        all_ok = false;
    }
    else
    {
        const auto path = planner_ptr->getRawPath();
        double max_z = -std::numeric_limits<double>::infinity();
        for (const auto &p : path)
            max_z = std::max(max_z, p(2));

        printf("Case 2: path found, %zu points, max z = %f\n", path.size(),
               max_z);

        if (max_z >= 4.0)
        {
            printf(ANSI_COLOR_RED
                   "FAILED case 2: path reaches z=%f, at or above the 4.0m "
                   "ceiling\n" ANSI_COLOR_RESET,
                   max_z);
            all_ok = false;
        }
        else
        {
            printf("Case 2 OK: path stays below the ceiling\n");
        }
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_planner_ceiling FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_planner_ceiling PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
