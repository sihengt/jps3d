#include <jps_basis/data_utils.h>
#include <jps_planner/jps_planner/jps_planner.h>

#include "read_map.hpp"

using namespace JPS;

// Regression test for repeated plan() calls reusing one JPSPlanner instance.
// Exercises the persisted-GraphSearch / pool-reuse / token-array path that
// test_planner_3d.cpp (only two plan() calls) doesn't fully cover.
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

    std::unique_ptr<JPSPlanner3D> planner_ptr =
        std::make_unique<JPSPlanner3D>(false);
    planner_ptr->setMapUtil(map_util);
    planner_ptr->updateMap();

    struct Case
    {
        Vec3f start, goal;
        bool use_jps;
    };
    // All three pairs are confirmed reachable on data/simple3d.yaml (verified
    // by running the planner directly while writing this test): (0.5,0.5,0.5)
    // and (4.5,4.5,4.5) are the map's own start/goal (cell centers -- see the
    // simple3d.yaml fix in Task 1); (0.5,3.5,0.5) is the 4th waypoint on the
    // JPS path actually returned for the first pair, so it is reachable from
    // the same start by construction.
    std::vector<Case> cases = {
        {Vec3f(0.5, 0.5, 0.5), Vec3f(4.5, 4.5, 4.5), true},
        {Vec3f(4.5, 4.5, 4.5), Vec3f(0.5, 0.5, 0.5), false},
        {Vec3f(0.5, 0.5, 0.5), Vec3f(0.5, 3.5, 0.5), true},
    };

    bool all_ok = true;
    for (size_t i = 0; i < cases.size(); ++i)
    {
        const auto &c = cases[i];
        bool valid = planner_ptr->plan(c.start, c.goal, 1, c.use_jps);
        const auto path = planner_ptr->getRawPath();

        printf("Case %zu (%s): valid=%d, path size=%zu\n", i,
               c.use_jps ? "JPS" : "A*", valid, path.size());

        if (!valid || path.empty())
        {
            printf(ANSI_COLOR_RED
                   "Case %zu FAILED: no path found\n" ANSI_COLOR_RESET,
                   i);
            all_ok = false;
            continue;
        }

        const Vec3f &path_start = path.front();
        const Vec3f &path_end = path.back();
        if ((path_start - c.start).norm() > reader.resolution() * 1.5 ||
            (path_end - c.goal).norm() > reader.resolution() * 1.5)
        {
            printf(ANSI_COLOR_RED
                   "Case %zu FAILED: path endpoints don't match "
                   "start/goal\n" ANSI_COLOR_RESET,
                   i);
            all_ok = false;
        }
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_planner_replan FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_planner_replan PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
