#include <jps3d/jps3d_frontend.hpp>
#include <jps_collision/map_util_voxel.h>

using namespace JPS;

int main()
{
    // A 1-wide, 10-long corridor along x: cells 0-4 known-free, 5-9 unknown.
    // No obstacles at all -- the planner can search straight through to the
    // goal either way, since unknown reads as free. The only question this
    // test answers is whether planPath's frontier truncation stops the
    // returned path at the free/unknown boundary.
    Vec3i dim(10, 1, 1);
    Vec3f origin(0, 0, 0);
    std::vector<signed char> data(10, 0);
    for (int i = 5; i < 10; ++i)
        data[i] = -1; // unknown

    auto map_util = std::make_shared<VoxelMapUtil>();
    map_util->setMap(origin, dim, data, 1.0);

    const Vec3f start(0.5, 0.5, 0.5);
    const Vec3f goal(9.5, 0.5, 0.5);

    bool all_ok = true;

    // block_unknown = true, frontier_seed_radius = 0.0 (no seed exemption):
    // path must stop before entering the unknown region (x >= 5.0).
    {
        Jps3dFrontend frontend(map_util, false, /*block_unknown=*/true,
                               /*frontier_seed_radius=*/0.0);
        frontend.updateMap();
        vec_Vec3f path;
        if (!frontend.planPath(start, goal, 1.0, true, path) || path.empty())
        {
            printf(ANSI_COLOR_RED
                   "FAILED: block_unknown=true case should still return a "
                   "(truncated) path\n" ANSI_COLOR_RESET);
            all_ok = false;
        }
        else if (path.back()(0) >= 5.0)
        {
            printf(ANSI_COLOR_RED
                   "FAILED: path should be truncated before x=5.0 (got last "
                   "point x=%.2f)\n" ANSI_COLOR_RESET,
                   path.back()(0));
            all_ok = false;
        }
    }

    // block_unknown = false: path should reach the goal, unknown or not.
    {
        Jps3dFrontend frontend(map_util, false, /*block_unknown=*/false,
                               /*frontier_seed_radius=*/0.0);
        frontend.updateMap();
        vec_Vec3f path;
        if (!frontend.planPath(start, goal, 1.0, true, path) || path.empty())
        {
            printf(ANSI_COLOR_RED
                   "FAILED: block_unknown=false case should return a path\n"
                   ANSI_COLOR_RESET);
            all_ok = false;
        }
        else if ((path.back() - goal).norm() > 1e-6)
        {
            printf(ANSI_COLOR_RED
                   "FAILED: block_unknown=false should reach the goal "
                   "unmodified (got last point (%.2f,%.2f,%.2f))\n"
                   ANSI_COLOR_RESET,
                   path.back()(0), path.back()(1), path.back()(2));
            all_ok = false;
        }
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_jps3d_frontend FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_jps3d_frontend PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
