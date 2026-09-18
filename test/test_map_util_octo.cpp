#include <jps_collision/map_util_octo.h>
#include <octomap/octomap.h>

using namespace JPS;

int main()
{
    // 4x4x4 grid, 1m resolution, origin at 0 — must be bootstrapped with
    // setMap() before the first updateFromOctree(), same as jps3d_node's
    // init_map() will do.
    Vec3i dim(4, 4, 4);
    Vec3f origin(0, 0, 0);
    std::vector<signed char> bootstrap(
        static_cast<size_t>(dim(0)) * dim(1) * dim(2), -1); // all unknown

    OctomapMapUtil<3> map_util;
    map_util.setMap(origin, dim, bootstrap, 1.0);

    bool all_ok = true;

    // Octree at the SAME 1m resolution as the grid. floatToInt uses
    // round((pt-origin)/res - 0.5), so a 1m leaf's true extent can map to a
    // 2-cell-wide index range at these boundaries (e.g. [1.0,2.0] -> indices
    // {1,2}, not just {1}) -- keep the occupied and free leaves far enough
    // apart that their rounded index ranges don't touch, or "occupied wins"
    // would make the free assertion below fail.
    octomap::OcTree tree(1.0);
    tree.updateNode(octomap::point3d(1.5, 1.5, 1.5), true);  // occupied: world [1.0,2.0]^3 -> cells {1,2}^3
    tree.updateNode(octomap::point3d(3.5, 3.5, 3.5), false); // free: world [3.0,4.0]^3 -> cell (3,3,3) (4 is outside dim=4, clipped)

    map_util.updateFromOctree(&tree);

    if (!map_util.isOccupied(Vec3i(1, 1, 1), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (1,1,1) should be occupied after updateFromOctree\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }
    if (map_util.isUnknown(Vec3i(3, 3, 3)) ||
        !map_util.isFree(Vec3i(3, 3, 3), 0.0) ||
        map_util.isOccupied(Vec3i(3, 3, 3), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (3,3,3) should be known-free after updateFromOctree\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }

    // A cell the octree never touched stays unknown, but is still treated
    // as free by isFree() (optimistic search) -- the same single-grid
    // duality SimpleMapUtil already provides.
    if (!map_util.isUnknown(Vec3i(0, 0, 0)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (0,0,0) was never touched, should be unknown\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }
    if (!map_util.isFree(Vec3i(0, 0, 0), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: unknown cell (0,0,0) should still read as free "
               "(optimistic search)\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    // A coarser leaf spanning multiple grid cells: reset with a fresh 2m
    // octree. A single leaf at (1,1,1) with a 2m edge covers world
    // [0,2]^3, which floatToInt maps to (at least) grid cells {0,1}^3 --
    // this checks that every one of those 8 cells got marked, i.e. one
    // coarse leaf really did fan out over multiple grid cells (it may
    // also reach a couple of index-2 neighbors at the rounding boundary;
    // this test only asserts the cells it must cover, not the ones it
    // must not).
    octomap::OcTree coarse_tree(2.0);
    coarse_tree.updateNode(octomap::point3d(1.0, 1.0, 1.0), true);

    std::vector<signed char> bootstrap2(
        static_cast<size_t>(dim(0)) * dim(1) * dim(2), -1);
    map_util.setMap(origin, dim, bootstrap2, 1.0); // reset before re-ingesting
    map_util.updateFromOctree(&coarse_tree);

    const Vec3i covered[8] = {
        Vec3i(0, 0, 0), Vec3i(1, 0, 0), Vec3i(0, 1, 0), Vec3i(1, 1, 0),
        Vec3i(0, 0, 1), Vec3i(1, 0, 1), Vec3i(0, 1, 1), Vec3i(1, 1, 1)};
    for (const auto &c : covered)
    {
        if (!map_util.isOccupied(c, 0.0))
        {
            printf(ANSI_COLOR_RED
                   "FAILED: coarse leaf should mark grid cell (%d,%d,%d) "
                   "occupied\n" ANSI_COLOR_RESET,
                   c(0), c(1), c(2));
            all_ok = false;
        }
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_map_util_octo FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_map_util_octo PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
