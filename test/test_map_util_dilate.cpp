#include <jps_collision/map_util_voxel.h>

using namespace JPS;

int main()
{
    // 5x5x5 all-free grid except one occupied cell at (2,2,2).
    Vec3i dim(5, 5, 5);
    Vec3f origin(0, 0, 0);
    std::vector<signed char> data(static_cast<size_t>(dim(0)) * dim(1) * dim(2), 0);
    data[2 + 5 * 2 + 25 * 2] = 100; // occupied

    VoxelMapUtil map_util;
    map_util.setMap(origin, dim, data, 1.0);

    bool all_ok = true;

    // Before dilation, a face neighbor of the occupied cell is still free.
    if (!map_util.isFree(Vec3i(3, 2, 2), 0.0))
    {
        printf(ANSI_COLOR_RED "FAILED: (3,2,2) should start free\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    map_util.dilateByRadius(1);

    // After dilating by 1 cell, every 6-connected face neighbor of (2,2,2)
    // must now be occupied.
    const Vec3i neighbors[6] = {
        Vec3i(3, 2, 2), Vec3i(1, 2, 2), Vec3i(2, 3, 2),
        Vec3i(2, 1, 2), Vec3i(2, 2, 3), Vec3i(2, 2, 1)};
    for (const auto &n : neighbors)
    {
        if (!map_util.isOccupied(n, 0.0))
        {
            printf(ANSI_COLOR_RED
                   "FAILED: neighbor (%d,%d,%d) should be occupied after "
                   "dilateByRadius(1)\n" ANSI_COLOR_RESET,
                   n(0), n(1), n(2));
            all_ok = false;
        }
    }

    // A cell 2 away (outside the radius-1 neighborhood) must remain free.
    if (!map_util.isFree(Vec3i(0, 2, 2), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (0,2,2) is 2 cells away, should remain free after "
               "dilateByRadius(1)\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_map_util_dilate FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_map_util_dilate PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
