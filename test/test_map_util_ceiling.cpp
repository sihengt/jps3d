#include <jps_collision/map_util.h>

#include <limits>

using namespace JPS;

int main()
{
    // 3x3x5 all-free voxel grid, origin at 0, 1m resolution.
    Vec3i dim(3, 3, 5);
    Vec3f origin(0, 0, 0);
    Tmap data(static_cast<size_t>(dim(0)) * dim(1) * dim(2), 0); // all free

    VoxelMapUtil map_util;
    map_util.setMap(origin, dim, data, 1.0);

    bool all_ok = true;

    // Before any ceiling is set, every cell is free (disabled by default).
    if (!map_util.isFree(Vec3i(1, 1, 4)) || map_util.isOccupied(Vec3i(1, 1, 4)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: default (no ceiling) should leave z=4 free\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }

    // Ceiling at world Z = 3.0 -> floor((3.0 - 0) / 1.0) = cell index 3.
    // Cells with index < 3 stay free; index >= 3 become occupied.
    map_util.setCeiling(3.0);

    if (!map_util.isFree(Vec3i(1, 1, 2)) || map_util.isOccupied(Vec3i(1, 1, 2)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: z=2 should remain free below the ceiling\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }

    if (map_util.isFree(Vec3i(1, 1, 3)) || !map_util.isOccupied(Vec3i(1, 1, 3)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: z=3 (the cell containing the ceiling height) should "
               "be blocked\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    if (map_util.isFree(Vec3i(1, 1, 4)) || !map_util.isOccupied(Vec3i(1, 1, 4)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: z=4 should remain blocked above the ceiling\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }

    // Disabling the ceiling (non-finite) restores pre-ceiling behavior.
    map_util.setCeiling(std::numeric_limits<decimal_t>::infinity());
    if (!map_util.isFree(Vec3i(1, 1, 4)) || map_util.isOccupied(Vec3i(1, 1, 4)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: z=4 should be free again once the ceiling is "
               "disabled\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_map_util_ceiling FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_map_util_ceiling PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
