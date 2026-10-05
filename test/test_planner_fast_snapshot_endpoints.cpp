// In fast mode the search runs on the snapshot taken by updateMap(), so
// plan() must judge the start and goal cells by the snapshot, not by the live
// map (which can have changed since). Uses an all-free voxel map and rewrites
// the live map between updateMap() and plan().
#include <cstdio>
#include <jps_collision/map_util_voxel.h>
#include <jps_planner/jps_planner/jps_planner.h>

using namespace JPS;

static const Veci<3> kDim(10, 10, 10);
static const Vec3f kStart(0.55, 0.55, 0.55); 
static const Vec3f kGoal(4.25, 4.25, 4.25);
static int failures = 0;

#define CHECK(cond, ...)                                                       \
    do                                                                         \
    {                                                                          \
        if (!(cond))                                                           \
        {                                                                      \
            printf("FAIL line %d: ", __LINE__);                                \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
            ++failures;                                                        \
        }                                                                      \
    } while (0)

// 10^3 map, 0.5 m cells, origin 0; cells in `blocked` are occupied.
static void loadMap(VoxelMapUtil &mu, const std::vector<Vec3f> &blocked)
{
    std::vector<signed char> data(10 * 10 * 10, 0);
    for (const auto &p : blocked)
    {
        const Veci<3> c = mu.floatToInt(p);
        data[c(0) + 10 * c(1) + 100 * c(2)] = 100;
    }
    mu.setMap(Vec3f::Zero(), kDim, data, 0.5);
}

// Builds a planner whose snapshot was taken with `snap_blocked` occupied,
// then rewrites the live map with `live_blocked` occupied and plans.
static bool run(const std::vector<Vec3f> &snap_blocked,
                const std::vector<Vec3f> &live_blocked, int &status)
{
    auto mu = std::make_shared<VoxelMapUtil>();
    loadMap(*mu, snap_blocked);
    JPSPlanner3D planner(false);
    planner.setMapUtil(mu);
    planner.setFastMode(true);
    planner.updateMap();
    loadMap(*mu, live_blocked);
    const bool ok = planner.plan(kStart, kGoal, 1.0, false);
    status = planner.status();
    return ok;
}

int main()
{
    int st = 0;

    CHECK(run({}, {}, st), "baseline: free map should plan (status %d)", st);

    // Start blocked in the snapshot but free live: must be rejected.
    CHECK(!run({kStart}, {}, st) && st == 1,
          "start blocked in snapshot, free live: expected status 1, got %d", st);

    // Goal blocked in the snapshot but free live: must be rejected.
    CHECK(!run({kGoal}, {}, st) && st == 2,
          "goal blocked in snapshot, free live: expected status 2, got %d", st);

    // Start/goal free in the snapshot but blocked live: the search only ever
    // sees the snapshot, so this plans.
    CHECK(run({}, {kStart, kGoal}, st),
          "free in snapshot, blocked live: expected success (status %d)", st);

    if (failures)
        return 1;
    printf("test_planner_fast_snapshot_endpoints PASSED\n");
    return 0;
}
