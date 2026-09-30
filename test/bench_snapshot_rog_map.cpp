// Micro-benchmark for the fast-mode occupancy snapshot
// (JPSPlanner::updateMap -> ROGMapUtil::snapshotOccupancy) on the real
// 0.05 m scene.
// Usage: bench_snapshot_rog <yaml> [reps]
// Times updateMap() at several snapshot thread counts and checks each
// snapshot against the generic per-cell isFree() sweep
// (MapUtil<3>::snapshotOccupancy) as ground truth.
#include "rog_map_fixture.hpp"
#include "timer.hpp"

#include <jps_planner/jps_planner/jps_planner.h>

#include <algorithm>
#include <cstdio>
#include <functional>
#include <thread>
#include <vector>

using namespace JPS;

namespace
{
constexpr double kThresh = 0.3; // see test_map_util_super.cpp

double timeMedianMs(int reps, const std::function<void()> &f)
{
    std::vector<double> ms;
    f(); // warm
    for (int i = 0; i < reps; ++i)
    {
        Timer t(true);
        f();
        ms.push_back(t.ElapsedMs());
    }
    std::sort(ms.begin(), ms.end());
    return ms[ms.size() / 2];
}
} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("usage: %s <yaml> [reps]\n", argv[0]);
        return 1;
    }
    const int reps = argc > 2 ? atoi(argv[2]) : 10;
    auto rog = std::make_shared<jps_test::TestROGMap>(argv[1]);
    rog->insert(jps_test::loadPcdScene(
        "super_planner/jps/data/output_with_ground.pcd"));
    auto mu = std::make_shared<ROGMapUtil<3>>(rog);
    mu->setThreshVal(kThresh);
    mu->updateVirtualCeilingFloor();

    JPSPlanner3D pl(false);
    pl.setMapUtil(mu);
    pl.setThreshVal(kThresh);
    pl.setFastMode(true);
    pl.updateMap();
    const Vec3i lo = pl.debugSnapLo();
    const Vec3i n = pl.debugSnapDim();
    printf("# snapshot box lo %d %d %d  n %d %d %d  (%.1f M cells, %.0f MB of "
           "ESDF reads)\n",
           lo(0), lo(1), lo(2), n(0), n(1), n(2), n.prod() / 1e6,
           n.prod() * 8 / 1e6);

    std::vector<uint8_t> truth;
    Timer tg(true);
    mu->MapUtil<3>::snapshotOccupancy(lo, lo + n - Vec3i::Ones(), kThresh,
                                      truth);
    printf("# generic per-cell sweep (ground truth): %.1f ms\n",
           tg.ElapsedMs());

    printf("%-28s %8s %s\n", "updateMap()", "med_ms", "check");
    const int hw = static_cast<int>(std::thread::hardware_concurrency());
    for (int th : {1, 2, 4, 8, hw / 2, hw})
    {
        mu->setSnapshotThreads(th);
        const double ms = timeMedianMs(reps, [&] { pl.updateMap(); });
        char name[64];
        snprintf(name, sizeof name, "%d snapshot thread(s)", th);
        printf("%-28s %8.2f %s\n", name, ms,
               pl.debugSnapshot() == truth ? "== generic" : "MISMATCH");
    }
    rog.reset();
    return 0;
}
