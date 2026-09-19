// JPS/A* planning benchmark over a rog_map::ROGMap (via ROGMapUtil) and an
// identical SimpleMapUtil copy of the same grid.
// Usage: bench_jps_rog_map <yaml> [out.csv] [reps] [scene-filter] [mode]
//   mode: base (default) | fast | fastbox | budget
#include "rog_map_fixture.hpp"
#include "timer.hpp"

#include <jps3d/jps3d_frontend.hpp>
#include <jps_collision/map_util_voxel.h>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <vector>

using namespace JPS;

namespace
{
constexpr double kThreshRog = 0.3; // see test_map_util_super.cpp
constexpr double kZ = 1.0;

struct PlanCase
{
    std::string name;
    Vec3f start, goal;
    bool expect_path = true;
};

struct Scene
{
    std::string name;
    std::function<rog_map::PointCloud()> build;
    std::vector<PlanCase> cases;
};

struct Algo
{
    std::string name;
    bool jps;
    double eps;
};

double pathLength(const vec_Vec3f &p)
{
    double l = 0;
    for (size_t i = 1; i < p.size(); ++i)
        l += (p[i] - p[i - 1]).norm();
    return l;
}

// Copy the ESDF window into a SimpleMapUtil with the same dims and cell
// classification (occupied / outside-ceiling / outside-bbox all -> occupied),
// so the two backends search exactly the same grid.
std::shared_ptr<VoxelMapUtil> cloneToSimple(ROGMapUtil<3> &rog,
                                            const Vec3i &min_id_g)
{
    const Vec3i dim = rog.getDim();
    const double res = rog.getRes();
    std::vector<signed char> data(1LL * dim(0) * dim(1) * dim(2), 100);
    for (int z = 0; z < dim(2); ++z)
        for (int y = 0; y < dim(1); ++y)
            for (int x = 0; x < dim(0); ++x)
            {
                Vec3i g = min_id_g + Vec3i(x, y, z);
                if (rog.isFree(g, kThreshRog))
                    data[x + dim(0) * (y + 1LL * dim(1) * z)] = 0;
            }
    auto mu = std::make_shared<VoxelMapUtil>();
    mu->setMap(min_id_g.cast<double>() * res, dim, data, res);
    return mu;
}

struct Result
{
    bool ok = false;
    double len = 0, first_ms = 0, median_ms = 0, p95_ms = 0;
    int attempts = 1, status = 0;
    JPSPlanner3D::Timings med; // phase medians + counters from median run
};

Result runCase(Jps3dFrontend &fe, JPSPlanner3D::Timings (*tim)(Jps3dFrontend &),
               const PlanCase &c, const Algo &a, int reps)
{
    Result r;
    vec_Vec3f path;
    Timer t(true);
    r.ok = fe.planPath(c.start, c.goal, a.eps, a.jps, path);
    r.first_ms = t.ElapsedMs();
    r.len = r.ok ? pathLength(path) : 0;
    r.attempts = fe.lastAttempts();
    r.status = fe.status();
    std::vector<std::pair<double, JPSPlanner3D::Timings>> runs;
    for (int i = 0; i < reps; ++i)
    {
        fe.planPath(c.start, c.goal, a.eps, a.jps, path);
        runs.emplace_back(fe.lastPlanMs(), tim(fe));
    }
    std::sort(runs.begin(), runs.end(),
              [](auto &x, auto &y) { return x.first < y.first; });
    r.median_ms = runs[runs.size() / 2].first;
    r.p95_ms = runs[std::min(runs.size() - 1, (size_t)(runs.size() * 0.95))].first;
    r.med = runs[runs.size() / 2].second;
    return r;
}
} // namespace

// Jps3dFrontend hides its planner; expose timings through a tiny accessor.
static JPSPlanner3D::Timings frontendTimings(Jps3dFrontend &fe)
{
    return fe.lastTimings();
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("usage: %s <yaml> [csv] [reps] [scene]\n", argv[0]);
        return 1;
    }
    const std::string yaml = argv[1];
    const std::string csv_path = argc > 2 ? argv[2] : "";
    const int reps = argc > 3 ? atoi(argv[3]) : 20;
    const std::string filter = argc > 4 ? argv[4] : "";
    const std::string mode = argc > 5 ? argv[5] : "base";
    const bool fast = mode != "base";

    auto diag = [](double a, double b, const char *n) {
        return PlanCase{n, Vec3f(a, a, kZ), Vec3f(b, b, kZ)};
    };
    std::vector<PlanCase> diag_cases = {diag(-2, 1.5, "5m"), diag(-5, 5.6, "15m"),
                                        diag(-10.6, 10.6, "30m"),
                                        diag(-19, 19, "55m")};
    std::vector<Scene> scenes = {
        {"pillars", [] { return jps_test::openPillars(60); }, diag_cases},
        {"rooms", [] { return jps_test::roomsAndDoors(); }, diag_cases},
        {"pocket",
         [] { return jps_test::deadEndPocket(15.0, 4.0); },
         {PlanCase{"trap2m", Vec3f(0, -0.5, kZ), Vec3f(0, 1.5, kZ)},
          PlanCase{"trap2m_rev", Vec3f(0, 1.5, kZ), Vec3f(0, -0.5, kZ)},
          PlanCase{"nopath", Vec3f(0, -0.5, kZ), Vec3f(10, 10, kZ), false}}},
    };
    std::vector<Algo> algos = {{"jps", true, 1.0}, {"jps_e1.5", true, 1.5},
                               {"astar", false, 1.0}};

    std::ofstream csv;
    if (!csv_path.empty())
    {
        csv.open(csv_path);
        csv << "mode,backend,scene,case,algo,eps,ok,len_m,first_ms,median_ms,p95_ms,"
               "check_ms,build_ms,search_ms,convert_ms,corner_ms,line_ms,"
               "expand,succ,jump_steps,cell_queries,heap_push\n";
    }

    printf("%-7s %-7s %-8s %-11s %-9s %3s %8s %8s %8s | %8s %8s %8s | %8s %9s %10s %3s\n",
           "mode", "backend", "scene", "case", "algo", "ok", "len_m", "med_ms",
           "p95_ms", "search", "corner", "build1st", "expand", "jumpstep",
           "queries", "att");

    for (auto &sc : scenes)
    {
        if (!filter.empty() && sc.name != filter)
            continue;
        Timer t(true);
        auto rog = std::make_shared<jps_test::TestROGMap>(yaml);
        rog->insert(sc.build());
        printf("# scene %s: map build %.0f ms\n", sc.name.c_str(), t.ElapsedMs());

        auto mu_rog = std::make_shared<ROGMapUtil<3>>(rog);
        mu_rog->setThreshVal(kThreshRog);
        mu_rog->updateVirtualCeilingFloor();
        // global index of local cell (0,0,0): window origin minus half size
        Vec3i origin_i;
        rog->esdfMapPosToGlobalIndex(rog->config().fix_map_origin, origin_i);
        const Vec3i min_id_g = origin_i - (mu_rog->getDim() - Vec3i::Ones()) / 2;
        t.Reset();
        auto mu_simple = cloneToSimple(*mu_rog, min_id_g);
        printf("# scene %s: simple clone %.0f ms\n", sc.name.c_str(),
               t.ElapsedMs());

        // snap start/goal on the ROG util (same cells on both)
        for (auto &c : sc.cases)
        {
            Vec3f s, g;
            if (!jps_test::snapToFree(*mu_rog, c.start, s) ||
                !jps_test::snapToFree(*mu_rog, c.goal, g))
                printf("# WARN %s/%s: could not snap start/goal\n",
                       sc.name.c_str(), c.name.c_str());
            else
                c.start = s, c.goal = g;
        }

        struct Backend
        {
            const char *name;
            std::shared_ptr<MapUtil<3>> mu;
            double thresh;
        } backends[] = {{"rog", mu_rog, kThreshRog}, {"simple", mu_simple, 0.0}};

        for (auto &be : backends)
        {
            Jps3dFrontend fe(be.mu, false, /*block_unknown=*/false, 0.0);
            fe.setThreshVal(be.thresh);
            fe.setFastMode(fast);
            if (mode == "fastbox")
                fe.setSearchBoxWidening(3.0);
            if (mode == "budget")
                fe.setMaxExpand(2000);
            fe.updateMap();
            if (fast)
            {
                // re-run to time a warm snapshot
                fe.updateMap();
                printf("# %s/%s snapshot: %.2f ms\n", be.name, sc.name.c_str(),
                       fe.lastSnapshotMs());
            }
            for (auto &c : sc.cases)
                for (auto &a : algos)
                {
                    const int n = a.jps ? reps : std::max(3, reps / 4);
                    Result r = runCase(fe, frontendTimings, c, a, n);
                    if (r.ok != c.expect_path && mode != "budget")
                        printf("# WARN %s/%s/%s: ok=%d expected %d\n", be.name,
                               sc.name.c_str(), c.name.c_str(), r.ok,
                               c.expect_path);
                    printf("%-7s %-7s %-8s %-11s %-9s %3d %8.2f %8.2f %8.2f | %8.2f "
                           "%8.2f %8.2f | %8lld %9lld %10lld %3d%s\n",
                           mode.c_str(), be.name, sc.name.c_str(), c.name.c_str(),
                           a.name.c_str(), r.ok, r.len, r.median_ms, r.p95_ms,
                           r.med.search_ms, r.med.corner_ms, r.first_ms,
                           r.med.expand, r.med.jump_steps, r.med.cell_queries,
                           r.attempts, r.status == 3 ? " partial" : "");
                    if (csv.is_open())
                        csv << mode << ',' << be.name << ',' << sc.name << ',' << c.name << ','
                            << a.name << ',' << a.eps << ',' << r.ok << ','
                            << r.len << ',' << r.first_ms << ',' << r.median_ms
                            << ',' << r.p95_ms << ',' << r.med.check_ms << ','
                            << r.med.build_ms << ',' << r.med.search_ms << ','
                            << r.med.convert_ms << ',' << r.med.corner_ms << ','
                            << r.med.line_ms << ',' << r.med.expand << ','
                            << r.med.succ << ',' << r.med.jump_steps << ','
                            << r.med.cell_queries << ',' << r.med.heap_push
                            << '\n';
                }
        }
    }
    return 0;
}
