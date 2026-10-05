// JPS/A* planning benchmark over a rog_map::ROGMap (via ROGMapUtil) and an
// identical SimpleMapUtil copy of the same grid.
// Usage: bench_jps_rog_map <yaml> [out.csv] [reps] [scene-filter] [mode] [plot]
//   mode: base (default) | fast | fastbox | budget | cycle | compare
//   "cycle" = fast mode with updateMap()+planPath() both timed every rep
//   (honest end-to-end per-planning-cycle cost); the others amortize one
//   updateMap() over all reps (steady-state search-only cost).
//   "compare" = runs base and fast back-to-back per backend, sharing the
//   same scene/map (no rebuild), so both land in one table/CSV with `mode`
//   distinguishing them -- filter/group by that column instead of diffing
//   two separate runs.
//   plot: pass "plot" as the 6th arg to also write one SVG per case
//   (plot_<scene>_<case>.svg, cwd) with the obstacle cloud (top-down, near
//   the case's z) plus each algo's route overlaid -- same idea as the
//   vendored jps/test/test_planner_2d.cpp boost::geometry svg_mapper plot,
//   adapted to draw obstacles from the scene's point cloud directly instead
//   of looping the whole (much larger, 3D) occupancy grid.
#include "rog_map_fixture.hpp"
#include <jps_basis/timer.hpp>

#include <jps3d/jps3d_frontend.hpp>
#include <jps_collision/map_util_voxel.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <functional>
#include <memory>
#include <string>
#include <unordered_set>
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

// Copy ESDF to an occupancy map that the other backend can use (for fair comparison algo-wise)
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

// Top-down SVG
// start (orange) / goal (blue), one coloured route per algo.
// Obstacles are drawn from the scene's own point cloud within [z_lo, z_hi]
void plotCasePaths(const std::string &svg_path, const rog_map::PointCloud &cloud,
                   double z_lo, double z_hi, const Vec3f &start, const Vec3f &goal,
                   const std::vector<std::pair<std::string, vec_Vec3f>> &algo_paths)
{
    typedef boost::geometry::model::d2::point_xy<double> point_2d;

    double xmin = 1e18, xmax = -1e18, ymin = 1e18, ymax = -1e18;
    for (auto &p : cloud.points)
    {
        if (!std::isfinite(p.x) || !std::isfinite(p.y))
            continue;
        xmin = std::min(xmin, (double)p.x);
        xmax = std::max(xmax, (double)p.x);
        ymin = std::min(ymin, (double)p.y);
        ymax = std::max(ymax, (double)p.y);
    }
    xmin = std::min({xmin, (double)start(0), (double)goal(0)}) - 1.0;
    xmax = std::max({xmax, (double)start(0), (double)goal(0)}) + 1.0;
    ymin = std::min({ymin, (double)start(1), (double)goal(1)}) - 1.0;
    ymax = std::max({ymax, (double)start(1), (double)goal(1)}) + 1.0;

    std::ofstream svg(svg_path);
    boost::geometry::svg_mapper<point_2d> mapper(svg, 1000, 1000);

    boost::geometry::model::polygon<point_2d> bound;
    std::vector<point_2d> corners = {
        point_2d(xmin, ymin), point_2d(xmin, ymax), point_2d(xmax, ymax),
        point_2d(xmax, ymin), point_2d(xmin, ymin)};
    boost::geometry::assign_points(bound, corners);
    boost::geometry::correct(bound);
    mapper.add(bound);
    mapper.map(bound, "fill-opacity:1.0;fill:rgb(255,255,255);"
                      "stroke:rgb(0,0,0);stroke-width:2");

    // Obstacles: dedupe to a 0.1 m bucket so the SVG stays small/fast even
    // for a multi-million-point cloud.
    const double bucket = 0.1;
    std::unordered_set<int64_t> seen;
    for (auto &p : cloud.points)
    {
        if (p.z < z_lo || p.z > z_hi)
            continue;
        const int64_t ix = static_cast<int64_t>(std::floor(p.x / bucket));
        const int64_t iy = static_cast<int64_t>(std::floor(p.y / bucket));
        const int64_t key = (ix << 32) ^ (iy & 0xffffffffLL);
        if (!seen.insert(key).second)
            continue;
        point_2d a(p.x, p.y);
        mapper.add(a);
        mapper.map(a, "fill-opacity:0.6;fill:rgb(90,90,90);", 1);
    }

    point_2d s(start(0), start(1)), g(goal(0), goal(1));
    mapper.add(s);
    mapper.map(s, "fill-opacity:1.0;fill:rgb(255,140,0);", 8);
    mapper.add(g);
    mapper.map(g, "fill-opacity:1.0;fill:rgb(0,120,255);", 8);

    // Same colour convention as jps/test/test_planner_2d.cpp (jps=red,
    // astar=green), plus blue for eps>1 JPS.
    static const std::vector<std::string> colors = {
        "rgb(212,0,0)", "rgb(0,60,220)", "rgb(1,150,0)", "rgb(160,0,200)"};
    size_t ci = 0;
    for (auto &ap : algo_paths)
    {
        boost::geometry::model::linestring<point_2d> line;
        for (auto &pt : ap.second)
            line.push_back(point_2d(pt(0), pt(1)));
        mapper.add(line);
        const std::string &color = colors[ci % colors.size()];
        mapper.map(line, "opacity:0.7;fill:none;stroke:" + color + ";stroke-width:3");
        mapper.text(point_2d(xmin + 0.3, ymax - 0.5 - 0.7 * ci), ap.first,
                    "fill-opacity:1.0;fill:" + color + ";");
        ++ci;
    }
}

struct Result
{
    bool ok = false;
    double len = 0, first_ms = 0, median_ms = 0, p95_ms = 0;
    int attempts = 1, status = 0;
    JPSPlanner3D::Timings med; // phase medians + counters from median run
};

Result runCase(Jps3dFrontend &fe, JPSPlanner3D::Timings (*tim)(Jps3dFrontend &),
               const PlanCase &c, const Algo &a, int reps, bool per_cycle_update)
{
    Result r;
    vec_Vec3f path;
    Timer t(true);
    if (per_cycle_update)
        fe.updateMap();
    r.ok = fe.planPath(c.start, c.goal, a.eps, a.jps, path);
    r.first_ms = t.ElapsedMs();
    r.len = r.ok ? pathLength(path) : 0;
    r.attempts = fe.lastAttempts();
    r.status = fe.status();
    std::vector<std::pair<double, JPSPlanner3D::Timings>> runs;
    for (int i = 0; i < reps; ++i)
    {
        // rebuild snapshot (updateMap, if FAST) AND plan, if per_cycle_update is on.
        Timer tc(true);
        if (per_cycle_update)
            fe.updateMap();
        fe.planPath(c.start, c.goal, a.eps, a.jps, path);
        // kind of dangerous, we always want per_cycle_update
        const double ms = per_cycle_update ? tc.ElapsedMs() : fe.lastPlanMs();
        runs.emplace_back(ms, tim(fe));
    }
    std::sort(runs.begin(), runs.end(),
              [](auto &x, auto &y) { return x.first < y.first; });
    r.median_ms = runs[runs.size() / 2].first;
    r.p95_ms = runs[std::min(runs.size() - 1, (size_t)(runs.size() * 0.95))].first;
    r.med = runs[runs.size() / 2].second;
    return r;
}
// ---------------------------------------------------------------------------
// "prof" mode: where does an A* expansion's time go?
//  1. In-search phase split (needs -DJPS3D_PHASE_TIMING=ON): pop / successor
//     generation (queries + node alloc) / successor processing (heap).
//  2. Replay of the recorded expansion order through each layer of the
//     per-neighbor query chain, so each layer's cost is measured on the real
//     access pattern, isolated from the heap/bookkeeping.

// This function calibrates the tsc (CPU fixed clock) to nanoseconds for an estimate
// of how fast each function is running.
double tscPerNs()
{
#ifdef JPS_PHASE_TIMING
    auto c0 = std::chrono::steady_clock::now();
    unsigned long long t0 = __rdtsc();
    while (std::chrono::steady_clock::now() - c0 < std::chrono::milliseconds(200))
    {
    }
    unsigned long long t1 = __rdtsc();
    auto c1 = std::chrono::steady_clock::now();
    return (t1 - t0) /
           (double)std::chrono::duration_cast<std::chrono::nanoseconds>(c1 - c0).count();
#else
    return 0;
#endif
}

template <class F> double timeNs(F &&f)
{
    f(); // warm
    auto c0 = std::chrono::steady_clock::now();
    f();
    auto c1 = std::chrono::steady_clock::now();
    return std::chrono::duration_cast<std::chrono::nanoseconds>(c1 - c0).count();
}

void profileCase(Jps3dFrontend &fe, jps_test::TestROGMap &rog,
                 ROGMapUtil<3> &mu, const PlanCase &c, const Algo &a,
                 double tsc_ns, int reps, bool replay)
{
    // --- 1. phase split, base then fast (same frontend, toggled) ----------
    std::vector<Vec3i> trace; // base-mode (global index) expansion order
    for (bool fast : {false, true})
    {
        fe.setFastMode(fast);
        fe.updateMap();
        vec_Vec3f path;
        fe.planPath(c.start, c.goal, a.eps, a.jps, path); // warm
        std::vector<double> times;
        bool ok = false;
        for (int i = 0; i < std::max(1, reps); ++i)
        {
            Timer t(true);
            ok = fe.planPath(c.start, c.goal, a.eps, a.jps, path);
            times.push_back(t.ElapsedMs());
        }
        std::sort(times.begin(), times.end());
        const double ms = times[times.size() / 2];
        const auto &gs = fe.debugPlanner()->debugGraphSearch();
        const auto &st = gs->stats();
        const auto &tm = fe.lastTimings();
        const double cyc2ms = tsc_ns > 0 ? 1e-6 / tsc_ns : 0;
        const double pop = st.cyc_pop * cyc2ms, succ = st.cyc_succ * cyc2ms,
                     proc = st.cyc_proc * cyc2ms;
        printf("%-5s %-6s %-10s ok=%d len=%.4f med=%8.2fms search=%8.2f | pop=%8.2f "
               "succ=%8.2f proc=%8.2f other=%7.2f | exp=%8lld succ=%9lld "
               "push=%8lld incr=%8lld | ns/exp: pop=%5.0f succ=%5.0f proc=%5.0f\n",
               fast ? "fast" : "base", c.name.c_str(), a.name.c_str(), ok,
               ok ? pathLength(path) : 0.0, ms,
               tm.search_ms, pop, succ, proc,
               tm.search_ms - pop - succ - proc, st.expand, st.succ,
               st.heap_push, st.heap_increase, pop * 1e6 / st.expand,
               succ * 1e6 / st.expand, proc * 1e6 / st.expand);
        if (!fast)
            trace.assign(gs->expandTrace().begin(), gs->expandTrace().end());
    }
    if (!replay)
        return;
    if (trace.empty())
    {
        printf("  (no trace: rebuild with -DJPS3D_PHASE_TIMING=ON for replay)\n");
        return;
    }

    // --- 2. replay the expansion order through each query-chain layer -----
    const size_t n_exp = std::min<size_t>(trace.size(), 1000000);
    std::vector<Vec3i> nb;
    for (int dx = -1; dx <= 1; ++dx)
        for (int dy = -1; dy <= 1; ++dy)
            for (int dz = -1; dz <= 1; ++dz)
                if (dx || dy || dz)
                    nb.emplace_back(dx, dy, dz);
    const double nq = double(n_exp) * nb.size();
    const double th = kThreshRog;
    MapUtil<3> &base_mu = mu; // force virtual dispatch like GraphSearch does
    const auto &buf = mu.map_;
    const int zc = mu.getVirtualCeilingIdZ(), zf = mu.getVirtualFloorIdZ();
    const Vec3i dim = mu.getDim();
    const Vec3i half = (dim - Vec3i::Ones()) / 2;
    const int sy = dim(2), sx = dim(1) * dim(2);
    auto toLocal = [&](int g, int ax) {
        int l = g % dim(ax);
        if (l > half(ax))
            l -= dim(ax);
        else if (l < -half(ax))
            l += dim(ax);
        return l + half(ax);
    };
    auto inlineHash = [&](const Vec3i &p) {
        return toLocal(p(0), 0) * sx + toLocal(p(1), 1) * sy + toLocal(p(2), 2);
    };
    auto inside = [&](const Vec3i &p) {
        return !(p(2) > zc || p(2) < zf) && rog.insideESDFMap(p);
    };

    std::vector<int> pre_idx(n_exp * nb.size());
    for (size_t i = 0, k = 0; i < n_exp; ++i)
        for (auto &d : nb)
        {
            const Vec3i p = trace[i] + d;
            pre_idx[k++] = inside(p) ? inlineHash(p) : -1;
        }

    const auto &snap = fe.debugPlanner()->debugSnapshot();
    const Vec3i slo = fe.debugPlanner()->debugSnapLo();
    const Vec3i sdim = fe.debugPlanner()->debugSnapDim();

    struct Layer
    {
        const char *name;
        std::function<long long()> run;
    };
    std::vector<Layer> layers = {
        {"empty loop (coord gen only)",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                 {
                     const Vec3i p = trace[i] + d;
                     acc += p(0) ^ p(1) ^ p(2);
                 }
             return acc;
         }},
        {"base getSucc: v.isFree(Veci)+v.getIndex",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                 {
                     const Vec3i p = trace[i] + d;
                     if (base_mu.isFree(p, th))
                         acc += base_mu.getIndex(p) & 1;
                 }
             return acc;
         }},
        {"v.isFree(Veci) only",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                     acc += base_mu.isFree(trace[i] + d, th);
             return acc;
         }},
        {"rog direct: z+insideESDF+hash+load",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                 {
                     const Vec3i p = trace[i] + d;
                     acc += inside(p) &&
                            buf[rog.getESDFBufferIndexFromGlobalIndex(p)] >= th;
                 }
             return acc;
         }},
        {"  rog z+insideESDFMap only",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                     acc += inside(trace[i] + d);
             return acc;
         }},
        {"  rog getESDFBufferIndex only",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                     acc += rog.getESDFBufferIndexFromGlobalIndex(trace[i] + d);
             return acc;
         }},
        {"  inline modulo hash only",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                     acc += inlineHash(trace[i] + d);
             return acc;
         }},
        {"  ESDF double load only (pre-hashed)",
         [&] {
             long long acc = 0;
             for (int idx : pre_idx)
                 acc += idx >= 0 && buf[idx] >= th;
             return acc;
         }},
        {"fast: snapshot byte (bounds+load)",
         [&] {
             long long acc = 0;
             for (size_t i = 0; i < n_exp; ++i)
                 for (auto &d : nb)
                 {
                     const Vec3i p = trace[i] + d - slo;
                     acc += (p.array() >= 0).all() && (p.array() < sdim.array()).all() &&
                            snap[((size_t)p(0) * sdim(1) + p(1)) * sdim(2) + p(2)] == 0;
                 }
             return acc;
         }},
    };
    printf("  replay %zu expansions x %zu nbrs = %.0f queries:\n", n_exp,
           nb.size(), nq);
    for (auto &L : layers)
    {
        long long r = 0;
        const double ns = timeNs([&] { r = L.run(); });
        printf("    %-42s %7.2f ns/query  %8.1f ms total   (chk %lld)\n",
               L.name, ns / nq, ns * 1e-6, r);
    }
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
    const std::string mode_arg = argc > 5 ? argv[5] : "base";

    const std::vector<std::string> variants =
        mode_arg == "compare" ? std::vector<std::string>{"base", "fast"}
                              : std::vector<std::string>{mode_arg};
    const bool plot = argc > 6 && std::string(argv[6]) == "plot";

    auto diag = [](double a, double b, const char *n) {
        return PlanCase{n, Vec3f(a, a, kZ), Vec3f(b, b, kZ)};
    };
    std::vector<PlanCase> diag_cases = {diag(-2, 1.5, "5m"),
                                        diag(-5, 5.6, "15m"),
                                        diag(-10.6, 10.6, "30m"),
                                        diag(-19, 19, "55m")};
    std::vector<PlanCase> trap_cases = {
        PlanCase{"trap2m", Vec3f(0, -0.5, kZ), Vec3f(0, 1.5, kZ)},
        PlanCase{"trap2m_rev", Vec3f(0, 1.5, kZ), Vec3f(0, -0.5, kZ)},
        PlanCase{"nopath", Vec3f(0, -0.5, kZ), Vec3f(10, 10, kZ), false}
    };

    // real map waypoints
    const double rz = 1.0;
    std::vector<PlanCase> real_cases = {
        PlanCase{"6m", Vec3f(-8.0, -3.0, rz), Vec3f(-8.0, -9.0, rz)},
        PlanCase{"12m", Vec3f(-8.0, -3.0, rz), Vec3f(-2.0, -13.0, rz)},
        PlanCase{"20m", Vec3f(-8.0, -3.0, rz), Vec3f(6.0, -17.0, rz)},
        PlanCase{"28m", Vec3f(-8.0, -3.0, rz), Vec3f(13.0, -19.0, rz)},
        PlanCase{"weave", Vec3f(-8.0, -8.0, rz), Vec3f(-2.0, -8.0, rz)},
    };
    std::vector<Scene> scenes = {
        // {"pillars", [] { return jps_test::openPillars(500); }, diag_cases},
        // {"rooms", [] { return jps_test::roomsAndDoors(); }, diag_cases},
        // {"pocket", [] { return jps_test::deadEndPocket(15.0, 4.0); }, trap_cases,},
        {"real", [] {
             return jps_test::loadPcdScene(
                 "super_planner/jps/data/output_with_ground.pcd");
         }, real_cases},
    };
    std::vector<Algo> algos = {
        // {"jps", true, 1.0},
        // {"jps_e1.5", true, 1.5},
        {"astar", false, 1.0},
        {"astar_e1.5", false, 1.5},
        {"astar_e2.0", false, 2.0},
        {"astar_e5.0", false, 5.0}
    };

    std::ofstream csv;
    if (!csv_path.empty())
    {
        csv.open(csv_path);
        csv << "mode,backend,scene,case,algo,eps,ok,len_m,first_ms,median_ms,p95_ms,"
               "check_ms,build_ms,search_ms,convert_ms,corner_ms,line_ms,"
               "expand,succ,jump_steps,cell_queries,heap_push\n";
    }

    // each scene contains a name, a pointcloud building function, and cases.
    for (auto &sc : scenes)
    {
        if (!filter.empty() && sc.name != filter)
            continue;
        Timer t(true);
        auto rog = std::make_shared<jps_test::TestROGMap>(yaml);
        auto scene_cloud = sc.build();
        rog->insert(scene_cloud);
        printf("# scene %s: map build %.0f ms\n", sc.name.c_str(), t.ElapsedMs());

        auto mu_rog = std::make_shared<ROGMapUtil<3>>(rog);
        mu_rog->setThreshVal(kThreshRog);
        // fast-mode snapshot threads (ROGMapUtil::setSnapshotThreads)
        if (const char *st = std::getenv("JPS_SNAPSHOT_THREADS"))
            mu_rog->setSnapshotThreads(atoi(st));
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
        // checks all start/goals before continuing
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

        if (mode_arg == "prof")
        {
            const double tsc_ns = tscPerNs();
            printf("# prof: TSC %.3f cycles/ns%s\n", tsc_ns,
                   tsc_ns > 0 ? "" : " (phase timers OFF)");
            Jps3dFrontend fe(mu_rog, false, false, 0.0);
            fe.setThreshVal(kThreshRog);
            // argv[6] = "replay" also replays the query-chain layers
            const bool replay = argc > 6 && std::string(argv[6]) == "replay";
            for (auto &c : sc.cases)
                for (auto &a : algos)
                    if (!a.jps && a.eps == 1.5)
                        profileCase(fe, *rog, *mu_rog, c, a, tsc_ns, reps,
                                    replay);
            rog.reset();
            continue;
        }

        // "suite": fixed config matrix. Each rep times updateMap() (the
        // fast-mode snapshot; a no-op in base mode) + planPath() together,
        // then reports the median cycle plus the median run's phase split
        // (pop/succ/proc are TSC cycles, only with -DJPS3D_PHASE_TIMING=ON).
        if (mode_arg == "suite")
        {
            struct Cfg
            {
                const char *name;
                bool fast, jps;
                double eps;
            } cfgs[] = {{"astar_base_e1.0", false, false, 1.0},
                        {"astar_base_e1.5", false, false, 1.5},
                        {"jps_fast_e1.0", true, true, 1.0},
                        {"jps_fast_e1.5", true, true, 1.5},
                        {"astar_fast_e1.0", true, false, 1.0},
                        {"astar_fast_e1.5", true, false, 1.5}};
            const double tsc_ns = tscPerNs();
            const double cyc2ms = tsc_ns > 0 ? 1e-6 / tsc_ns : 0;
            printf("# suite: reps=%d snapshot_threads=%s phase_timers=%s\n",
                   reps, std::getenv("JPS_SNAPSHOT_THREADS") ?: "1",
                   tsc_ns > 0 ? "on" : "off");
            printf("%-16s %-6s %2s %8s | %9s %9s %9s | %8s %8s %9s %7s %7s %7s | "
                   "%9s %9s %9s %8s | %9s %10s %9s %9s %9s\n",
                   "cfg", "case", "ok", "len_m", "cycle_ms", "p95_ms", "first_ms",
                   "snap_ms", "check", "search", "convert", "corner", "line",
                   "pop", "succ", "proc", "other", "expand", "jump_steps",
                   "queries", "push", "incr");
            for (auto &cf : cfgs)
            {
                Jps3dFrontend fe(mu_rog, false, false, 0.0);
                fe.setThreshVal(kThreshRog);
                fe.setFastMode(cf.fast);
                fe.updateMap(); // warm (first allocation of the snapshot)
                for (auto &c : sc.cases)
                {
                    struct Run
                    {
                        double cycle, snap;
                        JPSPlanner3D::Timings tm;
                        JPS::GraphSearch<3>::Stats st;
                    };
                    vec_Vec3f path;
                    Timer t1(true);
                    fe.updateMap();
                    bool ok = fe.planPath(c.start, c.goal, cf.eps, cf.jps, path);
                    const double first = t1.ElapsedMs();
                    const double len = ok ? pathLength(path) : 0;
                    std::vector<Run> runs;
                    for (int i = 0; i < reps; ++i)
                    {
                        Timer tc(true);
                        fe.updateMap();
                        fe.planPath(c.start, c.goal, cf.eps, cf.jps, path);
                        const double ms = tc.ElapsedMs();
                        runs.push_back({ms, cf.fast ? fe.lastSnapshotMs() : 0.0,
                                        fe.lastTimings(),
                                        fe.debugPlanner()->debugGraphSearch()->stats()});
                    }
                    std::sort(runs.begin(), runs.end(),
                              [](auto &x, auto &y) { return x.cycle < y.cycle; });
                    const Run &m = runs[runs.size() / 2];
                    const double p95 =
                        runs[std::min(runs.size() - 1, (size_t)(runs.size() * 0.95))].cycle;
                    const double pop = m.st.cyc_pop * cyc2ms,
                                 succ = m.st.cyc_succ * cyc2ms,
                                 proc = m.st.cyc_proc * cyc2ms;
                    printf("%-16s %-6s %2d %8.2f | %9.2f %9.2f %9.2f | %8.2f %8.3f "
                           "%9.2f %7.3f %7.3f %7.3f | %9.2f %9.2f %9.2f %8.2f | "
                           "%9lld %10lld %9lld %9lld %9lld\n",
                           cf.name, c.name.c_str(), ok, len, m.cycle, p95, first,
                           m.snap, m.tm.check_ms, m.tm.search_ms, m.tm.convert_ms,
                           m.tm.corner_ms, m.tm.line_ms, pop, succ, proc,
                           tsc_ns > 0 ? m.tm.search_ms - pop - succ - proc : 0.0,
                           m.st.expand, m.st.jump_steps, m.st.cell_queries,
                           m.st.heap_push, m.st.heap_increase);
                    fflush(stdout);
                }
            }
            rog.reset();
            continue;
        }

        struct Backend
        {
            const char *name;
            std::shared_ptr<MapUtil<3>> mu;
            double thresh;
        } backends[] ={{"rog", mu_rog, kThreshRog}}; //, {"simple", mu_simple, 0.0}};

        // headers for table to come
        printf("%-7s %-7s %-8s %-11s %-9s %3s %8s %8s %8s | %8s %8s %8s | %8s %9s %10s %3s\n",
            "mode", "backend", "scene", "case", "algo", "ok", "len_m", "med_ms",
            "p95_ms", "search", "corner", "build1st", "expand", "jumpstep",
            "queries", "att");
        for (auto &be : backends)
        {
            // for comparing between fast mode and normal mode
            for (auto &variant : variants)
            {
                const bool fast = variant != "base"; // as long as variant is not base, it is fast mode.
                Jps3dFrontend fe(be.mu, false, false, 0.0);
                fe.setThreshVal(be.thresh);
                fe.setFastMode(fast);
                if (variant == "fastbox")
                    fe.setSearchBoxWidening(3.0);
                if (variant == "budget")
                    fe.setMaxExpand(2000);
                fe.updateMap();
                if (fast)
                {
                    // re-run to time a warm snapshot
                    fe.updateMap();
                    printf("# %s/%s/%s snapshot: %.2f ms\n", variant.c_str(),
                           be.name, sc.name.c_str(), fe.lastSnapshotMs());
                }
                for (auto &c : sc.cases)
                    for (auto &a : algos)
                    {
                        Result r = runCase(fe, frontendTimings, c, a, reps,
                                           true); // TODO: hardcoded to always cycle
                        if (r.ok != c.expect_path && variant != "budget")
                            printf("# WARN %s/%s/%s/%s: ok=%d expected %d\n",
                                   variant.c_str(), be.name, sc.name.c_str(),
                                   c.name.c_str(), r.ok, c.expect_path);
                        printf("%-7s %-7s %-8s %-11s %-9s %3d %8.2f %8.2f %8.2f | %8.2f "
                               "%8.2f %8.2f | %8lld %9lld %10lld %3d%s\n",
                               variant.c_str(),
                               be.name,
                               sc.name.c_str(),
                               c.name.c_str(),
                               a.name.c_str(), // algo name
                               r.ok,
                               r.len,
                               r.median_ms,
                               r.p95_ms,
                               r.med.search_ms,
                               r.med.corner_ms,
                               r.first_ms,
                               r.med.expand,
                               r.med.jump_steps,
                               r.med.cell_queries,
                               r.attempts,
                               r.status == 3 ? " partial" : "");
                        if (csv.is_open())
                            csv << variant << ',' << be.name << ',' << sc.name << ',' << c.name << ','
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

        if (plot)
        {
            Jps3dFrontend fe(mu_rog, false, false, 0.0);
            fe.setThreshVal(kThreshRog);
            fe.setFastMode(true);
            fe.updateMap();
            for (auto &c : sc.cases)
            {
                std::vector<std::pair<std::string, vec_Vec3f>> algo_paths;
                for (auto &a : algos)
                {
                    vec_Vec3f path;
                    if (fe.planPath(c.start, c.goal, a.eps, a.jps, path))
                        algo_paths.emplace_back(a.name, path);
                }
                const std::string svg_path =
                    "plot_" + sc.name + "_" + c.name + ".svg";
                plotCasePaths(svg_path, scene_cloud, c.start(2) - 0.5,
                              c.start(2) + 0.5, c.start, c.goal, algo_paths);
                printf("# wrote %s\n", svg_path.c_str());
            }
        }
        rog.reset();
    }
    return 0;
}
 