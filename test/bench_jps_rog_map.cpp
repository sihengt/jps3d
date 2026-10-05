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
#include "timer.hpp"

#include <jps3d/jps3d_frontend.hpp>
#include <jps_collision/map_util_voxel.h>

#include <boost/geometry.hpp>
#include <boost/geometry/geometries/linestring.hpp>
#include <boost/geometry/geometries/point_xy.hpp>
#include <boost/geometry/geometries/polygon.hpp>

#include <algorithm>
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

// Diagnostic: build the fast-mode snapshot, then re-query every cell in it
// live (through ROGMapUtil::isFree, the same path base/non-fast mode uses)
// and compare classifications. If base mode's huge expand/jump_steps blowup
// (vs fast/cycle mode, same case) came from the live path seeing different
// occupancy than the snapshot it was built from, this shows exactly where
// and how many cells disagree.
void diffOccupancy(std::shared_ptr<ROGMapUtil<3>> mu_rog, double thresh)
{
    Jps3dFrontend fe(mu_rog, false, false, 0.0);
    fe.setThreshVal(thresh);
    fe.setFastMode(true);
    fe.updateMap();
    auto planner = fe.debugPlanner();
    const auto &occ = planner->debugSnapshot();
    const Vec3i lo = planner->debugSnapLo();
    const Vec3i dim = planner->debugSnapDim();
    printf("# diffOccupancy: snapshot lo=%d %d %d dim=%d %d %d (%lld cells)\n",
           lo(0), lo(1), lo(2), dim(0), dim(1), dim(2),
           (long long)dim(0) * dim(1) * dim(2));
    if ((long long)occ.size() != (long long)dim(0) * dim(1) * dim(2))
    {
        printf("# diffOccupancy: ABORT, occ.size()=%zu != dim product\n", occ.size());
        return;
    }
    long long mismatches = 0, checked = 0;
    int printed = 0;
    Timer t(true);
    for (int z = 0; z < dim(2); ++z)
        for (int y = 0; y < dim(1); ++y)
            for (int x = 0; x < dim(0); ++x)
            {
                const Vec3i g = lo + Vec3i(x, y, z);
                const bool snap_free = occ[(1LL * x * dim(1) + y) * dim(2) + z] == 0;
                const bool live_free = mu_rog->isFree(g, thresh);
                ++checked;
                if (snap_free != live_free)
                {
                    ++mismatches;
                    if (printed < 20)
                    {
                        printf("#   mismatch g=%d %d %d snap_free=%d live_free=%d\n",
                               g(0), g(1), g(2), snap_free, live_free);
                        ++printed;
                    }
                }
            }
    printf("# diffOccupancy: checked=%lld mismatches=%lld (%.4f%%) in %.0f ms\n",
           checked, mismatches, 100.0 * mismatches / std::max<long long>(1, checked),
           t.ElapsedMs());
}

// Diagnostic: the ESDF ring-buffer hash is id(0)*mapY*mapZ + id(1)*mapZ +
// id(2), so a long straight walk along X strides ~mapY*mapZ*8 bytes per
// step (multi-MB at 0.05m res -- a guaranteed cache/DRAM miss every step),
// while a walk along Z strides 8 bytes (contiguous). JPS's jump() does long
// straight-line corridor walks; this checks whether that axis asymmetry is
// large enough to matter at this map size.
void axisStrideBench(std::shared_ptr<ROGMapUtil<3>> mu_rog, double thresh,
                     const Vec3i &center, int span, int reps)
{
    volatile int sink = 0;
    for (int axis = 0; axis < 3; ++axis)
    {
        Vec3i step = Vec3i::Zero();
        step(axis) = 1;
        // Each rep sweeps [-span/2, span/2] along `axis`, offset along a
        // *different* axis so every rep touches fresh addresses (no
        // cache/page reuse across reps -- this is meant to model a fresh
        // long jump corridor scan, not a repeated hot-cache walk).
        const int other_axis = (axis + 1) % 3;
        Vec3i rep_step = Vec3i::Zero();
        rep_step(other_axis) = 1;
        long long total = 0;
        Timer t(true);
        for (int r = 0; r < reps; ++r)
        {
            Vec3i base = center + rep_step * (r - reps / 2);
            for (int i = -span / 2; i <= span / 2; ++i)
            {
                Vec3i g = base + step * i;
                sink += mu_rog->isFree(g, thresh) ? 1 : 0;
                ++total;
            }
        }
        const double ms = t.ElapsedMs();
        printf("# axisStrideBench: axis=%c span=%d reps=%d queries=%lld time=%.2f ms "
               "(%.2f ns/query)\n",
               "xyz"[axis], span, reps, total, ms, ms * 1e6 / total);
    }
    (void)sink;
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
    // Waypoints through the real captured scene (output_with_ground.pcd,
    // bounds x[-9.93,15.50] y[-20,-2] z[0,4.07], 13.9M pts). All anchored at
    // (-8,-3), a clear spot near the top-left of the covered area, and
    // walked along the scene's diagonal at increasing distance; "weave" is
    // a short leg straight through the densest cluster instead of around
    // the edge of the captured area, to force real maneuvering. z=1.0 sits
    // inside the surveyed flight band [0.3,1.8] and well under the virtual
    // ceiling (2.7). See docs/perf/2026-09-24-fast-mode-budget-gap.md.
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
        {"jps", true, 1.0},
        {"jps_e1.5", true, 1.5},
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

        if (const char *e = std::getenv("JPS_DEBUG_DIFF"); e && std::string(e) == "1")
            diffOccupancy(mu_rog, kThreshRog);
        if (const char *e = std::getenv("JPS_DEBUG_STRIDE"); e && std::string(e) == "1")
        {
            Vec3i center_id;
            rog->esdfMapPosToGlobalIndex(Vec3f(-8.0, -6.0, 1.0), center_id);
            axisStrideBench(mu_rog, kThreshRog, center_id, 400, 400);
        }

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

        struct Backend
        {
            const char *name;
            std::shared_ptr<MapUtil<3>> mu;
            double thresh;
        } backends[] = {{"rog", mu_rog, kThreshRog}}; //, {"simple", mu_simple, 0.0}};

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
 