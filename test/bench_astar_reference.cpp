// Reference comparison for A* on the real 0.05 m ROG-Map scene: separates
// map-representation cost, search data-structure cost, and problem size.
// Usage: bench_astar_reference <yaml> [reps] [out.csv]
//
// For each case x heuristic weight it reports, all in one process on the same
// snapshot and the same (snapped) start/goal cells:
//   jps_base  : JPSPlanner A*, live ESDF (no snapshot), search time only
//   jps_fast  : JPSPlanner A*, fast-mode snapshot, search time only
//   ref_pw    : minimal A* on the same snapshot -- flat arrays, lazy-deletion
//               std::priority_queue -- with jps3d's exact f (piecewise XDP),
//               heuristic, neighbours, costs and tie-break
//   ref_wa    : same, with standard weighted A*: f = g + w*h
//   ref_pw/wa @0.10 m, @0.20 m : same on the snapshot max-pooled by 2x / 4x
//               (a coarse cell is blocked if any fine cell in it is blocked;
//               0.2 m is the resolution SUPER's own A* plans at)
#include "rog_map_fixture.hpp"
#include "timer.hpp"

#include <jps_planner/jps_planner/jps_planner.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <functional>
#include <queue>
#include <string>
#include <vector>

using namespace JPS;

namespace
{
constexpr double kThresh = 0.3; // see test_map_util_super.cpp
constexpr double kZ = 1.0;

// Occupancy grid, z-fastest like the fast-mode snapshot, padded with one
// blocked cell on every side so neighbour loops need no bounds checks.
struct Grid
{
    int nx = 0, ny = 0, nz = 0; // padded dims
    std::vector<uint8_t> occ;   // 1 = blocked
    size_t id(int x, int y, int z) const
    {
        return (size_t(x) * ny + y) * nz + z;
    }
};

Grid padSnapshot(const std::vector<uint8_t> &snap, const Vec3i &n)
{
    Grid g;
    g.nx = n(0) + 2, g.ny = n(1) + 2, g.nz = n(2) + 2;
    g.occ.assign(size_t(g.nx) * g.ny * g.nz, 1);
    for (int x = 0; x < n(0); ++x)
        for (int y = 0; y < n(1); ++y)
            std::copy_n(&snap[(size_t(x) * n(1) + y) * n(2)], n(2),
                        &g.occ[g.id(x + 1, y + 1, 1)]);
    return g;
}

// Max-pool the (unpadded) snapshot by factor k, then pad.
Grid coarsen(const std::vector<uint8_t> &snap, const Vec3i &n, int k)
{
    const Vec3i m((n(0) + k - 1) / k, (n(1) + k - 1) / k, (n(2) + k - 1) / k);
    std::vector<uint8_t> c(size_t(m.prod()), 0);
    for (int x = 0; x < n(0); ++x)
        for (int y = 0; y < n(1); ++y)
            for (int z = 0; z < n(2); ++z)
                if (snap[(size_t(x) * n(1) + y) * n(2) + z])
                    c[(size_t(x / k) * m(1) + y / k) * m(2) + z / k] = 1;
    return padSnapshot(c, m);
}

enum class FMode
{
    PiecewiseXDP, // jps3d's compare_state::fval
    WeightedAStar // f = g + w*h
};

struct RefResult
{
    bool ok = false;
    long long expand = 0, push = 0;
    double len_cells = 0; // path cost in cells
    double ms = 0;
};

// Minimal A*: flat g/parent/stamp arrays sized to the grid, lazy-deletion
// binary heap of (f, g, id). Same neighbour set (26), costs (1, sqrt2,
// sqrt3), octile heuristic and tie-break (lower f, then higher g) as
// jps3d's GraphSearch. Closed nodes are never reopened (same as jps3d).
class RefAStar
{
public:
    explicit RefAStar(const Grid &g) : g_(g)
    {
        const size_t N = g.occ.size();
        gval_.resize(N);
        parent_.resize(N);
        stamp_.assign(N, 0);
        for (int dx = -1; dx <= 1; ++dx)
            for (int dy = -1; dy <= 1; ++dy)
                for (int dz = -1; dz <= 1; ++dz)
                {
                    if (!dx && !dy && !dz)
                        continue;
                    off_.push_back((long(dx) * g.ny + dy) * g.nz + dz);
                    d_.push_back({dx, dy, dz});
                    cost_.push_back(std::sqrt(double(dx * dx + dy * dy + dz * dz)));
                }
    }

    // s, t: padded-grid coordinates
    RefResult plan(const Vec3i &s, const Vec3i &t, double w, FMode mode)
    {
        Timer tm(true);
        RefResult r;
        epoch_ += 2; // stamp == epoch_: open/seen, epoch_ + 1: closed
        const size_t sid = g_.id(s(0), s(1), s(2)), tid = g_.id(t(0), t(1), t(2));
        if (g_.occ[sid] || g_.occ[tid])
            return r;
        auto heur = [&](int x, int y, int z)
        {
            const int dx = std::abs(x - t(0)), dy = std::abs(y - t(1)),
                      dz = std::abs(z - t(2));
            const int dmin = std::min({dx, dy, dz}), dmax = std::max({dx, dy, dz});
            const int dmid = dx + dy + dz - dmin - dmax;
            return std::sqrt(3.0) * dmin + std::sqrt(2.0) * (dmid - dmin) +
                   (dmax - dmid);
        };
        auto fval = [&](double gv, double h)
        {
            if (mode == FMode::WeightedAStar)
                return gv + w * h;
            return h > gv ? gv + h : (gv + (2.0 * w - 1.0) * h) / w;
        };
        struct E
        {
            double f, g;
            size_t id;
        };
        auto worse = [](const E &a, const E &b)
        {
            if (a.f >= b.f - 1e-6 && a.f <= b.f + 1e-6)
                return a.g < b.g;
            return a.f > b.f;
        };
        std::priority_queue<E, std::vector<E>, decltype(worse)> pq(worse);
        gval_[sid] = 0;
        parent_[sid] = sid;
        stamp_[sid] = epoch_;
        pq.push({fval(0, heur(s(0), s(1), s(2))), 0, sid});
        r.push = 1;
        while (!pq.empty())
        {
            const E e = pq.top();
            pq.pop();
            if (stamp_[e.id] == epoch_ + 1 || e.g > gval_[e.id])
                continue; // closed, or a stale duplicate
            stamp_[e.id] = epoch_ + 1;
            ++r.expand;
            if (e.id == tid)
            {
                r.ok = true;
                r.len_cells = e.g;
                break;
            }
            // decode the expanded cell once; neighbours add the offsets
            const int cz = int(e.id % g_.nz), cy = int((e.id / g_.nz) % g_.ny),
                      cx = int(e.id / (size_t(g_.nz) * g_.ny));
            for (size_t k = 0; k < off_.size(); ++k)
            {
                const size_t nid = size_t(long(e.id) + off_[k]);
                if (g_.occ[nid])
                    continue;
                const uint32_t st = stamp_[nid];
                if (st == epoch_ + 1)
                    continue;
                const double ng = e.g + cost_[k];
                if (st == epoch_ && ng >= gval_[nid] - 1e-9)
                    continue;
                stamp_[nid] = epoch_;
                gval_[nid] = ng;
                parent_[nid] = e.id;
                pq.push({fval(ng, heur(cx + d_[k][0], cy + d_[k][1], cz + d_[k][2])), ng, nid});
                ++r.push;
            }
        }
        r.ms = tm.ElapsedMs();
        return r;
    }

private:
    const Grid &g_;
    std::vector<double> gval_;
    std::vector<size_t> parent_;
    std::vector<uint32_t> stamp_;
    uint32_t epoch_ = 0;
    std::vector<long> off_;
    std::vector<std::array<int, 3>> d_;
    std::vector<double> cost_;
};

// Nearest free cell (padded coords) within r cells of c, or c if none.
Vec3i snapFree(const Grid &g, const Vec3i &c, int r = 3)
{
    if (!g.occ[g.id(c(0), c(1), c(2))])
        return c;
    Vec3i best = c;
    int bd = 1 << 30;
    for (int dx = -r; dx <= r; ++dx)
        for (int dy = -r; dy <= r; ++dy)
            for (int dz = -r; dz <= r; ++dz)
            {
                const Vec3i q = c + Vec3i(dx, dy, dz);
                if ((q.array() < 0).any() || q(0) >= g.nx || q(1) >= g.ny ||
                    q(2) >= g.nz || g.occ[g.id(q(0), q(1), q(2))])
                    continue;
                const int d = dx * dx + dy * dy + dz * dz;
                if (d < bd)
                    bd = d, best = q;
            }
    return best;
}

template <typename F> double medianOf(int reps, F f)
{
    std::vector<double> v;
    for (int i = 0; i < reps; ++i)
        v.push_back(f());
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

struct Case
{
    std::string name;
    Vec3f start, goal;
};
} // namespace

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        printf("usage: %s <yaml> [reps] [out.csv]\n", argv[0]);
        return 1;
    }
    const int reps = argc > 2 ? atoi(argv[2]) : 3;
    std::ofstream csv;
    if (argc > 3)
    {
        csv.open(argv[3]);
        csv << "case,eps,impl,res_m,ok,expand,push,search_ms,ns_per_exp,len_m\n";
    }

    auto rog = std::make_shared<jps_test::TestROGMap>(argv[1]);
    rog->insert(jps_test::loadPcdScene(
        "super_planner/jps/data/output_with_ground.pcd"));
    auto mu = std::make_shared<ROGMapUtil<3>>(rog);
    mu->setThreshVal(kThresh);
    mu->setSnapshotThreads(4);
    mu->updateVirtualCeilingFloor();
    const double res = mu->getRes();

    // Same cases as bench_astar_rog "real"
    std::vector<Case> cases = {
        {"6m", Vec3f(-8.0, -3.0, kZ), Vec3f(-8.0, -9.0, kZ)},
        {"12m", Vec3f(-8.0, -3.0, kZ), Vec3f(-2.0, -13.0, kZ)},
        {"20m", Vec3f(-8.0, -3.0, kZ), Vec3f(6.0, -17.0, kZ)},
        {"28m", Vec3f(-8.0, -3.0, kZ), Vec3f(13.0, -19.0, kZ)},
        {"weave", Vec3f(-8.0, -8.0, kZ), Vec3f(-2.0, -8.0, kZ)},
    };
    for (auto &c : cases)
    {
        Vec3f s, t;
        if (jps_test::snapToFree(*mu, c.start, s) &&
            jps_test::snapToFree(*mu, c.goal, t))
            c.start = s, c.goal = t;
    }

    JPSPlanner3D fast(false), base(false);
    for (auto *p : {&fast, &base})
    {
        p->setMapUtil(mu);
        p->setThreshVal(kThresh);
    }
    fast.setFastMode(true);
    fast.updateMap();
    const Vec3i slo = fast.debugSnapLo(), sn = fast.debugSnapDim();
    const std::vector<uint8_t> snap = fast.debugSnapshot();

    Timer tg(true);
    const Grid g1 = padSnapshot(snap, sn), g2 = coarsen(snap, sn, 2),
               g4 = coarsen(snap, sn, 4);
    printf("# grids: 0.05 m %dx%dx%d, 0.10 m %dx%dx%d, 0.20 m %dx%dx%d "
           "(built in %.0f ms, not timed below)\n",
           sn(0), sn(1), sn(2), g2.nx - 2, g2.ny - 2, g2.nz - 2, g4.nx - 2,
           g4.ny - 2, g4.nz - 2, tg.ElapsedMs());
    RefAStar r1(g1), r2(g2), r4(g4);

    printf("%-6s %4s %-16s %3s %9s %9s %10s %8s %7s\n", "case", "eps", "impl",
           "ok", "expand", "search_ms", "ns/exp", "len_m", "vs_fast");
    const double eps_list[] = {1.0, 1.5, 2.0, 5.0};
    for (const auto &c : cases)
        for (double w : eps_list)
        {
            double fast_ms = 0;
            auto emit = [&](const char *impl, double res_m, bool ok,
                            long long exp, long long push, double ms,
                            double len_m)
            {
                const double nspe = exp ? ms * 1e6 / exp : 0;
                printf("%-6s %4.1f %-16s %3d %9lld %9.2f %10.1f %8.2f %6.2fx\n",
                       c.name.c_str(), w, impl, ok, exp, ms, nspe, len_m,
                       fast_ms > 0 ? ms / fast_ms : 0.0);
                if (csv.is_open())
                    csv << c.name << ',' << w << ',' << impl << ',' << res_m
                        << ',' << ok << ',' << exp << ',' << push << ',' << ms
                        << ',' << nspe << ',' << len_m << '\n';
            };
            // jps3d, fast mode (snapshot), search phase only
            long long fexp = 0;
            bool fok = false;
            double flen = 0;
            fast_ms = medianOf(reps, [&] {
                fok = fast.plan(c.start, c.goal, w, false);
                fexp = fast.lastTimings().expand;
                flen = 0;
                const auto p = fast.getRawPath();
                for (size_t i = 1; i < p.size(); ++i)
                    flen += (p[i] - p[i - 1]).norm();
                return fast.lastTimings().search_ms;
            });
            emit("jps_fast", res, fok, fexp, fast.lastTimings().heap_push,
                 fast_ms, flen);
            // jps3d, base mode (live ESDF); one rep when it is slow
            long long bexp = 0;
            bool bok = false;
            const int breps = fast_ms > 300 ? 1 : reps;
            const double base_ms = medianOf(breps, [&] {
                bok = base.plan(c.start, c.goal, w, false);
                bexp = base.lastTimings().expand;
                return base.lastTimings().search_ms;
            });
            emit("jps_base", res, bok, bexp, base.lastTimings().heap_push,
                 base_ms, flen);
            // reference A*, three resolutions, both f formulas
            struct R
            {
                const char *tag;
                RefAStar *a;
                const Grid *g;
                int k;
            } rs[] = {{"0.05", &r1, &g1, 1}, {"0.10", &r2, &g2, 2}, {"0.20", &r4, &g4, 4}};
            for (auto &rr : rs)
                for (FMode m : {FMode::PiecewiseXDP, FMode::WeightedAStar})
                {
                    const Vec3i s = snapFree(*rr.g, (mu->floatToInt(c.start) - slo) / rr.k + Vec3i::Ones());
                    const Vec3i t = snapFree(*rr.g, (mu->floatToInt(c.goal) - slo) / rr.k + Vec3i::Ones());
                    RefResult res_r;
                    const double ms = medianOf(reps, [&] {
                        res_r = rr.a->plan(s, t, w, m);
                        return res_r.ms;
                    });
                    char name[32];
                    snprintf(name, sizeof name, "ref_%s@%s",
                             m == FMode::PiecewiseXDP ? "pw" : "wa", rr.tag);
                    emit(name, res * rr.k, res_r.ok, res_r.expand, res_r.push,
                         ms, res_r.len_cells * res * rr.k);
                }
        }
    rog.reset();
    return 0;
}
