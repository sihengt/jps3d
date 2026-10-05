// Regression test: the heuristic weight belongs to each GraphSearch instance.
//
// compare_state's f-value used to read a process-global weight that every
// GraphSearch overwrote with its own eps in its ctor and setEps(). A search
// planned after another instance with a different eps was constructed (or
// had setEps() called) therefore ran with the other instance's weight.
//
// Every query is planned by a GraphSearch(eps_a) while a GraphSearch(eps_b) is
// created, reconfigured and planned in between; the result must be identical
// to planning with eps_a alone.
#include <jps_collision/map_util_voxel.h>
#include <jps_planner/jps_planner/graph_search.h>

#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace JPS;

namespace
{
struct Result
{
    bool ok = false;
    double g = 0.0;
    std::vector<Veci<3>> pts;
    bool operator==(const Result &o) const
    {
        if (ok != o.ok)
            return false;
        if (!ok)
            return true;
        if (std::fabs(g - o.g) > 1e-12 || pts.size() != o.pts.size())
            return false;
        for (size_t i = 0; i < pts.size(); ++i)
            if (pts[i] != o.pts[i])
                return false;
        return true;
    }
};

Result run(GraphSearch<3> &gs, const int *s, const int *g, bool jps)
{
    Result r;
    r.ok = gs.plan(s[0], s[1], s[2], g[0], g[1], g[2], jps, -1);
    if (r.ok)
    {
        const auto path = gs.getPath();
        r.g = path.front()->g;
        for (const auto &n : path)
            r.pts.emplace_back(n->x, n->y, n->z);
    }
    return r;
}
} // namespace

int main()
{
    const int X = 40, Y = 40, Z = 12;
    const int kTrials = 100;
    std::mt19937 rng(11);
    int compared = 0, differing = 0;

    for (int t = 0; t < kTrials; ++t)
    {
        std::vector<signed char> cells((size_t)X * Y * Z, 0);
        std::uniform_real_distribution<double> u(0, 1);
        const double density = 0.10 + 0.05 * (t % 5);
        for (auto &c : cells)
            c = u(rng) < density ? 1 : 0;
        auto mu = std::make_shared<SimpleMapUtil<3>>();
        mu->setMap(Vec3f::Zero(), Vec3i(X, Y, Z), cells, 1.0);

        std::uniform_int_distribution<int> ux(0, X - 1), uy(0, Y - 1),
            uz(0, Z - 1);
        int s[3], g[3];
        auto pick = [&](int *p) {
            do { p[0] = ux(rng); p[1] = uy(rng); p[2] = uz(rng); }
            while (cells[p[0] + X * (p[1] + Y * p[2])]);
        };
        pick(s);
        pick(g);

        for (bool jps : {false, true})
        {
            const double eps_a = 1.0, eps_b = 5.0;

            GraphSearch<3> alone(mu, X, Y, Z, eps_a, false);
            const Result expect = run(alone, s, g, jps);

            // Same config as `alone`, but a differently-weighted search is
            // built, reconfigured and run before and after it plans.
            GraphSearch<3> a(mu, X, Y, Z, eps_a, false);
            GraphSearch<3> b(mu, X, Y, Z, eps_b, false);
            b.setEps(eps_b);
            run(b, s, g, jps);
            const Result got = run(a, s, g, jps);

            ++compared;
            if (!(got == expect))
            {
                ++differing;
                if (differing <= 5)
                    printf("FAIL: trial %d jps=%d: eps=%.1f search changed "
                           "(g %.6f vs %.6f) after an eps=%.1f search ran\n",
                           t, jps, eps_a, got.g, expect.g, eps_b);
            }
        }
    }
    printf("compared %d, differing %d\n", compared, differing);
    printf("%s\n", differing == 0 ? "PASS" : "FAIL");
    return differing == 0 ? 0 : 1;
}
