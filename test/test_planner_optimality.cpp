// Regression test: JPS must return exactly the same path cost as A*.
//
// Both search the same 26-connected grid with the same edge costs and the same
// consistent heuristic, so JPS -- which only prunes provably redundant
// expansions -- must be cost-identical to A*, never merely close. Two separate
// regressions were caught only by this invariant:
//
//   * the travel direction on a node was not refreshed when the node was
//     relaxed through a cheaper parent (JPS only), so pruning ran against a
//     stale incoming direction;
//   * corner-cut prevention was applied inside jump() but not in getSucc(),
//     which both desynchronised the two searches and invalidated the
//     forced-neighbour tables JPS prunes with.
//
// Also checks that every returned path is actually traversable, that a
// persisted GraphSearch (reused across plan() calls, as the node does) agrees
// with a fresh one, and that JPS never fails where A* succeeds.
#include <jps_collision/map_util_voxel.h>
#include <jps_planner/jps_planner/graph_search.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

using namespace JPS;

namespace
{
struct Grid
{
    int X, Y, Z;
    std::vector<signed char> cells;
    bool occupied(int x, int y, int z) const
    {
        return cells[x + y * X + z * X * Y] != 0;
    }
};

Grid randomGrid(std::mt19937 &rng, int X, int Y, int Z, double density)
{
    Grid g{X, Y, Z, std::vector<signed char>((size_t)X * Y * Z, 0)};
    std::uniform_int_distribution<int> ux(0, X - 1), uy(0, Y - 1), uz(0, Z - 1);
    std::uniform_int_distribution<int> box(1, 4);
    const int target = (int)(density * X * Y * Z);
    int filled = 0;
    while (filled < target)
    {
        int x0 = ux(rng), y0 = uy(rng), z0 = uz(rng);
        int wx = box(rng), wy = box(rng), wz = box(rng);
        for (int x = x0; x < std::min(X, x0 + wx); ++x)
            for (int y = y0; y < std::min(Y, y0 + wy); ++y)
                for (int z = z0; z < std::min(Z, z0 + wz); ++z)
                    if (!g.occupied(x, y, z))
                    {
                        g.cells[x + y * X + z * X * Y] = 100;
                        filled++;
                    }
    }
    return g;
}

/// Build a GraphSearch over g through a SimpleMapUtil, the way JPSPlanner does.
GraphSearch<3> makeSearch(const Grid &g)
{
    auto mu = std::make_shared<SimpleMapUtil<3>>();
    mu->setMap(Vec3f::Zero(), Vec3i(g.X, g.Y, g.Z), g.cells, 1.0);
    return GraphSearch<3>(mu, g.X, g.Y, g.Z, 1.0, false);
}

/// Walk the waypoint path cell by cell: every segment must be a straight
/// cardinal/diagonal run through free cells.
bool traversable(const std::vector<StatePtr> &path, const Grid &g)
{
    if (path.empty())
        return false;
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const int dx = path[i + 1]->x - path[i]->x;
        const int dy = path[i + 1]->y - path[i]->y;
        const int dz = path[i + 1]->z - path[i]->z;
        const int steps = std::max({std::abs(dx), std::abs(dy), std::abs(dz)});
        if (steps == 0)
            return false;
        if ((dx && std::abs(dx) != steps) || (dy && std::abs(dy) != steps) ||
            (dz && std::abs(dz) != steps))
            return false;
        const int sx = (dx > 0) - (dx < 0);
        const int sy = (dy > 0) - (dy < 0);
        const int sz = (dz > 0) - (dz < 0);
        for (int s = 0; s <= steps; ++s)
        {
            const int cx = path[i]->x + sx * s;
            const int cy = path[i]->y + sy * s;
            const int cz = path[i]->z + sz * s;
            if (cx < 0 || cx >= g.X || cy < 0 || cy >= g.Y || cz < 0 ||
                cz >= g.Z)
                return false;
            if (g.occupied(cx, cy, cz))
                return false;
        }
    }
    return true;
}
} // namespace

int main()
{
    const int X = 40, Y = 40, Z = 12;
    const int kTrials = 400;
    std::mt19937 rng(2026);

    int checked = 0, cost_mismatch = 0, jps_missing = 0, untraversable = 0;
    int persisted_mismatch = 0;
    double worst_excess = 0.0;

    for (int t = 0; t < kTrials; ++t)
    {
        const double density = 0.10 + 0.05 * (t % 5); // 0.10 .. 0.30
        Grid g = randomGrid(rng, X, Y, Z, density);

        std::uniform_int_distribution<int> ux(0, X - 1), uy(0, Y - 1),
            uz(0, Z - 1);
        int sx, sy, sz, gx, gy, gz, guard = 0;
        do { sx = ux(rng); sy = uy(rng); sz = uz(rng); }
        while (g.occupied(sx, sy, sz) && ++guard < 2000);
        guard = 0;
        do { gx = ux(rng); gy = uy(rng); gz = uz(rng); }
        while (g.occupied(gx, gy, gz) && ++guard < 2000);
        if (g.occupied(sx, sy, sz) || g.occupied(gx, gy, gz))
            continue;

        GraphSearch<3> astar = makeSearch(g);
        const bool a_ok = astar.plan(sx, sy, sz, gx, gy, gz, false, -1);

        GraphSearch<3> jps = makeSearch(g);
        const bool j_ok = jps.plan(sx, sy, sz, gx, gy, gz, true, -1);

        if (!a_ok)
        {
            if (j_ok)
            {
                printf("FAIL: JPS solved a query A* called unreachable\n");
                return 1;
            }
            continue;
        }
        checked++;

        if (!j_ok)
        {
            jps_missing++;
            printf("FAIL: JPS found no path where A* did: "
                   "start(%d,%d,%d) goal(%d,%d,%d)\n", sx, sy, sz, gx, gy, gz);
            continue;
        }

        const double a_cost = astar.getPath().front()->g;
        const double j_cost = jps.getPath().front()->g;
        if (std::fabs(a_cost - j_cost) > 1e-6)
        {
            cost_mismatch++;
            worst_excess =
                std::max(worst_excess, std::fabs(j_cost - a_cost) / a_cost);
            if (cost_mismatch <= 5)
                printf("FAIL: cost differs  A*=%.6f JPS=%.6f (%+.2f%%)  "
                       "start(%d,%d,%d) goal(%d,%d,%d)\n",
                       a_cost, j_cost, 100.0 * (j_cost - a_cost) / a_cost,
                       sx, sy, sz, gx, gy, gz);
        }

        if (!traversable(astar.getPath(), g) || !traversable(jps.getPath(), g))
        {
            untraversable++;
            printf("FAIL: returned path is not traversable\n");
        }

        // A GraphSearch reused across plan() calls must match a fresh one.
        GraphSearch<3> reused = makeSearch(g);
        reused.plan(gx, gy, gz, sx, sy, sz, true, -1); // warm the pool first
        if (reused.plan(sx, sy, sz, gx, gy, gz, true, -1) != j_ok ||
            std::fabs(reused.getPath().front()->g - j_cost) > 1e-9)
        {
            persisted_mismatch++;
            printf("FAIL: persisted GraphSearch disagrees with a fresh one\n");
        }
    }

    printf("\nqueries compared            : %d\n", checked);
    printf("JPS cost != A* cost         : %d (worst %.2f%%)\n", cost_mismatch,
           100.0 * worst_excess);
    printf("JPS failed where A* solved  : %d\n", jps_missing);
    printf("untraversable paths         : %d\n", untraversable);
    printf("persisted != fresh          : %d\n", persisted_mismatch);

    if (checked < kTrials / 2)
    {
        printf("FAIL: too few usable queries (%d)\n", checked);
        return 1;
    }
    const int failures =
        cost_mismatch + jps_missing + untraversable + persisted_mismatch;
    printf("%s\n", failures == 0 ? "PASS" : "FAIL");
    return failures == 0 ? 0 : 1;
}
