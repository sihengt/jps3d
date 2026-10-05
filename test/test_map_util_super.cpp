// Correctness gate for JPS::ROGMapUtil over a real rog_map::ROGMap.
// Usage: test_map_util_super <rog_map_bench.yaml>
#include "rog_map_fixture.hpp"
#include <jps_basis/timer.hpp>

#include <jps3d/jps3d_frontend.hpp>

#include <cstdio>

using namespace JPS;

static int failures = 0;
#define CHECK(cond, ...)                                                       \
    do                                                                         \
    {                                                                          \
        if (!(cond))                                                           \
        {                                                                      \
            printf(ANSI_COLOR_RED "FAIL %s:%d: ", __FILE__, __LINE__);         \
            printf(__VA_ARGS__);                                               \
            printf("\n" ANSI_COLOR_RESET);                                     \
            failures++;                                                        \
        }                                                                      \
    } while (0)

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("usage: %s <rog_map yaml>\n", argv[0]);
        return 1;
    }

    Timer t(true);
    auto rog = std::make_shared<jps_test::TestROGMap>(argv[1]);
    printf("ROGMap init: %.1f ms\n", t.ElapsedMs());

    t.Reset();
    rog->insert(jps_test::roomsAndDoors());
    printf("insert rooms scene + ESDF: %.1f ms\n", t.ElapsedMs());

    auto mu = std::make_shared<ROGMapUtil<3>>(rog);
    // This ROG-Map fork's ESDF reports one resolution (0.2) on occupied
    // cells, not 0, and sqrt(2)*0.2 on their face neighbours; a 0.3 m
    // threshold (robot_r, one inflation step) reproduces isOccupiedInflate()
    // exactly, so that is the planner's free/occupied boundary.
    const double kThresh = 0.3;
    mu->setThreshVal(kThresh);
    mu->updateVirtualCeilingFloor();
    mu->info();

    // 1. dims agree with the ESDF buffer
    const Vec3i dim = mu->getDim();
    const long long prod = 1LL * dim(0) * dim(1) * dim(2);
    CHECK(prod == (long long)rog->getESDFBuffer().size(),
          "dim prod %lld != buffer %zu", prod, rog->getESDFBuffer().size());
    printf("dim = [%d %d %d] (%lld cells)\n", dim(0), dim(1), dim(2), prod);

    // 2. every reachable cell hashes inside [0, prod)
    Vec3f bmin, bmax;
    mu->getLocalMapBound(bmin, bmax);
    printf("updated bbox: [%.2f %.2f %.2f] .. [%.2f %.2f %.2f]\n", bmin(0),
           bmin(1), bmin(2), bmax(0), bmax(1), bmax(2));
    const Vec3i imin = mu->floatToInt(bmin), imax = mu->floatToInt(bmax);
    long long n_inside = 0, n_free = 0, n_bad = 0;
    for (int x = imin(0); x <= imax(0); ++x)
        for (int y = imin(1); y <= imax(1); ++y)
            for (int z = imin(2); z <= imax(2); ++z)
            {
                Vec3i pn(x, y, z);
                if (mu->isOutside(pn))
                    continue;
                n_inside++;
                int idx = mu->getIndex(pn);
                if (idx < 0 || idx >= prod)
                {
                    n_bad++;
                    continue;
                }
                if (mu->isFree(idx, kThresh))
                    n_free++;
            }
    CHECK(n_bad == 0, "%lld cells hashed outside the node table", n_bad);
    CHECK(n_free > 0, "no free cells at all");
    printf("cells inside ceiling/floor+bbox: %lld, free: %lld\n", n_inside,
           n_free);

    // 3. known cells
    auto at = [&](double x, double y, double z) { return mu->floatToInt(Vec3f(x, y, z)); };
    printf("esdf at wall: %.3f, at room centre: %.3f\n",
           rog->getESDFBuffer()[mu->getIndex(at(-0.1, 7.0, 1.0))],
           rog->getESDFBuffer()[mu->getIndex(at(5.0, 5.0, 1.0))]);
    CHECK(mu->isOccupied(at(-0.1, 7.0, 1.0), kThresh), "wall x=0 should be occupied");
    CHECK(mu->isFree(at(5.0, 5.0, 1.0), 0.0), "room centre should be free");
    CHECK(mu->isFree(at(5.0, 5.0, 1.0), 2.0), "room centre should be >=2 m clear");
    CHECK(!mu->isFree(at(0.4, 7.0, 1.0), 0.5), "0.4 m from wall is < 0.5 clear");
    CHECK(mu->isOutside(at(5.0, 5.0, 3.5)), "above virtual ceiling is outside");
    CHECK(mu->isOutside(at(5.0, 5.0, -0.5)), "below virtual ground is outside");
    // ROG-Map lowers the virtual ceiling by robot_r (3.0 -> 2.7 here).
    CHECK(!mu->isOutside(at(5.0, 5.0, 2.5)), "just under ceiling is inside");

    // 4. float<->int round trip lands in the same cell
    {
        Vec3f p(3.37, -8.21, 1.13);
        Vec3i c = mu->floatToInt(p);
        Vec3f q = mu->intToFloat(c);
        CHECK((mu->floatToInt(q) - c).norm() == 0, "round trip changed cell");
        CHECK((q - p).cwiseAbs().maxCoeff() <= mu->getRes(),
              "cell centre too far from point");
    }

    // 5. plan short and long JPS paths and re-validate them
    Jps3dFrontend fe(mu, /*verbose=*/false, /*block_unknown=*/false, 0.0);
    fe.setThreshVal(kThresh);
    fe.updateMap();
    struct Case
    {
        Vec3f s, g;
        const char *name;
    } cases[] = {{Vec3f(-15, -15, 1), Vec3f(-12, -12, 1), "short"},
                 {Vec3f(-18, -18, 1), Vec3f(18, 18, 1), "long"}};
    for (auto &c : cases)
    {
        Vec3f s, g;
        CHECK(jps_test::snapToFree(*mu, c.s, s), "%s: start not snappable", c.name);
        CHECK(jps_test::snapToFree(*mu, c.g, g), "%s: goal not snappable", c.name);
        vec_Vec3f path;
        t.Reset();
        bool ok = fe.planPath(s, g, 1.0, true, path);
        double ms = t.ElapsedMs();
        CHECK(ok, "%s: plan failed (status %d)", c.name, fe.status());
        if (!ok)
            continue;
        double len = 0;
        for (size_t i = 0; i < path.size(); ++i)
        {
            CHECK(mu->isFree(mu->floatToInt(path[i]), kThresh),
                  "%s: waypoint %zu occupied", c.name, i);
            if (i)
            {
                CHECK(!mu->isBlocked(path[i - 1], path[i], kThresh),
                      "%s: segment %zu blocked", c.name, i);
                len += (path[i] - path[i - 1]).norm();
            }
        }
        printf("%s: %zu waypoints, length %.2f m, %.2f ms\n", c.name,
               path.size(), len, ms);
    }

    // 6. fast mode (flat snapshot) and search-box widening must reproduce the
    //    baseline path cost exactly; snapshot must agree with isFree per cell.
    {
        Jps3dFrontend fast(mu, false, false, 0.0);
        fast.setThreshVal(kThresh);
        fast.setFastMode(true);
        fast.updateMap();
        printf("snapshot build: %.2f ms\n", fast.lastSnapshotMs());
        Jps3dFrontend widen(mu, false, false, 0.0);
        widen.setThreshVal(kThresh);
        widen.setFastMode(true);
        widen.setSearchBoxWidening(2.0);
        widen.updateMap();
        for (auto &c : cases)
        {
            Vec3f s, g;
            jps_test::snapToFree(*mu, c.s, s);
            jps_test::snapToFree(*mu, c.g, g);
            vec_Vec3f p0, p1, p2;
            bool ok0 = fe.planPath(s, g, 1.0, true, p0);
            bool ok1 = fast.planPath(s, g, 1.0, true, p1);
            bool ok2 = widen.planPath(s, g, 1.0, true, p2);
            CHECK(ok0 && ok1 && ok2, "%s: fast/widen plan failed", c.name);
            auto len = [](const vec_Vec3f &p) {
                double l = 0;
                for (size_t i = 1; i < p.size(); ++i)
                    l += (p[i] - p[i - 1]).norm();
                return l;
            };
            CHECK(std::abs(len(p0) - len(p1)) < 1e-6,
                  "%s: fast path length %.4f != baseline %.4f", c.name,
                  len(p1), len(p0));
            CHECK(std::abs(len(p0) - len(p2)) < 1e-6,
                  "%s: widened path length %.4f != baseline %.4f (attempts %d)",
                  c.name, len(p2), len(p0), widen.lastAttempts());
            printf("%s: baseline %.2f ms, fast %.2f ms, widen %.2f ms (%d "
                   "attempts)\n",
                   c.name, fe.lastPlanMs(), fast.lastPlanMs(), widen.lastPlanMs(),
                   widen.lastAttempts());
        }
        // per-cell agreement between the generic and the ROG sweep
        std::vector<uint8_t> a, b;
        const Vec3i lo = imin, hi(imax(0), imax(1), imin(2) + 14);
        mu->snapshotOccupancy(lo, hi, kThresh, a);
        mu->MapUtil<3>::snapshotOccupancy(lo, hi, kThresh, b);
        CHECK(a == b, "ROG snapshot sweep differs from generic per-cell sweep");
    }

    if (failures)
    {
        printf(ANSI_COLOR_RED "test_map_util_super: %d failure(s)\n" ANSI_COLOR_RESET, failures);
        return 1;
    }
    printf(ANSI_COLOR_GREEN "test_map_util_super PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
