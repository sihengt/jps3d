#include <jps_collision/map_util_octo.h>
#include <jps_collision/map_util_voxel.h>
#include <octomap/octomap.h>
#include <random>

using namespace JPS;

/// Load the same random free/occupied/unknown grid into an OctomapMapUtil and
/// a SimpleMapUtil, apply the same ceiling and dilation, and check that every
/// MapUtil query agrees. Guards OctomapMapUtil's standalone reimplementation
/// of the dense-grid queries against drifting from SimpleMapUtil.
template <int Dim> bool parityCheck(unsigned seed)
{
    std::mt19937 rng(seed);
    Veci<Dim> dim;
    Vecf<Dim> origin;
    for (int i = 0; i < Dim; ++i)
    {
        dim(i) = 7 + 2 * i;
        origin(i) = -1.25 + 0.7 * i;
    }
    const decimal_t res = 0.5;

    size_t n = 1;
    for (int i = 0; i < Dim; ++i)
        n *= dim(i);
    std::vector<signed char> data(n);
    std::uniform_int_distribution<int> pick(0, 99);
    for (auto &c : data)
    {
        const int r = pick(rng);
        c = r < 10 ? 100 : (r < 30 ? -1 : 0); // 10% occ, 20% unknown
    }

    OctomapMapUtil<Dim> octo;
    SimpleMapUtil<Dim> simple;
    octo.setMap(origin, dim, data, res);
    simple.setMap(origin, dim, data, res);
    const decimal_t ceiling_z = origin(Dim - 1) + (dim(Dim - 1) - 2) * res;
    octo.setCeiling(ceiling_z);
    simple.setCeiling(ceiling_z);
    octo.dilateByRadius(1);
    simple.dilateByRadius(1);

    bool ok = true;
    auto fail = [&](const char *what)
    {
        printf(ANSI_COLOR_RED "FAILED: %dD parity mismatch in %s\n"
                              ANSI_COLOR_RESET,
               Dim, what);
        ok = false;
    };

    if (octo.getMap() != simple.getMap())
        fail("getMap");
    if (octo.getCloud() != simple.getCloud())
        fail("getCloud");
    if (octo.getFreeCloud() != simple.getFreeCloud())
        fail("getFreeCloud");
    if (octo.getUnknownCloud() != simple.getUnknownCloud())
        fail("getUnknownCloud");

    // Every cell plus a one-cell ring outside the window.
    Veci<Dim> pn;
    const decimal_t thresh = octo.getThreshDist();
    auto checkCell = [&]()
    {
        if (octo.isOutside(pn) != simple.isOutside(pn))
            fail("isOutside");
        if (octo.isFree(pn, thresh) != simple.isFree(pn, thresh))
            fail("isFree");
        if (octo.isOccupied(pn, thresh) != simple.isOccupied(pn, thresh))
            fail("isOccupied");
        if (octo.isUnknown(pn) != simple.isUnknown(pn))
            fail("isUnknown");
    };
    if constexpr (Dim == 3)
    {
        for (pn(0) = -1; pn(0) <= dim(0); ++pn(0))
            for (pn(1) = -1; pn(1) <= dim(1); ++pn(1))
                for (pn(2) = -1; pn(2) <= dim(2); ++pn(2))
                    checkCell();
    }
    else
    {
        for (pn(0) = -1; pn(0) <= dim(0); ++pn(0))
            for (pn(1) = -1; pn(1) <= dim(1); ++pn(1))
                checkCell();
    }

    // Random world points (some outside the window) for the float queries.
    Vecf<Dim> lo_w, hi_w;
    simple.getLocalMapBound(lo_w, hi_w);
    std::uniform_real_distribution<decimal_t> unit(-0.1, 1.1);
    auto randPt = [&]()
    {
        Vecf<Dim> p;
        for (int i = 0; i < Dim; ++i)
            p(i) = lo_w(i) + unit(rng) * (hi_w(i) - lo_w(i));
        return p;
    };
    for (int t = 0; t < 200; ++t)
    {
        const Vecf<Dim> a = randPt(), b = randPt();
        if (octo.floatToInt(a) != simple.floatToInt(a))
            fail("floatToInt");
        if (octo.rayTrace(a, b) != simple.rayTrace(a, b))
            fail("rayTrace");
        if (octo.isBlocked(a, b) != simple.isBlocked(a, b))
            fail("isBlocked");
        Vecf<Dim> ho = Vecf<Dim>::Zero(), hs = Vecf<Dim>::Zero();
        const bool io = octo.lineIntersectMapBound(a, b, ho);
        const bool is = simple.lineIntersectMapBound(a, b, hs);
        if (io != is || (io && ho != hs))
            fail("lineIntersectMapBound");
        Vecf<Dim> no = Vecf<Dim>::Zero(), ns = Vecf<Dim>::Zero();
        const bool fo = octo.getNearestKnownFreePos(a, no);
        const bool fs = simple.getNearestKnownFreePos(a, ns);
        if (fo != fs || (fo && no != ns))
            fail("getNearestKnownFreePos");
    }
    return ok;
}

int main()
{
    // 4x4x4 grid, 1m resolution, origin at 0 — must be bootstrapped with
    // setMap() before the first updateFromOctree(), same as jps3d_node's
    // init_map() will do.
    Vec3i dim(4, 4, 4);
    Vec3f origin(0, 0, 0);
    std::vector<signed char> bootstrap(
        static_cast<size_t>(dim(0)) * dim(1) * dim(2), -1); // all unknown

    OctomapMapUtil<3> map_util;
    map_util.setMap(origin, dim, bootstrap, 1.0);

    bool all_ok = true;

    // Octree at the SAME 1m resolution as the grid: each leaf is aligned to
    // exactly one grid cell and must mark only that cell -- its +x/+y/+z
    // faces sit on the neighbors' boundaries but must not spill into them.
    octomap::OcTree tree(1.0);
    tree.updateNode(octomap::point3d(1.5, 1.5, 1.5), true);  // occupied: world [1.0,2.0)^3 -> cell (1,1,1)
    tree.updateNode(octomap::point3d(3.5, 3.5, 3.5), false); // free: world [3.0,4.0)^3 -> cell (3,3,3)

    map_util.updateFromOctree(&tree);

    if (!map_util.isOccupied(Vec3i(1, 1, 1), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (1,1,1) should be occupied after updateFromOctree\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }
    if (map_util.isUnknown(Vec3i(3, 3, 3)) ||
        !map_util.isFree(Vec3i(3, 3, 3), 0.0) ||
        map_util.isOccupied(Vec3i(3, 3, 3), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (3,3,3) should be known-free after updateFromOctree\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }

    // Boundary neighbors of the two leaves must stay untouched (unknown).
    const Vec3i untouched[6] = {Vec3i(2, 1, 1), Vec3i(1, 2, 1),
                                Vec3i(1, 1, 2), Vec3i(2, 2, 2),
                                Vec3i(2, 3, 3), Vec3i(3, 3, 2)};
    for (const auto &c : untouched)
    {
        if (!map_util.isUnknown(c))
        {
            printf(ANSI_COLOR_RED
                   "FAILED: (%d,%d,%d) only touches a leaf face, should stay "
                   "unknown\n" ANSI_COLOR_RESET,
                   c(0), c(1), c(2));
            all_ok = false;
        }
    }

    // A cell the octree never touched stays unknown, but is still treated
    // as free by isFree() (optimistic search).
    if (!map_util.isUnknown(Vec3i(0, 0, 0)))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (0,0,0) was never touched, should be unknown\n"
               ANSI_COLOR_RESET);
        all_ok = false;
    }
    if (!map_util.isFree(Vec3i(0, 0, 0), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: unknown cell (0,0,0) should still read as free "
               "(optimistic search)\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    // A coarser leaf spanning multiple grid cells: reset with a fresh 2m
    // octree. A single leaf at (1,1,1) with a 2m edge covers world
    // [0,2)^3, which must mark exactly grid cells {0,1}^3 -- every one of
    // those 8 cells, and none of the index-2 cells its faces touch.
    octomap::OcTree coarse_tree(2.0);
    coarse_tree.updateNode(octomap::point3d(1.0, 1.0, 1.0), true);

    std::vector<signed char> bootstrap2(
        static_cast<size_t>(dim(0)) * dim(1) * dim(2), -1);
    map_util.setMap(origin, dim, bootstrap2, 1.0); // reset before re-ingesting
    map_util.updateFromOctree(&coarse_tree);

    const Vec3i covered[8] = {
        Vec3i(0, 0, 0), Vec3i(1, 0, 0), Vec3i(0, 1, 0), Vec3i(1, 1, 0),
        Vec3i(0, 0, 1), Vec3i(1, 0, 1), Vec3i(0, 1, 1), Vec3i(1, 1, 1)};
    for (const auto &c : covered)
    {
        if (!map_util.isOccupied(c, 0.0))
        {
            printf(ANSI_COLOR_RED
                   "FAILED: coarse leaf should mark grid cell (%d,%d,%d) "
                   "occupied\n" ANSI_COLOR_RESET,
                   c(0), c(1), c(2));
            all_ok = false;
        }
    }
    const Vec3i beyond[4] = {Vec3i(2, 0, 0), Vec3i(0, 2, 0), Vec3i(0, 0, 2),
                             Vec3i(2, 2, 2)};
    for (const auto &c : beyond)
    {
        if (!map_util.isUnknown(c))
        {
            printf(ANSI_COLOR_RED
                   "FAILED: coarse leaf should not reach grid cell "
                   "(%d,%d,%d)\n" ANSI_COLOR_RESET,
                   c(0), c(1), c(2));
            all_ok = false;
        }
    }

    for (unsigned seed = 1; seed <= 5; ++seed)
    {
        if (!parityCheck<3>(seed))
            all_ok = false;
        if (!parityCheck<2>(seed))
            all_ok = false;
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_map_util_octo FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_map_util_octo PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
