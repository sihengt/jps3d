// The live isOutside() and the fast-mode snapshot must agree about cells
// outside the ESDF's updated bbox. Uses a config whose update box (8x8x6 m) is
// smaller than the map window (20x20x10 m) so such cells exist inside the
// window. Usage: test_rog_map_bbox_consistency <rog_map_small_box.yaml>
#include "rog_map_fixture.hpp"

#include <algorithm>
#include <cstdio>

using namespace JPS;

static int failures = 0;
#define CHECK(cond, ...)                                                       \
    do                                                                         \
    {                                                                          \
        if (!(cond))                                                           \
        {                                                                      \
            printf("FAIL line %d: ", __LINE__);                                \
            printf(__VA_ARGS__);                                               \
            printf("\n");                                                      \
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
    auto rog = std::make_shared<jps_test::TestROGMap>(argv[1]);
    rog->insert(jps_test::openPillars(0));
    auto mu = std::make_shared<ROGMapUtil<3>>(rog);
    mu->updateVirtualCeilingFloor();

    const Vec3i bmin = mu->getUpdatedBboxMinId();
    const Vec3i bmax = mu->getUpdatedBboxMaxId();
    // z centre of the part of the bbox between the virtual floor and ceiling
    const int zlo = std::max(bmin(2), mu->getVirtualFloorIdZ());
    const int zhi = std::min(bmax(2), mu->getVirtualCeilingIdZ());
    Vec3i mid = (bmin + bmax) / 2;
    mid(2) = (zlo + zhi) / 2;
    printf("bbox ids [%d %d %d] .. [%d %d %d]\n", bmin(0), bmin(1), bmin(2),
           bmax(0), bmax(1), bmax(2));

    // Box 20 cells wider than the bbox in x/y, still inside the 20 m window.
    const Vec3i pad(20, 20, 0);
    Vec3i lo = bmin - pad, hi = bmax + pad;
    // Keep only cells the ESDF window and virtual ceiling/floor allow, so the
    // comparison isolates the bbox rule.
    CHECK(rog->insideESDFMap(Vec3i(lo(0), mid(1), mid(2))) &&
              rog->insideESDFMap(Vec3i(hi(0), mid(1), mid(2))),
          "test setup: padded box left the ESDF window");

    // Cells just outside the bbox are blocked by isOutside(); inside is not.
    CHECK(!mu->isOutside(mid), "bbox centre should be inside");
    CHECK(mu->isOutside(Vec3i(bmin(0) - 1, mid(1), mid(2))),
          "cell just below bbox min x should be outside");
    CHECK(mu->isOutside(Vec3i(bmax(0) + 1, mid(1), mid(2))),
          "cell just above bbox max x should be outside");
    CHECK(mu->isOutside(Vec3i(mid(0), bmax(1) + 1, mid(2))),
          "cell just above bbox max y should be outside");

    // The fast sweep must equal the generic per-cell sweep over the padded box,
    // and every cell outside the bbox must be blocked.
    std::vector<uint8_t> fast, generic;
    mu->snapshotOccupancy(lo, hi, 0.0, fast);
    mu->MapUtil<3>::snapshotOccupancy(lo, hi, 0.0, generic);
    CHECK(fast == generic,
          "fast snapshot differs from live isOutside()/isFree() outside the bbox");
    const Vec3i n = hi - lo + Vec3i::Ones();
    long long outside_free = 0;
    for (int x = 0; x < n(0); ++x)
        for (int y = 0; y < n(1); ++y)
            for (int z = 0; z < n(2); ++z)
            {
                const Vec3i p = lo + Vec3i(x, y, z);
                const bool in = (p.array() >= bmin.array()).all() &&
                                (p.array() <= bmax.array()).all();
                if (!in && fast[(static_cast<size_t>(x) * n(1) + y) * n(2) + z] == 0)
                    ++outside_free;
            }
    CHECK(outside_free == 0, "%lld snapshot cells outside the bbox are free",
          outside_free);

    if (failures)
        return 1;
    printf("test_rog_map_bbox_consistency PASSED\n");
    return 0;
}
