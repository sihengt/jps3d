// Asserts the claim in ROGMapUtil::updateVirtualCeilingFloor()'s doc comment:
// the virtual ceiling/floor z-index is a pure function of a fixed world
// height and resolution, so it must stay identical across a map slide, while
// the ESDF's updated-bbox indices are odom-centered and must move with it.
// Usage: test_rog_map_slide_invariants <rog_map_bench.yaml>
#include "rog_map_fixture.hpp"

#include <cstdio>

using namespace JPS;

static int failures = 0;
#define CHECK(cond, ...)                                                      \
    do                                                                        \
    {                                                                         \
        if (!(cond))                                                          \
        {                                                                     \
            printf(ANSI_COLOR_RED "FAIL %s:%d: ", __FILE__, __LINE__);        \
            printf(__VA_ARGS__);                                              \
            printf("\n" ANSI_COLOR_RESET);                                    \
            failures++;                                                       \
        }                                                                     \
    } while (0)

int main(int argc, char **argv)
{
    if (argc != 2)
    {
        printf("usage: %s <rog_map yaml>\n", argv[0]);
        return 1;
    }

    auto rog = std::make_shared<jps_test::TestROGMap>(argv[1]);
    // No obstacles needed: this test only checks index bookkeeping. insert()
    // with an empty cloud still runs rebuildESDF() so the initial updated
    // bbox is non-degenerate (the ctor alone leaves it at zero, per
    // TestROGMap's comment).
    rog->insert(jps_test::openPillars(0));

    auto mu = std::make_shared<ROGMapUtil<3>>(rog);
    mu->updateVirtualCeilingFloor();

    const int ceil0 = mu->getVirtualCeilingIdZ();
    const int floor0 = mu->getVirtualFloorIdZ();
    const Vec3i bbox_min0 = mu->getUpdatedBboxMinId();
    const Vec3i bbox_max0 = mu->getUpdatedBboxMaxId();
    printf("before slide: ceil_id_z=%d floor_id_z=%d bbox=[%d %d %d]..[%d %d "
           "%d]\n",
           ceil0, floor0, bbox_min0(0), bbox_min0(1), bbox_min0(2),
           bbox_max0(0), bbox_max0(1), bbox_max0(2));

    // Slide the map's window 5 m in x (well within map_size=[40,40,20], so
    // this is an incremental shift, not a full-window reset) and re-center
    // the ESDF's updated-bbox on the new odom.
    const Vec3f fix_origin(0, 0, 10);
    const Vec3f new_odom = fix_origin + Vec3f(5, 0, 0);
    rog->slideTo(new_odom);
    mu->updateVirtualCeilingFloor();

    const int ceil1 = mu->getVirtualCeilingIdZ();
    const int floor1 = mu->getVirtualFloorIdZ();
    const Vec3i bbox_min1 = mu->getUpdatedBboxMinId();
    const Vec3i bbox_max1 = mu->getUpdatedBboxMaxId();
    printf("after slide:  ceil_id_z=%d floor_id_z=%d bbox=[%d %d %d]..[%d %d "
           "%d]\n",
           ceil1, floor1, bbox_min1(0), bbox_min1(1), bbox_min1(2),
           bbox_max1(0), bbox_max1(1), bbox_max1(2));

    // Ceiling/floor come from a fixed world height (config, not odom) run
    // through posToGlobalIndex, which depends only on (pos, resolution) --
    // never on the sliding window's current origin. They must be identical.
    CHECK(ceil0 == ceil1,
          "virtual ceiling z-index changed across slide: %d -> %d", ceil0,
          ceil1);
    CHECK(floor0 == floor1,
          "virtual floor z-index changed across slide: %d -> %d", floor0,
          floor1);

    // The updated bbox is centered on the live odom (getESDFUpdatedBbox()),
    // so it must move when the map slides. Eigen's Matrix type has no
    // operator!=, so compare via the difference (mirrors the existing
    // round-trip check in test_map_util_super.cpp).
    CHECK((bbox_min0 - bbox_min1).squaredNorm() != 0 ||
              (bbox_max0 - bbox_max1).squaredNorm() != 0,
          "updated bbox did not change after a 5 m slide");

    if (failures)
    {
        printf(ANSI_COLOR_RED
               "test_rog_map_slide_invariants: %d failure(s)\n" ANSI_COLOR_RESET,
               failures);
        return 1;
    }
    printf(ANSI_COLOR_GREEN "test_rog_map_slide_invariants PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
