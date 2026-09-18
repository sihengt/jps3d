# jps3d_node Octomap Refactor Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Move octomap ingestion/classification out of `jps3d_node.cpp` into a new `OctomapMapUtil` (extends `SimpleMapUtil`), introduce a backend-agnostic `Jps3dFrontend` planning facade, drop the redundant pointcloud path, and get `jps3d_node` building and running again.

**Architecture:** `OctomapMapUtil<Dim,ValueT> : public SimpleMapUtil<Dim,ValueT>` adds one method (`updateFromOctree`) and inherits every query method unchanged. `Jps3dFrontend` holds only the abstract `shared_ptr<JPS::MapUtil<3>>` (never the concrete type) plus a `JPSPlanner3D`, and does frontier-truncation via `isUnknown()`/`isOccupied()` on that abstract interface — so it works for any backend with zero backend-specific code. `jps3d_node.cpp` becomes pure ROS glue: parameters, one octomap subscription, one service.

**Tech Stack:** C++17, ROS 2 (rclcpp), octomap (C++ library, not octomap_msgs, inside jps_collision), Eigen3, existing `jps_lib`.

**Spec:** `docs/superpowers/specs/2026-09-18-jps3d-node-octomap-refactor-design.md`

## Global Constraints

- `OctomapMapUtil` depends only on `<octomap/octomap.h>` (the core octomap library) — never on `octomap_msgs`. ROS message deserialization stays in `jps3d_node.cpp`.
- `Jps3dFrontend` depends only on `jps_basis`, `jps_collision/map_util.h` (abstract), and `jps_planner` — never on ROS, `octomap`, or the concrete `OctomapMapUtil` type.
- Pointcloud support (`voxel_sub_`, `voxel_callback`, `voxel_sub_topic`) is being removed, not kept behind a flag.
- `map_mutex_` stays exactly where it is today (node-owned, guards `octomap_callback` and `plan_callback`) — no new locking is introduced anywhere else.
- `inflate_cell_size_` (existing ROS param) now also governs octomap's inflation radius, replacing the hardcoded `constexpr int inflate_cells = 2` that `octomap_callback` currently has.

---

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `include/jps_collision/map_util_voxel.h` | Modify | Add `SimpleMapUtil::dilateByRadius(int cells)`. |
| `test/test_map_util_dilate.cpp` | Create | Standalone test for `dilateByRadius`. |
| `include/jps_collision/map_util_octo.h` | Rewrite | `OctomapMapUtil<Dim,ValueT> : SimpleMapUtil<Dim,ValueT>` + `updateFromOctree()`. |
| `test/test_map_util_octo.cpp` | Create | Standalone test for `OctomapMapUtil`. |
| `include/jps3d/jps3d_frontend.hpp` | Create | `Jps3dFrontend` declaration. |
| `src/jps3d_frontend.cpp` | Create | `Jps3dFrontend` implementation. |
| `test/test_jps3d_frontend.cpp` | Create | Standalone test for `Jps3dFrontend` frontier truncation. |
| `include/jps3d/jps3d_node.hpp` | Rewrite | Drop pointcloud members; hold `OctomapMapUtil<3>` + `Jps3dFrontend`. |
| `src/jps3d_node.cpp` | Rewrite | Params, one octomap subscription, service, marker publishing. |
| `test/octomap_test_publisher.cpp` | Create | One-shot ROS publisher for the smoke test. |
| `CMakeLists.txt` | Modify | `find_package(octomap)`, new test targets, restore ament/ROS wiring, `jps3d_node` executable. |

---

## Task 1: `SimpleMapUtil::dilateByRadius`

**Files:**
- Modify: `include/jps_collision/map_util_voxel.h` (add method inside `SimpleMapUtil`, near existing `dilate()` at line 364)
- Test: `test/test_map_util_dilate.cpp`
- Modify: `CMakeLists.txt` (register the new test target)

**Interfaces:**
- Produces: `void SimpleMapUtil<Dim,ValueT>::dilateByRadius(int cells)` — public method on `SimpleMapUtil`, callable on `VoxelMapUtil`/`OccMapUtil` and (after Task 2) `OctomapMapUtil`.
- Consumes: existing `dilate(const vec_Veci<Dim>&)`, `isFree`/`isOccupied` — all already present.

- [ ] **Step 1: Write the failing test**

Create `test/test_map_util_dilate.cpp`:

```cpp
#include <jps_collision/map_util_voxel.h>

using namespace JPS;

int main()
{
    // 5x5x5 all-free grid except one occupied cell at (2,2,2).
    Vec3i dim(5, 5, 5);
    Vec3f origin(0, 0, 0);
    std::vector<signed char> data(static_cast<size_t>(dim(0)) * dim(1) * dim(2), 0);
    data[2 + 5 * 2 + 25 * 2] = 100; // occupied

    VoxelMapUtil map_util;
    map_util.setMap(origin, dim, data, 1.0);

    bool all_ok = true;

    // Before dilation, a face neighbor of the occupied cell is still free.
    if (!map_util.isFree(Vec3i(3, 2, 2), 0.0))
    {
        printf(ANSI_COLOR_RED "FAILED: (3,2,2) should start free\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    map_util.dilateByRadius(1);

    // After dilating by 1 cell, every 6-connected face neighbor of (2,2,2)
    // must now be occupied.
    const Vec3i neighbors[6] = {
        Vec3i(3, 2, 2), Vec3i(1, 2, 2), Vec3i(2, 3, 2),
        Vec3i(2, 1, 2), Vec3i(2, 2, 3), Vec3i(2, 2, 1)};
    for (const auto &n : neighbors)
    {
        if (!map_util.isOccupied(n, 0.0))
        {
            printf(ANSI_COLOR_RED
                   "FAILED: neighbor (%d,%d,%d) should be occupied after "
                   "dilateByRadius(1)\n" ANSI_COLOR_RESET,
                   n(0), n(1), n(2));
            all_ok = false;
        }
    }

    // A cell 2 away (outside the radius-1 neighborhood) must remain free.
    if (!map_util.isFree(Vec3i(0, 2, 2), 0.0))
    {
        printf(ANSI_COLOR_RED
               "FAILED: (0,2,2) is 2 cells away, should remain free after "
               "dilateByRadius(1)\n" ANSI_COLOR_RESET);
        all_ok = false;
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_map_util_dilate FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_map_util_dilate PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
```

- [ ] **Step 2: Register the test target and confirm it fails to build**

In `CMakeLists.txt`, after the existing `test_map_util_ceiling` block (around line 68), add:

```cmake
add_executable(test_map_util_dilate test/test_map_util_dilate.cpp)
target_link_libraries(test_map_util_dilate jps_lib)
add_test(test_map_util_dilate test_map_util_dilate)
```

Run: `cmake --build <build_dir> --target test_map_util_dilate 2>&1 | tail -30`
Expected: FAIL — `'class JPS::SimpleMapUtil<3>' has no member named 'dilateByRadius'`.

- [ ] **Step 3: Implement `dilateByRadius`**

In `include/jps_collision/map_util_voxel.h`, add this method immediately after `dilate()` (which ends at line 386 with `map_ = map;` then `}`):

```cpp
    /// Convenience wrapper around dilate(): inflates every occupied cell by
    /// `cells` in each dimension (Chebyshev radius, i.e. the same cube
    /// neighborhood dilate() already takes explicitly), so callers don't
    /// have to hand-build the offset list themselves.
    void dilateByRadius(int cells)
    {
        vec_Veci<Dim> neighbors;
        if constexpr (Dim == 3)
        {
            for (int dx = -cells; dx <= cells; ++dx)
                for (int dy = -cells; dy <= cells; ++dy)
                    for (int dz = -cells; dz <= cells; ++dz)
                        if (dx || dy || dz)
                            neighbors.push_back(Veci<Dim>(dx, dy, dz));
        }
        else
        {
            for (int dx = -cells; dx <= cells; ++dx)
                for (int dy = -cells; dy <= cells; ++dy)
                    if (dx || dy)
                        neighbors.push_back(Veci<Dim>(dx, dy));
        }
        dilate(neighbors);
    }
```

- [ ] **Step 4: Build and run the test**

Run: `cmake --build <build_dir> --target test_map_util_dilate -j4 && <build_dir>/test_map_util_dilate`
Expected: `test_map_util_dilate PASSED`

- [ ] **Step 5: Commit**

```bash
git add include/jps_collision/map_util_voxel.h test/test_map_util_dilate.cpp CMakeLists.txt
git commit -m "feat: add SimpleMapUtil::dilateByRadius, dedupe inflate-neighbor-list construction"
```

---

## Task 2: `OctomapMapUtil`

**Files:**
- Rewrite: `include/jps_collision/map_util_octo.h` (currently a stale, unused pre-refactor `MapUtil<Dim>` copy — replace entirely)
- Test: `test/test_map_util_octo.cpp`
- Modify: `CMakeLists.txt` (add `find_package(octomap REQUIRED)`, register the test target)

**Interfaces:**
- Consumes: `SimpleMapUtil<Dim,ValueT>` (Task 1's `dilateByRadius`, plus inherited `setMap`, `val_free_`/`val_occ_`/`val_unknown_`, `origin_d_`/`dim_`/`res_`, `floatToInt`, `isOutside`, `getIndex`) — all `protected`, accessible from a subclass.
- Produces: `class OctomapMapUtil<Dim, ValueT=double> : public SimpleMapUtil<Dim, ValueT>` with `void updateFromOctree(const octomap::OcTree *tree)`. Typedef `OctomapMapUtil<3> Octo3DMapUtil` for symmetry with `VoxelMapUtil`.

- [ ] **Step 1: Write the failing test**

Create `test/test_map_util_octo.cpp`:

```cpp
#include <jps_collision/map_util_octo.h>
#include <octomap/octomap.h>

using namespace JPS;

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

    // Octree at the SAME 1m resolution as the grid. floatToInt uses
    // round((pt-origin)/res - 0.5), so a 1m leaf's true extent can map to a
    // 2-cell-wide index range at these boundaries (e.g. [1.0,2.0] -> indices
    // {1,2}, not just {1}) -- keep the occupied and free leaves far enough
    // apart that their rounded index ranges don't touch, or "occupied wins"
    // would make the free assertion below fail.
    octomap::OcTree tree(1.0);
    tree.updateNode(octomap::point3d(1.5, 1.5, 1.5), true);  // occupied: world [1.0,2.0]^3 -> cells {1,2}^3
    tree.updateNode(octomap::point3d(3.5, 3.5, 3.5), false); // free: world [3.0,4.0]^3 -> cell (3,3,3) (4 is outside dim=4, clipped)

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

    // A cell the octree never touched stays unknown, but is still treated
    // as free by isFree() (optimistic search) -- the same single-grid
    // duality SimpleMapUtil already provides.
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
    // [0,2]^3, which floatToInt maps to (at least) grid cells {0,1}^3 --
    // this checks that every one of those 8 cells got marked, i.e. one
    // coarse leaf really did fan out over multiple grid cells (it may
    // also reach a couple of index-2 neighbors at the rounding boundary;
    // this test only asserts the cells it must cover, not the ones it
    // must not).
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

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_map_util_octo FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_map_util_octo PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
```

- [ ] **Step 2: Add octomap to the build and register the test target**

In `CMakeLists.txt`, after the existing `find_package(PkgConfig REQUIRED)` (line 17), add:

```cmake
find_package(octomap REQUIRED)
```

After the `test_map_util_dilate` block from Task 1, add:

```cmake
add_executable(test_map_util_octo test/test_map_util_octo.cpp)
target_include_directories(test_map_util_octo PRIVATE ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(test_map_util_octo jps_lib ${OCTOMAP_LIBRARIES})
add_test(test_map_util_octo test_map_util_octo)
```

Run: `cmake <source_dir> -B <build_dir> && cmake --build <build_dir> --target test_map_util_octo 2>&1 | tail -30`
Expected: FAIL — `jps_collision/map_util_octo.h` doesn't declare `OctomapMapUtil` yet (it still holds the stale pre-refactor `MapUtil<Dim>` copy, which itself won't even parse against the current `SimpleMapUtil`/`MapUtil` headers).

- [ ] **Step 3: Rewrite `map_util_octo.h`**

Replace the entire contents of `include/jps_collision/map_util_octo.h` with:

```cpp
/**
 * @file map_util_octo.h
 * @brief OctomapMapUtil, a MapUtil implementation backed by an
 * octomap::OcTree, extending SimpleMapUtil's array storage rather than
 * reimplementing the abstract MapUtil interface from scratch.
 */
#ifndef JPS_MAP_UTIL_OCTO_H
#define JPS_MAP_UTIL_OCTO_H

#include <jps_collision/map_util_voxel.h>
#include <octomap/octomap.h>

namespace JPS
{
/**
 * @brief MapUtil implementation that ingests an octomap::OcTree.
 *
 * Adds exactly one method beyond SimpleMapUtil: updateFromOctree(). Every
 * query (isFree/isOccupied/isUnknown/getCloud/dilate/setCeiling/
 * setThreshVal) is inherited unchanged, so anything holding this through the
 * abstract MapUtil<Dim,ValueT> interface (GraphSearch, JPSPlanner,
 * Jps3dFrontend) sees identical semantics to any other backend.
 *
 * Depends only on the core octomap library, not octomap_msgs -- ROS message
 * deserialization (octomap_msgs::fullMsgToMap) happens one layer up, in the
 * ROS node, mirroring how SimpleMapUtil's setMap() takes plain typed args
 * rather than a sensor_msgs type.
 */
template <int Dim, typename ValueT = double>
class OctomapMapUtil : public SimpleMapUtil<Dim, ValueT>
{
public:
    using Base = SimpleMapUtil<Dim, ValueT>;

    /**
     * @brief Rebuild the grid from an octomap octree.
     *
     * Must be called after an initial setMap(ori, dim, data, res) has
     * established origin_d_/dim_/res_ (see SimpleMapUtil::setMap). Builds a
     * fresh classification buffer seeded fully unknown, walks every octree
     * leaf, and marks the grid cells that leaf's (possibly coarser-than-grid)
     * bounding box covers -- occupied leaves always win over a previously
     * written free/unknown value for the same cell (coarser occupied leaves
     * take precedence, matching a leaf never being downgraded once occupied).
     * Ends by calling the inherited setMap(), which converts the 0/negative/
     * positive convention below into this instance's val_free_/val_occ_/
     * val_unknown_ representation.
     */
    void updateFromOctree(const octomap::OcTree *tree)
    {
        size_t n = static_cast<size_t>(this->dim_(0)) * this->dim_(1);
        if constexpr (Dim == 3)
            n *= this->dim_(2);
        std::vector<signed char> data(n, -1); // start fully unknown

        for (auto it = tree->begin_leafs(), end = tree->end_leafs(); it != end;
             ++it)
        {
            const bool occ = tree->isNodeOccupied(*it);
            const double s = it.getSize(); // leaf edge length in meters
            const octomap::point3d c = it.getCoordinate();

            Vecf<Dim> lo_f, hi_f;
            if constexpr (Dim == 3)
            {
                lo_f = Vecf<Dim>(c.x() - s / 2, c.y() - s / 2, c.z() - s / 2);
                hi_f = Vecf<Dim>(c.x() + s / 2, c.y() + s / 2, c.z() + s / 2);
            }
            else
            {
                lo_f = Vecf<Dim>(c.x() - s / 2, c.y() - s / 2);
                hi_f = Vecf<Dim>(c.x() + s / 2, c.y() + s / 2);
            }
            const Veci<Dim> lo = this->floatToInt(lo_f);
            const Veci<Dim> hi = this->floatToInt(hi_f);

            Veci<Dim> pn;
            if constexpr (Dim == 3)
            {
                for (pn(0) = lo(0); pn(0) <= hi(0); ++pn(0))
                    for (pn(1) = lo(1); pn(1) <= hi(1); ++pn(1))
                        for (pn(2) = lo(2); pn(2) <= hi(2); ++pn(2))
                        {
                            if (this->isOutside(pn))
                                continue;
                            const int idx = this->getIndex(pn);
                            if (occ)
                                data[idx] = 100; // occupied always wins
                            else if (data[idx] != 100)
                                data[idx] = 0; // observed free
                        }
            }
            else
            {
                for (pn(0) = lo(0); pn(0) <= hi(0); ++pn(0))
                    for (pn(1) = lo(1); pn(1) <= hi(1); ++pn(1))
                    {
                        if (this->isOutside(pn))
                            continue;
                        const int idx = this->getIndex(pn);
                        if (occ)
                            data[idx] = 100;
                        else if (data[idx] != 100)
                            data[idx] = 0;
                    }
            }
        }

        this->setMap(this->origin_d_, this->dim_, data, this->res_);
    }
};

typedef OctomapMapUtil<3> Octo3DMapUtil;

} // namespace JPS
#endif
```

- [ ] **Step 4: Build and run the test**

Run: `cmake --build <build_dir> --target test_map_util_octo -j4 && <build_dir>/test_map_util_octo`
Expected: `test_map_util_octo PASSED`

- [ ] **Step 5: Commit**

```bash
git add include/jps_collision/map_util_octo.h test/test_map_util_octo.cpp CMakeLists.txt
git commit -m "feat: implement OctomapMapUtil extending SimpleMapUtil, replacing stale pre-refactor copy"
```

---

## Task 3: `Jps3dFrontend`

**Files:**
- Create: `include/jps3d/jps3d_frontend.hpp`
- Create: `src/jps3d_frontend.cpp`
- Test: `test/test_jps3d_frontend.cpp`
- Modify: `CMakeLists.txt` (register the test target)

**Interfaces:**
- Consumes: `JPS::MapUtil<3>` (abstract — `isUnknown(const Veci<3>&)`, `isOccupied(const Veci<3>&, TmapValue)`, `getThreshDist()`, `getRes()`, `floatToInt()`, `isOutside()`), `JPSPlanner3D` (`setMapUtil`, `updateMap`, `plan`, `getPath`, `status`).
- Produces: `class Jps3dFrontend` with constructor `Jps3dFrontend(std::shared_ptr<JPS::MapUtil<3>> map_util, bool verbose, bool block_unknown, double frontier_seed_radius)`, `void updateMap()`, `void setThreshVal(double)`, `bool planPath(const Vec3f&, const Vec3f&, double eps, bool use_jps, vec_Vec3f& out_path)`, `int status() const`. Task 4 (the node) consumes this directly.

- [ ] **Step 1: Write the failing test**

Create `test/test_jps3d_frontend.cpp`:

```cpp
#include <jps3d/jps3d_frontend.hpp>
#include <jps_collision/map_util_voxel.h>

using namespace JPS;

int main()
{
    // A 1-wide, 10-long corridor along x: cells 0-4 known-free, 5-9 unknown.
    // No obstacles at all -- the planner can search straight through to the
    // goal either way, since unknown reads as free. The only question this
    // test answers is whether planPath's frontier truncation stops the
    // returned path at the free/unknown boundary.
    Vec3i dim(10, 1, 1);
    Vec3f origin(0, 0, 0);
    std::vector<signed char> data(10, 0);
    for (int i = 5; i < 10; ++i)
        data[i] = -1; // unknown

    auto map_util = std::make_shared<VoxelMapUtil>();
    map_util->setMap(origin, dim, data, 1.0);

    const Vec3f start(0.5, 0.5, 0.5);
    const Vec3f goal(9.5, 0.5, 0.5);

    bool all_ok = true;

    // block_unknown = true, frontier_seed_radius = 0.0 (no seed exemption):
    // path must stop before entering the unknown region (x >= 5.0).
    {
        Jps3dFrontend frontend(map_util, false, /*block_unknown=*/true,
                               /*frontier_seed_radius=*/0.0);
        frontend.updateMap();
        vec_Vec3f path;
        if (!frontend.planPath(start, goal, 1.0, true, path) || path.empty())
        {
            printf(ANSI_COLOR_RED
                   "FAILED: block_unknown=true case should still return a "
                   "(truncated) path\n" ANSI_COLOR_RESET);
            all_ok = false;
        }
        else if (path.back()(0) >= 5.0)
        {
            printf(ANSI_COLOR_RED
                   "FAILED: path should be truncated before x=5.0 (got last "
                   "point x=%.2f)\n" ANSI_COLOR_RESET,
                   path.back()(0));
            all_ok = false;
        }
    }

    // block_unknown = false: path should reach the goal, unknown or not.
    {
        Jps3dFrontend frontend(map_util, false, /*block_unknown=*/false,
                               /*frontier_seed_radius=*/0.0);
        frontend.updateMap();
        vec_Vec3f path;
        if (!frontend.planPath(start, goal, 1.0, true, path) || path.empty())
        {
            printf(ANSI_COLOR_RED
                   "FAILED: block_unknown=false case should return a path\n"
                   ANSI_COLOR_RESET);
            all_ok = false;
        }
        else if ((path.back() - goal).norm() > 1e-6)
        {
            printf(ANSI_COLOR_RED
                   "FAILED: block_unknown=false should reach the goal "
                   "unmodified (got last point (%.2f,%.2f,%.2f))\n"
                   ANSI_COLOR_RESET,
                   path.back()(0), path.back()(1), path.back()(2));
            all_ok = false;
        }
    }

    if (!all_ok)
    {
        printf(ANSI_COLOR_RED "test_jps3d_frontend FAILED\n" ANSI_COLOR_RESET);
        return 1;
    }

    printf(ANSI_COLOR_GREEN "test_jps3d_frontend PASSED\n" ANSI_COLOR_RESET);
    return 0;
}
```

- [ ] **Step 2: Add the header, stub source, and test target; confirm it fails**

Create `include/jps3d/jps3d_frontend.hpp`:

```cpp
#pragma once

#include <memory>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util.h>
#include <jps_planner/jps_planner/jps_planner.h>

/**
 * @brief Thin planning facade over an abstract JPS::MapUtil<3> + JPSPlanner3D.
 *
 * Holds map_util_ ONLY as the abstract JPS::MapUtil<3> interface -- never
 * the concrete backend type (OctomapMapUtil, SimpleMapUtil, or any future
 * one) -- so every method here, including frontier truncation, works
 * identically for any backend without a single backend-specific branch.
 */
class Jps3dFrontend
{
public:
    Jps3dFrontend(std::shared_ptr<JPS::MapUtil<3>> map_util, bool verbose,
                  bool block_unknown, double frontier_seed_radius);

    /// Forwards to map_util_->setThreshVal(); affects both the planner's
    /// obstacle margin and (indirectly, via getThreshDist()) frontier
    /// truncation's occupied check.
    void setThreshVal(double thresh_val);

    /// Refresh the planner's internal snapshot after the map changed. Must
    /// be called after every map_util_ mutation before the next planPath().
    void updateMap();

    /// Plans start->goal. On success, fills out_path (world-space waypoints,
    /// start to goal) and returns true. If block_unknown_ is set, the
    /// returned path is truncated at the first point beyond
    /// frontier_seed_radius_ of start that is unknown or occupied. Returns
    /// false (out_path untouched) if no path exists.
    bool planPath(const Vec3f &start, const Vec3f &goal, double eps,
                  bool use_jps, vec_Vec3f &out_path);

    /// Last plan()'s status code (0=ok, -1=no path, 1=start blocked,
    /// 2=goal blocked), for the caller's logging.
    int status() const;

private:
    std::shared_ptr<JPS::MapUtil<3>> map_util_;
    std::shared_ptr<JPSPlanner3D> planner_;
    bool block_unknown_;
    double frontier_seed_radius_;
};
```

Create a stub `src/jps3d_frontend.cpp` (just enough to link, not yet correct):

```cpp
#include <jps3d/jps3d_frontend.hpp>

Jps3dFrontend::Jps3dFrontend(std::shared_ptr<JPS::MapUtil<3>> map_util,
                             bool verbose, bool block_unknown,
                             double frontier_seed_radius)
    : map_util_(map_util), planner_(std::make_shared<JPSPlanner3D>(verbose)),
      block_unknown_(block_unknown),
      frontier_seed_radius_(frontier_seed_radius)
{
    planner_->setMapUtil(map_util_);
}

void Jps3dFrontend::setThreshVal(double thresh_val)
{
    map_util_->setThreshVal(thresh_val);
}

void Jps3dFrontend::updateMap() { planner_->updateMap(); }

bool Jps3dFrontend::planPath(const Vec3f &, const Vec3f &, double, bool,
                             vec_Vec3f &)
{
    return false; // TODO Step 3
}

int Jps3dFrontend::status() const { return planner_->status(); }
```

In `CMakeLists.txt`, after the `test_map_util_octo` block from Task 2, add:

```cmake
add_executable(test_jps3d_frontend test/test_jps3d_frontend.cpp src/jps3d_frontend.cpp)
target_link_libraries(test_jps3d_frontend jps_lib)
add_test(test_jps3d_frontend test_jps3d_frontend)
```

Run: `cmake --build <build_dir> --target test_jps3d_frontend -j4 && <build_dir>/test_jps3d_frontend`
Expected: FAIL — first assertion fails because `planPath` always returns `false` (the stub).

- [ ] **Step 3: Implement `planPath`**

Replace the stub body in `src/jps3d_frontend.cpp` with:

```cpp
#include <algorithm>
#include <cmath>

bool Jps3dFrontend::planPath(const Vec3f &start, const Vec3f &goal,
                             double eps, bool use_jps, vec_Vec3f &out_path)
{
    if (!planner_->plan(start, goal, eps, use_jps))
        return false;

    vec_Vec3f path = planner_->getPath();

    if (!block_unknown_ || path.size() < 2)
    {
        out_path = path;
        return true;
    }

    const double step = std::max(0.5 * map_util_->getRes(), 1e-3);
    const double thresh = map_util_->getThreshDist();

    auto blocked = [&](const Vec3f &p) -> bool
    {
        Vec3i pn = map_util_->floatToInt(p);
        if (map_util_->isOutside(pn))
            return true; // off-map == never observed
        return map_util_->isUnknown(pn) || map_util_->isOccupied(pn, thresh);
    };

    vec_Vec3f truncated;
    truncated.push_back(path.front());
    for (size_t i = 0; i + 1 < path.size(); ++i)
    {
        const Vec3f a = path[i], b = path[i + 1];
        const double len = (b - a).norm();
        const int steps = std::max(1, static_cast<int>(std::ceil(len / step)));
        bool cut = false;
        for (int s = 1; s <= steps; ++s)
        {
            const Vec3f p = a + (b - a) * (static_cast<double>(s) / steps);
            const bool seeded = (p - start).norm() <= frontier_seed_radius_;
            if (!seeded && blocked(p))
            {
                cut = true;
                break;
            }
            truncated.push_back(p);
        }
        if (cut)
            break;
    }
    out_path = truncated;
    return true;
}
```

Add `#include <algorithm>` and `#include <cmath>` at the top of `src/jps3d_frontend.cpp` (alongside the existing `#include <jps3d/jps3d_frontend.hpp>`), replacing the placeholder body from Step 2 entirely — the file should now contain the constructor/`setThreshVal`/`updateMap`/`status` from Step 2 unchanged, plus this real `planPath`.

- [ ] **Step 4: Build and run the test**

Run: `cmake --build <build_dir> --target test_jps3d_frontend -j4 && <build_dir>/test_jps3d_frontend`
Expected: `test_jps3d_frontend PASSED`

- [ ] **Step 5: Commit**

```bash
git add include/jps3d/jps3d_frontend.hpp src/jps3d_frontend.cpp test/test_jps3d_frontend.cpp CMakeLists.txt
git commit -m "feat: add Jps3dFrontend planning facade with backend-agnostic frontier truncation"
```

---

## Task 4: Rewrite `jps3d_node`

**Files:**
- Rewrite: `include/jps3d/jps3d_node.hpp`
- Rewrite: `src/jps3d_node.cpp`

**Interfaces:**
- Consumes: `JPS::OctomapMapUtil<3>` (Task 2: `updateFromOctree`, `dilateByRadius`, inherited `setMap`/`setCeiling`/`getCloud`/`getDim`/`getRes`), `Jps3dFrontend` (Task 3: constructor, `updateMap`, `planPath`).
- Produces: `class Jps3dNode : public rclcpp::Node` — same public shape as today (default-constructible, `main()` spins it). No other file depends on its internals.

This task has no standalone unit test of its own (it's ROS glue over already-tested components); it's verified by Task 5's build and Task 6's smoke test.

- [ ] **Step 1: Rewrite the header**

Replace the entire contents of `include/jps3d/jps3d_node.hpp` with:

```cpp
#pragma once

#include <cmath>
#include <limits>
#include <memory>
#include <mutex>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/srv/get_plan.hpp>
#include <std_msgs/msg/color_rgba.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

#include <jps_basis/data_type.h>
#include <jps_collision/map_util_octo.h>
#include <jps3d/jps3d_frontend.hpp>

class Jps3dNode : public rclcpp::Node
{
public:
    Jps3dNode();

private:
    std::shared_ptr<JPS::OctomapMapUtil<3>> octo_map_util_;
    std::shared_ptr<Jps3dFrontend> frontend_;

    // Protects octo_map_util_/frontend_ across octomap_callback/plan_callback.
    std::mutex map_mutex_;

    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr
        voxel_pub_;
    rclcpp::Subscription<octomap_msgs::msg::Octomap>::SharedPtr octomap_sub_;
    rclcpp::Service<nav_msgs::srv::GetPlan>::SharedPtr plan_srv_;

    visualization_msgs::msg::Marker occ_marker_template_;
    int inflate_cell_size_ = 2;

    /**
     * @brief Bootstraps octo_map_util_ (bounds/origin/resolution/ceiling,
     * all-unknown until the first octomap message) and frontend_.
     */
    void init_map();
    void octomap_callback(const octomap_msgs::msg::Octomap::SharedPtr msg);
    void plan_callback(
        const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
        std::shared_ptr<nav_msgs::srv::GetPlan::Response> response);
};
```

- [ ] **Step 2: Rewrite the source file**

Replace the entire contents of `src/jps3d_node.cpp` with:

```cpp
#include "jps3d/jps3d_node.hpp"

Jps3dNode::Jps3dNode() : Node("jps3d_node")
{
    this->declare_parameter<double>("map_origin_x", -20.0);
    this->declare_parameter<double>("map_origin_y", -20.0);
    this->declare_parameter<double>("map_origin_z", 0.0);
    this->declare_parameter<double>("map_size_x", 40.0);
    this->declare_parameter<double>("map_size_y", 40.0);
    this->declare_parameter<double>("map_size_z", 10.0);
    this->declare_parameter<double>("map_resolution", 0.1);
    this->declare_parameter<double>(
        "ceiling_height_z", std::numeric_limits<double>::infinity());
    this->declare_parameter<double>("eps", 1.0);
    this->declare_parameter<bool>("use_jps", true);
    this->declare_parameter<std::string>("path_pub_topic", "/global_plan");
    this->declare_parameter<std::string>("plan_srv_topic", "/plan");
    this->declare_parameter<std::string>("octomap_sub_topic", "/octomap_full");
    this->declare_parameter<bool>("block_unknown", true);
    this->declare_parameter<double>("frontier_seed_radius", 0.3);
    this->declare_parameter<int>("inflate_cell_size", 2);

    init_map();

    std::string octomap_sub_topic =
        this->get_parameter("octomap_sub_topic").as_string();
    std::string path_pub_topic = this->get_parameter("path_pub_topic").as_string();
    std::string plan_srv_topic = this->get_parameter("plan_srv_topic").as_string();
    inflate_cell_size_ = this->get_parameter("inflate_cell_size").as_int();

    path_pub_ = this->create_publisher<nav_msgs::msg::Path>(path_pub_topic,
                                                             rclcpp::QoS(10));
    voxel_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
        "jps3d_voxels", rclcpp::QoS(10));

    octomap_sub_ = this->create_subscription<octomap_msgs::msg::Octomap>(
        octomap_sub_topic, 10,
        std::bind(&Jps3dNode::octomap_callback, this, std::placeholders::_1));

    plan_srv_ = this->create_service<nav_msgs::srv::GetPlan>(
        plan_srv_topic,
        std::bind(&Jps3dNode::plan_callback, this, std::placeholders::_1,
                  std::placeholders::_2));

    auto dim = octo_map_util_->getDim();
    RCLCPP_INFO(this->get_logger(),
                "JPS3D node ready. Map dim: [%d, %d, %d], resolution: %.3f m, "
                "ceiling: %.2f m",
                dim(0), dim(1), dim(2), octo_map_util_->getRes(),
                this->get_parameter("ceiling_height_z").as_double());
}

void Jps3dNode::init_map()
{
    double ox = this->get_parameter("map_origin_x").as_double();
    double oy = this->get_parameter("map_origin_y").as_double();
    double oz = this->get_parameter("map_origin_z").as_double();
    double sx = this->get_parameter("map_size_x").as_double();
    double sy = this->get_parameter("map_size_y").as_double();
    double sz = this->get_parameter("map_size_z").as_double();
    double res = this->get_parameter("map_resolution").as_double();

    if (res <= 0.0)
    {
        RCLCPP_FATAL(this->get_logger(), "map_resolution must be > 0, got %f",
                    res);
        throw std::invalid_argument("map_resolution must be positive");
    }

    Vec3f origin(ox, oy, oz);
    Vec3i dim(static_cast<int>(std::ceil(sx / res)),
             static_cast<int>(std::ceil(sy / res)),
             static_cast<int>(std::ceil(sz / res)));

    // All-unknown until the first octomap message arrives.
    std::vector<signed char> data(
        static_cast<size_t>(dim(0)) * dim(1) * dim(2), -1);

    octo_map_util_ = std::make_shared<JPS::OctomapMapUtil<3>>();
    octo_map_util_->setMap(origin, dim, data, res);

    double ceiling_z = this->get_parameter("ceiling_height_z").as_double();
    if (std::isfinite(ceiling_z) && ceiling_z <= oz)
        RCLCPP_WARN(this->get_logger(),
                    "ceiling_height_z (%.2f) is at or below map_origin_z "
                    "(%.2f); the entire map will be blocked",
                    ceiling_z, oz);
    octo_map_util_->setCeiling(ceiling_z);

    bool block_unknown = this->get_parameter("block_unknown").as_bool();
    double frontier_seed_radius =
        this->get_parameter("frontier_seed_radius").as_double();
    frontend_ = std::make_shared<Jps3dFrontend>(
        octo_map_util_, /*verbose=*/true, block_unknown, frontier_seed_radius);
    frontend_->updateMap();

    std_msgs::msg::ColorRGBA occupied_color;
    occupied_color.r = 1.0;
    occupied_color.g = 0.0;
    occupied_color.b = 0.0;
    occupied_color.a = 0.5;

    geometry_msgs::msg::Vector3 scale;
    scale.x = res;
    scale.y = res;
    scale.z = res;

    occ_marker_template_.pose.orientation.w = 1;
    occ_marker_template_.type = visualization_msgs::msg::Marker::CUBE;
    occ_marker_template_.action = visualization_msgs::msg::Marker::ADD;
    occ_marker_template_.scale = scale;
    occ_marker_template_.color = occupied_color;
}

void Jps3dNode::octomap_callback(
    const octomap_msgs::msg::Octomap::SharedPtr msg)
{
    std::unique_ptr<octomap::AbstractOcTree> abstract(
        octomap_msgs::fullMsgToMap(*msg));
    if (!abstract)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "octomap_callback: failed to deserialize %s",
                            this->get_parameter("octomap_sub_topic")
                                .as_string()
                                .c_str());
        return;
    }
    auto *tree = dynamic_cast<octomap::OcTree *>(abstract.get());
    if (!tree)
    {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                            "octomap_callback: octree is not an OcTree");
        return;
    }

    std::lock_guard<std::mutex> lock(map_mutex_);

    octo_map_util_->updateFromOctree(tree);
    octo_map_util_->dilateByRadius(inflate_cell_size_);
    frontend_->updateMap();

    visualization_msgs::msg::Marker cube_list;
    cube_list.header = msg->header;
    cube_list.ns = "jps3d_voxels";
    cube_list.id = 0;
    cube_list.type = visualization_msgs::msg::Marker::CUBE_LIST;
    cube_list.action = visualization_msgs::msg::Marker::ADD;
    cube_list.scale = occ_marker_template_.scale;
    cube_list.color = occ_marker_template_.color;
    cube_list.pose.orientation.w = 1.0;

    auto occ = octo_map_util_->getCloud();
    cube_list.points.reserve(occ.size());
    for (const auto &p : occ)
    {
        geometry_msgs::msg::Point gp;
        gp.x = p(0);
        gp.y = p(1);
        gp.z = p(2);
        cube_list.points.push_back(gp);
    }

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(cube_list);
    voxel_pub_->publish(markers);

    RCLCPP_DEBUG(this->get_logger(), "Octomap map updated: %zu occupied cells",
                occ.size());
}

void Jps3dNode::plan_callback(
    const std::shared_ptr<nav_msgs::srv::GetPlan::Request> request,
    std::shared_ptr<nav_msgs::srv::GetPlan::Response> response)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    Vec3f start(request->start.pose.position.x, request->start.pose.position.y,
               request->start.pose.position.z);
    Vec3f goal(request->goal.pose.position.x, request->goal.pose.position.y,
              request->goal.pose.position.z);

    std::string frame_id = request->start.header.frame_id.empty()
                              ? request->goal.header.frame_id
                              : request->start.header.frame_id;
    if (frame_id.empty())
    {
        RCLCPP_ERROR(this->get_logger(), "frame_id not set on start or goal");
        return;
    }

    double eps = this->get_parameter("eps").as_double();
    bool use_jps = this->get_parameter("use_jps").as_bool();

    vec_Vec3f path_pts;
    if (!frontend_->planPath(start, goal, eps, use_jps, path_pts))
    {
        RCLCPP_WARN(this->get_logger(), "Planning failed (status %d)",
                   frontend_->status());
        return;
    }

    nav_msgs::msg::Path path_msg;
    rclcpp::Time stamp = this->now();
    path_msg.header.frame_id = frame_id;
    path_msg.header.stamp = stamp;

    for (const auto &pt : path_pts)
    {
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = frame_id;
        pose.header.stamp = stamp;
        pose.pose.position.x = pt(0);
        pose.pose.position.y = pt(1);
        pose.pose.position.z = pt(2);
        pose.pose.orientation.w = 1.0;
        path_msg.poses.push_back(pose);
    }

    response->plan = path_msg;
    path_pub_->publish(path_msg);
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<Jps3dNode>());
    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 3: Commit**

```bash
git add include/jps3d/jps3d_node.hpp src/jps3d_node.cpp
git commit -m "refactor: rewrite jps3d_node as pure ROS glue over OctomapMapUtil + Jps3dFrontend, drop pointcloud path"
```

(This won't build yet — `jps3d_node` isn't wired into `CMakeLists.txt`. That's Task 5.)

---

## Task 5: Restore build wiring and build `jps3d_node`

**Files:**
- Modify: `CMakeLists.txt`

**Interfaces:**
- Consumes: every file from Tasks 1-4.
- Produces: a `jps3d_node` executable target.

- [ ] **Step 1: Restore ament/ROS `find_package`s**

In `CMakeLists.txt`, replace:

```cmake
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)

find_package(octomap REQUIRED)
```

with:

```cmake
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
```

- [ ] **Step 2: Replace the commented-out `jps3d_node` block**

Find (from the original file, still present as comments):

```cmake
# add_executable(jps3d_node src/jps3d_node.cpp)
# target_include_directories(jps3d_node PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
# target_compile_features(jps3d_node PUBLIC c_std_99 cxx_std_17)
# target_include_directories(jps3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
# target_link_libraries(jps3d_node jps_lib ${OCTOMAP_LIBRARIES})
# ament_target_dependencies(jps3d_node
#   rclcpp nav_msgs geometry_msgs sensor_msgs visualization_msgs std_msgs
#   octomap_msgs octomap)
```

Replace it with (uncommented, `jps3d_frontend.cpp` added as a second source, `sensor_msgs` dropped since pointcloud is gone but kept in `ament_target_dependencies` is unnecessary now):

```cmake
add_executable(jps3d_node src/jps3d_node.cpp src/jps3d_frontend.cpp)
target_include_directories(jps3d_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>)
target_compile_features(jps3d_node PUBLIC c_std_99 cxx_std_17)
target_include_directories(jps3d_node PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(jps3d_node jps_lib ${OCTOMAP_LIBRARIES})
ament_target_dependencies(jps3d_node
  rclcpp nav_msgs geometry_msgs visualization_msgs std_msgs octomap_msgs)
```

- [ ] **Step 3: Restore `ament_package()` and the `jps3d_node` install rule**

Uncomment the line near the end of the file:

```cmake
# ament_package()
```
becomes
```cmake
ament_package()
```

Uncomment the install block:

```cmake
# install(TARGETS jps3d_node
#   DESTINATION lib/${PROJECT_NAME})
```
becomes
```cmake
install(TARGETS jps3d_node
  DESTINATION lib/${PROJECT_NAME})
```

Leave every other still-commented block (`dmp_lib`, `create_map`, the launch/config installs, the scripts install) exactly as-is — they're pre-existing, unrelated TODOs, not part of this refactor.

- [ ] **Step 4: Build everything and confirm no regressions**

Run: `cmake <source_dir> -B <build_dir> -DCMAKE_BUILD_TYPE=Release && cmake --build <build_dir> -j4 2>&1 | tail -100`
Expected: `[100%] Built target jps3d_node` with no errors, and every previously-passing target (`jps_lib`, `test_planner_2d/3d/replan/ceiling`, `test_map_util_ceiling`, `test_map_util_dilate`, `test_map_util_octo`, `test_jps3d_frontend`) still builds.

- [ ] **Step 5: Run the full existing test suite to confirm no regressions**

Run: `cd <build_dir> && ctest --output-on-failure`
Expected: all tests pass, including the three new ones from Tasks 1-3.

- [ ] **Step 6: Commit**

```bash
git add CMakeLists.txt
git commit -m "build: restore ament/ROS wiring and re-enable jps3d_node executable"
```

---

## Task 6: Smoke-test `jps3d_node` end-to-end

**Files:**
- Create: `test/octomap_test_publisher.cpp`
- Modify: `CMakeLists.txt` (register the publisher executable — test-only tool, not installed)

**Interfaces:**
- Consumes: `octomap_msgs`, `octomap`, `rclcpp` (already found in Task 5).
- Produces: a one-shot executable `octomap_test_publisher` that publishes a single `octomap_msgs::msg::Octomap` on `/octomap_full` and exits.

- [ ] **Step 1: Write the publisher**

Create `test/octomap_test_publisher.cpp`:

```cpp
// One-shot publisher used to smoke-test jps3d_node: builds a small octree
// (5x5x5 world, 1m resolution, free except a wall at x=2 blocking y in
// [0,5) except at z=4, which is left open), serializes it, and publishes
// once on /octomap_full.
#include <chrono>
#include <thread>

#include <rclcpp/rclcpp.hpp>
#include <octomap_msgs/msg/octomap.hpp>
#include <octomap_msgs/conversions.h>
#include <octomap/octomap.h>

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("octomap_test_publisher");
    auto pub = node->create_publisher<octomap_msgs::msg::Octomap>(
        "/octomap_full", rclcpp::QoS(10));

    octomap::OcTree tree(1.0);
    // Free space everywhere in [0,5)^3.
    for (int x = 0; x < 5; ++x)
        for (int y = 0; y < 5; ++y)
            for (int z = 0; z < 5; ++z)
                tree.updateNode(
                    octomap::point3d(x + 0.5, y + 0.5, z + 0.5), false);
    // Wall at x=2 across all y, blocking z=0..3; z=4 stays open (the gap).
    for (int y = 0; y < 5; ++y)
        for (int z = 0; z < 4; ++z)
            tree.updateNode(octomap::point3d(2.5, y + 0.5, z + 0.5), true);

    octomap_msgs::msg::Octomap msg;
    octomap_msgs::fullMapToMsg(tree, msg);
    msg.header.frame_id = "map";
    msg.header.stamp = node->now();

    // Give the subscriber time to match before publishing (no history QoS
    // durability here, matching jps3d_node's default depth-10 subscription).
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    pub->publish(msg);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    rclcpp::shutdown();
    return 0;
}
```

- [ ] **Step 2: Wire it into the build**

In `CMakeLists.txt`, after the `jps3d_node` block, add:

```cmake
add_executable(octomap_test_publisher test/octomap_test_publisher.cpp)
target_include_directories(octomap_test_publisher PUBLIC ${OCTOMAP_INCLUDE_DIRS})
target_link_libraries(octomap_test_publisher ${OCTOMAP_LIBRARIES})
ament_target_dependencies(octomap_test_publisher rclcpp octomap_msgs)
```

Run: `cmake --build <build_dir> --target jps3d_node octomap_test_publisher -j4 2>&1 | tail -60`
Expected: both link successfully.

- [ ] **Step 3: Run the node and publisher, then call the plan service**

```bash
cd <build_dir>
./jps3d_node --ros-args \
  -p map_origin_x:=0.0 -p map_origin_y:=0.0 -p map_origin_z:=0.0 \
  -p map_size_x:=5.0 -p map_size_y:=5.0 -p map_size_z:=5.0 \
  -p map_resolution:=1.0 -p block_unknown:=false \
  -p inflate_cell_size:=0 &
JPS3D_PID=$!
sleep 2
./octomap_test_publisher
sleep 1
ros2 service call /plan nav_msgs/srv/GetPlan \
  "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {w: 1.0}}}, \
    goal: {header: {frame_id: 'map'}, pose: {position: {x: 4.5, y: 0.5, z: 4.5}, orientation: {w: 1.0}}}, \
    tolerance: 0.0}"
kill $JPS3D_PID
```

Expected: the `ros2 service call` output shows `plan.poses` with at least 3 waypoints; the path's z values should reach up toward 4.5 near x=2 (routing through the gap at z=4 in the wall) rather than a straight line, since a straight line from (0.5,0.5,0.5) to (4.5,0.5,4.5) would pass through the blocked z=0..3 section of the wall at x=2. `inflate_cell_size:=0` avoids the inflated wall eating the one-cell-wide gap.

- [ ] **Step 4: Re-run with `block_unknown:=true` and no publisher message, confirm graceful no-path**

```bash
cd <build_dir>
./jps3d_node --ros-args \
  -p map_origin_x:=0.0 -p map_origin_y:=0.0 -p map_origin_z:=0.0 \
  -p map_size_x:=5.0 -p map_size_y:=5.0 -p map_size_z:=5.0 \
  -p map_resolution:=1.0 -p block_unknown:=true \
  -p frontier_seed_radius:=0.3 &
JPS3D_PID=$!
sleep 2
ros2 service call /plan nav_msgs/srv/GetPlan \
  "{start: {header: {frame_id: 'map'}, pose: {position: {x: 0.5, y: 0.5, z: 0.5}, orientation: {w: 1.0}}}, \
    goal: {header: {frame_id: 'map'}, pose: {position: {x: 4.5, y: 0.5, z: 4.5}, orientation: {w: 1.0}}}, \
    tolerance: 0.0}"
kill $JPS3D_PID
```

Expected: with no octomap message ever published, the entire map is unknown; `plan.poses` in the response should be empty or contain only the start point (frontier truncation with `frontier_seed_radius:=0.3` cuts the path almost immediately, since (0.5,0.5,0.5) is within 0.3m of itself but the very next sampled point along any straight line leaves that radius into unknown space). No crash, no hang.

- [ ] **Step 5: Commit**

```bash
git add test/octomap_test_publisher.cpp CMakeLists.txt
git commit -m "test: add octomap_test_publisher and smoke-test jps3d_node end-to-end"
```

---

## Self-Review Notes

- **Spec coverage:** `OctomapMapUtil : SimpleMapUtil` (Task 2), `Jps3dFrontend` abstract-only (Task 3), `octomap::OcTree*` ingestion boundary (Task 2's `updateFromOctree` signature), pointcloud dropped (Task 4's rewrite has no `voxel_*`), `inflate_cell_size_` governs octomap inflation (Task 1 + Task 4's `octomap_callback`), build wiring restored (Task 5) — every spec section maps to a task.
- **Type consistency checked:** `Jps3dFrontend`'s constructor signature in Task 3 Step 2 matches its use in Task 4 Step 2 (`std::make_shared<Jps3dFrontend>(octo_map_util_, true, block_unknown, frontier_seed_radius)` — 4 args, same order). `OctomapMapUtil<3>` used consistently in Tasks 2 and 4. `planPath`'s signature matches between declaration (Task 3 Step 2) and the node's call site (Task 4 Step 2).
- **No placeholders remain** other than the intentional Task-3-Step-2 stub, which Step 3 of the same task explicitly replaces before that task's own build/test cycle completes.
