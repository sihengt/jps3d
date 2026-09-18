# jps3d_node Octomap Refactor — Design

## Goal

`src/jps3d_node.cpp` currently has octomap-specific ingestion logic (octree
deserialization, occupied/free/unknown classification, a hand-rolled
optimistic-vs-true-map split, obstacle inflation) written directly inline in
the ROS node, duplicated/inconsistent with the simpler pointcloud path it
sits next to. This spec moves that logic into the `jps_collision` MapUtil
layer, drops the pointcloud path (confirmed redundant — see below), and
introduces a thin planning facade so `jps3d_node.cpp` becomes ROS glue only:
parameters, one subscription, one service, nothing else.

## Background / why pointcloud is being dropped

`voxel_callback`'s default topic (`/octomap_point_cloud_centers`) is itself
a standard `octomap_server` output — the same underlying map as
`/octomap_full`, just published as an occupied-cells-only point cloud
instead of the full serialized octree. Neither path is incremental; both
rebuild the whole grid from a complete snapshot on every callback. Pointcloud
mode's only functional difference is that it can't distinguish
free/unknown (so `block_unknown` never engages on that path today). Since it
adds nothing octomap's path doesn't already provide, it is dropped rather
than parameterized alongside octomap.

## Architecture

```
jps3d_node.cpp / jps3d_node.hpp        (ROS I/O only)
  │
  ├─ owns: shared_ptr<JPS::OctomapMapUtil<3>> octo_map_util_   (CONCRETE)
  │         — needed for ingestion-only methods: updateFromOctree(),
  │           dilateByRadius(), setCeiling(), setMap() bootstrap
  │
  ├─ owns: shared_ptr<Jps3dFrontend> frontend_
  │
  │  octomap_callback(msg):
  │    1. deserialize msg -> octomap::OcTree*  (unchanged: octomap_msgs::fullMsgToMap)
  │    2. octo_map_util_->updateFromOctree(tree)
  │    3. octo_map_util_->dilateByRadius(inflate_cell_size_)
  │    4. frontend_->updateMap()
  │    5. publish voxel markers (unchanged, via octo_map_util_->getCloud())
  │
  │  plan_callback(req, resp):
  │    1. frontend_->planPath(start, goal, eps, use_jps, out_path)
  │    2. build nav_msgs::msg::Path from out_path, publish + respond
  │
  └─ std::mutex map_mutex_   (UNCHANGED: still guards both callbacks)

Jps3dFrontend                            (include/jps3d/jps3d_frontend.hpp,
                                           src/jps3d_frontend.cpp)
  │  holds shared_ptr<JPS::MapUtil<3>>  <- ABSTRACT. Never the concrete type.
  │  holds shared_ptr<JPSPlanner3D>
  │
  │  planPath(start, goal, eps, use_jps, out_path):
  │    - planner_->plan(start, goal, eps, use_jps); false -> return false
  │    - path = planner_->getPath()
  │    - if block_unknown_: walk path in getRes()-sized steps, truncate at
  │      first sampled point where map_util_->isUnknown(pn) or
  │      map_util_->isOccupied(pn, thresh) is true, unless within
  │      frontier_seed_radius_ of start (same rule as today's plan_callback)
  │    - out_path = (possibly truncated) path; return true

OctomapMapUtil<Dim, ValueT=double> : public SimpleMapUtil<Dim, ValueT>
  (rewrites include/jps_collision/map_util_octo.h, which currently holds a
   stale pre-refactor copy of MapUtil<Dim> that isn't included anywhere)
  │
  │  void updateFromOctree(const octomap::OcTree* tree);
  │    - builds a fresh Tmap seeded val_unknown_ everywhere (inherited)
  │    - walks tree->begin_leafs()..end_leafs(); for each leaf, expands its
  │      (possibly-coarser-than-grid) bbox into cell range via floatToInt
  │      (inherited) and marks val_occ_/val_free_, never downgrading an
  │      already-val_occ_ cell (same "coarser leaf" precedence as today)
  │    - calls inherited setMap(origin_d_, dim_, data, res_)
  │  — every query method (isFree/isOccupied/isUnknown/getCloud/dilate/
  │    setCeiling/setThreshVal) is INHERITED UNCHANGED from SimpleMapUtil.
  │    OctomapMapUtil adds exactly one new method beyond its parent.
  │  — depends on <octomap/octomap.h> only, not octomap_msgs. ROS message
  │    deserialization stays in jps3d_node.cpp.

SimpleMapUtil<Dim, ValueT>  (include/jps_collision/map_util_voxel.h)
  │  ADD: void dilateByRadius(int cells);
  │    Builds the (2*cells+1)^Dim - 1 neighbor-offset list and calls the
  │    existing dilate(). Both octomap_callback and (formerly) voxel_callback
  │    build this exact neighbor list inline today (duplicated); this pulls
  │    it onto the class that already owns dilate(), used by OctomapMapUtil
  │    and any future backend without re-deriving it in the node.
```

## Data flow

```
/octomap_full (octomap_msgs::msg::Octomap)
        │
        ▼
octomap_callback()
  • octomap_msgs::fullMsgToMap -> octomap::OcTree*        (unchanged, node-owned)
  • octo_map_util_->updateFromOctree(tree)                (NEW — was inline in node)
  • octo_map_util_->dilateByRadius(inflate_cell_size_)     (NEW helper, was inline duplicated loop)
  • frontend_->updateMap()                                 (was planner_->updateMap() directly)
  • publish voxel MarkerArray from octo_map_util_->getCloud()  (unchanged)

/plan (GetPlan service)
  • frontend_->planPath(start, goal, eps, use_jps, out_path)
      └─ internally: planner_->plan(...), then frontier truncation via
         map_util_->isUnknown()/isOccupied() through the ABSTRACT interface
  • build nav_msgs::msg::Path from out_path, publish + respond   (unchanged)
```

## Parameters (after refactor)

Removed: `voxel_sub_topic`. `octomap_sub_topic` stops being an
empty-string-means-fallback flag — it is now the only map input and is
always subscribed (still a parameter, so the topic name itself stays
configurable; default unchanged at `/octomap_full`).

Everything else (`map_origin_*`, `map_size_*`, `map_resolution`,
`ceiling_height_z`, `eps`, `use_jps`, `path_pub_topic`, `plan_srv_topic`,
`block_unknown`, `frontier_seed_radius`, `inflate_cell_size`) is unchanged.
Per the earlier decision, `inflate_cell_size` now also governs octomap's
inflation (previously hardcoded to `constexpr int inflate_cells = 2` in
`octomap_callback`, inconsistent with the parameterized value
`voxel_callback` used).

## Out of scope (explicitly not doing)

- **Searching-horizon / `REACH_HORIZON`-style truncation** from the SUPER
  reference `JpsFrontend`. `jps3d_node` always plans straight to the
  requested goal; `planPath` returns plain success/fail, matching current
  `plan_callback` behavior.
- **Slide-retry-on-origin-shift** from the reference. That guards against
  ROGMap's egocentric *sliding* window moving mid-search. `jps3d`'s grid is
  fixed-origin; nothing slides.
- **New locking scheme.** `map_mutex_` stays exactly where it is, at the
  node level, guarding both callbacks — `Jps3dFrontend` and
  `OctomapMapUtil` are not thread-safe on their own, same as today's
  `map_util_`/`planner_`.
- **jps3d_node_lib / testability restructuring** described in
  `docs/superpowers/plans/2026-04-28-test-jps3d-node.md` (splitting `main()`
  out, `NodeOptions` support). Unrelated to this refactor; not touched.
- **2D backend.** Everything here is 3D-only (`OctomapMapUtil<3>`,
  `Jps3dFrontend` concrete on `Dim=3`), matching the current node.

## Build integration

`jps3d_node`'s `add_executable` block, and the ament/ROS `find_package`
calls and `ament_target_dependencies`/`ament_package()` machinery it needs,
are currently **entirely commented out** in `CMakeLists.txt` (confirmed:
no `find_package(ament_cmake ...)` etc. anywhere in the live file) — this
predates this refactor and isn't something this task introduced. Getting
`jps3d_node` building again requires restoring that wiring, not just
uncommenting the executable block. `package.xml` already declares every
dependency needed (`rclcpp`, `nav_msgs`, `geometry_msgs`, `sensor_msgs`,
`visualization_msgs`, `std_msgs`, `octomap_msgs`, `octomap`) and needs no
changes. The prior plan `docs/superpowers/plans/2026-04-24-jps3d-ros2-node.md`
(Task 1) shows the pattern this project used before; the implementation plan
will follow the same shape, extended for the additional deps now in use
(`octomap_msgs`, `octomap`, `visualization_msgs`, `std_msgs`).

## Testing

- **`OctomapMapUtil` unit test** (new, non-ROS, follows the existing
  `test/test_map_util_ceiling.cpp` pattern which already tests `SimpleMapUtil`
  standalone): build a small in-memory `octomap::OcTree`, call
  `updateFromOctree`, assert `isFree`/`isOccupied`/`isUnknown` match expected
  cells, and that a coarse occupied leaf correctly marks all of its
  constituent grid cells. Runs in this sandbox with no ROS runtime required
  (only links the `octomap` library, already a resolvable dependency here).
- **`jps3d_node` build + smoke test**: this sandbox already has ROS 2 Jazzy
  (`rclcpp`, `octomap_msgs`, etc. all resolve via plain `find_package`, as
  seen configuring the scratch build earlier in this session), so after
  restoring the CMake wiring the node can actually be built and started here
  — `ros2 run` / `ros2 topic pub` / `ros2 service call` against it, similar
  in spirit to the smoke tests in the 2026-04-24 plan, adapted for the
  octomap topic instead of a raw point cloud.
- Existing `test_planner_2d/3d/replan/ceiling` and `test_map_util_ceiling`
  targets are untouched by this refactor and must keep passing.

## File Structure

| File | Action | Responsibility |
|---|---|---|
| `include/jps_collision/map_util_octo.h` | **Rewrite** | `OctomapMapUtil<Dim,ValueT> : SimpleMapUtil<Dim,ValueT>` + `updateFromOctree()`. Replaces current stale/unused contents. |
| `include/jps_collision/map_util_voxel.h` | **Modify** | Add `dilateByRadius(int cells)` to `SimpleMapUtil`. |
| `include/jps3d/jps3d_frontend.hpp` | **Create** | `Jps3dFrontend` declaration. |
| `src/jps3d_frontend.cpp` | **Create** | `Jps3dFrontend` implementation (`planPath`, frontier truncation). |
| `include/jps3d/jps3d_node.hpp` | **Rewrite** | Drop pointcloud members/`voxel_sub_`/`true_map_`/`val_*` constants; hold `OctomapMapUtil<3>` + `Jps3dFrontend`. |
| `src/jps3d_node.cpp` | **Rewrite** | Params, one octomap subscription, service, marker publishing — no classification/truncation logic inline. |
| `CMakeLists.txt` | **Modify** | Restore ament/ROS `find_package`s, re-enable `jps3d_node` executable + new `jps3d_frontend.cpp` source, `ament_target_dependencies`, `ament_package()`. |
| `test/test_map_util_octo.cpp` | **Create** | Standalone `OctomapMapUtil` unit test. |

## Open items for plan-writing

None — all prior open questions were resolved during brainstorming
(facade layer: yes; octomap representation: `OctomapMapUtil : SimpleMapUtil`;
ingestion boundary: `octomap::OcTree*`, not the ROS message; backend
selection: dropped, octomap-only; parameterization target: `inflate_cell_size_`).
