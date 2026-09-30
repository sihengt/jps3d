/**
 * @file map_util.h
 * @brief MapUtil abstract base class
 */
#ifndef JPS_MAP_UTIL_H
#define JPS_MAP_UTIL_H

#include <algorithm>
#include <cstdint>
#include <jps_basis/data_type.h>
#include <vector>

namespace JPS
{
/// The type of map data Tmap is defined as a 1D array. Kept as free aliases
/// (= double) for backward compatibility with non-templated files that use
/// these names directly; MapUtil and its templated owners below use their
/// own nested TmapValue/Tmap (bound to ValueT) instead.
using TmapValue = double;
using Tmap = std::vector<TmapValue>;

/**
 * @brief Abstract interface for the map util classes used for collision
 * checking
 * @param Dim is the dimension of the workspace
 * @param ValueT is the type of a single map cell's value. Defaults to
 * double so every existing MapUtil<Dim> caller keeps compiling unchanged;
 * a future backend (e.g. an int8_t-based octomap) can swap this via
 * MapUtil<Dim, int8_t> without touching anything else.
 *
 * Map-implementation independent: has no dependency on any concrete map
 * backend.
 */
template <int Dim, typename ValueT = double> class MapUtil
{
public:
    /// Nested value-type aliases, bound to ValueT. Unqualified TmapValue/Tmap
    /// used below (and in derived classes) resolve to these rather than the
    /// free JPS::TmapValue/JPS::Tmap aliases above.
    using TmapValue = ValueT;
    using Tmap = std::vector<ValueT>;

    virtual ~MapUtil() = default;

    /// Refresh the virtual ceiling/floor and any other state that depends on
    /// the map having possibly slid. Must be called whenever the map may
    /// have slid (i.e. at the start of every plan()).
    virtual void updateVirtualCeilingFloor() = 0;

    /// Get map data
    virtual Tmap getMap() = 0;
    /// Get resolution
    virtual decimal_t getRes() = 0;
    /// Get dimensions
    virtual Veci<Dim> getDim() const = 0;
    /// Get origin
    virtual Vecf<Dim> getOrigin() = 0;
    /// Get the min/max corners of the *live* map's currently valid region
    virtual void getLocalMapBound(Vecf<Dim> &map_min, Vecf<Dim> &map_max) = 0;
    /// Get index of a cell
    virtual int getIndex(const Veci<Dim> &pn) = 0;

    /// Check if the given cell is outside of the map in i-the dimension
    virtual bool isOutsideXYZ(const Veci<Dim> &n, int i) = 0;
    /// Check if the cell is free by index
    virtual bool isFree(int idx, TmapValue val) = 0;
    /// Check if the cell is unknown by index
    virtual bool isUnknown(int idx) = 0;
    /// Check if the cell is occupied by index
    virtual bool isOccupied(int idx, TmapValue val) = 0;
    /// Check if the cell is outside by coordinate
    virtual bool isOutside(const Veci<Dim> &pn) = 0;
    /// Check if the given cell is free by coordinate
    virtual bool isFree(const Veci<Dim> &pn, TmapValue val) = 0;
    /// Check if the given cell is occupied by coordinate
    virtual bool isOccupied(const Veci<Dim> &pn, TmapValue val) = 0;
    /// Check if the given cell is unknown by coordinate
    virtual bool isUnknown(const Veci<Dim> &pn) = 0;

    /**
     * @brief Update map
     *
     * @param ori origin position
     */
    virtual void setMap(const Vecf<Dim> &ori) = 0;

    /// Print basic information about the util
    virtual void info() = 0;

    /// Float position to discrete cell coordinate
    virtual Veci<Dim> floatToInt(const Vecf<Dim> &pt) = 0;
    /// Discrete cell coordinate to float position
    virtual Vecf<Dim> intToFloat(const Veci<Dim> &pn) = 0;

    /// Raytrace from float point pt1 to pt2
    virtual vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1,
                                   const Vecf<Dim> &pt2) = 0;

    /// Check if the ray from p1 to p2 is occluded
    virtual bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2,
                           TmapValue val) = 0;
    virtual bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2) = 0;

    /// Search for the nearest free cell to seed_pos, within
    /// nearest_free_search_radius_m_.
    virtual bool getNearestKnownFreePos(const Vecf<Dim> &seed_pos,
                                        Vecf<Dim> &nearest_pos) = 0;

    /// Find where the segment (from, to) crosses the live local map's
    /// bounding box (see getLocalMapBound()), writing the crossing point
    /// to hit. Returns false if the segment never crosses the box.
    virtual bool lineIntersectMapBound(const Vecf<Dim> &from,
                                       const Vecf<Dim> &to, Vecf<Dim> &hit) = 0;

    /// Get occupied voxels
    virtual vec_Vecf<Dim> getCloud() = 0;
    /// Get free voxels
    virtual vec_Vecf<Dim> getFreeCloud() = 0;
    /// Get unknown voxels
    virtual vec_Vecf<Dim> getUnknownCloud() = 0;

    virtual void setThreshVal(decimal_t thresh_val) = 0;
    virtual decimal_t getThreshDist() = 0;

    /// Batched neighbor check for graph search: tests the n (<= 32) cells
    /// pn + offs[k]. For each one that is free at threshold `val`, sets bit k
    /// of the returned mask and writes its getIndex() to ids[k] (ids[k] is
    /// unspecified otherwise). Equivalent to n isFree()+getIndex() calls;
    /// backends override it to share the per-center work (one virtual call
    /// and one index computation per expansion instead of per neighbor).
    virtual uint32_t freeNeighbors(const Veci<Dim> &pn, const Veci<Dim> *offs,
                                   int n, TmapValue val, int *ids)
    {
        uint32_t mask = 0;
        for (int k = 0; k < n; ++k)
        {
            const Veci<Dim> q = pn + offs[k];
            if (isFree(q, val))
            {
                ids[k] = getIndex(q);
                mask |= 1u << k;
            }
        }
        return mask;
    }

    /// Flat occupancy snapshot of the cell box [lo, hi] (inclusive), for the
    /// planner's fast path. `out` is resized to prod(hi - lo + 1), indexed
    /// x-fastest by (pn - lo), and holds 0 for cells that are free at
    /// threshold `val` and 1 for everything else (occupied, unknown-as-
    /// occupied if the backend says so, outside the map, above the ceiling).
    /// The default queries isFree() per cell; backends should override with
    /// a direct buffer sweep.
    virtual void snapshotOccupancy(const Veci<Dim> &lo, const Veci<Dim> &hi,
                                   TmapValue val, std::vector<uint8_t> &out)
    {
        const Veci<Dim> n = hi - lo + Veci<Dim>::Ones();
        size_t total = 1;
        for (int i = 0; i < Dim; ++i)
            total *= static_cast<size_t>(std::max(0, n(i)));
        out.assign(total, 1);
        if (total == 0)
            return;
        if constexpr (Dim == 3)
        {
            size_t k = 0;
            for (int z = 0; z < n(2); ++z)
                for (int y = 0; y < n(1); ++y)
                    for (int x = 0; x < n(0); ++x, ++k)
                        out[k] = isFree(lo + Veci<Dim>(x, y, z), val) ? 0 : 1;
        }
        else
        {
            size_t k = 0;
            for (int y = 0; y < n(1); ++y)
                for (int x = 0; x < n(0); ++x, ++k)
                    out[k] = isFree(lo + Veci<Dim>(x, y), val) ? 0 : 1;
        }
    }
};

} // namespace JPS

#endif
