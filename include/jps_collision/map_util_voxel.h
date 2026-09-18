/**
 * @file map_util_voxel.h
 * @brief SimpleMapUtil, a plain array-backed MapUtil implementation used for
 * standalone testing and as the default concrete map backend when no
 * external map library (e.g. rog_map) is available.
 */
#ifndef JPS_MAP_UTIL_VOXEL_H
#define JPS_MAP_UTIL_VOXEL_H

#include <algorithm>
#include <cmath>
#include <iostream>
#include <jps_collision/map_util.h>
#include <limits>

namespace JPS
{
/**
 * @brief MapUtil implementation backed by a plain in-memory array
 * @param Dim is the dimension of the workspace
 * @param ValueT is the type of a single map cell's value, forwarded to
 * MapUtil<Dim, ValueT>. Defaults to double.
 */
template <int Dim, typename ValueT = double>
class SimpleMapUtil : public MapUtil<Dim, ValueT>
{
public:
    using TmapValue = typename MapUtil<Dim, ValueT>::TmapValue;
    using Tmap = typename MapUtil<Dim, ValueT>::Tmap;

    /// Simple constructor
    SimpleMapUtil() {}

    /// A static array-backed map never slides, so there's nothing to refresh.
    void updateVirtualCeilingFloor() override {}

    /// Get map data
    Tmap getMap() override { return map_; }
    /// Get resolution
    decimal_t getRes() override { return res_; }
    /// Get dimensions
    Veci<Dim> getDim() const override { return dim_; }
    /// Get origin
    Vecf<Dim> getOrigin() override { return origin_d_; }
    /// Get the min/max corners of the map -- the whole map is always "live"
    void getLocalMapBound(Vecf<Dim> &map_min, Vecf<Dim> &map_max) override
    {
        map_min = origin_d_;
        map_max = origin_d_ + dim_.template cast<decimal_t>() * res_;
    }
    /// Get index of a cell
    int getIndex(const Veci<Dim> &pn) override
    {
        return Dim == 2 ? pn(0) + dim_(0) * pn(1)
                        : pn(0) + dim_(0) * pn(1) + dim_(0) * dim_(1) * pn(2);
    }

    /// Check if the given cell is outside of the map in i-th dimension
    bool isOutsideXYZ(const Veci<Dim> &n, int i) override
    {
        return n(i) < 0 || n(i) >= dim_(i);
    }
    /// Check if the cell is free by index. val <= cell value : free.
    bool isFree(int idx, TmapValue val) override { return map_[idx] >= val; }
    /// Check if the cell is unknown by index
    bool isUnknown(int idx) override { return map_[idx] == val_unknown_; }
    /// Check if the cell is occupied by index. cell value < val : occupied.
    bool isOccupied(int idx, TmapValue val) override
    {
        return map_[idx] < val;
    }

    /// Check if the cell is outside by coordinate, or beyond the virtual
    /// ceiling (if set via setCeiling())
    bool isOutside(const Veci<Dim> &pn) override
    {
        for (int i = 0; i < Dim; i++)
            if (pn(i) < 0 || pn(i) >= dim_(i))
                return true;
        if (Dim == 3 && pn(Dim - 1) >= ceiling_cell_z_)
            return true;
        return false;
    }
    /// Check if the given cell is free by coordinate
    bool isFree(const Veci<Dim> &pn, TmapValue val) override
    {
        if (isOutside(pn))
            return false;
        return isFree(getIndex(pn), val);
    }
    /// Check if the given cell is occupied by coordinate
    bool isOccupied(const Veci<Dim> &pn, TmapValue val) override
    {
        if (isOutside(pn))
            return true;
        return isOccupied(getIndex(pn), val);
    }
    /// Check if the given cell is unknown by coordinate
    bool isUnknown(const Veci<Dim> &pn) override
    {
        if (isOutside(pn))
            return false;
        return isUnknown(getIndex(pn));
    }

    /**
     * @brief Update map origin only
     *
     * @param ori origin position
     */
    void setMap(const Vecf<Dim> &ori) override { origin_d_ = ori; }

    /**
     * @brief Set the full map (convenience overload, not part of the
     * abstract interface)
     *
     * @param ori origin position
     * @param dim number of cells in each dimension
     * @param map array of cell values: 0 -> free, negative -> unknown,
     * positive -> occupied
     * @param res map resolution
     */
    void setMap(const Vecf<Dim> &ori, const Veci<Dim> &dim,
                const std::vector<signed char> &map, decimal_t res)
    {
        dim_ = dim;
        origin_d_ = ori;
        res_ = res;

        map_.resize(map.size());
        for (size_t i = 0; i < map.size(); i++)
        {
            if (map[i] == 0)
                map_[i] = val_free_;
            else if (map[i] < 0)
                map_[i] = val_unknown_;
            else
                map_[i] = val_occ_;
        }

        initSortedNeighbors();
    }

    /**
     * @brief Set a virtual ceiling
     *
     * Any cell whose vertical extent reaches or exceeds this world-frame Z
     * height (meters) is treated as outside the map by isFree()/isOccupied(),
     * regardless of what the underlying map data says. Must be called after
     * setMap(), since it depends on origin/resolution. Pass a non-finite
     * value (e.g. +infinity, the default) to disable. No-op for Dim == 2
     * (no Z axis).
     */
    void setCeiling(decimal_t max_z)
    {
        if (Dim == 3 && std::isfinite(max_z))
            ceiling_cell_z_ = static_cast<int>(
                std::floor((max_z - origin_d_(Dim - 1)) / res_));
        else
            ceiling_cell_z_ = std::numeric_limits<int>::max();
    }

    /// Print basic information about the util
    void info() override
    {
        Vecf<Dim> range = dim_.template cast<decimal_t>() * res_;
        std::cout << "MapUtil Info ========================== " << std::endl;
        std::cout << "   res: [" << res_ << "]" << std::endl;
        std::cout << "   origin: [" << origin_d_.transpose() << "]"
                  << std::endl;
        std::cout << "   range: [" << range.transpose() << "]" << std::endl;
        std::cout << "   dim: [" << dim_.transpose() << "]" << std::endl;
    };

    /// Float position to discrete cell coordinate
    Veci<Dim> floatToInt(const Vecf<Dim> &pt) override
    {
        Veci<Dim> pn;
        for (int i = 0; i < Dim; i++)
            pn(i) = std::round((pt(i) - origin_d_(i)) / res_ - 0.5);
        return pn;
    }
    /// Discrete cell coordinate to float position
    Vecf<Dim> intToFloat(const Veci<Dim> &pn) override
    {
        return (pn.template cast<decimal_t>() + Vecf<Dim>::Constant(0.5)) *
                   res_ +
               origin_d_;
    }

    /// Raytrace from float point pt1 to pt2
    vec_Veci<Dim> rayTrace(const Vecf<Dim> &pt1, const Vecf<Dim> &pt2) override
    {
        Vecf<Dim> diff = pt2 - pt1;
        decimal_t k = 0.8;
        int max_diff = (diff / res_).template lpNorm<Eigen::Infinity>() / k;
        decimal_t s = 1.0 / max_diff;
        Vecf<Dim> step = diff * s;

        vec_Veci<Dim> pns;
        Veci<Dim> prev_pn = Veci<Dim>::Constant(-1);
        for (int n = 1; n < max_diff; n++)
        {
            Vecf<Dim> pt = pt1 + step * n;
            Veci<Dim> new_pn = floatToInt(pt);
            if (isOutside(new_pn))
                break;
            if (new_pn != prev_pn)
                pns.push_back(new_pn);
            prev_pn = new_pn;
        }
        return pns;
    }

    /// Check if the ray from p1 to p2 is occluded
    bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2,
                   TmapValue val) override
    {
        vec_Veci<Dim> pns = rayTrace(p1, p2);
        for (const auto &pn : pns)
        {
            if (!isFree(pn, val))
                return true;
        }
        return false;
    }
    bool isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2) override
    {
        return isBlocked(p1, p2, thresh_val_);
    }

    /// Search sorted_neighbors_ (nearest first, precomputed in the ctor) for
    /// the nearest free cell to seed_pos, within nearest_free_search_radius_m_.
    bool getNearestKnownFreePos(const Vecf<Dim> &seed_pos,
                                Vecf<Dim> &nearest_pos) override
    {
        const Veci<Dim> seed_id = floatToInt(seed_pos);
        if (isFree(seed_id, thresh_val_) && !isUnknown(seed_id))
        {
            nearest_pos = intToFloat(seed_id);
            return true;
        }
        for (const auto &delta : sorted_neighbors_)
        {
            Veci<Dim> cand = seed_id + delta;
            if (isFree(cand, thresh_val_) && !isUnknown(cand))
            {
                nearest_pos = intToFloat(cand);
                return true;
            }
        }
        return false;
    }

    /// Find where the segment (from, to) crosses the map's bounding box,
    /// writing the crossing point to hit. Returns false if the segment never
    /// crosses the box. Slab method.
    bool lineIntersectMapBound(const Vecf<Dim> &from, const Vecf<Dim> &to,
                               Vecf<Dim> &hit) override
    {
        Vecf<Dim> box_min, box_max;
        getLocalMapBound(box_min, box_max);
        const Vecf<Dim> dir = to - from;

        decimal_t t_enter = 0.0;
        decimal_t t_exit = 1.0;
        for (int i = 0; i < Dim; i++)
        {
            if (std::abs(dir(i)) < 1e-9)
            {
                if (from(i) < box_min(i) || from(i) > box_max(i))
                    return false;
                continue;
            }
            decimal_t t1 = (box_min(i) - from(i)) / dir(i);
            decimal_t t2 = (box_max(i) - from(i)) / dir(i);
            if (t1 > t2)
                std::swap(t1, t2);
            t_enter = std::max(t_enter, t1);
            t_exit = std::min(t_exit, t2);
            if (t_enter > t_exit)
                return false;
        }
        hit = from + dir * t_enter;
        return true;
    }

    /// Get occupied voxels
    vec_Vecf<Dim> getCloud() override
    {
        vec_Vecf<Dim> cloud;
        Veci<Dim> n;
        if (Dim == 3)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                        if (isOccupied(getIndex(n), thresh_val_))
                            cloud.push_back(intToFloat(n));
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    if (isOccupied(getIndex(n), thresh_val_))
                        cloud.push_back(intToFloat(n));
        }
        return cloud;
    }

    /// Get free voxels
    vec_Vecf<Dim> getFreeCloud() override
    {
        vec_Vecf<Dim> cloud;
        Veci<Dim> n;
        if (Dim == 3)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                        if (isFree(getIndex(n), thresh_val_))
                            cloud.push_back(intToFloat(n));
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    if (isFree(getIndex(n), thresh_val_))
                        cloud.push_back(intToFloat(n));
        }
        return cloud;
    }

    /// Get unknown voxels
    vec_Vecf<Dim> getUnknownCloud() override
    {
        vec_Vecf<Dim> cloud;
        Veci<Dim> n;
        if (Dim == 3)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                        if (isUnknown(getIndex(n)))
                            cloud.push_back(intToFloat(n));
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    if (isUnknown(getIndex(n)))
                        cloud.push_back(intToFloat(n));
        }
        return cloud;
    }

    void setThreshVal(decimal_t thresh_val) override
    {
        thresh_val_ = thresh_val;
    }
    decimal_t getThreshDist() override { return thresh_val_; }

    /// Dilate occupied cells
    void dilate(const vec_Veci<Dim> &dilate_neighbor)
    {
        Tmap map = map_;
        Veci<Dim> n = Veci<Dim>::Zero();
        if (Dim == 3)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                        if (isOccupied(getIndex(n), thresh_val_))
                            for (const auto &it : dilate_neighbor)
                                if (!isOutside(n + it))
                                    map[getIndex(n + it)] = val_occ_;
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    if (isOccupied(getIndex(n), thresh_val_))
                        for (const auto &it : dilate_neighbor)
                            if (!isOutside(n + it))
                                map[getIndex(n + it)] = val_occ_;
        }
        map_ = map;
    }

    /// Free unknown voxels
    void freeUnknown()
    {
        Veci<Dim> n;
        if (Dim == 3)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                        if (isUnknown(getIndex(n)))
                            map_[getIndex(n)] = val_free_;
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                    if (isUnknown(getIndex(n)))
                        map_[getIndex(n)] = val_free_;
        }
    }

protected:
    /// Map entity
    Tmap map_;
    /// Resolution
    decimal_t res_;
    /// Origin, float type
    Vecf<Dim> origin_d_;
    /// Dimension, int type
    Veci<Dim> dim_;
    /// Cell value assigned to free cells by setMap()
    decimal_t val_free_ = 1.0;
    /// Cell value assigned to occupied cells by setMap()
    decimal_t val_occ_ = -1.0;
    /// Cell value assigned to unknown cells by setMap()
    decimal_t val_unknown_ = std::numeric_limits<decimal_t>::max();
    /// val <= thresh_val_ : occupied; val > thresh_val_ : free. Set via
    /// setThreshVal().
    decimal_t thresh_val_ = 0;
    /// Cell index at/above which a cell is treated as outside the map by
    /// isFree()/isOccupied(), regardless of map data. Set via setCeiling().
    int ceiling_cell_z_ = std::numeric_limits<int>::max();

    /// Search radius (in meters) for getNearestKnownFreePos().
    static constexpr decimal_t nearest_free_search_radius_m_ = 3.0;
    /// Cell offsets within nearest_free_search_radius_m_, sorted nearest-first
    vec_Veci<Dim> sorted_neighbors_;

    void initSortedNeighbors()
    {
        const int r =
            static_cast<int>(std::ceil(nearest_free_search_radius_m_ / res_));
        sorted_neighbors_.clear();
        if constexpr (Dim == 3)
        {
            for (int dx = -r; dx <= r; dx++)
                for (int dy = -r; dy <= r; dy++)
                    for (int dz = -r; dz <= r; dz++)
                    {
                        if (dx == 0 && dy == 0 && dz == 0)
                            continue;
                        if (dx * dx + dy * dy + dz * dz > r * r)
                            continue;
                        sorted_neighbors_.emplace_back(dx, dy, dz);
                    }
        }
        else
        {
            for (int dx = -r; dx <= r; dx++)
                for (int dy = -r; dy <= r; dy++)
                {
                    if (dx == 0 && dy == 0)
                        continue;
                    if (dx * dx + dy * dy > r * r)
                        continue;
                    sorted_neighbors_.emplace_back(dx, dy);
                }
        }
        std::sort(sorted_neighbors_.begin(), sorted_neighbors_.end(),
                  [](const Veci<Dim> &a, const Veci<Dim> &b)
                  { return a.squaredNorm() < b.squaredNorm(); });
    }
};

typedef SimpleMapUtil<2> OccMapUtil;

typedef SimpleMapUtil<3> VoxelMapUtil;

} // namespace JPS

#endif
