/**
 * @file map_util_super.h
 * @brief ROGMapUtil, the MapUtil implementation built on rog_map::ROGMap
 */
#ifndef JPS_MAP_UTIL_SUPER_H
#define JPS_MAP_UTIL_SUPER_H

#include <algorithm>
#include <iostream>
#include <jps_collision/map_util.h>
#include <rog_map/rog_map.h>
#include <stdexcept>
#include <string>

namespace JPS
{
/**
 * @biref MapUtil implementation for collision checking, backed by a
 * rog_map::ROGMap
 * @param Dim is the dimension of the workspace
 * @param ValueT is the type of a single map cell's value, forwarded to
 * MapUtil<Dim, ValueT>. Defaults to double.
 */
template <int Dim, typename ValueT = double>
class ROGMapUtil final : public MapUtil<Dim, ValueT>
{
public:
    using TmapValue = typename MapUtil<Dim, ValueT>::TmapValue;
    using Tmap = typename MapUtil<Dim, ValueT>::Tmap;

    /// Simple constructor
    ROGMapUtil(std::shared_ptr<rog_map::ROGMap> map_struct_ptr)
        : map_(map_struct_ptr->getESDFBuffer())
    {
        map_class_ptr_ = map_struct_ptr;
        res_ = map_class_ptr_->getESDFResolution();
        virtual_ceiling_ = map_class_ptr_->getVirtualCeilingHeight();
        virtual_floor_ = map_class_ptr_->getVirtualFloorHeight();

        // Hard coded values
        val_unknown_ = std::numeric_limits<double>::max();
        thresh_val_ = 0.0;

        // dim_ must equal the ESDF ring buffer's own grid size, since
        // getIndex() hashes into that buffer and GraphSearch sizes its node
        // table from getDim(). ROG-Map's CounterMap pads the ESDF grid beyond
        // map_size/res (see CounterMap::initCounterMap: half = floor(half_d /
        // res) + (inflation_step + 1), size = 2*half + 1, with inflation_step
        // 0 for the ESDF), so replicate that here and cross-check against the
        // buffer length rather than rounding map_size/res.
        {
            const Vecf<Dim> half_d = map_class_ptr_->getLocalMapSize() / 2.0;
            for (int i = 0; i < Dim; ++i)
                dim_(i) = 2 * (static_cast<int>(half_d(i) / res_) + 1) + 1;
            long long prod = 1;
            for (int i = 0; i < Dim; ++i)
                prod *= dim_(i);
            if (prod != static_cast<long long>(map_.size()))
                throw std::runtime_error(
                    "ROGMapUtil: derived ESDF dims (" + std::to_string(prod) +
                    " cells) do not match ESDF buffer size (" +
                    std::to_string(map_.size()) + ")");
        }

        // Precompute cell offsets within nearest_free_search_radius_m_, sorted
        // nearest-first, so getNearestKnownFreePos() can scan them directly.
        // The radius is specified in meters and converted to cells here using
        // the map resolution (rounded up so the full metric radius is covered).
        const int r =
            static_cast<int>(std::ceil(nearest_free_search_radius_m_ / res_));
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

    /// Refresh the virtual ceiling/floor as ESDF global z indices from the
    /// current world heights, and the ESDF's actually-updated bbox as global
    /// indices. Must be called whenever the map may have slid (i.e. at the
    /// start of every plan()), since the world-height/pos -> global-index
    /// mapping is not stable across slides.
    void updateVirtualCeilingFloor() override
    {
        if constexpr (Dim == 3)
        {
            virtual_ceiling_ = map_class_ptr_->getVirtualCeilingHeight();
            virtual_floor_ = map_class_ptr_->getVirtualFloorHeight();
            // posToGlobalIndex converts each axis independently, so the z
            // result depends only on virtual_ceiling_/virtual_floor_ — the
            // x/y=0 args just fill the discarded ceil_id.x()/.y(). A global
            // index is an absolute lattice coordinate with no bounds check,
            // so this is valid even if world XY=(0,0) is outside the map.
            Vec3i ceil_id, floor_id;
            map_class_ptr_->esdfMapPosToGlobalIndex(
                Vec3f(0, 0, virtual_ceiling_), ceil_id);
            map_class_ptr_->esdfMapPosToGlobalIndex(Vec3f(0, 0, virtual_floor_),
                                                    floor_id);
            virtual_ceiling_id_z_ = ceil_id.z();
            virtual_floor_id_z_ = floor_id.z();

            // insideESDFMap()/insideLocalMap() only bounds-check against the
            // full allocated sliding window (rog_map.map_size), which is far
            // larger than the region the ESDF solver actually refreshed this
            // frame (rog_map.esdf.local_update_box, centered on current odom).
            // A cell outside the updated bbox but inside the sliding window
            // holds a stale distance value from a prior odom position, not a
            // validated one, so isOutside() must reject it too. Cache as
            // global indices here (once per plan) rather than converting on
            // every isOutside() call in the graph-search hot path.
            Vec3f bbox_min, bbox_max;
            map_class_ptr_->getESDFUpdatedBbox(bbox_min, bbox_max);
            map_class_ptr_->esdfMapPosToGlobalIndex(bbox_min,
                                                    updated_bbox_min_id_);
            map_class_ptr_->esdfMapPosToGlobalIndex(bbox_max,
                                                    updated_bbox_max_id_);
        }
    }

    /// Get map data
    Tmap getMap() override { return map_; }
    /// Get resolution
    decimal_t getRes() override { return res_; }
    /// Get dimensions
    Veci<Dim> getDim() const override { return dim_; }
    /// Get origin
    Vecf<Dim> getOrigin() override { return origin_d_; }
    /// Get the min/max corners of the *live* ESDF map's currently valid region
    void getLocalMapBound(Vecf<Dim> &map_min, Vecf<Dim> &map_max) override
    {
        if constexpr (Dim == 3)
        {
            Vec3f box_min, box_max;
            map_class_ptr_->getESDFUpdatedBbox(box_min, box_max);
            map_min = box_min;
            map_max = box_max;
        }
        else
        {
            map_min = origin_d_;
            map_max = origin_d_ + dim_.template cast<decimal_t>() * res_;
        }
    }
    /// Get index of a cell
    int getIndex(const Veci<Dim> &pn) override
    {
        if constexpr (Dim == 2)
        {
            return pn(0) + dim_(0) * pn(1);
        }
        else
        {
            // pn is already an ESDF global index (it comes from floatToInt /
            // the search), so hash it directly — no float round-trip.
            return map_class_ptr_->getESDFBufferIndexFromGlobalIndex(pn);
        }
    }

    /// Check if the given cell is outside of the map in i-the dimension
    bool isOutsideXYZ(const Veci<Dim> &n, int i) override
    {
        return n(i) < 0 || n(i) >= dim_(i);
    }
    /// Check if the cell is free by index
    bool isFree(int idx, TmapValue val) override
    {
        return map_[idx] >= val;
    } // Implicit assumption that unknown is free
    /// Check if the cell is unknown by index
    ///  Query unknown status from occupancy state instead of dist value
    bool isUnknown(int idx) override
    {
        if constexpr (Dim == 3)
        {
            return map_class_ptr_->esdfMapIsUnknownByBufferIndex(idx);
        }
        else
        {
            return map_[idx] ==
                   val_unknown_; // Hard coded default (2D path unchanged)
        }
    }
    /// Check if the cell is occupied by index
    bool isOccupied(int idx, TmapValue val) override { return map_[idx] < val; }
    /// Check if the cell is outside by coordinate
    bool isOutside(const Veci<Dim> &pn) override
    {
        if constexpr (Dim == 3)
        {
            /// Take into consideration virtual ceil and floor.
            /// pn is an ESDF global index; compare its z directly against the
            /// precomputed ceil/floor z indices and use the integer
            /// inside-map test — no per-cell float conversion.
            if (pn(2) > virtual_ceiling_id_z_ || pn(2) < virtual_floor_id_z_)
                return true;
            if (!map_class_ptr_->insideESDFMap(pn))
                return true;
            // insideESDFMap() alone only bounds pn against the full sliding
            // window, not the (smaller) region the ESDF actually refreshed
            // this frame — see updateVirtualCeilingFloor(). Reject cells
            // outside that updated bbox too, since their distance values are
            // stale rather than validated.
            if ((pn.array() < updated_bbox_min_id_.array()).any() ||
                (pn.array() > updated_bbox_max_id_.array()).any())
                return true;
            return false;
        }
        return false;
    }
    /// Check if the given cell is free by coordinate
    bool isFree(const Veci<Dim> &pn, TmapValue val) override
    {
        if (isOutside(pn))
            return false;
        else
            return isFree(getIndex(pn), val);
    }
    /// Check if the given cell is occupied by coordinate
    bool isOccupied(const Veci<Dim> &pn, TmapValue val) override
    {
        if (isOutside(pn))
            return true;
        else
            return isOccupied(getIndex(pn), val);
    }
    /// Check if the given cell is unknown by coordinate
    bool isUnknown(const Veci<Dim> &pn) override
    {
        if (isOutside(pn))
            return true;
        return isUnknown(getIndex(pn));
    }

    /**
     * @brief Update map
     *
     * @param ori origin position
     * @param map array of cell values
     */
    void setMap(const Vecf<Dim> &ori) override
    {
        origin_d_ = ori;
#ifdef ORIGIN_AT_CENTER
        for (int i = 0; i < Dim; ++i)
            min_id_g_(i) =
                static_cast<int>(ori(i) / res_ + (ori(i) > 0 ? 0.5 : -0.5));
#else
        for (int i = 0; i < Dim; ++i)
            min_id_g_(i) = static_cast<int>(std::floor(ori(i) / res_));
#endif
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
        if constexpr (Dim == 3)
        {
            map_class_ptr_->esdfMapPosToGlobalIndex(pt, pn);
        }
        else
        {
            // TODO: This is not updated
            for (int i = 0; i < Dim; i++)
                pn(i) = std::round((pt(i) - origin_d_(i)) / res_);
        }
        return pn;
    }
    /// Discrete cell coordinate to float position
    Vecf<Dim> intToFloat(const Veci<Dim> &pn) override
    {
        if constexpr (Dim == 3)
        {
            Vecf<3> idx;
            map_class_ptr_->esdfMapGlobalIndexToPos(pn, idx);
            return idx;
        }
        else
        {
            return pn.template cast<decimal_t>() * res_ + origin_d_;
        }
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
    bool inline isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2,
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

    bool inline isBlocked(const Vecf<Dim> &p1, const Vecf<Dim> &p2) override
    {
        return isBlocked(p1, p2, thresh_val_);
    }

    /// Search sorted_neighbors_ (nearest first, precomputed in the ctor) for
    /// the nearest free cell to seed_pos, within nearest_free_search_radius_.
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

    /// Find where the segment (from, to) crosses the live local map's
    /// bounding box (see getLocalMapBound()), writing the crossing point
    /// to hit. Returns false if the segment never crosses the box.
    /// Slab method
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
            {
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                {
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                    {
                        if (isOccupied(getIndex(n), thresh_val_))
                            cloud.push_back(intToFloat(n));
                    }
                }
            }
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
            {
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                {
                    if (isOccupied(getIndex(n), thresh_val_))
                        cloud.push_back(intToFloat(n));
                }
            }
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
            {
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                {
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                    {
                        if (isFree(getIndex(n), thresh_val_))
                            cloud.push_back(intToFloat(n));
                    }
                }
            }
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
            {
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                {
                    if (isFree(getIndex(n), thresh_val_))
                        cloud.push_back(intToFloat(n));
                }
            }
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
            {
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                {
                    for (n(2) = 0; n(2) < dim_(2); n(2)++)
                    {
                        if (isUnknown(getIndex(n)))
                            cloud.push_back(intToFloat(n));
                    }
                }
            }
        }
        else if (Dim == 2)
        {
            for (n(0) = 0; n(0) < dim_(0); n(0)++)
            {
                for (n(1) = 0; n(1) < dim_(1); n(1)++)
                {
                    if (isUnknown(getIndex(n)))
                        cloud.push_back(intToFloat(n));
                }
            }
        }

        return cloud;
    }

    void setThreshVal(decimal_t thresh_val) override
    {
        thresh_val_ = thresh_val;
    }

    decimal_t getThreshDist() override { return thresh_val_; }

    /// Batched neighbor check (see MapUtil::freeNeighbors). When the whole
    /// 3x3x3 block around pn is inside the ESDF window and between the
    /// virtual floor/ceiling (almost always), the ring-buffer index is
    /// computed once for pn and each neighbor's index is built from
    /// per-axis terms with a compare-based wrap, so there is no modulo and
    /// no virtual call per neighbor. Offsets must be within [-1, 1];
    /// anything else falls back to the per-cell path.
    uint32_t freeNeighbors(const Veci<Dim> &pn, const Veci<Dim> *offs, int n,
                           TmapValue val, int *ids) override
    {
        if constexpr (Dim != 3)
        {
            return MapUtil<Dim, ValueT>::freeNeighbors(pn, offs, n, val, ids);
        }
        else
        {
            const Vec3i one = Vec3i::Ones();
            if (pn(2) + 1 > virtual_ceiling_id_z_ ||
                pn(2) - 1 < virtual_floor_id_z_ ||
                !map_class_ptr_->insideESDFMap(Vec3i(pn - one)) ||
                !map_class_ptr_->insideESDFMap(Vec3i(pn + one)))
                return MapUtil<Dim, ValueT>::freeNeighbors(pn, offs, n, val,
                                                           ids);
            const Vec3i half = (dim_ - one) / 2;
            // term[axis][d + 1] = contribution of local index (pn + d) on
            // that axis to the buffer hash, as in
            // SlidingMap::getHashIndexFromGlobalIndex.
            const int stride[3] = {dim_(1) * dim_(2), dim_(2), 1};
            int term[3][3];
            for (int a = 0; a < 3; ++a)
            {
                int l = pn(a) % dim_(a);
                if (l > half(a))
                    l -= dim_(a);
                else if (l < -half(a))
                    l += dim_(a);
                for (int d = -1; d <= 1; ++d)
                {
                    int ld = l + d;
                    if (ld > half(a))
                        ld -= dim_(a);
                    else if (ld < -half(a))
                        ld += dim_(a);
                    term[a][d + 1] = (ld + half(a)) * stride[a];
                }
            }
            uint32_t mask = 0;
            for (int k = 0; k < n; ++k)
            {
                const Vec3i &o = offs[k];
                if ((o.array().abs() > 1).any())
                {
                    const Vec3i q = pn + o;
                    if (isFree(q, val))
                    {
                        ids[k] = getIndex(q);
                        mask |= 1u << k;
                    }
                    continue;
                }
                const int idx =
                    term[0][o(0) + 1] + term[1][o(1) + 1] + term[2][o(2) + 1];
                if (map_[idx] >= val)
                {
                    ids[k] = idx;
                    mask |= 1u << k;
                }
            }
            return mask;
        }
    }

    /// Direct sweep of the ESDF ring buffer: no virtual calls and no modulo
    /// in the inner loop (the local index is advanced incrementally along
    /// x and wrapped once per row). Cells outside the ESDF's updated bbox or
    /// the virtual floor/ceiling are blocked, exactly as isOutside() does.
    void snapshotOccupancy(const Veci<Dim> &lo, const Veci<Dim> &hi,
                           TmapValue val, std::vector<uint8_t> &out) override
    {
        if constexpr (Dim != 3)
        {
            MapUtil<Dim, ValueT>::snapshotOccupancy(lo, hi, val, out);
        }
        else
        {
            const Veci<Dim> n = hi - lo + Veci<Dim>::Ones();
            if ((n.array() <= 0).any())
            {
                out.clear();
                return;
            }
            out.assign(static_cast<size_t>(n(0)) * n(1) * n(2), 1);
            const Vec3i half = (dim_ - Vec3i::Ones()) / 2;
            // Global index -> local index in [-half, half] (see
            // rog_map SlidingMap::globalIndexToLocalIndex).
            auto toLocal = [&](int g, int axis)
            {
                int l = g % dim_(axis);
                if (l > half(axis))
                    l -= dim_(axis);
                else if (l < -half(axis))
                    l += dim_(axis);
                return l;
            };
            const Vec3i vlo =
                lo.cwiseMax(updated_bbox_min_id_)
                    .cwiseMax(Vec3i(lo(0), lo(1), virtual_floor_id_z_));
            const Vec3i vhi =
                hi.cwiseMin(updated_bbox_max_id_)
                    .cwiseMin(Vec3i(hi(0), hi(1), virtual_ceiling_id_z_));
            const int sy = dim_(2), sx = dim_(1) * dim_(2);
            for (int z = vlo(2); z <= vhi(2); ++z)
            {
                const int lz = toLocal(z, 2) + half(2);
                for (int y = vlo(1); y <= vhi(1); ++y)
                {
                    const int ly = toLocal(y, 1) + half(1);
                    int lx = toLocal(vlo(0), 0);
                    uint8_t *row =
                        out.data() +
                        (static_cast<size_t>(z - lo(2)) * n(1) + (y - lo(1))) *
                            n(0) +
                        (vlo(0) - lo(0));
                    for (int x = vlo(0); x <= vhi(0); ++x, ++lx)
                    {
                        if (lx > half(0))
                            lx -= dim_(0);
                        const int idx = (lx + half(0)) * sx + ly * sy + lz;
                        row[x - vlo(0)] = (map_[idx] >= val) ? 0 : 1;
                    }
                }
            }
        }
    }

    /// Map entity (Raw data) -- aliases the live ROGMap ESDF buffer, which is
    /// updated in place as the sliding map moves, so no per-call resync needed.
    const Tmap &map_;

protected:
    /// Resolution
    decimal_t res_;
    /// Origin, float type
    Vecf<Dim> origin_d_ = Vecf<Dim>::Zero();
    /// Dimension, int type
    Veci<Dim> dim_;

    /// Minimum global index
    Veci<Dim> min_id_g_;
    decimal_t val_unknown_ = 0.5; // TODO: to remove
    /// val <= thresh_val_ : occupied
    /// val > thresh_val_ : free
    decimal_t thresh_val_ = 0;
    // Map class with helper functions
    std::shared_ptr<rog_map::ROGMap> map_class_ptr_;
    decimal_t virtual_ceiling_ = 4;
    decimal_t virtual_floor_ = -1;
    /// Virtual ceiling/floor as ESDF global z indices (precomputed in ctor)
    int virtual_ceiling_id_z_ = 0;
    int virtual_floor_id_z_ = 0;
    /// ESDF actually-updated bbox as global indices, refreshed each plan()
    /// by updateVirtualCeilingFloor() — see isOutside() for why this is
    /// checked separately from insideESDFMap().
    Vec3i updated_bbox_min_id_ = Vec3i::Zero();
    Vec3i updated_bbox_max_id_ = Vec3i::Zero();

    /// Search radius (in meters) for getNearestKnownFreePos(). Converted to
    /// cells in the ctor via the map resolution.
    static constexpr decimal_t nearest_free_search_radius_m_ = 3.0;
    /// Cell offsets within nearest_free_search_radius_m_, sorted nearest-first
    /// (precomputed once in the ctor, used by getNearestKnownFreePos())
    vec_Veci<Dim> sorted_neighbors_;
};

typedef ROGMapUtil<2> ROG2DMapUtil;

typedef ROGMapUtil<3> ROG3DmapUtil;

} // namespace JPS

#endif
