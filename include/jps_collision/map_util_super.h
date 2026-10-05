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
#include <thread>
#include <vector>

namespace JPS
{
/**
 * @brief MapUtil implementation for collision checking with rog_map::ROGMap
 * @param Dim is workspace dimension
 * @param ValueT is the type of a single map cell's value, forwarded to
 * MapUtil<Dim, ValueT>. Defaults to double.
 */
template <int Dim, typename ValueT = double>
class ROGMapUtil final : public MapUtil<Dim, ValueT>
{
    // rog_map is a 3D map: the 2D branches below only exist so the template
    // body parses, and index out of bounds if ever used. Fail at compile time.
    static_assert(Dim == 3, "ROGMapUtil only supports Dim == 3");

public:
    using TmapValue = typename MapUtil<Dim, ValueT>::TmapValue;
    using Tmap = typename MapUtil<Dim, ValueT>::Tmap;

    // In 3D, decides between ESDF / Inflation
    //
    // Esdf       distance >= thresh_val_, read from the ESDF ring buffer.
    //            The ESDF is seeded only from prob-map OCCUPIED cells and is
    //            not inflated, so its free set is larger than the inflation
    //            map's by (inflation_step * inflation_resolution -
    //            thresh_val_). To fully incorporate, TODO to make corridor
    //            generator use the same resolution as ESDF.
    //
    // Inflation  !isOccupiedInflate(), same metric corridor generator certifies with, so a
    //            guide path is free exactly when its consumer agrees.
    //
    // The search lattice is the ESDF grid either way; only the predicate
    // changes. Under Inflation the lattice is finer than the inflation grid
    // (0.1 m vs 0.2 m), which costs nothing and keeps the index/sliding logic
    // that Esdf already relies on.
    enum class Authority
    {
        Esdf,
        Inflation
    };

    void setAuthority(Authority a) { authority_ = a; }
    Authority getAuthority() const { return authority_; }

    /**
     * @brief Construct a new ROGMapUtil object, sets up ROGMap dim_ to address
     * ROGMap padding, and precomputes cell offsets within nearest_free_search_radius_m_.
     * 
     * @param map_struct_ptr 
     */
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

        // Computes ROGMap dim_
        // ROGMap pads its map and ensures that there's always a clear middle (config.hpp:398-421)
        {
            const Vecf<Dim> half_d = map_class_ptr_->getLocalMapSize() / 2.0;
            for (int i = 0; i < Dim; ++i)
                dim_(i) =
                    2 * (static_cast<int>(half_d(i) / res_) + 1) + 1;

            // multiplies per axis dimensions into a total cell count and asserts
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
        // sort by distance
        std::sort(sorted_neighbors_.begin(), sorted_neighbors_.end(),
                  [](const Veci<Dim> &a, const Veci<Dim> &b)
                  { return a.squaredNorm() < b.squaredNorm(); });

        // Without this the cached ceiling/floor z indices are 0 and isOutside()
        // rejects every cell but z = 0 until the caller refreshes them.
        updateVirtualCeilingFloor();
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

            // posToGlobalIndex converts each axis independently to a global
            // index, which is an absolute lattice coordinate independent of
            // sliding, so only z matters here.
            Vec3i ceil_id, floor_id;
            map_class_ptr_->esdfMapPosToGlobalIndex(
                Vec3f(0, 0, virtual_ceiling_), ceil_id);
            map_class_ptr_->esdfMapPosToGlobalIndex(Vec3f(0, 0, virtual_floor_),
                                                    floor_id);
            virtual_ceiling_id_z_ = ceil_id.z();
            virtual_floor_id_z_ = floor_id.z();

            // caches ESDF bbox current id for bounds checks
            // used for other things too, keep alive for now.
            // todo: validate if we want to persist this.
            Vec3f bbox_min, bbox_max;
            map_class_ptr_->getESDFUpdatedBbox(bbox_min, bbox_max);
            map_class_ptr_->esdfMapPosToGlobalIndex(bbox_min,
                                                    updated_bbox_min_id_);
            map_class_ptr_->esdfMapPosToGlobalIndex(bbox_max,
                                                    updated_bbox_max_id_);
        }
    }

    /// Cached virtual ceiling/floor z-indices and updated-bbox global indices
    /// from the last updateVirtualCeilingFloor() call. Exposed for tests that
    /// need to observe what does/doesn't change across a map slide.
    int getVirtualCeilingIdZ() const { return virtual_ceiling_id_z_; }
    int getVirtualFloorIdZ() const { return virtual_floor_id_z_; }
    Vec3i getUpdatedBboxMinId() const { return updated_bbox_min_id_; }
    Vec3i getUpdatedBboxMaxId() const { return updated_bbox_max_id_; }

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
        if constexpr (Dim == 3)
        {
            if (authority_ == Authority::Inflation)
                return !map_class_ptr_->isOccupiedInflate(posFromIndex(idx));
        }
        return map_[idx] >= val;
    } // Implicit assumption that unknown is free
    /// Check if the cell is unknown by index
    ///  Query unknown status from occupancy state instead of dist value
    bool isUnknown(int idx) override
    {
        if constexpr (Dim == 3)
        {
            if (authority_ == Authority::Inflation)
                return isUnknownAt(posFromIndex(idx));
            return map_class_ptr_->esdfMapIsUnknownByBufferIndex(idx);
        }
        else
        {
            return map_[idx] ==
                   val_unknown_; // Hard coded default (2D path unchanged)
        }
    }
    /// Check if the cell is occupied by index
    bool isOccupied(int idx, TmapValue val) override
    {
        if constexpr (Dim == 3)
        {
            if (authority_ == Authority::Inflation)
                return map_class_ptr_->isOccupiedInflate(posFromIndex(idx));
        }
        return map_[idx] < val;
    }
    /// Check if the cell is outside by coordinate
    bool isOutside(const Veci<Dim> &pn) override
    {
        if constexpr (Dim == 3)
        {
            // pn is an ESDF global index, check its z directly.
            // outside of virtual ceiling or floor
            if (pn(2) > virtual_ceiling_id_z_ || pn(2) < virtual_floor_id_z_)
                return true;
            // not inside ESDF map
            if (!map_class_ptr_->insideESDFMap(pn))
                return true;

            // Outside the ESDF's updated bbox the distance values are not
            // recomputed, so treat them as blocked. snapshotOccupancy() and
            // the snapshot box (getLocalMapBound()) use the same bbox, which
            // keeps fast and live mode in agreement.
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
        if constexpr (Dim == 3)
        {
            if (authority_ == Authority::Inflation)
                return !map_class_ptr_->isOccupiedInflate(posFromCell(pn));
        }
        return isFree(getIndex(pn), val);
    }
    /// Check if the given cell is occupied by coordinate
    bool isOccupied(const Veci<Dim> &pn, TmapValue val) override
    {
        if (isOutside(pn))
            return true;
        if constexpr (Dim == 3)
        {
            if (authority_ == Authority::Inflation)
                return map_class_ptr_->isOccupiedInflate(posFromCell(pn));
        }
        return isOccupied(getIndex(pn), val);
    }
    /// Check if the given cell is unknown by coordinate
    bool isUnknown(const Veci<Dim> &pn) override
    {
        if (isOutside(pn))
            return true;
        if constexpr (Dim == 3)
        {
            if (authority_ == Authority::Inflation)
                return isUnknownAt(posFromCell(pn));
        }
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

    /**
     * @brief Batched neighbor check that returns a mask corresponding to each neighbor.
     * Only usable when:
     * 1) The entire 3x3x3 block around pn is inside the ESDF window 
     * 2) 3x3x3 block is within the virtual floor / ceiling.
     * 3) Offsets must be within [-1, 1] strictly. 
     * 
     * Ring-buffer index is computed once for pn (global point).
     * Each neighbor's index offset / axis is built per-axis.
     * 
     * @param pn map coordinates (integer) corresponding to global point
     * @param offs precomputed grid offsets
     * @param n number of neighbors
     * @param val threshold for free/occupied
     * @param ids[out] ids that contain free
     * @return uint32_t 
     */
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
            // The ring-buffer shortcut below reads the ESDF directly, so it
            // does not apply when the inflation map decides free/occupied.
            if (authority_ == Authority::Inflation ||
                pn(2) + 1 > virtual_ceiling_id_z_ ||
                pn(2) - 1 < virtual_floor_id_z_ ||
                !map_class_ptr_->insideESDFMap(Vec3i(pn - one)) ||
                !map_class_ptr_->insideESDFMap(Vec3i(pn + one)))
                return MapUtil<Dim, ValueT>::freeNeighbors(pn, offs, n, val,
                                                           ids);
            const Vec3i half = (dim_ - one) / 2;
                
            // stride computes how much of the dimensions needs to be hopped over when flattening
            const int stride[3] = {dim_(1) * dim_(2), dim_(2), 1};
            // initializes lookup table for indices
            int term[3][3];

            // for each axis...
            for (int a = 0; a < 3; ++a)
            {
                // center l because ROG map centers its window at 0
                int l = pn(a) % dim_(a);
                if (l > half(a))
                    l -= dim_(a);
                else if (l < -half(a))
                    l += dim_(a);
                
                // offset
                for (int d = -1; d <= 1; ++d)
                {
                    int ld = l + d;
                    // centering for the same reason
                    if (ld > half(a))
                        ld -= dim_(a);
                    else if (ld < -half(a))
                        ld += dim_(a);
                    // inserts into term[axis][offset] the flattened index
                    term[a][d + 1] = (ld + half(a)) * stride[a];
                }
            }
            // mask corresponds to the occupancy read for all neighbors
            uint32_t mask = 0;
            for (int k = 0; k < n; ++k)
            {
                const Vec3i &o = offs[k];
                // this check skips batching for offsets that are > 1 (doesn't work)
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

    /// Worker threads for snapshotOccupancy() (<= 1: sweep on the calling
    /// thread). The sweep is memory-bound; 4 threads roughly halve it.
    void setSnapshotThreads(int n) { snapshot_threads_ = n; }

    /// Direct sweep of the ESDF ring buffer in its own memory order: the
    /// buffer is z-fastest, and so is the 3D snapshot layout (see
    /// MapUtil::snapshotOccupancy), so each (x, y) column is one contiguous
    /// read and one contiguous write, with no virtual calls or modulo per
    /// cell. Cells outside the ESDF's updated bbox or the virtual
    /// floor/ceiling are blocked, exactly as isOutside() does.
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
            const size_t total = static_cast<size_t>(n(0)) * n(1) * n(2);
            const Vec3i vlo = lo.cwiseMax(updated_bbox_min_id_)
                                  .cwiseMax(Vec3i(lo(0), lo(1), virtual_floor_id_z_));
            const Vec3i vhi = hi.cwiseMin(updated_bbox_max_id_)
                                  .cwiseMin(Vec3i(hi(0), hi(1), virtual_ceiling_id_z_));
            // Every cell is written below when the valid box is the whole
            // box (the usual case), so the 1-fill is only needed otherwise.
            if (out.size() != total || vlo != lo || vhi != hi)
                out.assign(total, 1);
            if ((vhi.array() < vlo.array()).any())
                return;
            // Under Inflation the ring-buffer sweep does not apply:
            if (authority_ == Authority::Inflation)
            {
                const int ny = n(1), nzs = n(2);
                for (int x = vlo(0); x <= vhi(0); ++x)
                {
                    for (int y = vlo(1); y <= vhi(1); ++y)
                    {
                        Vecf<3> pos;
                        map_class_ptr_->esdfMapGlobalIndexToPos(
                            Vec3i(x, y, vlo(2)), pos);
                        uint8_t *col =
                            out.data() +
                            (static_cast<size_t>(x - lo(0)) * ny + (y - lo(1))) *
                                nzs +
                            (vlo(2) - lo(2));
                        Vec3i prev_inf_id = Vec3i::Constant(
                            std::numeric_limits<int>::min());
                        uint8_t prev_val = 0;
                        for (int z = vlo(2); z <= vhi(2); ++z, pos.z() += res_)
                        {
                            Vec3i inf_id;
                            map_class_ptr_->infMapPosToGlobalIndex(pos, inf_id);
                            if (inf_id != prev_inf_id)
                            {
                                prev_val =
                                    map_class_ptr_->isOccupiedInflate(pos) ? 1
                                                                           : 0;
                                prev_inf_id = inf_id;
                            }
                            col[z - vlo(2)] = prev_val;
                        }
                    }
                }
                return;
            }
            const Vec3i half = (dim_ - Vec3i::Ones()) / 2;
            // Global index -> ring-buffer coordinate in [0, dim)
            // (rog_map SlidingMap::globalIndexToLocalIndex, shifted by half)
            auto toBuf = [&](int g, int axis)
            {
                int l = g % dim_(axis);
                if (l > half(axis))
                    l -= dim_(axis);
                else if (l < -half(axis))
                    l += dim_(axis);
                return l + half(axis);
            };
            // Plain locals, captured by value: the uint8_t stores below may
            // alias anything, so members / by-reference captures would be
            // reloaded after every store and block vectorization.
            const int dx = dim_(0), dy = dim_(1), dz = dim_(2);
            const int sy = dz, sx = dy * dz;
            const int ny = n(1), nzs = n(2);
            const int x0 = vlo(0), y0 = vlo(1), y1 = vhi(1);
            const int lx0 = toBuf(x0, 0), ly0 = toBuf(y0, 1),
                      lz0 = toBuf(vlo(2), 2);
            const int nz = vhi(2) - vlo(2) + 1;
            const size_t off0 = static_cast<size_t>(x0 - lo(0)) * ny * nzs +
                                static_cast<size_t>(y0 - lo(1)) * nzs +
                                (vlo(2) - lo(2));
            const TmapValue th = val;
            const TmapValue *const buf = map_.data();
            uint8_t *const dst = out.data();
            auto sweep = [=](int xb, int xe)
            {
                int lx = (lx0 + xb) % dx;
                for (int xi = xb; xi < xe; ++xi, ++lx)
                {
                    if (lx >= dx)
                        lx -= dx;
                    int ly = ly0;
                    for (int y = y0; y <= y1; ++y, ++ly)
                    {
                        if (ly >= dy)
                            ly -= dy;
                        uint8_t *__restrict col =
                            dst + off0 +
                            (static_cast<size_t>(xi) * ny + (y - y0)) * nzs;
                        const TmapValue *__restrict src =
                            buf + static_cast<size_t>(lx) * sx +
                            static_cast<size_t>(ly) * sy;
                        // The z column wraps at most once in the ring buffer
                        int lz = lz0, k = 0;
                        while (k < nz)
                        {
                            const int run = std::min(nz - k, dz - lz);
                            for (int j = 0; j < run; ++j)
                                col[k + j] = src[lz + j] >= th ? 0 : 1;
                            k += run;
                            lz = 0;
                        }
                    }
                }
            };
            const int nx = vhi(0) - vlo(0) + 1;
            const int threads = std::min(snapshot_threads_, nx);
            if (threads <= 1)
            {
                sweep(0, nx);
                return;
            }
            std::vector<std::thread> pool;
            pool.reserve(threads - 1);
            const int chunk = (nx + threads - 1) / threads;
            for (int t = 1; t < threads; ++t)
            {
                const int b = t * chunk, e = std::min(nx, b + chunk);
                if (b < e)
                    pool.emplace_back(sweep, b, e);
            }
            sweep(0, std::min(nx, chunk));
            for (auto &w : pool)
                w.join();
        }
    }

    // reference to ESDF map buffer within ROGMap, reads directly from buffer.
    const Tmap &map_;

protected:
    /// ESDF ring-buffer index -> world position of that cell's centre.
    Vecf<3> posFromIndex(int idx) const
    {
        Vecf<3> pos;
        map_class_ptr_->getESDFPosFromHashIndex(idx, pos);
        return pos;
    }

    /// ESDF global cell index -> world position of that cell's centre.
    Vecf<3> posFromCell(const Veci<3> &pn) const
    {
        Vecf<3> pos;
        map_class_ptr_->esdfMapGlobalIndexToPos(pn, pos);
        return pos;
    }

    // Has this cell been observed? Current only consumer is getNearestKnownFreePos()
    // which wants known-free start/goal seeds.
    bool isUnknownAt(const Vecf<3> &pos) const
    {
        return map_class_ptr_->getGridType(pos) == rog_map::GridType::UNKNOWN;
    }

    /// Which representation decides free/occupied -- see Authority.
    Authority authority_ = Authority::Esdf;
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
    /// Virtual ceiling/floor as ESDF global z indices; set in the ctor and
    /// refreshed by updateVirtualCeilingFloor()
    int virtual_ceiling_id_z_ = 0;
    int virtual_floor_id_z_ = 0;
    /// ESDF actually-updated bbox as global indices, refreshed each plan()
    /// by updateVirtualCeilingFloor() — see isOutside() for why this is
    /// checked separately from insideESDFMap().
    Vec3i updated_bbox_min_id_ = Vec3i::Zero();
    Vec3i updated_bbox_max_id_ = Vec3i::Zero();
    /// See setSnapshotThreads()
    int snapshot_threads_ = 1;

    /// Search radius (in meters) for getNearestKnownFreePos(). Converted to
    /// cells in the ctor via the map resolution.
    static constexpr decimal_t nearest_free_search_radius_m_ = 3.0;
    /// Cell offsets within nearest_free_search_radius_m_, sorted nearest-first
    /// (precomputed once in the ctor, used by getNearestKnownFreePos())
    vec_Veci<Dim> sorted_neighbors_;
};

typedef ROGMapUtil<3> ROG3DmapUtil;

} // namespace JPS

#endif
