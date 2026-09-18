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
