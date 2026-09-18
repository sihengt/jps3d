#include <jps3d/jps3d_frontend.hpp>

#include <algorithm>
#include <cmath>

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

int Jps3dFrontend::status() const { return planner_->status(); }
