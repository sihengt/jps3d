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
    // JPSPlanner/GraphSearch keep their own copy of the threshold for the
    // per-cell isFree/isOccupied queries in the search; without this the
    // search ran with the default 0, i.e. ESDF-occupied cells counted free.
    planner_->setThreshVal(thresh_val);
}

void Jps3dFrontend::updateMap() { planner_->updateMap(); }

void Jps3dFrontend::setFastMode(bool on) { planner_->setFastMode(on); }
void Jps3dFrontend::setMaxExpand(int n) { planner_->setMaxExpand(n); }
void Jps3dFrontend::setSearchBoxWidening(double margin_m)
{
    widen_margin_m_ = margin_m;
}

bool Jps3dFrontend::planPath(const Vec3f &start, const Vec3f &goal,
                             double eps, bool use_jps, vec_Vec3f &out_path)
{
    last_attempts_ = 0;
    last_plan_ms_ = 0.0;
    bool ok = false;
    if (widen_margin_m_ > 0.0 && planner_->fastMode())
    {
        const Vec3i s = map_util_->floatToInt(start);
        const Vec3i g = map_util_->floatToInt(goal);
        const Vec3i lo = s.cwiseMin(g), hi = s.cwiseMax(g);
        double margin = widen_margin_m_;
        for (;;)
        {
            const int mc = static_cast<int>(
                std::ceil(margin / std::max(map_util_->getRes(), 1e-9)));
            const bool full = planner_->setSearchBox(
                lo - Vec3i::Constant(mc), hi + Vec3i::Constant(mc));
            ok = planner_->plan(start, goal, eps, use_jps);
            last_attempts_++;
            last_plan_ms_ += planner_->lastTimings().total_ms;
            // Stop unless this was an exhaustive failure inside a box that
            // did not yet cover the whole map.
            if (ok || full || planner_->status() != -1)
                break;
            margin *= 2.0;
        }
        planner_->clearSearchBox();
    }
    else
    {
        ok = planner_->plan(start, goal, eps, use_jps);
        last_attempts_ = 1;
        last_plan_ms_ = planner_->lastTimings().total_ms;
    }
    if (!ok)
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
