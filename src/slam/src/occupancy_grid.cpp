#include "slam/occupancy_grid.hpp"
#include <cmath>
#include <algorithm>
#include <limits>

namespace slam {

OccupancyGrid::OccupancyGrid(int width, int height, double resolution,
                             double l_occ, double l_free, double l_min, double l_max,
                             double display_l_occ, double display_l_free,
                             double occupied_stop)
  : w_(width), h_(height), res_(resolution),
    l_occ_(l_occ), l_free_(l_free), l_min_(l_min), l_max_(l_max),
    display_l_occ_(display_l_occ), display_l_free_(display_l_free),
    occupied_stop_(occupied_stop) {
  origin_x_ = -(w_ * res_) / 2.0;
  origin_y_ = -(h_ * res_) / 2.0;
  log_.assign(static_cast<size_t>(w_) * h_, 0.0f);
  lf_.assign(log_.size(), 0.0f);
}

void OccupancyGrid::reset() {
  std::fill(log_.begin(), log_.end(), 0.0f);
  lf_dirty_ = true;
}

std::unique_ptr<OccupancyGrid> OccupancyGrid::cloneEmpty() const {
  return std::make_unique<OccupancyGrid>(w_, h_, res_, l_occ_, l_free_,
                                         l_min_, l_max_, display_l_occ_,
                                         display_l_free_, occupied_stop_);
}

void OccupancyGrid::adoptLogFrom(OccupancyGrid& other) {
  if (other.log_.size() != log_.size()) return;  // geometry mismatch guard
  log_ = std::move(other.log_);
  lf_dirty_ = true;
}

int OccupancyGrid::countOccupied() const {
  int n = 0;
  for (float v : log_) if (v > display_l_occ_) ++n;
  return n;
}

// Bresenham/DDA free-ray from (ox,oy) cell to (ex,ey) cell, voting free
// on traversed cells (excluding endpoint), stopping at strong walls.
void OccupancyGrid::integrateCloud(const Pose2& robot, const Cloud& world_ep,
                                   double laser_x, double laser_y) {
  if (world_ep.size() == 0) return;
  const double inv = 1.0 / res_;
  const double c = std::cos(robot.th), s = std::sin(robot.th);
  const double ox = robot.x + c * laser_x - s * laser_y;
  const double oy = robot.y + s * laser_x + c * laser_y;
  const int ox_c = static_cast<int>(std::floor((ox - origin_x_) * inv));
  const int oy_c = static_cast<int>(std::floor((oy - origin_y_) * inv));

  for (size_t i = 0; i < world_ep.size(); ++i) {
    const int ex_c = static_cast<int>(std::floor((world_ep.x[i] - origin_x_) * inv));
    const int ey_c = static_cast<int>(std::floor((world_ep.y[i] - origin_y_) * inv));

    // DDA from origin to endpoint, integer Bresenham.
    int x0 = ox_c, y0 = oy_c, x1 = ex_c, y1 = ey_c;
    int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
    int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
    int err = dx - dy;
    int cx = x0, cy = y0;
    while (true) {
      if (cx == x1 && cy == y1) break;  // stop before endpoint
      if (cx >= 0 && cx < w_ && cy >= 0 && cy < h_) {
        size_t idx = static_cast<size_t>(cy) * w_ + cx;
        // Stop the free-ray at established walls (don't erase them).
        if (log_[idx] > occupied_stop_) break;
        log_[idx] = std::max(l_min_, static_cast<double>(log_[idx]) + l_free_);
      }
      int e2 = 2 * err;
      if (e2 > -dy) { err -= dy; cx += sx; }
      if (e2 <  dx) { err += dx; cy += sy; }
    }
    // Occupied endpoint.
    if (ex_c >= 0 && ex_c < w_ && ey_c >= 0 && ey_c < h_) {
      size_t idx = static_cast<size_t>(ey_c) * w_ + ex_c;
      log_[idx] = std::min(l_max_, static_cast<double>(log_[idx]) + l_occ_);
    }
  }
  lf_dirty_ = true;
}

void OccupancyGrid::occupiedPoints(double rx, double ry, double radius,
                                   std::vector<float>& ox, std::vector<float>& oy,
                                   double threshold) const {
  if (threshold < 0.0) threshold = display_l_occ_;
  const double r2 = radius * radius;
  ox.clear(); oy.clear();
  // Scan only the cell window covering the query disc, not the whole grid.
  const double inv = 1.0 / res_;
  int x0 = std::max(0, static_cast<int>(std::floor((rx - radius - origin_x_) * inv)));
  int x1 = std::min(w_ - 1, static_cast<int>(std::floor((rx + radius - origin_x_) * inv)));
  int y0 = std::max(0, static_cast<int>(std::floor((ry - radius - origin_y_) * inv)));
  int y1 = std::min(h_ - 1, static_cast<int>(std::floor((ry + radius - origin_y_) * inv)));
  for (int yy = y0; yy <= y1; ++yy) {
    for (int xx = x0; xx <= x1; ++xx) {
      size_t idx = static_cast<size_t>(yy) * w_ + xx;
      if (log_[idx] > threshold) {
        double wx = origin_x_ + (xx + 0.5) * res_;
        double wy = origin_y_ + (yy + 0.5) * res_;
        double d2 = (wx - rx) * (wx - rx) + (wy - ry) * (wy - ry);
        if (d2 <= r2) { ox.push_back(static_cast<float>(wx)); oy.push_back(static_cast<float>(wy)); }
      }
    }
  }
}

// ── Felzenszwalb 1D squared-distance transform ──────────────────────
static void edt1d(const std::vector<float>& f, std::vector<float>& d, int n) {
  std::vector<int> v(n);
  std::vector<float> z(n + 1);
  int k = 0;
  v[0] = 0;
  z[0] = -std::numeric_limits<float>::infinity();
  z[1] =  std::numeric_limits<float>::infinity();
  for (int q = 1; q < n; ++q) {
    float s;
    while (true) {
      s = ((f[q] + q * q) - (f[v[k]] + v[k] * v[k])) / (2.0f * (q - v[k]));
      if (s <= z[k]) --k; else break;
    }
    ++k;
    v[k] = q;
    z[k] = s;
    z[k + 1] = std::numeric_limits<float>::infinity();
  }
  k = 0;
  for (int q = 0; q < n; ++q) {
    while (z[k + 1] < q) ++k;
    float dq = q - v[k];
    d[q] = dq * dq + f[v[k]];
  }
}

void OccupancyGrid::buildLikelihoodField(double sigma) {
  const float INF = 1e20f;
  // Empty-map guard: with NO occupied cells the separable EDT computes
  // INF − INF = NaN, which would poison the AMCL sensor model.  Return
  // an all-zero field (every cell maximally far from any wall).
  int n_occ = 0;
  for (float v : log_) if (v > display_l_occ_) { ++n_occ; break; }
  if (n_occ == 0) {
    lf_.assign(log_.size(), 0.0f);
    lf_sigma_ = sigma; lf_dirty_ = false;
    return;
  }

  std::vector<float> dist(log_.size());
  // Seed: 0 at occupied cells, INF elsewhere (squared-distance domain in cells).
  for (size_t i = 0; i < log_.size(); ++i)
    dist[i] = (log_[i] > display_l_occ_) ? 0.0f : INF;

  // Columns then rows (separable EDT).
  std::vector<float> col(h_), cold(h_);
  for (int x = 0; x < w_; ++x) {
    for (int y = 0; y < h_; ++y) col[y] = dist[static_cast<size_t>(y) * w_ + x];
    edt1d(col, cold, h_);
    for (int y = 0; y < h_; ++y) dist[static_cast<size_t>(y) * w_ + x] = cold[y];
  }
  std::vector<float> row(w_), rowd(w_);
  for (int y = 0; y < h_; ++y) {
    for (int x = 0; x < w_; ++x) row[x] = dist[static_cast<size_t>(y) * w_ + x];
    edt1d(row, rowd, w_);
    for (int x = 0; x < w_; ++x) dist[static_cast<size_t>(y) * w_ + x] = rowd[x];
  }

  // Convert squared-cell-distance → gaussian likelihood in metres.
  const double res2 = res_ * res_;
  const double denom = 2.0 * sigma * sigma;
  lf_.resize(log_.size());
  for (size_t i = 0; i < log_.size(); ++i) {
    double d2_m = dist[i] * res2;                 // metres^2
    lf_[i] = static_cast<float>(std::exp(-d2_m / denom));
  }
  lf_sigma_ = sigma;
  lf_dirty_ = false;
}

const std::vector<float>& OccupancyGrid::likelihoodField(double sigma) {
  if (lf_dirty_ || std::abs(lf_sigma_ - sigma) > 1e-9)
    buildLikelihoodField(sigma);
  return lf_;
}

void OccupancyGrid::toRosData(std::vector<int8_t>& out) const {
  out.resize(log_.size());
  for (size_t i = 0; i < log_.size(); ++i) {
    // Use >= / <= (not strict >/<): log-odds accumulate in exact l_occ steps
    // (e.g. 0.50), so a freshly-confirmed wall lands EXACTLY on display_l_occ_
    // (2.5 = 5 hits).  With strict '>' that cell published as -1 (unknown/grey)
    // instead of 100 (occupied) — walls got fog/holes that poisoned the A* grid.
    if (log_[i] >= display_l_occ_)        out[i] = 100;
    else if (log_[i] <= display_l_free_)  out[i] = 0;
    else                                  out[i] = -1;
  }
}

}  // namespace slam
