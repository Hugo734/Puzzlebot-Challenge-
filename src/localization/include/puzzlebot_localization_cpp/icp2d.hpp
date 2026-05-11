#pragma once

/*
 * icp2d.hpp — Header-only 2D ICP scan matcher (Eigen, from scratch)
 *
 * Algorithm (per iteration):
 *   1. Vectorised nearest-neighbour: for every source point find the
 *      closest target point using the identity
 *        ||a - b||² = ||a||² + ||b||² - 2 a·b
 *   2. Reject pairs whose distance exceeds reject_dist
 *   3. SVD best-fit rigid transform for the inlier set (A → B)
 *   4. Apply transform to source, accumulate total transform
 *   5. Repeat until convergence or max_iter
 *
 * All point clouds are Nx2 Eigen matrices (each row = one 2-D point).
 */

#include <Eigen/Dense>
#include <cmath>
#include <limits>
#include <vector>

namespace puzzlebot_localization_cpp {

struct IcpResult {
    double dx{0.0};
    double dy{0.0};
    double dtheta{0.0};
    double fitness{std::numeric_limits<double>::infinity()};
    bool   converged{false};
};

// ── Coordinate helpers ────────────────────────────────────────────────────────

// Convert a LaserScan to an Nx2 point cloud in the robot's base frame.
//
// (laser_x, laser_y, laser_yaw) is the static SE(2) offset of the LiDAR
// frame relative to the base frame (from TF).
inline Eigen::MatrixXd scan_to_points(
    const std::vector<float> & ranges,
    double angle_min,
    double angle_increment,
    double range_min,
    double range_max,
    double laser_x   = 0.0,
    double laser_y   = 0.0,
    double laser_yaw = 0.0)
{
    const double cL = std::cos(laser_yaw);
    const double sL = std::sin(laser_yaw);

    std::vector<double> xs, ys;
    xs.reserve(ranges.size());
    ys.reserve(ranges.size());

    for (std::size_t i = 0; i < ranges.size(); ++i) {
        double r = static_cast<double>(ranges[i]);
        if (!std::isfinite(r) || r < range_min || r > range_max) continue;
        double a  = angle_min + static_cast<double>(i) * angle_increment;
        double xl = r * std::cos(a);
        double yl = r * std::sin(a);
        // laser → base
        xs.push_back(laser_x + cL * xl - sL * yl);
        ys.push_back(laser_y + sL * xl + cL * yl);
    }

    if (xs.empty()) return Eigen::MatrixXd(0, 2);

    Eigen::MatrixXd out(static_cast<Eigen::Index>(xs.size()), 2);
    for (std::size_t i = 0; i < xs.size(); ++i) {
        out(static_cast<Eigen::Index>(i), 0) = xs[i];
        out(static_cast<Eigen::Index>(i), 1) = ys[i];
    }
    return out;
}

// Apply SE(2) pose (x, y, theta) to an Nx2 point cloud.
inline Eigen::MatrixXd transform_points(
    const Eigen::MatrixXd & pts,
    double x, double y, double theta)
{
    if (pts.rows() == 0) return pts;
    const double c = std::cos(theta), s = std::sin(theta);
    Eigen::Matrix2d R;
    R << c, -s,
         s,  c;
    Eigen::MatrixXd out = (R * pts.transpose()).transpose();
    out.col(0).array() += x;
    out.col(1).array() += y;
    return out;
}

// ── Core ICP ──────────────────────────────────────────────────────────────────

inline IcpResult icp(
    const Eigen::MatrixXd & source,
    const Eigen::MatrixXd & target,
    int    max_iter    = 20,
    double tolerance   = 1e-4,
    double reject_dist = 0.3,
    int    min_points  = 20)
{
    IcpResult result;
    const auto N_s = source.rows();
    const auto N_t = target.rows();

    if (N_s < min_points || N_t < min_points) return result;

    const double reject_sq = reject_dist * reject_dist;

    // Accumulate total transform
    Eigen::Matrix2d R_total = Eigen::Matrix2d::Identity();
    Eigen::Vector2d t_total = Eigen::Vector2d::Zero();

    Eigen::MatrixXd src = source;

    // Pre-compute target squared norms (reused every iteration)
    Eigen::VectorXd tgt_sq = target.rowwise().squaredNorm();  // (N_t,)

    for (int iter = 0; iter < max_iter; ++iter) {
        // ── Vectorised nearest-neighbour ──────────────────────────────
        // dist_sq(i,j) = ||src[i] - tgt[j]||²
        //              = ||src[i]||² + ||tgt[j]||² - 2 src[i]·tgt[j]
        Eigen::VectorXd  src_sq = src.rowwise().squaredNorm();   // (N_s,)
        Eigen::MatrixXd  cross  = src * target.transpose();      // (N_s, N_t)
        Eigen::MatrixXd  dist_sq =
            src_sq.replicate(1, N_t) +
            tgt_sq.transpose().replicate(N_s, 1) -
            2.0 * cross;
        dist_sq = dist_sq.cwiseMax(0.0);   // guard against floating-point negatives

        // ── Collect inliers ───────────────────────────────────────────
        std::vector<Eigen::Index> i_src, i_tgt;
        i_src.reserve(N_s);
        i_tgt.reserve(N_s);
        double sum_d = 0.0;

        for (Eigen::Index i = 0; i < N_s; ++i) {
            Eigen::Index j;
            double d_sq = dist_sq.row(i).minCoeff(&j);
            if (d_sq < reject_sq) {
                i_src.push_back(i);
                i_tgt.push_back(j);
                sum_d += std::sqrt(d_sq);
            }
        }

        if (static_cast<int>(i_src.size()) < min_points) break;
        result.fitness = sum_d / static_cast<double>(i_src.size());

        // ── Build matched subsets ────────────────────────────────────
        const auto K = static_cast<Eigen::Index>(i_src.size());
        Eigen::MatrixXd A(K, 2), B(K, 2);
        for (Eigen::Index k = 0; k < K; ++k) {
            A.row(k) = src.row(i_src[static_cast<std::size_t>(k)]);
            B.row(k) = target.row(i_tgt[static_cast<std::size_t>(k)]);
        }

        // ── SVD best-fit transform  A → B ────────────────────────────
        const Eigen::Vector2d cA = A.colwise().mean();
        const Eigen::Vector2d cB = B.colwise().mean();

        const Eigen::Matrix2d H =
            (A.rowwise() - cA.transpose()).transpose() *
            (B.rowwise() - cB.transpose());

        Eigen::JacobiSVD<Eigen::Matrix2d> svd(
            H, Eigen::ComputeFullU | Eigen::ComputeFullV);

        Eigen::Matrix2d R = svd.matrixV() * svd.matrixU().transpose();

        // Ensure proper rotation (det = +1, not a reflection)
        if (R.determinant() < 0.0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1.0;
            R = V * svd.matrixU().transpose();
        }

        const Eigen::Vector2d t = cB - R * cA;

        // ── Apply to source, accumulate ──────────────────────────────
        src = (R * src.transpose()).transpose();
        src.col(0).array() += t(0);
        src.col(1).array() += t(1);

        t_total = R * t_total + t;
        R_total = R * R_total;

        // ── Convergence check ────────────────────────────────────────
        const double dt     = t.norm();
        const double dangle = std::abs(std::atan2(R(1, 0), R(0, 0)));
        if (dt < tolerance && dangle < tolerance) {
            result.converged = true;
            break;
        }
    }

    result.dx     = t_total(0);
    result.dy     = t_total(1);
    result.dtheta = std::atan2(R_total(1, 0), R_total(0, 0));
    return result;
}

}  // namespace puzzlebot_localization_cpp
