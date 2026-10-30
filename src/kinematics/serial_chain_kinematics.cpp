#include "motion/kinematics/serial_chain_kinematics.hpp"
#include "motion/math/derivative_jets.hpp"
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <set>
namespace motion {
namespace {
Eigen::Isometry3d transform(const Pose &p) {
  Eigen::Isometry3d t = Eigen::Isometry3d::Identity();
  t.translation() = Eigen::Vector3d(p.position[0], p.position[1], p.position[2]);
  t.linear() = Eigen::Quaterniond(p.orientation[0], p.orientation[1], p.orientation[2], p.orientation[3])
                   .toRotationMatrix();
  return t;
}
Pose convert(const Eigen::Isometry3d &t) {
  const Eigen::Quaterniond q(t.linear());
  return {{t.translation()[0], t.translation()[1], t.translation()[2]}, {q.w(), q.x(), q.y(), q.z()}};
}
Eigen::MatrixXd matrix(const std::vector<Vector> &j) {
  Eigen::MatrixXd r(6, j[0].size());
  for (int i = 0; i < 6; ++i)
    for (std::size_t k = 0; k < j[i].size(); ++k)
      r(i, k) = j[i][k];
  return r;
}
} // namespace
SerialChainKinematics::SerialChainKinematics(std::vector<SerialJoint> j, Pose b, Pose t, SerialIKOptions o)
    : joints_(std::move(j)), base_(b), tool_(t), options_(o) {
  if (joints_.empty() || joints_.size() > 128 || !kinematic_detail::finite_pose(b) ||
      !kinematic_detail::finite_pose(t))
    throw std::invalid_argument("Invalid serial chain");
  if (o.max_iterations == 0 || o.max_iterations > 10000)
    throw std::invalid_argument("Invalid IK iteration budget");
  for (double x : {o.position_tolerance_m, o.orientation_tolerance_rad, o.orientation_length_m,
                   o.prismatic_scale_m, o.damping, o.max_normalized_step, o.singularity_threshold})
    if (!std::isfinite(x) || x <= 0)
      throw std::invalid_argument("Invalid IK options");
  std::set<std::string> names;
  for (std::size_t i = 0; i < joints_.size(); ++i) {
    auto &x = joints_[i];
    if (x.name.empty() || !names.insert(x.name).second || !kinematic_detail::finite_pose(x.origin) ||
        (i && x.parent_link != joints_[i - 1].child_link))
      throw std::invalid_argument("Invalid serial topology");
    if (x.type == SerialJointType::Fixed)
      continue;
    const double n = kinematic_detail::norm(x.axis);
    if (!std::isfinite(n) || n <= 0 || !std::isfinite(x.lower) || !std::isfinite(x.upper) ||
        x.lower >= x.upper || !std::isfinite(x.max_velocity) || x.max_velocity <= 0)
      throw std::invalid_argument("Invalid joint geometry/limits");
    for (auto &a : x.axis)
      a /= n;
    names_.push_back(x.name);
    limits_.lower.push_back(x.lower);
    limits_.upper.push_back(x.upper);
  }
  if (names_.empty() || names_.size() > 12)
    throw std::invalid_argument("Supported chain has 1..12 moving joints");
}
void SerialChainKinematics::require_q(const Vector &q) const {
  if (!kinematic_detail::finite(q, dof()))
    throw std::invalid_argument("Invalid joint dimension/value");
  for (std::size_t i = 0; i < q.size(); ++i)
    if (q[i] < limits_.lower[i] || q[i] > limits_.upper[i])
      throw std::invalid_argument("Joint outside model limits");
}
Pose SerialChainKinematics::fk(const Vector &q) const {
  require_q(q);
  auto t = transform(base_);
  std::size_t i = 0;
  for (const auto &j : joints_) {
    t = t * transform(j.origin);
    if (j.type == SerialJointType::Fixed)
      continue;
    const Eigen::Vector3d a(j.axis[0], j.axis[1], j.axis[2]);
    if (j.type == SerialJointType::Prismatic)
      t.translate(a * q[i]);
    else
      t.rotate(Eigen::AngleAxisd(q[i], a));
    ++i;
  }
  return convert(t * transform(tool_));
}
std::vector<Vector> SerialChainKinematics::jacobian(const Vector &q) const {
  require_q(q);
  auto t = transform(base_);
  std::vector<Eigen::Vector3d> axes, positions;
  std::vector<bool> linear;
  std::size_t i = 0;
  for (const auto &j : joints_) {
    t = t * transform(j.origin);
    if (j.type == SerialJointType::Fixed)
      continue;
    const Eigen::Vector3d a(j.axis[0], j.axis[1], j.axis[2]);
    axes.push_back(t.linear() * a);
    positions.push_back(t.translation());
    linear.push_back(j.type == SerialJointType::Prismatic);
    if (linear.back())
      t.translate(a * q[i]);
    else
      t.rotate(Eigen::AngleAxisd(q[i], a));
    ++i;
  }
  const Eigen::Vector3d end = (t * transform(tool_)).translation();
  std::vector<Vector> r(6, Vector(dof()));
  for (std::size_t k = 0; k < dof(); ++k) {
    const Eigen::Vector3d v = linear[k] ? axes[k] : axes[k].cross(end - positions[k]);
    for (int a = 0; a < 3; ++a) {
      r[a][k] = v[a];
      r[a + 3][k] = linear[k] ? 0 : axes[k][a];
    }
  }
  return r;
}
SingularityInfo SerialChainKinematics::singularity(const Vector &q) const {
  auto j = matrix(jacobian(q));
  j.bottomRows(3) *= options_.orientation_length_m;
  Eigen::JacobiSVD<Eigen::MatrixXd> s(j);
  const auto v = s.singularValues();
  const double ratio = v[0] > 0 ? v[v.size() - 1] / v[0] : 0;
  return {ratio, 1, ratio < options_.singularity_threshold};
}
KinematicDerivatives SerialChainKinematics::differentiate(const Vector &q, const Vector &v, const Vector &a,
                                                          const Vector &j) const {
  require_q(q);
  if (!kinematic_detail::finite(v, dof()) || !kinematic_detail::finite(a, dof()) ||
      !kinematic_detail::finite(j, dof()))
    throw std::invalid_argument("Invalid derivatives");
  using namespace differential_math;
  using D = Jet<double>;
  auto t = pose<D>(base_);
  std::size_t k = 0;
  for (const auto &joint : joints_) {
    t = multiply(t, pose<D>(joint.origin));
    if (joint.type == SerialJointType::Fixed)
      continue;
    D x({q[k], v[k], a[k], j[k]});
    auto m = identity<D>();
    if (joint.type == SerialJointType::Prismatic) {
      for (int i = 0; i < 3; ++i)
        m[i][3] = D(joint.axis[i]) * x;
    } else
      m = rotation(joint.axis, x);
    t = multiply(t, m);
    ++k;
  }
  t = multiply(t, pose<D>(tool_));
  auto w = angular(t);
  KinematicDerivatives r;
  std::array<Twist *, 3> out{&r.velocity, &r.acceleration, &r.jerk};
  for (int o = 0; o < 3; ++o)
    for (int i = 0; i < 3; ++i) {
      out[o]->data[i] = t[i][3].d[o + 1];
      out[o]->data[i + 3] = w[o][i];
    }
  kinematic_detail::require_finite(r);
  return r;
}
SerialIKResult SerialChainKinematics::ik_detailed(const Pose &goal, const Vector &seed) const {
  return ik_detailed(goal, seed, nullptr);
}
SerialIKResult SerialChainKinematics::ik_detailed(const Pose &goal, const Vector &seed,
                                                  PreparationContext *context) const {
  preparation_checkpoint(context, "ik");
  SerialIKResult r;
  if (!kinematic_detail::finite_pose(goal)) {
    r.diagnostic = "Invalid pose";
    return r;
  }
  try {
    require_q(seed);
    Vector q = seed;
    const auto target = transform(goal);
    const auto residual = [&](const Vector &x) {
      const auto actual = transform(fk(x));
      Eigen::Matrix<double, 6, 1> e;
      e.head<3>() = target.translation() - actual.translation();
      Eigen::AngleAxisd turn(target.linear() * actual.linear().transpose());
      e.tail<3>() = turn.axis() * turn.angle();
      return e;
    };
    for (std::size_t iteration = 0; iteration < options_.max_iterations; ++iteration) {
      preparation_consume(context, PreparationResource::Work, 1, "ik.iteration");
      const auto e = residual(q);
      r.iterations = iteration;
      r.position_error_m = e.head<3>().norm();
      r.orientation_error_rad = e.tail<3>().norm();
      const auto rank = singularity(q);
      r.rank_margin = rank.rank_margin;
      if (r.position_error_m <= options_.position_tolerance_m &&
          r.orientation_error_rad <= options_.orientation_tolerance_rad) {
        if (rank.singular) {
          r.status = SerialIKStatus::SingularSolution;
          r.diagnostic = "Singular IK solution";
          return r;
        }
        r.status = SerialIKStatus::Success;
        r.solution = q;
        r.diagnostic = "Local bounded IK converged";
        return r;
      }
      auto J = matrix(jacobian(q));
      Eigen::Matrix<double, 6, 1> weighted = e;
      J.bottomRows(3) *= options_.orientation_length_m;
      weighted.tail<3>() *= options_.orientation_length_m;
      Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
      const auto s = svd.singularValues();
      Eigen::VectorXd gains = s;
      for (Eigen::Index i = 0; i < s.size(); ++i)
        gains[i] = s[i] / (s[i] * s[i] + options_.damping * options_.damping);
      Eigen::VectorXd delta = svd.matrixV() * gains.asDiagonal() * svd.matrixU().transpose() * weighted;
      if (!delta.allFinite())
        break;
      const double norm = delta.norm();
      if (norm > options_.max_normalized_step)
        delta *= options_.max_normalized_step / norm;
      bool moved = false;
      for (int attempt = 0; attempt < 16; ++attempt) {
        preparation_consume(context, PreparationResource::Work, 1, "ik.line_search");
        const double scale = std::ldexp(1.0, -attempt);
        Vector next = q;
        for (std::size_t i = 0; i < q.size(); ++i)
          next[i] = std::clamp(q[i] + scale * delta[i], limits_.lower[i], limits_.upper[i]);
        auto trial = residual(next);
        trial.tail<3>() *= options_.orientation_length_m;
        if (trial.squaredNorm() < weighted.squaredNorm()) {
          q = std::move(next);
          moved = true;
          break;
        }
      }
      if (!moved)
        break;
    }
    r.status = SerialIKStatus::NoConvergence;
    r.diagnostic = "Local IK stalled or iteration budget exhausted";
  } catch (const PreparationError &) {
    throw;
  } catch (const std::exception &e) {
    r.status = SerialIKStatus::NumericalFailure;
    r.diagnostic = e.what();
  }
  return r;
}
} // namespace motion
