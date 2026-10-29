#include "motion/kinematics/inverse_differential.hpp"
#include <Eigen/Dense>
namespace motion {
InverseDifferentialResult inverse_differential(const IDifferentialKinematics &model, const Vector &q,
                                               const Twist &velocity, const Twist &acceleration) {
  InverseDifferentialResult result;
  try {
    if (!kinematic_detail::finite(q, model.dof()))
      throw std::invalid_argument("Invalid inverse differential state");
    for (auto t : {velocity, acceleration})
      for (double x : t.data)
        if (!std::isfinite(x))
          throw std::invalid_argument("Nonfinite spatial derivative");
    const auto raw = model.jacobian(q);
    if (raw.size() != 6)
      throw std::invalid_argument("Invalid Jacobian rows");
    Eigen::MatrixXd J(6, q.size());
    for (int r = 0; r < 6; ++r) {
      if (!kinematic_detail::finite(raw[r], q.size()))
        throw std::invalid_argument("Invalid Jacobian columns");
      for (std::size_t c = 0; c < q.size(); ++c)
        J(r, c) = raw[r][c];
    }
    Eigen::JacobiSVD<Eigen::MatrixXd> svd(J, Eigen::ComputeThinU | Eigen::ComputeThinV);
    const auto singular = svd.singularValues();
    result.rank_margin = singular[0] > 0 ? singular[singular.size() - 1] / singular[0] : 0;
    if (result.rank_margin <= 1e-8 || model.singularity(q).singular)
      throw std::domain_error("Inverse differential rank loss");
    svd.setThreshold(1e-10);
    Eigen::Matrix<double, 6, 1> v, a;
    for (int i = 0; i < 6; ++i) {
      v[i] = velocity.data[i];
      a[i] = acceleration.data[i];
    }
    const Eigen::VectorXd dq = svd.solve(v);
    JointTarget target{q, Vector(q.size()), Vector(q.size())};
    for (std::size_t i = 0; i < q.size(); ++i)
      target.dq[i] = dq[i];
    const auto bias = model.differentiate(q, target.dq, Vector(q.size()), Vector(q.size())).acceleration;
    for (int i = 0; i < 6; ++i)
      a[i] -= bias.data[i];
    const Eigen::VectorXd ddq = svd.solve(a);
    result.velocity_residual = (J * dq - v).norm();
    result.acceleration_residual = (J * ddq - a).norm();
    if (!dq.allFinite() || !ddq.allFinite() || result.velocity_residual > 1e-8 ||
        result.acceleration_residual > 1e-8)
      throw std::domain_error("Spatial derivative not representable");
    for (std::size_t i = 0; i < q.size(); ++i)
      target.ddq[i] = ddq[i];
    result.boundary = std::move(target);
    result.diagnostic = "SVD spatial residuals including Jdot*dq admitted";
  } catch (const std::exception &e) {
    result.diagnostic = e.what();
  }
  return result;
}
} // namespace motion
