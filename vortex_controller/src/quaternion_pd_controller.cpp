#include "vortex_controller/quaternion_pd_controller.h"


// Defines local variables in object from class.
QuaternionPdController::QuaternionPdController(double a, double b, double c, double W, double B,
                                               const Eigen::Vector3d &r_G, const Eigen::Vector3d &r_B)
: m_r_G(r_G), m_r_B(r_B), m_W(W), m_B(B)
{
  setGains(a, b, c);
}

// Sets gains and makes gain matrices.
void QuaternionPdController::setGains(double a, double b, double c)
{
  m_c   = c;
  m_K_d = a * Eigen::MatrixXd::Identity(6, 6);
  m_K_x = b * Eigen::MatrixXd::Identity(3, 3);
}

// Gives restoring force vector. Aka. Force vector needed to return to equilibrium. g(q)
Eigen::Vector6d QuaternionPdController::getRestoring(const Eigen::Quaterniond &q)
{
  Eigen::Matrix3d R = q.toRotationMatrix();
  return restoringForceVector(R);
}

// Gives the control vector. Aka. The vector to obtain wanted position. -K_d*v -K_p(q)*z
Eigen::Vector6d QuaternionPdController::getFeedback(const Eigen::Vector3d    &x,
                                                    const Eigen::Quaterniond &q,
                                                    const Eigen::Vector6d    &nu,
                                                    const Eigen::Vector3d    &x_d,
                                                    const Eigen::Quaterniond &q_d)
{
  Eigen::Matrix3d R   = q.toRotationMatrix();
  Eigen::Matrix6d K_p = proportionalGainMatrix(R);
  Eigen::Vector6d z   = errorVector(x, x_d, q, q_d);
  return (Eigen::Vector6d() << -m_K_d*nu - K_p*z).finished();
}

// Gives control vector and restoring force vector. Returns 
Eigen::Vector6d QuaternionPdController::compute(const Eigen::Vector3d    &x,
                                                const Eigen::Quaterniond &q,
                                                const Eigen::Vector6d    &nu,
                                                const Eigen::Vector3d    &x_d,
                                                const Eigen::Quaterniond &q_d)
{
  Eigen::Matrix3d R   = q.toRotationMatrix();
  Eigen::Matrix6d K_p = proportionalGainMatrix(R);
  Eigen::Vector6d z   = errorVector(x, x_d, q, q_d);
  Eigen::Vector6d g   = restoringForceVector(R);
  return (Eigen::Vector6d() << -m_K_d*nu - K_p*z + g).finished();
}

// Returns K_p(q), a 6x6 matrix with R^T(q) * Kx, 0_3x3, 0_3x3, c*I_3x3 
Eigen::Matrix6d QuaternionPdController::proportionalGainMatrix(const Eigen::Matrix3d &R)
{
  return (Eigen::Matrix6d() << R.transpose() * m_K_x,       Eigen::MatrixXd::Zero(3, 3),
                               Eigen::MatrixXd::Zero(3, 3), m_c*Eigen::MatrixXd::Identity(3, 3)).finished();
}

// Gives Position and wanted position, orientation and wanted orientation.
Eigen::Vector6d QuaternionPdController::errorVector(const Eigen::Vector3d    &x,
                                                    const Eigen::Vector3d    &x_d,
                                                    const Eigen::Quaterniond &q,
                                                    const Eigen::Quaterniond &q_d)
{
  // Returns error vector, with position error and orientation error.
  Eigen::Quaterniond q_tilde = q_d.conjugate()*q;
  q_tilde.normalize();
  return (Eigen::Vector6d() << x - x_d, sgn(q_tilde.w())*q_tilde.vec()).finished();
}

// Returns restoring force vector. 
Eigen::Vector6d QuaternionPdController::restoringForceVector(const Eigen::Matrix3d &R)
{
  Eigen::Vector3d f_G = R.transpose() * Eigen::Vector3d(0, 0, m_W);
  Eigen::Vector3d f_B = R.transpose() * Eigen::Vector3d(0, 0, -m_B);
  return (Eigen::Vector6d() << f_G + f_B, m_r_G.cross(f_G) + m_r_B.cross(f_B)).finished();
}

// Returns positiv or negativ value. 
int QuaternionPdController::sgn(double x)
{
  if (x < 0)
    return -1;
  return 1;
}
