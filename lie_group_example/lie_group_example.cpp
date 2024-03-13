#include <cassert>

#include <Eigen/Core>
#include <fmt/format.h>
#include <sophus/so3.hpp>
#include <sophus/se3.hpp>


int main() {
  /* ~~~~
    SO(3)
    ~~~~~ */
  /* Random rotation vector */
  Eigen::Vector3d rand_rot_vec = Eigen::Vector3d::Random() * 0.01;
  fmt::print("Random vector:\n{}\n\n", rand_rot_vec);

  /* hat operator (rotation vector to skew-symmetric matrix) */
  Eigen::Matrix3d hat = Sophus::SO3d::hat(rand_rot_vec);
  fmt::print("hat operator:\n{}\n\n", hat);

  /* vee operator (skew-symmetric matrix to rotation vector) */
  Eigen::Vector3d vee = Sophus::SO3d::vee(hat);
  fmt::print("vee operator:\n{}\n\n", vee);

  /* Convert rotation vector to SO3 via exponential map */
  Sophus::SO3d SO3 = Sophus::SO3d::exp(rand_rot_vec);
  fmt::print("SO3 matrix:\n{}\n\n", SO3.matrix());

  /* Convert SO3 to rotation vector via logarithm map */
  Eigen::Vector3d log_SO3 = SO3.log();
  fmt::print("Logarithm of SO3:\n{}\n\n", log_SO3);

  /* Compare the two vectors */
  assert((rand_rot_vec - vee).norm() < 1e-10);
  assert((rand_rot_vec - log_SO3).norm() < 1e-10);

  /* ~~~~
    SE(3)
    ~~~~~ */
  /* Random rotation & translation vector */
  Eigen::Vector<double, 6> rand_twist_vec = Eigen::Vector<double, 6>::Random() * 0.01;
  fmt::print("Random SE3 vector:\n{}\n\n", rand_twist_vec);

  /* Convert SE3 vector to SE3 via exponential map */
  Sophus::SE3d SE3 = Sophus::SE3d::exp(rand_twist_vec);
  fmt::print("SE3 matrix:\n{}\n\n", SE3.matrix());

  /* Convert SE3 to twist(rotation & translation) vector via logarithm map */
  Eigen::Vector<double, 6> log_SE3 = SE3.log();
  fmt::print("Logarithm of SE3:\n{}\n\n", log_SE3);

  /* Compare the two vectors */
  assert((log_SE3 - rand_twist_vec).norm() < 1e-10);

  /* ~~~~~~~~~~~~~~~
    SO3 Perturbation
    ~~~~~~~~~~~~~~~~ */
  Eigen::Vector3d perturb_rot_vec {0.0001, 0.0, 0.0};
  fmt::print("Perturbation vector:\n{}\n\n", perturb_rot_vec);

  /* Perturbation matrix */
  Sophus::SO3d perturbation = Sophus::SO3d::exp(perturb_rot_vec);
  fmt::print("Perturbation matrix:\n{}\n\n", perturbation.matrix());

  /* Apply perturbation to SO3 */
  Sophus::SO3d SO3_perturbed = SO3 * perturbation;
  fmt::print("Perturbed SO3 matrix:\n{}\n\n", SO3_perturbed.matrix());


  return 0;
}

