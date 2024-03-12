#include <cmath>
#include <iostream>

#include <Eigen/Dense>
#include <Eigen/Geometry>

#define RAD2DEG 180.0 / M_PI


int main() {
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    1. Initialize Axis-Angle, Rotation Matrix(SO(3)), and Quaternion
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
  /* Initialize axis-angle */
  Eigen::AngleAxisd rot_vec(M_PI / 2.0, Eigen::Vector3d(0, 0, 1.0));  // 90 degrees rotation about z-axis
  std::cout << "rot_vec = \n"
            << "angle = " << rot_vec.angle() * RAD2DEG << " deg\n"
            << "axis =\n" << rot_vec.axis()
            << std::endl << std::endl;

  /* Initialize rotation matrix */
  Eigen::Matrix3d rot_mat;
  Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();
  rot_mat << 0, -1, 0,  // 90 degrees rotation about z-axis
             1, 0, 0,
             0, 0, 1;
  std::cout << "identity(3x3) = \n"
            << identity
            << std::endl << std::endl;
  std::cout << "rot_mat(3x3) = \n"
            << rot_mat
            << std::endl << std::endl;

  /* Initialize quaternion */
  Eigen::Quaterniond quat = Eigen::Quaterniond(rot_vec);  // 90 degrees rotation about z-axis
  std::cout << "quat = \n"
            << quat.coeffs()
            << std::endl << std::endl;


  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    2. Convert between axis-angle, rotation matrix, and quaternion
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
  /* Convert axis-angle to rotation matrix */
  std::cout << "rot_vec.toRotationMatrix() = \n"
            << rot_vec.toRotationMatrix()
            << std::endl << std::endl;
  std::cout << "rot_vec.matrix() = \n"
            << rot_vec.matrix()
            << std::endl << std::endl;
  /* Convert axis-angle to quaternion */
  std::cout << "rot_vec to quaternion = \n"
            << Eigen::Quaterniond(rot_vec).coeffs()
            << std::endl << std::endl;

  /* Convert rotation matrix to axis-angle */
  std::cout << "rot_mat to axis-angle = \n"
            << "angle = " << Eigen::AngleAxisd(rot_mat).angle() * RAD2DEG << " deg\n"
            << "axis =\n" << Eigen::AngleAxisd(rot_mat).axis()
            << std::endl << std::endl;
  /* Convert rotation matrix to quaternion */
  std::cout << "rot_mat to quaternion = \n"
            << Eigen::Quaterniond(rot_mat).coeffs()
            << std::endl << std::endl;

  /* Convert quaternion to axis-angle */
  std::cout << "quat to axis-angle = \n"
            << "angle = " << Eigen::AngleAxisd(quat).angle() * RAD2DEG << " deg\n"
            << "axis =\n" << Eigen::AngleAxisd(quat).axis()
            << std::endl << std::endl;
  /* Convert quaternion to rotation matrix */
  std::cout << "quat to rotation matrix = \n"
            << quat.toRotationMatrix()
            << std::endl << std::endl;

  return 0;
}



