#include <cassert>
#include <iostream>

#include <Eigen/Dense>


int main() {
  /* ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
    1. Initialize matrix with Eigen::MatrixXd
    ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~ */
  /* Initialize a 2x2 matrix */
  Eigen::MatrixXd matrix(2, 2);
  matrix(0, 0) = 1;
  matrix(0, 1) = 2;
  matrix(1, 0) = 3;
  matrix(1, 1) = 4;
  std::cout << "matrix(2x2) = \n"
            << matrix
            << std::endl << std::endl;

  /* Initialize a 2x2 matrix with insertion operator */
  Eigen::MatrixXd matrix2(2, 2);
  matrix2 << 5, 6,
             7, 8;
  std::cout << "matrix2(2x2) = \n"
            << matrix2
            << std::endl << std::endl;

  /* Initialize identity matrix */
  Eigen::MatrixXd identity = Eigen::MatrixXd::Identity(2, 2);
  std::cout << "identity(2x2) = \n"
            << identity
            << std::endl << std::endl;

  /* Initialize a 2x2 matrix with random values */
  Eigen::Matrix<double, 2, 2> random = Eigen::Matrix<double, 2, 2>::Random();
  // Eigen::Matrix2d random = Eigen::Matrix2d::Random();  // Equivalent
  std::cout << "random(2x2) = \n"
            << random
            << std::endl << std::endl;

  /* Initialize a 2x2 matrix with zeros */
  Eigen::Matrix2d zeros = Eigen::Matrix2d::Zero();
  // Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(2, 2);   // Equivalent
  std::cout << "zeros(2x2) = \n"
            << zeros
            << std::endl << std::endl;

  /* ~~~~~~~~~~~~~~~~~~~
    2. Matrix operations
    ~~~~~~~~~~~~~~~~~~~~ */
  /* Matrix addition */
  Eigen::MatrixXd sum = matrix + matrix2;
  std::cout << "matrix + matrix2 = \n"
            << sum
            << std::endl << std::endl;

  /* Matrix multiplication */
  Eigen::MatrixXd product = matrix * matrix2;
  std::cout << "matrix * matrix2 = \n"
            << product
            << std::endl << std::endl;

  /* Matrix transpose */
  Eigen::MatrixXd transpose = matrix.transpose();
  std::cout << "matrix^T = \n"
            << transpose
            << std::endl << std::endl;

  /* Matrix inverse */
  Eigen::MatrixXd inverse = matrix.inverse();
  std::cout << "matrix^-1 = \n"
            << inverse
            << std::endl << std::endl;
  std::cout << "matrix * matrix^-1 = \n"
            << matrix * inverse
            << std::endl << std::endl;

  /* ~~~~~~~~~~~~~~~~~~~
    3. Linear system
    ~~~~~~~~~~~~~~~~~~~~ */
  /* Solve linear system */
  Eigen::VectorXd b(2);
  b << 1, 2;
  Eigen::VectorXd x = matrix2.colPivHouseholderQr().solve(b);
  std::cout << "Solution to matrix2 * x = b, where b = [1, 2]: \n"
            << "x = \n"
            << x
            << std::endl << std::endl;
  std::cout << "Error: matrix2 * x - b = \n"
            << (matrix2 * x - b).norm()
            << std::endl << std::endl;

  /* Eigenvalues and eigenvectors */
  Eigen::MatrixXd A(2, 2);
  A << 1, 2,
       2, 1;
  Eigen::EigenSolver<Eigen::MatrixXd> eigen_solver(A);
  assert(eigen_solver.info() == Eigen::Success);
  std::cout << "Eigenvalues of A = \n"
            << eigen_solver.eigenvalues()
            << std::endl << std::endl;
  std::cout << "Eigenvectors of A = \n"
            << eigen_solver.eigenvectors()
            << std::endl << std::endl;

  return 0;
}


