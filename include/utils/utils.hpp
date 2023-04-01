#pragma once
#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <casadi/casadi.hpp>

namespace Utils {
typedef Eigen::Matrix<casadi::SX, -1, 1> VectorXs;
typedef Eigen::Matrix<casadi::SX, -1, -1> MatrixXs;
typedef Eigen::SparseMatrix<casadi::SX, Eigen::RowMajor> SpMatrixXs;

typedef Eigen::Matrix<casadi::MX, -1, 1> VectorXm;
typedef Eigen::Matrix<casadi::MX, -1, -1> MatrixXm;
typedef Eigen::SparseMatrix<casadi::MX, Eigen::RowMajor> SpMatrixXm;

static casadi::SX eig_to_cas(const VectorXs &eig) {
  auto sx = casadi::SX(casadi::Sparsity::dense(eig.size()));
  for (int i = 0; i < eig.size(); i++) {
    sx(i) = eig(i);
  }
  return sx;
}
static casadi::MX eig_to_cas_MX(const VectorXm &eig) {
  auto sx = casadi::MX(casadi::Sparsity::dense(eig.size()));
  for (int i = 0; i < eig.size(); i++) {
    sx(i) = eig(i);
  }
  return sx;
}

static casadi::SX eigmat_to_cas(const MatrixXs &eig) {
  auto sx = casadi::SX(casadi::Sparsity::dense(eig.rows(), eig.cols()));
  for (int i = 0; i < eig.rows(); i++) {
    for (int j = 0; j < eig.cols(); j++) {
      sx(i, j) = eig(i, j);
    }
  }
  return sx;
}

static casadi::SX eig_spmat_to_cas(const SpMatrixXs &eig) {
  auto sx = casadi::SX(eig.rows(), eig.cols());
  for (int k = 0; k < eig.outerSize(); ++k) {
    for (Eigen::SparseMatrix<casadi::SX, Eigen::RowMajor>::InnerIterator it(eig,
                                                                            k);
         it; ++it) {
      sx(it.row(), it.col()) = it.value();
    }
  }
  return sx;
}

static casadi::MX eig_spmat_to_cas_MX(const SpMatrixXm &eig) {
  auto sx = casadi::MX(eig.rows(), eig.cols());
  for (int k = 0; k < eig.outerSize(); ++k) {
    for (Eigen::SparseMatrix<casadi::MX, Eigen::RowMajor>::InnerIterator it(eig,
                                                                            k);
         it; ++it) {
      sx(it.row(), it.col()) = it.value();
    }
  }
  return sx;
}

static VectorXs cas_to_eig(const casadi::SX &cas) {
  VectorXs eig(cas.size1());
  for (int i = 0; i < eig.size(); i++) {
    eig(i) = cas(i);
  }
  return eig;
}

template <int s1, int s2>
static Eigen::Matrix<casadi::SX, s1, s2> cas_to_eigmat(const casadi::SX &cas) {
  Eigen::Matrix<casadi::SX, s1, s2> eigmat;
  for (int i = 0; i < s1; i++) {
    for (int j = 0; j < s2; j++) {
      eigmat(i, j) = cas(i, j);
    }
  }
  return eigmat;
}

static VectorXm cas_to_eig_MX(const casadi::MX &cas) {
  VectorXm eig(cas.size1());
  for (int i = 0; i < eig.size(); i++) {
    eig(i) = cas(i);
  }
  return eig;
}

static Eigen::SparseMatrix<double> cas_DM_to_eig_sp(const casadi::DM &Matrx) {
  casadi::Sparsity SpA = Matrx.get_sparsity();
  // std::cout << "Nonzeros in rows: " << SpA.get_row() << "\n";
  // std::cout << "Nonzeros in columns: " << SpA.get_colind() << "\n";

  std::vector<casadi_int> output_row, output_col;
  SpA.get_triplet(output_row, output_col);
  std::vector<double> values = Matrx.get_nonzeros();

  // std::cout << "Output row: " << output_row << "\n";
  // std::cout << "Output col: " << output_col << "\n";
  // std::cout << "Nonzeros: " << Matrx.get_nonzeros() << "\n";

  using T = Eigen::Triplet<double>;
  std::vector<T> TripletList;
  TripletList.resize(values.size());
  for (int k = 0; k < values.size(); ++k)
    TripletList[k] = T(output_row[k], output_col[k], values[k]);

  for (std::vector<T>::const_iterator it = TripletList.begin();
       it != TripletList.end(); ++it) {
    // std::cout << "triplet: " << (*it).row() << " " << (*it).col() << " "
    //           << (*it).value() << "\n";
  }

  Eigen::SparseMatrix<double> SpMatrx(Matrx.size1(), Matrx.size2());
  SpMatrx.setFromTriplets(TripletList.begin(), TripletList.end());

  // std::cout << "Eigen sparse matrix: \n" << SpMatrx << "\n";
  return SpMatrx;
}

// convert casadi::DM to Eigen::MatrixXd
template <int s1, int s2>
static Eigen::MatrixXd cas_DM_to_eig_mat(const casadi::DM &Matrx) {
  auto vector_x = static_cast<std::vector<double>>(Matrx);
  Eigen::Matrix<double, s1, s2> eigMatrx(vector_x.data());
  return eigMatrx;
}

template <int s1, int s2>
static casadi::DM eig_to_cas_DM(const Eigen::Matrix<double, s1, s2> &eig) {
  casadi::DM cas = casadi::DM::zeros(s1, s2);
  for (int i = 0; i < s1; i++) {
    for (int j = 0; j < s2; j++) {
      cas(i, j) = eig(i, j);
    }
  }
  return cas;
}

template <int s1, int s2>
static Eigen::Matrix<double, s1, s2>
read_eig_mat_from_csv(std::string file_name) {
  // Load the CSV file saved by MATLAB
  std::ifstream file(file_name);

  // Read the data into an Eigen sparse matrix so we can adjust the size
  // dynamically
  Eigen::Matrix<double, s1, s2> A;
  A.setZero();
  std::string line;
  int row = 0;
  while (std::getline(file, line)) {
    std::stringstream line_stream(line);
    std::string cell;
    int col = 0;
    if (s2 == 1) {
      A(row, 0) = std::stod(line);
    } else {
      while (std::getline(line_stream, cell, ',')) {
        A(row, col) = std::stod(cell);
        col++;
      }
    }
    row++;
  }

  return A;
}

} // namespace Utils