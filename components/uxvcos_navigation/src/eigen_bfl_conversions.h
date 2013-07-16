#ifndef EIGEN_BFL_CONVERSIONS_H
#define EIGEN_BFL_CONVERSIONS_H

#include <bfl/wrappers/matrix/matrix_wrapper.h>
#include <Eigen/Geometry>

#ifndef __MATRIXWRAPPER_EIGEN__

template<typename BFLType, typename EigenType>
void toEigen(MatrixWrapper::Matrix const& matrix, EigenType &eigen) {
  assert(matrix.rows() == eigen.rows());
  assert(matrix.columns() == eigen.cols());

  for(int r = 0; r < eigen.rows(); ++r)
    for(int c = 0; c < eigen.cols(); ++c)
      eigen(r,c) = matrix(r+1,c+1);
}

template<typename EigenType>
void toEigen(MatrixWrapper::SymmetricMatrix const& matrix, EigenType &eigen) {
  assert(matrix.rows() == eigen.rows());
  assert(matrix.columns() == eigen.cols());

  for(int r = 0; r < eigen.rows(); ++r)
    for(int c = 0; c < eigen.cols(); ++c)
      eigen(r,c) = matrix(r+1,c+1);
}

template<typename EigenType>
void fromEigen(EigenType const& eigen, MatrixWrapper::Matrix &matrix) {
  matrix.resize(eigen.rows(), eigen.cols(), false, false);

  for(int r = 0; r < eigen.rows(); ++r)
    for(int c = 0; c < eigen.cols(); ++c)
      matrix(r+1,c+1) = eigen(r,c);
}

template<typename EigenType>
void fromEigen(EigenType const& eigen, MatrixWrapper::SymmetricMatrix &matrix) {
  assert(eigen.rows() == eigen.cols());
  matrix.resize(eigen.rows(), false, false);

  for(int r = 0; r < eigen.rows(); ++r)
    for(int c = 0; c < eigen.cols(); ++c)
      matrix(r+1,c+1) = eigen(r,c);
}

template<typename EigenType>
void toEigen(MatrixWrapper::ColumnVector const& vector, EigenType &eigen) {
  assert(vector.size() == eigen.rows());
  assert(eigen.cols() == 1);

  for(int r = 0; r < eigen.rows(); ++r)
    eigen(r) = vector(r+1);
}

template<typename EigenType>
void fromEigen(EigenType const& eigen, MatrixWrapper::ColumnVector &vector) {
  vector.resize(eigen.rows());

  for(int r = 0; r < eigen.rows(); ++r)
    vector(r+1) = eigen(r);
}

#else // __MATRIXWRAPPER_EIGEN__

template<typename BFLType, typename EigenType>
void toEigen(MatrixWrapper::Matrix const& matrix, EigenType &eigen) {
  eigen = matrix;
}

template<typename EigenType>
void toEigen(MatrixWrapper::SymmetricMatrix const& matrix, EigenType &eigen) {
  eigen = matrix;
}

template<typename EigenType>
void fromEigen(EigenType const& eigen, MatrixWrapper::Matrix &matrix) {
  (EigenMatrix&)matrix = eigen;
}

template<typename EigenType>
void fromEigen(EigenType const& eigen, MatrixWrapper::SymmetricMatrix &matrix) {
  (EigenMatrix&)matrix = eigen;
}

template<typename EigenType>
void toEigen(MatrixWrapper::ColumnVector const& vector, EigenType &eigen) {
  eigen = vector;
}

template<typename EigenType>
void fromEigen(EigenType const& eigen, MatrixWrapper::ColumnVector &vector) {
  (EigenColumnVector&)vector = eigen;
}

#endif // __MATRIXWRAPPER_EIGEN__

#endif // EIGEN_BFL_CONVERSIONS_H
