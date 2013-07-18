//=================================================================================================
// Copyright (c) 2013, Johannes Meyer and contributors, Technische Universitat Darmstadt
// All rights reserved.

// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of the Flight Systems and Automatic Control group,
//       TU Darmstadt, nor the names of its contributors may be used to
//       endorse or promote products derived from this software without
//       specific prior written permission.

// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
// ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
// WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//=================================================================================================

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
