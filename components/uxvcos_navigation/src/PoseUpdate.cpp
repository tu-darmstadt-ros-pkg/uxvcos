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

#include "PoseUpdate.h"
#include "Navigation.h"

#include <geometry_msgs/PoseStamped.h>
#include <Eigen/Geometry>
#include "eigen_bfl_conversions.h"

#include <rtt/Logger.hpp>

namespace uxvcos {
namespace Navigation {

PoseUpdate::PoseUpdate(Navigation *navigation)
  : navigation(navigation)
  , model(&pdf)
{
  subscriber = nh.subscribe("poseupdate", 10, &PoseUpdate::incomingPoseUpdate, this);
  // pose_publisher = nh.advertise<geometry_msgs::PoseStamped>("pose", 10, false);
}

PoseUpdate::~PoseUpdate()
{}

void PoseUpdate::incomingPoseUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseStamped)
{
  this->poseStamped = poseStamped;
}

void PoseUpdate::update()
{
  static const double TIMEOUT = 1.0;

  if (poseStamped) {
    update(navigation->pFilter, poseStamped->pose);
    lastTimestamp = navigation->NavData.header.stamp;
    poseStamped.reset();

    // disable zero velocity updates
    navigation->ExternalPoseUpdate = true;
  } else {
    if ((navigation->NavData.header.stamp - lastTimestamp).toSec() > TIMEOUT)
      navigation->ExternalPoseUpdate = false;
  }
}

/*
typedef Eigen::Matrix<double,6,6> Matrix6;
typedef Eigen::Matrix<double,6,1> Vector6;
typedef Eigen::Matrix<double,6,NUMBER_OF_STATES> MatrixH;
typedef Eigen::Matrix<double,NUMBER_OF_STATES,6> MatrixHT;
typedef Eigen::Matrix<double,NUMBER_OF_STATES,NUMBER_OF_STATES> MatrixX;
typedef Eigen::Matrix<double,NUMBER_OF_STATES,1> VectorX;

// Eigen, full state update
void PoseUpdate::update(BFL::KalmanFilter *filter, const geometry_msgs::PoseWithCovariance &pose, double alpha, double beta)
{
  RTT::Logger::In in("PoseUpdate");
  static const double initH[] = { 0.0, 0.0, 0.0, 0.0, 0.0, 1.0,
                                  0.0, 0.0, 0.0, 0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0, 1.0, 0.0, 0.0,
                                  1.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
  static const MatrixH  H(initH);
  static const MatrixHT HT(H.transpose());

  BFL::Gaussian *post = filter->PostGet();
  MatrixWrapper::ColumnVector    x_est(post->ExpectedValueGet());
  MatrixWrapper::SymmetricMatrix C_est(post->CovarianceGet());

  VectorX x_prior;
  toEigen(x_est, x_prior);

  MatrixX C_prior;
  toEigen(C_est, C_prior);

  MatrixX Ia(C_prior.inverse());

//  RTT::log(RTT::Debug)
//    << "prior: x_prior = [" << x_prior << "]" << RTT::nlog()
//    << "       C_prior = [" << C_prior << "]" << RTT::nlog()
//    << RTT::endlog();

  // debugging
  geometry_msgs::PoseStamped pose_prior;
  pose_prior.header.stamp = uxvcos::Time::now();
  pose_prior.header.frame_id = "/nav";
  pose_prior.pose.position.x = x_prior(3);
  pose_prior.pose.position.y = x_prior(4);
  pose_prior.pose.position.z = x_prior(5);
  Eigen::Quaterniond orientation(Eigen::AngleAxisd(x_prior(2), Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(x_prior(1), Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(x_prior(0), Eigen::Vector3d::UnitX()));
  pose_prior.pose.orientation.w = orientation.w();
  pose_prior.pose.orientation.x = orientation.x();
  pose_prior.pose.orientation.y = orientation.y();
  pose_prior.pose.orientation.z = orientation.z();
  pose_publisher.publish(pose_prior);

  // fetch external pose
  Vector6 xb;
  Matrix6 Ib;

  Eigen::Vector3d eulerb = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  xb(0)   = pose.pose.position.x;
  xb(1)   = pose.pose.position.y;
  xb(2)   = pose.pose.position.z;
  xb(3)   = eulerb[0];
  xb(4)   = eulerb[1];
  xb(5)   = eulerb[2];

  Matrix6 selection(Matrix6::Zero());
  for(unsigned int i = 0; i < 6; ++i) {
    for(unsigned int j = 0; j < 6; ++j)
      Ib(i,j) = pose.covariance[i*6+j];
    if (Ib(i,i) > 0) selection(i,i) = 1.0;
  }
  MatrixX selectionX(HT * selection * H);

  // compute alpha and beta if not given by the user
  if (alpha == 0.0 && beta == 0.0) {
    // double tr_a = C_prior.inverse().trace();
    // double tr_b = Ib.trace();

    // double tr_a = (H*C_prior*HT).inverse().trace();
    // double tr_b = Ib.trace();

    // double tr_a = (selectionX * Ia * selectionX).trace();
    double tr_a = Ia.trace();
    double tr_b = (HT * Ib * H).trace();

    alpha = tr_a / (tr_a + tr_b);
    beta  = tr_b / (tr_a + tr_b);
  }
  alpha = 1.0;
  beta  = 0.1;

//  RTT::log(RTT::Debug)
//    << "x_b = [" << xb << "]" << RTT::nlog()
//    << "I_b = [" << Ib << "]" << RTT::nlog()
//    << RTT::endlog();

  RTT::log(RTT::Debug) << "update: dx=" << (xb(0) - x_prior(3)) << "  dy=" << (xb(1) - x_prior(4)) << "  dpsi=" << (xb(3) - x_prior(2))*180.0/M_PI << RTT::nlog()
                       << "        alpha=" << alpha << "  beta=" << beta << RTT::endlog();

//  Ib.setZero();
//  Ib(0,0) = 1.0/ 100.0;
//  Ib(1,1) = 1.0/ 100.0;
//  Ib(3,3) = 1.0/ ((5.0*M_PI/180.0)*(5.0*M_PI/180.0));
//  beta = 1.0;

  // compute new covariance Matrix
  // Ia *= alpha;
  // Ib *= beta;
  MatrixX I_new(Ia - (1.0 - alpha) * selectionX * Ia * selectionX + beta * HT * Ib * H);
  // MatrixX I_new(alpha * Ia + beta * HT * Ib * H);
  MatrixX C_new(I_new.inverse());

  // compute new state vector
  VectorX x_new((MatrixX::Identity() - (1.0 - alpha) * selectionX) * Ia * x_prior + beta * HT * Ib * xb);
  // VectorX x_new(alpha * Ia * x_prior + beta * HT * Ib * xb);
  x_new = C_new * x_new;

//  RTT::log(RTT::Debug)
//    << "post:  x_new = [" << x_new << "]" << RTT::nlog()
//    << "       C_new = [" << C_new << "]" << RTT::nlog()
//    << RTT::endlog();

  // generate output
  fromEigen(x_new, x_est);
  fromEigen(C_new, C_est);
  post->ExpectedValueSet(x_est);
  post->CovarianceSet(C_est);
} */

typedef Eigen::Matrix<double,3,3> Matrix3;
typedef Eigen::Matrix<double,3,1> Vector3;
typedef Eigen::Matrix<double,3,NUMBER_OF_STATES> MatrixH;
typedef Eigen::Matrix<double,NUMBER_OF_STATES,3> MatrixHT;
typedef Eigen::Matrix<double,NUMBER_OF_STATES,NUMBER_OF_STATES> MatrixX;
typedef Eigen::Matrix<double,NUMBER_OF_STATES,1> VectorX;

// Eigen, covariance form
void PoseUpdate::update(BFL::KalmanFilter *filter, const geometry_msgs::PoseWithCovariance &pose, double alpha, double beta)
{
  RTT::Logger::In in("PoseUpdate");
  static const double initH[] = { 0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 1.0,
                                  1.0, 0.0, 0.0,
                                  0.0, 1.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0,
                                  0.0, 0.0, 0.0 };
  static const MatrixH  H(initH);
  static const MatrixHT HT(H.transpose());

  BFL::Gaussian *post = filter->PostGet();
  MatrixWrapper::ColumnVector    x_est(post->ExpectedValueGet());
  MatrixWrapper::SymmetricMatrix C_est(post->CovarianceGet());

  VectorX x_prior;
  toEigen(x_est, x_prior);

  MatrixX C_prior;
  toEigen(C_est, C_prior);

  // fetch external pose
  Vector3 xb;
  Matrix3 Ib;

  Eigen::Vector3d eulerb = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  xb(0)   = pose.pose.position.x;
  xb(1)   = pose.pose.position.y;
  xb(2)   = eulerb[0];

  static const unsigned int index[3] = { 0, 1, 3 };
  for(unsigned int i = 0; i < 3; ++i)
    for(unsigned int j = 0; j < 3; ++j)
      Ib(i,j) = pose.covariance[index[i]*6+index[j]];

  if (Ib.determinant() == 0.0) return;
  // Matrix3 R(Ib.inverse());
  Matrix3 R;
  R(0,0) = R(1,1) = pow(0.1, 2);
  R(2,2) = pow(5.0 * M_PI/180.0, 2);

  // compute alpha and beta if not given by the user
  if (alpha == 0.0 && beta == 0.0) {
    Matrix3 weights(Matrix3::Zero());
    weights(0,0) = weights(1,1) = 1.0 / 1.0;
    weights(2,2)                = 1.0 / 10.0 * M_PI/180.0;

    double tr_a = (weights*H*C_prior*HT).trace();
    double tr_b = (weights*R).trace();

    alpha = tr_b / (tr_a + tr_b);
    beta  = tr_a / (tr_a + tr_b);
  }
  alpha = 1.0;
  beta  = 1.0;

//  RTT::log(RTT::Debug)
//    << "x_b = [" << xb << "]" << RTT::nlog()
//    << "I_b = [" << Ib << "]" << RTT::nlog()
//    << RTT::endlog();

  RTT::log(RTT::Debug) << "update: dx=" << (xb(0) - x_prior(3)) << "  dy=" << (xb(1) - x_prior(4)) << "  dpsi=" << (xb(2) - x_prior(2))*180.0/M_PI << RTT::nlog()
                       << "        alpha=" << alpha << "  beta=" << beta << RTT::endlog();

  C_prior /= alpha;
  R       /= beta;

  MatrixHT K(C_prior * HT * (R + (H * C_prior * HT)).inverse());
  MatrixX C_new((MatrixX::Identity() - K * H) * C_prior);
  Vector3 e(xb - H*x_prior);
  e(2) -= 2*M_PI*round(e(2) / (2*M_PI));
  VectorX x_new(x_prior + K*e);

//  RTT::log(RTT::Debug)
//    << "post:  x_new = [" << x_new << "]" << RTT::nlog()
//    << "       C_new = [" << C_new << "]" << RTT::nlog()
//    << RTT::endlog();

  // generate output
  fromEigen(x_new, x_est);
  fromEigen(C_new, C_est);
  post->ExpectedValueSet(x_est);
  post->CovarianceSet(C_est);
}

/* // Direct update
void PoseUpdate::update(BFL::KalmanFilter *filter, const geometry_msgs::PoseWithCovariance &pose, double alpha, double beta)
{
  RTT::Logger::In in("PoseUpdate");

  BFL::Gaussian *post = filter->PostGet();
  MatrixWrapper::ColumnVector    x_est(post->ExpectedValueGet());

  MatrixWrapper::SymmetricMatrix Ib(6);
  for(unsigned int i = 0; i < 6; ++i)
    for(unsigned int j = 0; j < 6; ++j)
      Ib(i+1,j+1) = pose.covariance[i*6+j];

  Eigen::Vector3d eulerb = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
  if (Ib(1,1) > 0.0) x_est(4) = pose.pose.position.x;
  if (Ib(2,2) > 0.0) x_est(5) = pose.pose.position.x;
  if (Ib(3,3) > 0.0) x_est(6) = pose.pose.position.x;
  if (Ib(4,4) > 0.0) x_est(3) = eulerb[0];
  if (Ib(5,5) > 0.0) x_est(2) = eulerb[1];
  if (Ib(6,6) > 0.0) x_est(1) = eulerb[2];

  // generate output
  post->ExpectedValueSet(x_est);
} */

/* // BFL measurement model version
void PoseUpdate::update(BFL::KalmanFilter *filter, const geometry_msgs::PoseWithCovariance &pose, double alpha, double beta)
{
  MatrixWrapper::SymmetricMatrix R(3);
  R(1,1) = pose.covariance[0*6+0];
  R(1,2) = pose.covariance[0*6+1];
  R(1,3) = pose.covariance[0*6+3];
  R(2,2) = pose.covariance[1*6+1];
  R(2,3) = pose.covariance[1*6+3];
  R(3,3) = pose.covariance[3*6+3];

  if (R.determinant() > 0) {
    R = R.inverse();


    MatrixWrapper::ColumnVector y(3);
    Eigen::Vector3d eulerb = Eigen::Quaterniond(pose.pose.orientation.w, pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z).toRotationMatrix().eulerAngles(2,1,0);
    y(1)   = pose.pose.position.x;
    y(2)   = pose.pose.position.y;
    y(3)   = eulerb[0];

    pdf.AdditiveNoiseSigmaSet(R);
    filter->Update(&model, y);
  }
} */

PoseUpdate::Pdf::Pdf() : BFL::AnalyticConditionalGaussianAdditiveNoise(3,1)
{}

//--> Measurement equation y = h(x)
MatrixWrapper::ColumnVector PoseUpdate::Pdf::ExpectedValueGet() const
{
  MatrixWrapper::ColumnVector x = ConditionalArgumentGet(0);
  MatrixWrapper::ColumnVector y(3);

  y(1) = x(PX);
  y(2) = x(PY);
  // y(3) = x(6);
  y(3) = x(YAW);
  // y(5) = x(2);
  // y(6) = x(1);

  return y + AdditiveNoiseMuGet();
}

//--> Jacobian matrix H
MatrixWrapper::Matrix PoseUpdate::Pdf::dfGet(unsigned int i) const
{
  //--> Derivative to the first conditional argument (x)
  if (i==0)
  {
    //ColumnVector x = ConditionalArgumentGet(0);

    //--> Enhance visibility
    //----------------------------------------------------------

    //----------------------------------------------------------

    //--> Set H-Matrix
    //----------------------------------------------------------
    MatrixWrapper::Matrix H(3,NUMBER_OF_STATES);

    //--> Clear H-Matrix
    H = 0.0;

    H(1,PX) = 1.0;
    H(2,PY) = 1.0;
    // H(3,6) = 1.0;
    H(3,YAW) = 1.0;
    // H(5,2) = 1.0;
    // H(6,1) = 1.0;
    //----------------------------------------------------------

    return H;
  }

  return MatrixWrapper::Matrix();
}

} // namespace Navigation
} // namespace uxvcos
