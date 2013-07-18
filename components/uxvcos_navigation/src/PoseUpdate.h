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

#ifndef UXVCOS_NAVIGATION_POSEUPDATE_H
#define UXVCOS_NAVIGATION_POSEUPDATE_H

#include <ros/ros.h>

#include <bfl/model/analyticmeasurementmodel_gaussianuncertainty.h>
#include <bfl/pdf/analyticconditionalgaussian_additivenoise.h>
#include <bfl/filter/kalmanfilter.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <uxvcos/Time.h>

namespace uxvcos {
namespace Navigation {

class Navigation;

class PoseUpdate
{
private:
  ros::NodeHandle nh;
  ros::Subscriber subscriber;
  ros::Publisher pose_publisher;

  Navigation *navigation;
  geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseStamped;
  uxvcos::Time lastTimestamp;

public:
  PoseUpdate(Navigation *navigation);
  virtual ~PoseUpdate();

  void incomingPoseUpdate(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr poseStamped);

  void update();
  void update(BFL::KalmanFilter* filter, geometry_msgs::PoseWithCovariance const& pose, double alpha = 0.0, double beta = 0.0);

private:
  class Pdf : public BFL::AnalyticConditionalGaussianAdditiveNoise
  {
  public:
    Pdf();
    virtual MatrixWrapper::ColumnVector ExpectedValueGet() const;
    virtual MatrixWrapper::Matrix dfGet(unsigned int i) const;
  } pdf;
  BFL::AnalyticMeasurementModelGaussianUncertainty model;
};

} // namespace Navigation
} // namespace uxvcos

#endif // UXVCOS_NAVIGATION_POSEUPDATE_H
