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
