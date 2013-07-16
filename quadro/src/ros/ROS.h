#ifndef QUADRO_ROS_H
#define QUADRO_ROS_H

#include <uxvcos.h>
#include <ros_integration/ros_taskcontext.hpp>
#include <base/PortListener.h>

#include <tf/transform_broadcaster.h>

namespace quadro {

class UXVCOS_EXPORT ROS : public ros_integration::TaskContext
{
public:
  ROS(const std::string& name = "ROS");
  virtual ~ROS();

protected:
  bool startHook();
  void stopHook();

  void updateHook();
  bool breakUpdateHook();

private:
  uxvcos::Time startTime;
  double timeOffset;
  std::vector<geometry_msgs::TransformStamped> transforms;
  ::PortListener listener;
};

} // namespace quadro

#endif // QUADRO_ROS_H
