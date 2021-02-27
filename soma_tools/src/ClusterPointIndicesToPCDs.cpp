#include <ros/ros.h>
#include <nodelet/nodelet.h>
#include <pluginlib/class_list_macros.h>

namespace soma_tools
{
  class ClusterPointIndicesToPCDs : public nodelet::Nodelet
  {
  public:
    ClusterPointIndicesToPCDs();

  private:
    virtual void onInit();

  private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
  };

  ClusterPointIndicesToPCDs::ClusterPointIndicesToPCDs()
      : nh(getNodeHandle()),
        pnh(getPrivateNodeHandle())
  {
  }

  //
  //
  //
  void ClusterPointIndicesToPCDs::onInit()
  {
  }
}

PLUGINLIB_EXPORT_CLASS(soma_tools::ClusterPointIndicesToPCDs, nodelet::Nodelet)