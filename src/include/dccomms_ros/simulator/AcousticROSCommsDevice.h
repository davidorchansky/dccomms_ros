#ifndef DCCOMMS_ROS_ACOUSTIC_ROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_ACOUSTIC_ROSCOMMSDEVICE_H_

#include <dccomms_ros/simulator/AcousticCommsChannel.h>
#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <ns3/aqua-sim-helper.h>
#include <ns3/aqua-sim-net-device.h>
#include <ns3/mobility-helper.h>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

enum AQS_ROUTING_TYPE { AQS_ROUTING_DUMMY, AQS_ROUTING_VBF };
class AcousticROSCommsDevice : public ROSCommsDevice {
public:
  AcousticROSCommsDevice(ROSCommsSimulatorPtr, PacketBuilderPtr);

  DEV_TYPE GetDevType();

protected:
  void DoSetMac(uint32_t mac);
  void DoSend(PacketPtr dlf);
  void DoLinkToChannel(CommsChannelPtr channel);
  void DoStart();
  void DoSetPosition(const tf::Vector3 & position);
  bool DoStarted();

private:
  ns3::Ptr<ns3::AquaSimChannel> _channel;
  ns3::Ptr<ns3::AquaSimNetDevice> _device;
  ns3::Ptr<ns3::AquaSimMac> _macLayer;
  ns3::Ptr<ns3::AquaSimRouting> _routingLayer;
  ns3::Ptr<ns3::Node> _node;
  ns3::Ptr<ns3::MobilityModel> _mobility;
  ns3::AquaSimHelper _asHelper;
  ns3::AquaSimAddress _aquaSimAddr;
  ns3::MobilityHelper _mobh;

  bool _started;
  uint32_t _nodeListIndex;

  AQS_ROUTING_TYPE _routingType;

  void _Recv(std::string context, ns3::Ptr<const ns3::Packet>);
  void _SendTrace(std::string context, ns3::Ptr<const ns3::Packet>);
  void _RxError(std::string context, ns3::Ptr<const ns3::Packet>);
  void _PositionUpdated(std::string context, ns3::Ptr<const MobilityModel> model);
};
}
#endif // COMMSNODE_H
