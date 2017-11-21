#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;

namespace dccomms_ros {

CustomCommsChannel::CustomCommsChannel(uint32_t id) { _rosChannelId = id; }

void CustomCommsChannel::SetMinPrTime(double prTime) { _minPrTime = prTime; }

void CustomCommsChannel::SetPrTimeInc(double inc) { _prTimeIncPerMeter = inc; }

void CustomCommsChannel::AddDevice(CustomROSCommsDevicePtr dev) {
  _devices.push_back(dev);
}

void CustomCommsChannel::SendPacket(CustomROSCommsDevicePtr dev,
                                    dccomms::PacketPtr pkt) {
  auto txpos = dev->GetPosition();
  auto pktSize = pkt->GetPacketSize();
  auto byteTrt = dev->GetNextTt();
  auto trTime = pktSize * byteTrt;

  for (CustomROSCommsDevicePtr dst : _devices) {
    if (dst->GetMac() != dev->GetMac()) {
      auto rxpos = dst->GetPosition();
      auto distance = txpos.distance(rxpos);
      auto delay = _minPrTime + _prTimeIncPerMeter * distance;
      auto totalTime = static_cast<uint32_t>(ceil(delay + trTime));
      ns3::Simulator::ScheduleWithContext(
          dev->GetMac(), MilliSeconds(totalTime), &ROSCommsDevice::ReceiveFrame,
          dst.get(), pkt);
    }
  }
}
}
