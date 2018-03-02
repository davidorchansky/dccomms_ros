#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;

namespace dccomms_ros {

CustomCommsChannel::CustomCommsChannel(uint32_t id) { _rosChannelId = id; }

void CustomCommsChannel::SetMinPrTime(double prTime) {
  _minPrTime = prTime * 1000000;
} // from millis to nanos

void CustomCommsChannel::SetPrTimeInc(double inc) {
  _prTimeIncPerMeter = inc * 1000000;
} // from millis to nanos

void CustomCommsChannel::AddDevice(CustomROSCommsDevicePtr dev) {
  _devices.push_back(dev);
}

void CustomCommsChannel::SendPacket(CustomROSCommsDevicePtr dev,
                                    dccomms::PacketPtr pkt) {
  auto txpos = dev->GetPosition();
  auto minErrRate = dev->GetMinPktErrorRate();
  auto errRateInc = dev->GetPktErrorRateInc();

  for (CustomROSCommsDevicePtr dst : _devices) {
    if (dst->GetMac() != dev->GetMac()) {
      auto rxpos = dst->GetPosition();
      auto distance = txpos.distance(rxpos); // distance = meters
      auto dm = distance * 10;               // dm = decimeters
      auto maxdm = dev->GetMaxDistance();
      auto mindm = dev->GetMinDistance();
      if (dm <= maxdm && dm >= mindm) { // dst is in range
        auto delay = _minPrTime + _prTimeIncPerMeter * distance;
        auto totalTime = static_cast<uint64_t>(round(delay));
        auto errRate = minErrRate + errRateInc * distance;
        auto propagationError = dev->ErrOnNextPkt(errRate);
//        if (error) {
//          auto pBuffer = pkt->GetPayloadBuffer();
//          *pBuffer = ~*pBuffer;
//        }
        ns3::Simulator::ScheduleWithContext(
            dev->GetMac(), NanoSeconds(totalTime),
            &CustomROSCommsDevice::AddNewPacket, dst, pkt, propagationError);
      }
    }
  }
}
}
