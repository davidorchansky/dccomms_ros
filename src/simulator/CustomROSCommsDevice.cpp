#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

namespace dccomms_ros {

CustomROSCommsDevice::CustomROSCommsDevice(ROSCommsSimulatorPtr sim,
                                           PacketBuilderPtr txpb,
                                           PacketBuilderPtr rxpb)
    : ROSCommsDevice(sim, txpb, rxpb), _ownPtr(this), _erDist(0.0, 1.0) {
  SetStatus(IDLE);
}

DEV_TYPE CustomROSCommsDevice::GetDevType() { return DEV_TYPE::CUSTOM_DEV; }

void CustomROSCommsDevice::SetVariableBitRate(double mean, double sd) {
  _bitRateMean = mean;
  _bitRateSd = sd;
  auto _ttMean = 8 * 1e9 / _bitRateMean; // nanos / byte
  auto _ttSd = _bitRateSd > 0 ? 8 * 1e9 / _bitRateSd : 0;
  _ttDist = NormalDist(_ttMean, _ttSd);
}

void CustomROSCommsDevice::GetBitRate(double &mean, double &sd) {
  mean = _bitRateMean;
  sd = _bitRateSd;
}

void CustomROSCommsDevice::SetMinPktErrorRate(double minPktErrorRate) {
  _minPktErrorRate = minPktErrorRate;
}

double CustomROSCommsDevice::GetMinPktErrorRate() { return _minPktErrorRate; }

void CustomROSCommsDevice::SetPktErrorRateInc(double inc) {
  _pktErrorRateIncPerMeter = inc;
}

double CustomROSCommsDevice::GetPktErrorRateInc() {
  return _pktErrorRateIncPerMeter;
}

uint64_t CustomROSCommsDevice::GetNextTt() {
  auto tt = _ttDist(_ttGenerator);
  if (tt < 0) {
    Log->warn("trRate < 0: {} . Changing to its abs({}) value", tt, tt);
    tt = -tt;
  }
  return tt;
}

bool CustomROSCommsDevice::ErrOnNextPkt(double errRate) {
  auto rand = _erDist(_erGenerator);
  return rand < errRate;
}

void CustomROSCommsDevice::SetMaxDistance(uint32_t d) { _maxDistance = d; }

uint32_t CustomROSCommsDevice::GetMaxDistance() { return _maxDistance; }

void CustomROSCommsDevice::SetMinDistance(uint32_t d) { _minDistance = d; }

uint32_t CustomROSCommsDevice::GetMinDistance() { return _minDistance; }

inline void CustomROSCommsDevice::EnqueueTxPacket(PacketPtr pkt) {
  _txFifo.push(pkt);
}
PacketPtr CustomROSCommsDevice::PopTxPacket() {
  auto pkt = _txFifo.front();
  _txFifo.pop();
  return pkt;
}
inline bool CustomROSCommsDevice::TxFifoEmpty() { return _txFifo.empty(); }

void CustomROSCommsDevice::MarkIncommingPacketsAsCollisioned() {
  // Mark all packets invalid (collisioned)
  for (auto ipkt : _incommingPackets) {
    ipkt->collisionError = true;
  }
}

void CustomROSCommsDevice::HandleNextIncommingPacket() {
  if (!_incommingPackets.empty()) {
    IncommingPacketPtr ptr = _incommingPackets.front();
    _incommingPackets.pop_front();
    if (ptr->Error()) {
      // TODO: increase packet errors counter (traced value)
      if (ptr->collisionError) {
        // TODO: increase collision errors counter (traced value)
      }
      if (ptr->propagationError) {
        // TODO: increase propagation errors counter (traced value)
      }
    } else {
      ReceiveFrame(ptr->packet);
    }
  } else {
    Critical("internal error: incomming packets queue empty when "
             "HandleNextIncommingPacket!");
  }
}

void CustomROSCommsDevice::AddNewPacket(PacketPtr pkt, bool propagationError) {
  IncommingPacketPtr ipkt = dccomms::CreateObject<IncommingPacket>();
  ipkt->propagationError = propagationError;
  _incommingPackets.push_back(ipkt);

  if (_txChannel == _rxChannel && GetStatus() != SEND)
    ipkt->collisionError = true;

  MarkIncommingPacketsAsCollisioned();

  auto pktSize = pkt->GetPacketSize();
  auto byteTrt = GetNanosPerByte();
  auto trTime = pktSize * byteTrt;
  ns3::Simulator::ScheduleWithContext(
      GetMac(), ns3::NanoSeconds(trTime),
      &CustomROSCommsDevice::HandleNextIncommingPacket, this);
}

void CustomROSCommsDevice::SetStatus(DEV_STATUS status) {
  _status = status;
  if (!_txFifo.empty()) {
    auto pkt = PopTxPacket();
    TransmitPacket(pkt);
  }
}

void CustomROSCommsDevice::PropagateNextPacket() {

  if (!TxFifoEmpty()) {
    auto packet = PopTxPacket();
    static_pointer_cast<CustomCommsChannel>(_txChannel)
        ->SendPacket(_ownPtr, packet);
    if (TxFifoEmpty())
      SetStatus(IDLE);
  } else
    Critical("internal error: TX fifo emtpy when calling TransmitNextPacket");
}

void CustomROSCommsDevice::TransmitPacket(PacketPtr pkt) {
  if (_rxChannel != _txChannel || GetStatus() != RECV) {
    auto pktSize = pkt->GetPacketSize();
    auto byteTrt =
        GetNextTt(); // It's like GetNanosPerByte but with a variation
    auto trTime = pktSize * byteTrt;
    SetStatus(SEND);
    ns3::Simulator::ScheduleWithContext(
        GetMac(), ns3::NanoSeconds(trTime),
        &CustomROSCommsDevice::PropagateNextPacket, this);
  } else {
    EnqueueTxPacket(pkt);
  }
}

void CustomROSCommsDevice::DoSetMac(uint32_t mac) { _mac = mac; }
void CustomROSCommsDevice::DoSend(PacketPtr dlf) {
  ns3::Simulator::ScheduleWithContext(GetMac(), ns3::NanoSeconds(0),
                                      &CustomROSCommsDevice::TransmitPacket,
                                      this, dlf);
}
void CustomROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel,
                                           CHANNEL_LINK_TYPE linkType) {
  if (channel->GetType() == CHANNEL_TYPE::CUSTOM_CHANNEL) {
    //_channel = static_pointer_cast<CustomCommsChannel>(channel);

    if (linkType == CHANNEL_TX)
      _txChannel = channel;
    else if (linkType == CHANNEL_RX) {
      _rxChannel = channel;
      static_pointer_cast<CustomCommsChannel>(_rxChannel)->AddDevice(_ownPtr);
    } else if (linkType == CHANNEL_TXRX) {
      _txChannel = channel;
      _rxChannel = channel;
      static_pointer_cast<CustomCommsChannel>(_rxChannel)->AddDevice(_ownPtr);
    }

  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}
void CustomROSCommsDevice::DoStart() {}
bool CustomROSCommsDevice::DoStarted() { return true; }
void CustomROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  _position = position;
}
}
