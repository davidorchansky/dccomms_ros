#include <dccomms_ros/simulator/CustomCommsChannel.h>
#include <dccomms_ros/simulator/CustomROSCommsDevice.h>
#include <ns3/core-module.h>
#include <ns3/simulator.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE(
    "CustomROSCommsDevice"); // NS3 LOG DOES NOT WORK HERE (TODO: FIX IT)
namespace dccomms_ros {

NS_OBJECT_ENSURE_REGISTERED(CustomROSCommsDevice);

TypeId CustomROSCommsDevice::GetTypeId(void) {
  static TypeId tid =
      TypeId("dccomms_ros::CustomROSCommsDevice").SetParent<ROSCommsDevice>();

  return tid;
}

CustomROSCommsDevice::CustomROSCommsDevice(ROSCommsSimulatorPtr sim,
                                           PacketBuilderPtr txpb,
                                           PacketBuilderPtr rxpb)
    : ROSCommsDevice(sim, txpb, rxpb), _erDist(0.0, 1.0) {
  Transmitting(false);
  Receiving(false);
  LogComponentEnable("CustomROSCommsDevice",
                     LOG_LEVEL_ALL); // NS3 DOES NOT WORK (TODO: FIX IT)
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

void SimpleVarExprEval::CompileExpr(const string &expr,
                                    const std::string &var) {
  _sexpr = expr;
  _symbol_table.add_variable(var, _var);
  _symbol_table.add_constants();
  _expression.register_symbol_table(_symbol_table);
  _parser.compile(_sexpr, _expression);
}

SimpleVarExprEval::SimpleVarExprEval() {}
double SimpleVarExprEval::ComputeVal(double var) {
  _var = var;
  return _expression.value();
}

double CustomROSCommsDevice::_GetErrorRate(double meters) {
  return _mExprEval.ComputeVal(meters);
}
void CustomROSCommsDevice::GetRateErrorModel(std::string &expr,
                                             std::string &unit) {
  RateErrorModel::ErrorUnit eunit = _rem->GetUnit();
  switch (eunit) {
  case RateErrorModel::ERROR_UNIT_BIT:
    unit = "ERROR_UNIT_BIT";
    break;
  case RateErrorModel::ERROR_UNIT_BYTE:
    unit = "ERROR_UNIT_BYTE";
    break;
  default:
    unit = "ERROR_UNIT_PACKET";
  }
  expr = _eexpr;
}

void CustomROSCommsDevice::SetRateErrorModel(const std::string &expr,
                                             const std::string &unit) {
  ns3::Ptr<UniformRandomVariable> uv = CreateObject<UniformRandomVariable>();
  // Set this variable to a specific stream
  uv->SetStream(50);

  _rem = CreateObject<RateErrorModel>();
  _rem->SetRandomVariable(uv);
  if (unit == "bit")
    _rem->SetUnit(RateErrorModel::ERROR_UNIT_BIT);
  else if (unit == "byte")
    _rem->SetUnit(RateErrorModel::ERROR_UNIT_BYTE);
  else
    _rem->SetUnit(RateErrorModel::ERROR_UNIT_PACKET);

  _rem->Enable();
  if (expr == "")
    _eexpr = "0.01*m";
  else
    _eexpr = expr;

  Debug("SetRateErrorModel: expression = {}", _eexpr);
  _mExprEval.CompileExpr(_eexpr, "m");
}

bool CustomROSCommsDevice::ErrOnPkt(double range, ns3PacketPtr pkt) {
  double rate = _GetErrorRate(range);
  Debug("ErrOnPkt: range = {} meters --> rate = {}", range, rate);
  _rem->SetRate(rate);
  return _rem->IsCorrupt(pkt);
}

void CustomROSCommsDevice::SetMaxDistance(double d) { _maxDistance = d; }

double CustomROSCommsDevice::GetMaxDistance() { return _maxDistance; }

void CustomROSCommsDevice::SetMinDistance(double d) { _minDistance = d; }

double CustomROSCommsDevice::GetMinDistance() { return _minDistance; }

void CustomROSCommsDevice::SetIntrinsicDelay(double d) { _intrinsicDelay = d; }

double CustomROSCommsDevice::GetIntrinsicDelay() { return _intrinsicDelay; }

inline void CustomROSCommsDevice::EnqueueTxPacket(ns3PacketPtr pkt) {
  _txFifo.push(pkt);
}
ns3PacketPtr CustomROSCommsDevice::PopTxPacket() {
  auto pkt = _txFifo.front();
  _txFifo.pop();
  return pkt;
}
inline bool CustomROSCommsDevice::TxFifoEmpty() { return _txFifo.empty(); }

void CustomROSCommsDevice::MarkIncommingPacketsAsCollisioned() {
  // Mark all packets invalid (collisioned)
  Debug("CustomROSCommsDevice({}): MarkIncommingPacketsAsCollisioned",
        GetDccommsId());
  for (auto ipkt : _incommingPackets) {
    ipkt->collisionError = true;
  }
}

void CustomROSCommsDevice::HandleNextIncommingPacket() {
  NS_LOG_FUNCTION(this);
  Debug("CustomROSCommsDevice({}): HandleNextIncommingPacket", GetDccommsId());
  if (!_incommingPackets.empty()) {
    IncommingPacketPtr ptr = _incommingPackets.front();
    _incommingPackets.pop_front();
    if (ptr->Error()) {
      // TODO: increase packet errors counter (traced value)
      if (ptr->collisionError) {
        // TODO: increase collision errors counter (traced value)
        Debug("CustomROSCommsDevice({}): Collision!", GetDccommsId());
        _collisionCbTrace(this, ptr->packet);
      }
      if (ptr->propagationError) {
        // TODO: increase propagation errors counter (traced value)
        Debug("CustomROSCommsDevice({}): Propagation error!", GetDccommsId());
        _propErrorCbTrace(this, ptr->packet);
      }
    } else {
      ReceiveFrame(ptr->packet);
    }
    Receiving(false);
  } else {
    Critical("internal error: incomming packets queue empty when "
             "HandleNextIncommingPacket!");
  }
}

void CustomROSCommsDevice::AddNewPacket(ns3PacketPtr pkt,
                                        bool propagationError) {
  NS_LOG_FUNCTION(this);
  Debug("CustomROSCommsDevice({}): AddNewPacket", GetDccommsId());
  IncommingPacketPtr ipkt = dccomms::CreateObject<IncommingPacket>();
  ipkt->propagationError = propagationError;
  NetsimHeader header;
  pkt->PeekHeader(header);
  // TODO: check if propagation error and increase traced value
  if (Receiving() || _txChannel == _rxChannel && Transmitting()) {
    // TODO: increase colission errors traced value
    MarkIncommingPacketsAsCollisioned(); // Should be a maximum of 1 packet in
                                         // the _incommingPackets queue
  } else {
    Receiving(true);
    ipkt->packet = pkt;
    _incommingPackets.push_back(ipkt);

    auto pktSize = header.GetPacketSize();
    auto byteTrt = GetNanosPerByte();
    auto trTime = pktSize * byteTrt;
    Debug("CustomROSCommsDevice({}): Receiving packet: size({} bytes) ; "
          "rcTime({} "
          "secs)",
          GetDccommsId(), pktSize, trTime / 1e9);
    ns3::Simulator::ScheduleWithContext(
        GetMac(), ns3::NanoSeconds(trTime),
        &CustomROSCommsDevice::HandleNextIncommingPacket, this);
  }
}

bool CustomROSCommsDevice::Transmitting() { return _transmitting; }

void CustomROSCommsDevice::Transmitting(bool transmitting) {
  Debug("CustomROSCommsDevice({}): Setting transmitting status: {}",
        GetDccommsId(), transmitting);
  _transmitting = transmitting;
  if (!TxFifoEmpty()) {
    if (!_transmitting) {
      auto pkt = PopTxPacket();
      TransmitPacket(pkt);
    } else {
      Critical("internal error: TX fifo not empty when calling SetStatus with "
               "not SEND");
    }
  }
}

bool CustomROSCommsDevice::Receiving() { return _receiving; }

void CustomROSCommsDevice::Receiving(bool receiving) {
  _receiving = receiving;
  NS_LOG_DEBUG("Receiving: " << receiving ? "TRUE" : "FALSE");
  Debug("CustomROSCommsDevice({}): Setting receiving state to {}",
        GetDccommsId(), receiving);
}

void CustomROSCommsDevice::PropagateNextPacket() {
  NS_LOG_FUNCTION(this);
  if (!TxFifoEmpty()) {
    auto packet = PopTxPacket();
    static_cast<CustomCommsChannel *>(ns3::PeekPointer(_txChannel))
        ->SendPacket(this, packet);
    Transmitting(false);
  } else
    Critical("internal error: TX fifo emtpy when calling TransmitNextPacket");
}

void CustomROSCommsDevice::PropagatePacket(ns3PacketPtr pkt) {
  static_cast<CustomCommsChannel *>(ns3::PeekPointer(_txChannel))
      ->SendPacket(this, pkt);
}

void CustomROSCommsDevice::TransmitPacket(ns3PacketPtr pkt) {
  NS_LOG_FUNCTION(this);
  Debug("CustomROSCommsDevice: Transmit packet");
  NetsimHeader header;
  pkt->PeekHeader(header);
  if (_rxChannel != _txChannel || _rxChannel == _txChannel && !Receiving()) {
    auto pktSize = header.GetPacketSize();
    auto byteTrt =
        GetNextTt(); // It's like GetNanosPerByte but with a variation
    auto trTime = pktSize * byteTrt;
    Transmitting(true);
    Debug("CustomROSCommsDevice({}): Transmitting packet: size({} bytes) ; "
          "trTime({} secs)",
          GetDccommsId(), pktSize, trTime / 1e9);
    ns3::Simulator::ScheduleWithContext(GetMac(), ns3::NanoSeconds(trTime),
                                        &CustomROSCommsDevice::SetTransmitting,
                                        this, false);
    PropagatePacket(pkt);
  } else {
    Debug("CustomROSCommsDevice({}): Enqueue packet", GetDccommsId());
    EnqueueTxPacket(pkt);
  }
}

void CustomROSCommsDevice::DoSetMac(uint32_t mac) { _mac = mac; }
void CustomROSCommsDevice::DoSend(ns3PacketPtr dlf) {
  ns3::Simulator::ScheduleWithContext(GetMac(), ns3::NanoSeconds(_intrinsicDelay*1e6),
                                      &CustomROSCommsDevice::TransmitPacket,
                                      this, dlf);
}
void CustomROSCommsDevice::DoLinkToChannel(CommsChannelPtr channel,
                                           CHANNEL_LINK_TYPE linkType) {
  //  if (!_ownPtr)
  //    _ownPtr =
  //        this; //
  //        std::dynamic_pointer_cast<CustomROSCommsDevice>(ROSCommsDevice::shared_from_this());//https://stackoverflow.com/questions/16082785/use-of-enable-shared-from-this-with-multiple-inheritance
  if (channel->GetType() == CHANNEL_TYPE::CUSTOM_CHANNEL) {
    //_channel = static_pointer_cast<CustomCommsChannel>(channel);
    if (linkType == CHANNEL_TX)
      _txChannel = channel;
    else if (linkType == CHANNEL_RX) {
      _rxChannel = channel;
      static_cast<CustomCommsChannel *>(ns3::PeekPointer(_rxChannel))
          ->AddDevice(this);
    } else if (linkType == CHANNEL_TXRX) {
      _txChannel = channel;
      _rxChannel = channel;
      static_cast<CustomCommsChannel *>(ns3::PeekPointer(_rxChannel))
          ->AddDevice(this);
    }

  } else {
    Log->critical(
        "internal error: attempted to link device to a wrong channel type");
  }
}
void CustomROSCommsDevice::DoStart() {
  //  if (!_ownPtr)
  //    _ownPtr =
  //        this; //
  //        std::dynamic_pointer_cast<CustomROSCommsDevice>(ROSCommsDevice::shared_from_this());//https://stackoverflow.com/questions/16082785/use-of-enable-shared-from-this-with-multiple-inheritance
}
bool CustomROSCommsDevice::DoStarted() { return true; }
void CustomROSCommsDevice::DoSetPosition(const tf::Vector3 &position) {
  _position = position;
}

std::string CustomROSCommsDevice::DoToString() {
  int maxBuffSize = 2048;
  char buff[maxBuffSize];
  string txChannelLinked;
  if (_txChannel)
    txChannelLinked = "Type: " + ChannelType2String(_txChannel->GetType()) +
                      " ; Id: " + to_string(_txChannel->GetId());
  else
    txChannelLinked = "not linked";

  string rxChannelLinked;
  if (_rxChannel)
    rxChannelLinked = "Type: " + ChannelType2String(_rxChannel->GetType()) +
                      " ; Id: " + to_string(_rxChannel->GetId());
  else
    rxChannelLinked = "not linked";

  double bitrateD, bitrateSd;
  GetBitRate(bitrateD, bitrateSd);
  uint32_t bitrate = (uint32_t)bitrateD;
  std::string expr, eunit;
  GetRateErrorModel(expr, eunit);

  int n;
  n = snprintf(buff, maxBuffSize, "\tdccomms ID: ............... '%s'\n"
                                  "\tMAC ....................... %d\n"
                                  "\tDevice type ............... %s\n"
                                  "\tFrame ID: ................. '%s'\n"
                                  "\tTX channel: ............... '%s'\n"
                                  "\tRX channel: ............... '%s'\n"
                                  "\tMax. distance: ............ %.2f m\n"
                                  "\tMin. distance: ............ %.2f m\n"
                                  "\tBitrate: .................. %d bps\n"
                                  "\tBitrate SD: ............... %.3f\n"
                                  "\tTx Fifo Size: ............. %d bytes\n"
                                  "\tError Expression: ......... %s\n"
                                  "\tError Unit: ............... %s",
               _name.c_str(), _mac, DevType2String(GetDevType()).c_str(),
               _tfFrameId.c_str(), txChannelLinked.c_str(),
               rxChannelLinked.c_str(), _maxDistance, _minDistance, bitrate,
               bitrateSd, GetMaxTxFifoSize(), expr.c_str(), eunit.c_str());

  return std::string(buff);
}
}
