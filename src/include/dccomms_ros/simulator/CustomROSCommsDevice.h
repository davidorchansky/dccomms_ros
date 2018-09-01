#ifndef DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_

#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <exprtk.hpp>
#include <ns3/error-model.h>
#include <random>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

// enum DEV_STATUS { RECV, SEND, IDLE };
class CustomROSCommsDevice;
typedef ns3::Ptr<CustomROSCommsDevice> CustomROSCommsDeviceNs3Ptr;
// typedef dccomms::Ptr<CustomROSCommsDevice> CustomROSCommsDevicePtr;
typedef CustomROSCommsDevice *CustomROSCommsDevicePtr;

class IncomingPacket;
typedef dccomms::Ptr<IncomingPacket> IncomingPacketPtr;
class IncomingPacket {
public:
  bool propagationError;
  bool collisionError;
  ns3PacketPtr packet;
  IncomingPacket() {
    propagationError = false;
    collisionError = false;
    packet = NULL;
  }
  bool Error() { return propagationError || collisionError; }
};

class OutcomingPacket;
typedef dccomms::Ptr<OutcomingPacket> OutcomingPacketPtr;
class OutcomingPacket {
public:
  uint32_t packetSize;
  ns3PacketPtr packet;
  OutcomingPacket() { packet = NULL; }
};

class SimpleVarExprEval {
public:
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

  SimpleVarExprEval();
  void CompileExpr(const std::string &expr, const std::string &var);
  double ComputeVal(double var);

private:
  std::string _sexpr;
  symbol_table_t _symbol_table;
  expression_t _expression;
  double _var;
  parser_t _parser;
};

class CustomROSCommsDevice : public ROSCommsDevice {
public:
  CustomROSCommsDevice(ROSCommsSimulatorPtr, PacketBuilderPtr txpb,
                       PacketBuilderPtr rxpb);

  static CustomROSCommsDeviceNs3Ptr Build(ROSCommsSimulatorPtr sim,
                                       PacketBuilderPtr txpb,
                                       PacketBuilderPtr rxpb) {
    auto dev = ns3::CreateObject<CustomROSCommsDevice>(sim, txpb, rxpb);
    // auto dev = dccomms::CreateObject<CustomROSCommsDevice>(sim, txpb, rxpb);
    return dev;
    // return new CustomROSCommsDevice(sim, txpb, rxpb);
  }
  void SetJitter(double tx, double rx);
  void SetMinPktErrorRate(double minPktErrorRate);
  void SetPktErrorRateInc(double pktErrorRateInc);
  void SetMaxDistance(double d);
  void SetMinDistance(double d);
  void SetIntrinsicDelay(double d);

  void GetBitRate(double &bps); // as bps
  double GetMinPktErrorRate();
  double GetPktErrorRateInc();
  double GetMaxDistance();
  double GetMinDistance();
  double GetIntrinsicDelay();

  void PropagatePacket(ns3PacketPtr pkt);
  void TransmitPacket();
  void TransmitEnqueuedPacket();
  void StartPacketTransmission(const OutcomingPacketPtr &opkt);

  inline void SetTransmitting(bool v) { Transmitting(v); }
  bool ErrOnPkt(double range, ns3PacketPtr pkt);
  uint64_t GetNextTxJitter();
  uint64_t GetNextRxJitter();
  void SetRateErrorModel(const std::string &expr, const std::string &unit);
  void GetRateErrorModel(std::string &expr, std::string &unit);
  //  inline DEV_STATUS GetStatus() { return _status; }
  //  inline void SetStatus(DEV_STATUS status);

  virtual DEV_TYPE GetDevType();

  typedef std::normal_distribution<double> NormalRealDist;
  typedef std::uniform_int_distribution<int> UniformIntDist;
  typedef std::default_random_engine RandEngGen;
  typedef std::uniform_real_distribution<double> UniformRealDist;

  inline void EnqueueTxPacket(const OutcomingPacketPtr &opkt);
  inline bool TxFifoEmpty();
  OutcomingPacketPtr PopTxPacket();

  void AddNewPacket(ns3PacketPtr pkt, bool propagationError);
  void HandleNextIncomingPacket();

  void MarkIncommingPacketsAsCollisioned();
  bool Transmitting();
  void Transmitting(bool);

  bool Receiving();
  void Receiving(bool);

  static ns3::TypeId GetTypeId(void);
  void SchedulePacketTransmissionAfterJitter(const ns3PacketPtr & pkt);
  void ReceivePacketAfterJitter();

protected:
  virtual void DoSetMac(uint32_t mac);
  virtual void DoSend(ns3PacketPtr dlf);
  virtual void DoLinkToChannel(CommsChannelNs3Ptr channel,
                               CHANNEL_LINK_TYPE linkType);
  virtual void DoStart();
  virtual void DoSetPosition(const tf::Vector3 &position);
  virtual bool DoStarted();
  virtual void DoSetMaxTxFifoSize(uint32_t size);
  std::string DoToString();

private:
  uint32_t _mac;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;
  double _intrinsicDelay; // ms

  double _maxDistance, _minDistance; // in meters
  tf::Vector3 _position;
  // CustomROSCommsDevicePtr _ownPtr;

  NormalRealDist _ttDist;
  UniformRealDist _erDist;
  UniformIntDist _txJitterDist, _rxJitterDist;
  RandEngGen _ttGenerator, _erGenerator, _txJitterGenerator, _rxJitterGenerator;

  std::list<IncomingPacketPtr> _incomingPackets, _rxJitteredPackets;
  std::list<OutcomingPacketPtr> _outcomingPackets, _txJitteredPackets;


  CommsChannelNs3Ptr _txChannel, _rxChannel;
  // DEV_STATUS _status;
  bool _transmitting, _receiving;
  ns3::Ptr<ns3::RateErrorModel> _rem;

  double _GetErrorRate(double meters);
  std::string _eexpr;
  SimpleVarExprEval _mExprEval;
  double _txJitter, _rxJitter;
};
}
#endif // COMMSNODE_H
