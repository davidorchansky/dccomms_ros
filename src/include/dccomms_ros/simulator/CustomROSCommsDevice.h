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
typedef ns3::Ptr<CustomROSCommsDevice> CustomROSCommsDevicePtr;

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

  void SetVariableBitRate(double trTimeMean, double trTimeSd = 0); // as bps
  void SetMinPktErrorRate(double minPktErrorRate);
  void SetPktErrorRateInc(double pktErrorRateInc);
  void SetMaxDistance(double d);
  void SetMinDistance(double d);
  void SetIntrinsicDelay(double d);

  void GetBitRate(double &trTimeMean, double &trTimeSd); // as bps
  double GetMinPktErrorRate();
  double GetPktErrorRateInc();
  double GetMaxDistance();
  double GetMinDistance();
  double GetIntrinsicDelay();

  void PropagatePacket(ns3PacketPtr pkt);
  void TransmitPacket(ns3PacketPtr pkt);
  void TransmitEnqueuedPacket();
  void BeginPacketTransmission(const ns3PacketPtr & pkt, const uint32_t & pktSize);

  inline void SetTransmitting(bool v) { Transmitting(v); }
  bool ErrOnPkt(double range, ns3PacketPtr pkt);
  uint64_t GetNextTt(); // get next ms/byte
  void SetRateErrorModel(const std::string &expr, const std::string &unit);
  void GetRateErrorModel(std::string &expr, std::string &unit);
  //  inline DEV_STATUS GetStatus() { return _status; }
  //  inline void SetStatus(DEV_STATUS status);

  virtual DEV_TYPE GetDevType();

  typedef std::normal_distribution<double> NormalDist;
  typedef std::default_random_engine RandEngGen;
  typedef std::uniform_real_distribution<double> UniformRealDist;

  inline void EnqueueTxPacket(ns3PacketPtr pkt, uint32_t size);
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

protected:
  virtual void DoSetMac(uint32_t mac);
  virtual void DoSend(ns3PacketPtr dlf);
  virtual void DoLinkToChannel(CommsChannelPtr channel,
                               CHANNEL_LINK_TYPE linkType);
  virtual void DoStart();
  virtual void DoSetPosition(const tf::Vector3 &position);
  virtual bool DoStarted();
  virtual void DoSetMaxTxFifoSize(uint32_t size);
  std::string DoToString();

private:
  uint32_t _mac;
  double _bitRateMean, _bitRateSd;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;
  double _intrinsicDelay; // ms

  double _maxDistance, _minDistance; // in meters
  tf::Vector3 _position;
  // CustomROSCommsDevicePtr _ownPtr;

  NormalDist _ttDist;
  UniformRealDist _erDist;
  RandEngGen _ttGenerator, _erGenerator;

  std::list<IncomingPacketPtr> _incomingPackets;
  std::list<OutcomingPacketPtr> _outcomingPackets;

  CommsChannelPtr _txChannel, _rxChannel;
  // DEV_STATUS _status;
  bool _transmitting, _receiving;
  ns3::Ptr<ns3::RateErrorModel> _rem;

  double _GetErrorRate(double meters);
  std::string _eexpr;
  SimpleVarExprEval _mExprEval;
};
}
#endif // COMMSNODE_H
