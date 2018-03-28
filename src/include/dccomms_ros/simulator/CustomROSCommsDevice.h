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

class IncommingPacket;
typedef dccomms::Ptr<IncommingPacket> IncommingPacketPtr;
class IncommingPacket {
public:
  bool propagationError;
  bool collisionError;
  ns3PacketPtr packet;
  IncommingPacket() {
    propagationError = false;
    collisionError = false;
    packet = NULL;
  }
  bool Error() { return propagationError || collisionError; }
};

class SimpleVarExprEval {
public:
  typedef exprtk::symbol_table<double> symbol_table_t;
  typedef exprtk::expression<double> expression_t;
  typedef exprtk::parser<double> parser_t;

  SimpleVarExprEval();
  void CompileExpr(const std::string & expr, const std::string &var);
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
  void SetMaxDistance(uint32_t d);
  void SetMinDistance(uint32_t d);

  void GetBitRate(double &trTimeMean, double &trTimeSd); // as bps
  double GetMinPktErrorRate();
  double GetPktErrorRateInc();
  uint32_t GetMaxDistance();
  uint32_t GetMinDistance();

  void PropagateNextPacket();
  void PropagatePacket(ns3PacketPtr pkt);
  void TransmitPacket(ns3PacketPtr pkt);
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

  inline void EnqueueTxPacket(ns3PacketPtr pkt);
  inline bool TxFifoEmpty();
  ns3PacketPtr PopTxPacket();

  void AddNewPacket(ns3PacketPtr pkt, bool propagationError);
  void HandleNextIncommingPacket();

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
  std::string DoToString();

private:
  uint32_t _mac;
  double _bitRateMean, _bitRateSd;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;

  uint32_t _maxDistance, _minDistance; // in dm
  tf::Vector3 _position;
  //CustomROSCommsDevicePtr _ownPtr;

  NormalDist _ttDist;
  UniformRealDist _erDist;
  RandEngGen _ttGenerator, _erGenerator;

  std::queue<ns3PacketPtr> _txFifo;
  std::list<IncommingPacketPtr> _incommingPackets;

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
