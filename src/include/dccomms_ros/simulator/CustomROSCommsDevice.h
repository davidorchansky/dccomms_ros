#ifndef DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_
#define DCCOMMS_ROS_CUSTOMROSCOMMSDEVICE_H_

#include <dccomms_ros/simulator/ROSCommsDevice.h>
#include <random>

using namespace dccomms;
using namespace cpplogging;

namespace dccomms_ros {

//enum DEV_STATUS { RECV, SEND, IDLE };
class CustomROSCommsDevice;
typedef ns3::Ptr<CustomROSCommsDevice> CustomROSCommsDevicePtr;

class IncommingPacket;
typedef dccomms::Ptr<IncommingPacket> IncommingPacketPtr;
class IncommingPacket {
public:
  bool propagationError;
  bool collisionError;
  PacketPtr packet;
  IncommingPacket() {
    propagationError = false;
    collisionError = false;
    packet = NULL;
  }
  bool Error() { return propagationError || collisionError; }
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
  void PropagatePacket(PacketPtr pkt);
  void TransmitPacket(PacketPtr pkt);
  bool ErrOnNextPkt(double errRate);
  uint64_t GetNextTt(); // get next ms/byte

//  inline DEV_STATUS GetStatus() { return _status; }
//  inline void SetStatus(DEV_STATUS status);

  virtual DEV_TYPE GetDevType();

  typedef std::normal_distribution<double> NormalDist;
  typedef std::default_random_engine RandEngGen;
  typedef std::uniform_real_distribution<double> UniformRealDist;

  inline void EnqueueTxPacket(PacketPtr pkt);
  inline bool TxFifoEmpty();
  PacketPtr PopTxPacket();

  void AddNewPacket(PacketPtr pkt, bool propagationError);
  void HandleNextIncommingPacket();

  void MarkIncommingPacketsAsCollisioned();
  bool Transmitting();
  void Transmitting(bool);

  bool Receiving();
  void Receiving(bool );
protected:
  virtual void DoSetMac(uint32_t mac);
  virtual void DoSend(PacketPtr dlf);
  virtual void DoLinkToChannel(CommsChannelPtr channel, CHANNEL_LINK_TYPE linkType);
  virtual void DoStart();
  virtual void DoSetPosition(const tf::Vector3 &position);
  virtual bool DoStarted();

private:
  uint32_t _mac;
  double _bitRateMean, _bitRateSd;
  double _minPktErrorRate, _pktErrorRateIncPerMeter;

  uint32_t _maxDistance, _minDistance; // in dm
  tf::Vector3 _position;
  CustomROSCommsDevicePtr _ownPtr;

  NormalDist _ttDist;
  UniformRealDist _erDist;
  RandEngGen _ttGenerator, _erGenerator;

  std::queue<PacketPtr> _txFifo;
  std::list<IncommingPacketPtr> _incommingPackets;

  CommsChannelPtr _txChannel, _rxChannel;
  //DEV_STATUS _status;
  bool _transmitting, _receiving;
};
}
#endif // COMMSNODE_H
