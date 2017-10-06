#ifndef TESTING_COMMS_UTILS_HPP
#define TESTING_COMMS_UTILS_HPP

#include <dccomms/dccomms.h>

using namespace dccomms;
namespace teleop1 {
class OperatorPacket : public Packet {
public:
  OperatorPacket();
  void CopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(IStream *comms);

  bool PacketIsOk();

private:
  uint8_t *_order;
  int _packetSize;
  void _Init();
};

OperatorPacket::OperatorPacket() {
  _AllocBuffer(1);
  _Init();
}

void OperatorPacket::_Init() {
  _order = GetBuffer();
  _packetSize = 1;
}

void OperatorPacket::CopyFromRawBuffer(void *buffer) {
  _SetBuffer(buffer);
  _Init();
}

inline uint8_t *OperatorPacket::GetPayloadBuffer() { return _order; }

inline uint32_t OperatorPacket::GetPayloadSize() { return _packetSize; }

inline int OperatorPacket::GetPacketSize() { return _packetSize; }

void OperatorPacket::Read(IStream *stream) {
  stream->Read(_order, _packetSize);
}

bool OperatorPacket::PacketIsOk() { return true; }

class ROVPacket : public Packet {
public:
  ROVPacket();
  void CopyFromRawBuffer(void *buffer);
  uint8_t *GetPayloadBuffer();
  uint32_t GetPayloadSize();
  int GetPacketSize();
  void Read(IStream *comms);

  bool PacketIsOk();

  void GetPayload(void *copy);
  void SetPayload(void *data, int size = PAYLOAD_SIZE);

private:
  static const int PRE_SIZE = 1, PAYLOAD_SIZE = 20, FCS_SIZE = 2;

  uint8_t *_pre;
  uint8_t *_payload;
  uint8_t *_fcs;
  int _packetSize;
  void _Init();
  void _SetFCS();
  bool _CheckFCS();
};

ROVPacket::ROVPacket() {
  _packetSize = PRE_SIZE + PAYLOAD_SIZE + FCS_SIZE;
  _AllocBuffer(_packetSize);
  _Init();
}

void ROVPacket::_Init() {
  _pre = GetBuffer();
  *_pre = 0x55;
  _payload = _pre + PRE_SIZE;
  _fcs = _payload + PAYLOAD_SIZE;
}

void ROVPacket::CopyFromRawBuffer(void *buffer) {
  _SetBuffer(buffer);
  _Init();
}

inline uint8_t *ROVPacket::GetPayloadBuffer() { return _payload; }

inline uint32_t ROVPacket::GetPayloadSize() { return PAYLOAD_SIZE; }

inline int ROVPacket::GetPacketSize() { return _packetSize; }

void ROVPacket::Read(IStream *stream) {
  stream->WaitFor(_pre, PRE_SIZE);
  stream->Read(_payload, PAYLOAD_SIZE + FCS_SIZE);
}

void ROVPacket::GetPayload(void *copy) { memcpy(copy, _payload, PAYLOAD_SIZE); }

void ROVPacket::SetPayload(void *data, int size) {
  memset(_payload, 0, PAYLOAD_SIZE);
  memcpy(_payload, data, size);
}

void ROVPacket::_SetFCS() {
  uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE);
  *_fcs = (uint8_t)(crc >> 8);
  *(_fcs + 1) = (uint8_t)(crc & 0xff);
}

bool ROVPacket::_CheckFCS() {
  uint16_t crc = Checksum::crc16(_payload, PAYLOAD_SIZE + FCS_SIZE);
  return crc == 0;
}
bool ROVPacket::PacketIsOk() { return _CheckFCS(); }

class ROVPacketBuilder : public IPacketBuilder {
public:
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<ROVPacket>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create(){
      return CreateObject<ROVPacket>();
  }
};

class OperatorPacketBuilder : public IPacketBuilder {
public:
  PacketPtr CreateFromBuffer(void *buffer) {
    auto pkt = CreateObject<OperatorPacket>();
    pkt->CopyFromRawBuffer(buffer);
    return pkt;
  }
  PacketPtr Create(){
      return CreateObject<OperatorPacket>();
  }
};
}
#endif
