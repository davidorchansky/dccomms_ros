#ifndef DCCOMMSROS_PACKETBUILDERFACTORYLOADER_H
#define DCCOMMSROS_PACKETBUILDERFACTORYLOADER_H

#include <string>
#include <class_loader/multi_library_class_loader.h>
#include <dccomms_ros/simulator/PacketBuilderLoader.h>
#include <dccomms/IPacketBuilder.h>

namespace dccomms_ros {

class PacketBuilderLoader {
public:
   static dccomms::PacketBuilderPtr LoadPacketBuilder(const std::string & libName, const std::string className);
private:
   //static class_loader::MultiLibraryClassLoader _loader;
};
}
#endif
