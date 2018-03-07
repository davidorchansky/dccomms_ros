#include <dccomms_ros/simulator/PacketBuilderLoader.h>

namespace dccomms_ros {

// class_loader::MultiLibraryClassLoader PacketBuilderLoader::_loader(true);

dccomms::PacketBuilderPtr
PacketBuilderLoader::LoadPacketBuilder(const std::string &libName,
                                       const std::string className) {
  class_loader::ClassLoader loader(libName);
  //_loader.loadLibrary(libName);
  std::vector<std::string> classes =
      loader.getAvailableClasses<dccomms::IPacketBuilder>();
  for (unsigned int c = 0; c < classes.size(); ++c) {
    if (classes[c] == className) {
      dccomms::IPacketBuilder *pb =
          loader.createUnmanagedInstance<dccomms::IPacketBuilder>(classes[c]);
      return dccomms::PacketBuilderPtr(pb);
    }
  }
}
}
