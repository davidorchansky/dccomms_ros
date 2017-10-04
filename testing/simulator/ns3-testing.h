#pragma once

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <spdlog/tweakme.h>
#include <string>

namespace pruebas {

namespace spd = std;

enum LogLevel { critical, debug, err, info, off, trace, warn };

class Pruebas {
public:
  Pruebas(std::string name = "asd");

  virtual ~Pruebas();

  virtual void LogToFile(const std::string &filename);
  virtual void LogToConsole(bool);
  virtual void SetLogLevel(LogLevel);
  virtual std::string GetLogName() { return LogName; }
  virtual void FlushLogOn(LogLevel);
  virtual void FlushLog();

protected:
  std::shared_ptr<std::string> dist_sink;
  std::string LogName;
};

} /* namespace cpplogging */
