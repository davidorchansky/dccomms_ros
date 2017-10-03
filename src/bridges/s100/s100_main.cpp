/*
 * main.cpp
 *
 *  Created on: 22 oct. 2016
 *      Author: Diego Centelles Beltran
 */

#include <cstdio>
#include <dccomms/CommsBridge.h>
#include <dccomms/Utils.h>
#include <dccomms_utils/S100Stream.h>
#include <iostream>

#include <cstdio>
#include <signal.h>
#include <sys/types.h>

// ROS
#include "ros/ros.h"
// end ROS

#include <cpplogging/cpplogging.h>

using namespace std;
using namespace dccomms;
using namespace dccomms_utils;
using namespace cpplogging;
CommsBridge *comms;
S100Stream *stream;
LoggerPtr Log;

void SIGINT_handler(int sig) {
  printf("Received %d signal\nClosing device socket...\n", sig);
  printf("Device closed.\n");
  fflush(stdout);
  comms->FlushLog();
  stream->FlushLog();
  Utils::Sleep(2000);
  printf("Log messages flushed.\n");

  exit(0);
}

void setSignals() {
  if (signal(SIGINT, SIGINT_handler) == SIG_ERR) {
    printf("SIGINT install error\n");
    exit(1);
  }
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "dccomms_S100_bridge");
  ros::NodeHandle nh("~");
  Log = CreateLogger("S100Bridge");
  Log->SetLogName(Log->GetLogName() + ":Main");
  Log->SetLogLevel(cpplogging::LogLevel::debug);

  std::string logPrefix;
  bool logToFileEnabled;
  if (!nh.getParam("logPrefix", logPrefix)) {
    Log->Error("Failed to get param 'logPrefix'");
    return 1;
  } else {
    if (logPrefix == "") {
      Log->Info("Do not log to file");
      logToFileEnabled = false;
    } else {
      Log->Info("Log prefix: {}", logPrefix);
      logToFileEnabled = true;
    }
  }

  if (logToFileEnabled) {
    Log->LogToFile(logPrefix + "_main");
  }

  std::string modemPort;
  if (!nh.getParam("modemPort", modemPort)) {
    Log->Error("Failed to get param 'modemPort'");
    return 1;
  } else {
    Log->Info("modem port: {}", modemPort);
  }

  int modemBaudrate;
  if (!nh.getParam("modemBaudrate", modemBaudrate)) {
    Log->Error("Failed to get param 'modemBaudrate'");
    return 1;
  } else {
    Log->Info("modem baudrate: {}", modemBaudrate);
  }

  std::string ns;
  if (!nh.getParam("ns", ns)) {
    Log->Error("Failed to get param 'ns'");
    return 1;
  } else {
    Log->Info("namespace: {}", ns);
  }

  setSignals();

  auto portBaudrate = SerialPortStream::BAUD_2400;
  stream = new S100Stream(modemPort, portBaudrate, modemBaudrate);
  comms = new CommsBridge(stream, 0, DataLinkFrame::fcsType::crc16);

  comms->SetLogLevel(cpplogging::LogLevel::debug);
  comms->SetNamespace(ns);
  comms->SetLogName("S100Bridge");
  stream->SetLogName(comms->GetLogName() + ":S100Stream");
  comms->FlushLogOn(cpplogging::LogLevel::info);
  stream->FlushLogOn(cpplogging::LogLevel::info);

  if (logToFileEnabled) {

    comms->LogToFile(logPrefix);
    stream->LogToFile(logPrefix + "_device");
  }
  stream->LogConfig();

  comms->Start();
  while (1) {
    Utils::Sleep(10000);
  }
}
