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

using namespace std;
using namespace dccomms;
using namespace dccomms_utils;
CommsBridge *comms;
S100Stream *stream;

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

  std::string modemPort;
  if (!nh.getParam("modemPort", modemPort)) {
    ROS_ERROR("Failed to get param 'modemPort'");
    return 1;
  } else {
    ROS_INFO("modem port: %s", modemPort.c_str());
  }

  int modemBaudrate;
  if (!nh.getParam("modemBaudrate", modemBaudrate)) {
    ROS_ERROR("Failed to get param 'modemBaudrate'");
    return 1;
  } else {
    ROS_INFO("modem baudrate: %d", modemBaudrate);
  }

  std::string ns;
  if (!nh.getParam("ns", ns)) {
    ROS_ERROR("Failed to get param 'ns'");
    return 1;
  } else {
    ROS_INFO("namespace: %s", ns.c_str());
  }

  std::string logToFile;
  bool logToFileEnabled;
  if (!nh.getParam("logToFile", logToFile)) {
    ROS_ERROR("Failed to get param 'logToFile'");
    return 1;
  } else {
    if (logToFile == "") {
      ROS_INFO("Do not log to file");
      logToFileEnabled = false;
    } else {
      ROS_INFO("logging to file: %s", logToFile.c_str());
      logToFileEnabled = true;
    }
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
  if (logToFileEnabled)
    comms->LogToFile(logToFile);

  stream->FlushLogOn(cpplogging::LogLevel::info);
  if (logToFileEnabled)
    stream->LogToFile("device_" + logToFile);

  comms->Start();
  while (1) {
    Utils::Sleep(10000);
  }
}
