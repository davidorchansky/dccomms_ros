#include <dccomms_ros/ROSCommsNode.h>
#include <dccomms_ros/ROSCommsSimulator.h>

namespace dccomms_ros
{
ROSCommsDevice::ROSCommsDevice(ROSCommsSimulator *s, CommsDeviceServicePtr d) :
    _sim(s),
    _device(d),
    _txserv(this)
{
    _txdlf = DataLinkFrame::BuildDataLinkFrame (DataLinkFrame::crc16);
    _txserv.SetWork (&ROSCommsDevice::_TxWork);
    _name = "node";
    SetLogName (_name);
    _device->LogToConsole (false);
}
void ROSCommsDevice::StartDeviceService ()
{
    _device->Start ();
    _device->SetPhyLayerState(CommsDeviceService::READY);
}

void ROSCommsDevice::StartNodeWorker ()
{
    _txserv.Start ();
}

void ROSCommsDevice::ReceiveFrame (DataLinkFramePtr dlf)
{
    _receiveFrameMutex.lock();
    *_device << dlf;
    _receiveFrameMutex.unlock();
}

void ROSCommsDevice::SetName(const std::string name)
{
    _name = name;
}

std::string ROSCommsDevice::GetName()
{
    return _name;
}

void ROSCommsDevice::SetMaxBitRate(int mbr)
{
    _maxBitRate = mbr;
}

int  ROSCommsDevice::GetMaxBitRate ()
{
    return _maxBitRate;
}

void ROSCommsDevice::SetTrTime(float mean, float sd)
{
    _trTimeMean = mean;
    _trTimeSd = sd;
}

void ROSCommsDevice::GetTrTime(float &mean, float &sd)
{
    mean = _trTimeMean;
    sd = _trTimeSd;
}

void ROSCommsDevice::SetMinPrTime(float prTime)
{
    _minPrTime = prTime;
}

float ROSCommsDevice::GetMinPrTime ()
{
    return _minPrTime;
}

void ROSCommsDevice::SetPrTimeInc(float inc)
{
    _prTimeIncPerMeter = inc;
}

float ROSCommsDevice::GetPrTimeInc ()
{
    return _prTimeIncPerMeter;
}

void ROSCommsDevice::SetMinPktErrorRate(float minPktErrorRate)
{
    _minPktErrorRate = minPktErrorRate;
}

float ROSCommsDevice::GetMinPktErrorRate ()
{
    return _minPktErrorRate;
}

void ROSCommsDevice::SetPktErrorRateInc(float inc)
{
    _pktErrorRateIncPerMeter = inc;
}

float ROSCommsDevice::GetPktErrorRateInc ()
{
    return _pktErrorRateIncPerMeter;
}

void ROSCommsDevice::SetMac(int mac)
{
    _mac = mac;
}

int ROSCommsDevice::GetMac ()
{
    return _mac;
}

void ROSCommsDevice::_TxWork ()
{
    _device->WaitForFramesFromRxFifo();
    _device->SetPhyLayerState(CommsDeviceService::BUSY);
    do
    {
        *_device >> _txdlf;
        auto txdlf = DataLinkFrame::Copy (_txdlf);
        if(txdlf->checkFrame())
        {
            //PACKET OK
           // std::this_thread::sleep_for(std::chrono::milliseconds(2000));
            _sim->TransmitFrame(txdlf);

        }
        else
        {
            //PACKET WITH ERRORS
            //Log->critical("TX: INTERNAL ERROR: frame received with errors from the upper layer!");
        }
    }while(_device->GetRxFifoSize() > 0);

    _device->SetPhyLayerState(CommsDeviceService::READY);
}

void ROSCommsDevice::SetLogName(std::string name)
{
    Loggable::SetLogName(name);
    _device->SetLogName (name + ":CommsDeviceService");
}

void ROSCommsDevice::SetLogLevel(Loggable::LogLevel _level)
{
    Loggable::SetLogLevel(_level);
    _device->SetLogLevel (_level);

}

void ROSCommsDevice::LogToConsole(bool c)
{
    Loggable::LogToConsole(c);
    //_device->LogToConsole(c);

}

void ROSCommsDevice::LogToFile(const string &filename)
{
    Loggable::LogToFile(filename);
    _device->LogToFile(filename);
}

void ROSCommsDevice::FlushLog()
{
    Loggable::FlushLog();
    _device->FlushLog();
}

void ROSCommsDevice::FlushLogOn(LogLevel level)
{
    Loggable::FlushLogOn(level);
    _device->FlushLogOn(level);
}

}


