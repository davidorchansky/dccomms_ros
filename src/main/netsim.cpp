#include <iostream>
#include <cstdio>
#include  <cstdio>
#include  <sys/types.h>
#include  <signal.h>

#include <dccomms_ros/ROSCommsSimulator.h>

//ROS
#include <ros/ros.h>
//end ROS

using namespace dccomms;
using namespace dccomms_ros;
using namespace std;


static std::shared_ptr<spd::logger> Log = spd::stdout_color_mt("MERBOTSCommsSimulator");
ROSCommsSimulator * sim;

void SIGINT_handler (int sig)
{
        printf("Received %d signal\n",sig);
        printf("Log messages flushed.\n");
        exit(0);
}

void setSignals()
{
    if (signal(SIGINT, SIGINT_handler) == SIG_ERR) {
         printf("SIGINT install error\n");
         exit(1);
    }
}

int main(int argc, char ** argv)
{
    setSignals();

    //// GET PARAMS
    ros::init(argc, argv, "dccomms_netsim");
    ros::NodeHandle nh("~");

    sim = new ROSCommsSimulator(nh);
    sim->SetLogName ("netsim");
    sim->LogToFile ("netsim_log");


    Log->set_level(spdlog::level::debug);
    Log->flush_on(spd::level::info);

    sim->SetTransmitPDUCb ([](int linkType, dccomms::DataLinkFramePtr dlf){
        Log->info("Transmitting PDU");
    });
    sim->SetReceivePDUCb ([](int linkType, dccomms::DataLinkFramePtr dlf){
        Log->info("PDU Received");
    });
    sim->SetErrorPDUCb ([](int linkType, dccomms::DataLinkFramePtr dlf){
        Log->warn("PDU Received with errors");
    });

    sim->Start();

    ros::Rate loop_rate(30);
    while(ros::ok())
    {
        loop_rate.sleep();
        ros::spinOnce();
    }

}

