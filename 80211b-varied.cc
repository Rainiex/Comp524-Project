/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

/* 
 *802.11b Protocol (Single Channel)
 *
 *by Jie
 */

#include "ns3/core-module.h"
#include "ns3/propagation-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/internet-module.h"
#include "ns3/flow-monitor-module.h"
#include "ns3/wifi-module.h"
#include <iostream>
#include <cmath>
#include <string>
#include <sstream>

#define PI 3.14159265

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("80211b");

uint32_t NSta = 5; //number of stations
uint32_t Sim_time = 10; //simulation time

//Downlink Interface AP ----> Stations
void SingleChannel(uint16_t rate){
    
    uint32_t nSta = NSta; //number of stations
    double radius = 50; //distance between ap and stations
    uint32_t sim_time = Sim_time;
    double down_Tx = 0;
    double down_Tput = 0;
    double down_Load;
    double up_Tx = 0;
    double up_Tput = 0;
    double up_Load;
    uint16_t datarate;
    StringValue Data_Rate;
    
    switch (rate) {
        case 20:
            Data_Rate = StringValue("DsssRate20Mbps");
            datarate = 20;
            break;
            
        default:
            break;
    }
    
    //Disable CTS/RTS
    UintegerValue ctsThr = UintegerValue (100000);
    Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);
    
    //Setup nodes
    NodeContainer ApNode;
    ApNode.Create(1);
    NodeContainer StaNodes;
    StaNodes.Create(nSta);
    
    //Place nodes
    MobilityHelper mobility;
    Ptr<ListPositionAllocator> positionalloc1 = CreateObject<ListPositionAllocator>();
    for (size_t i = 0; i < nSta; i++){
        positionalloc1->Add (Vector(radius*cos(2*PI/(nSta)*i), radius*cos(2*PI/(nSta)*i), 0));
    }
    mobility.SetPositionAllocator(positionalloc1);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (StaNodes);
    
    Ptr<ListPositionAllocator> positionalloc2 = CreateObject<ListPositionAllocator>();
    positionalloc2->Add (Vector(0.0,0.0,0.0));
    mobility.SetPositionAllocator(positionalloc2);
    mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
    mobility.Install (ApNode);
    
    //Setup WiFi devices, channel, PHY, MAC
    WifiHelper wifi1;
    wifi1.SetStandard(WIFI_PHY_STANDARD_80211b); //use 802.11b
    wifi1.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", Data_Rate, "ControlMode", Data_Rate);
    
    NqosWifiMacHelper wifiMac1 = NqosWifiMacHelper::Default ();
    YansWifiPhyHelper wifiPhy1 = YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel1 = YansWifiChannelHelper::Default ();
    wifiPhy1.SetChannel (wifiChannel1.Create ());
    Ssid ssid1 = Ssid ("wifi-default");
    
    wifiMac1.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid1), "ActiveProbing", BooleanValue (false));
    NetDeviceContainer staDevs1;
    staDevs1 = wifi1.Install (wifiPhy1, wifiMac1, StaNodes);

    wifiMac1.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid1));
    NetDeviceContainer apDev1;
    apDev1 = wifi1.Install (wifiPhy1, wifiMac1, ApNode);
    
    //Internet protocol stack
    InternetStackHelper stack;
    stack.Install (ApNode);
    stack.Install (StaNodes);
    Ipv4AddressHelper address;
    address.SetBase ("10.0.0.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface1 = address.Assign (apDev1);
    Ipv4InterfaceContainer staInterface1 = address.Assign (staDevs1);
    
    ApplicationContainer cbrApps1;
    uint16_t cbrPort1;
    uint16_t cbrPort2;
    
    for (uint16_t i = 0; i < nSta; i++){
        cbrPort1 = 12345 + i;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(staInterface1.GetAddress(i), cbrPort1));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
        
        switch (i) {
            case 0:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.125572]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.62471Mbps"));
                break;
            case 1:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.265721]"));
                onOffHelper.SetAttribute("DataRate", StringValue("0.839792Mbps"));
                break;
            case 2:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.00401028]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.65294Mbps"));
                break;
            case 3:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.25218]"));
                onOffHelper.SetAttribute("DataRate", StringValue("2.08426Mbps"));
                break;
            case 4:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.302625]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.17075Mbps"));
                break;
                
            default:
                break;
        }

        
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
        cbrApps1.Add (onOffHelper.Install (ApNode.Get (0)));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (staInterface1.GetAddress(i), cbrPort1));
        cbrApps1.Add (sink.Install(StaNodes.Get(i)));
    }
    
    for (uint16_t i = 0; i < nSta; i++){
        cbrPort2 = 9;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(apInterface1.GetAddress(0), cbrPort2));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
    
        switch (i) {
            case 0:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.612669]"));
                onOffHelper.SetAttribute("DataRate", StringValue("0.969407Mbps"));
                break;
            case 1:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.473699]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.08011Mbps"));
                break;
            case 2:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1.23564]"));
                onOffHelper.SetAttribute("DataRate", StringValue("0.943724Mbps"));
                break;
            case 3:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.512989]"));
                onOffHelper.SetAttribute("DataRate", StringValue("0.645602Mbps"));
                break;
            case 4:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=1.48545]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.72832Mbps"));
                break;
                
            default:
                break;
        }

        
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
        cbrApps1.Add (onOffHelper.Install (StaNodes.Get (i)));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (apInterface1.GetAddress(0), cbrPort2));
        cbrApps1.Add (sink.Install(ApNode.Get(0)));
    }
    
    //Install flow monitor on all nodes
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    
    //Run simulation
    Simulator::Stop (Seconds (sim_time));
    Simulator::Run ();
    
    //Print per flow statistics
    monitor->CheckForLostPackets ();
    Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
    std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
    for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i){
        Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
        if (t.sourceAddress == "10.0.0.1"){
            down_Tx = down_Tx + i->second.txBytes;
            down_Tput = down_Tput + i->second.rxBytes * 8.0 / sim_time / 1024/ 1024;
        }
        else{
            up_Tx = up_Tx + i->second.txBytes;
            up_Tput = up_Tput + i->second.rxBytes * 8.0 / sim_time / 1024/ 1024;
        }
    }
    
    std::cout << "Channel Rate: " << datarate << " Mbps\n";
    down_Load = down_Tx * 8.0 / sim_time / 1024 / 1024;
    up_Load = up_Tx * 8.0 / sim_time / 1024 / 1024;
    std::cout << "Downlink Load: " << down_Load << " Mbps\n";
    std::cout << "Downlink Throughput: " << down_Tput << " Mbps\n";
    std::cout << "Uplink Load: " << up_Load << " Mbps\n";
    std::cout << "Uplink Throughput: " << up_Tput << " Mbps\n";
    
    std::cout << "------------------------------------------------\n";
    
    std::cout << "Traffic Load: " << down_Load + up_Load << " Mbps\n";
    std::cout << "Throughput: " << down_Tput + up_Tput << " Mbps\n";
    std::cout << "------------------------------------------------\n";
    
    std::cout << "**********************************************" << std::endl;
    std::cout << "Throughput/Traffic Load: " << (down_Tput / up_Tput) / (down_Load / up_Load) << "\n";
    std::cout << "**********************************************" << std::endl;
    
    //Clean up
    Simulator::Destroy ();
}

int main (int argc, char **argv){

    SingleChannel(20);
    
    return 0;
}
