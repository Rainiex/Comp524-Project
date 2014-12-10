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
 *Dual WiFi Protocol
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

NS_LOG_COMPONENT_DEFINE ("DualWiFi");

uint32_t NSta = 0; //number of stations
uint32_t Sim_time = 0; //simulation time
double UpDataRate = 0; //uplink data flow rate
double DownDataRate = 0; //downlink data flow rate

//Downlink Interface AP ----> Stations
double Downlink(uint16_t down_rate){
    
    uint32_t nSta = NSta; //number of stations
    double radius = 50; //distance between ap and stations
    uint32_t sim_time = Sim_time;
    double total_Tx = 0;
    double total_Tput = 0;
    double total_Load;
    uint16_t datarate;
    StringValue Data_Rate;
    
    switch (down_rate) {
        case 1:
            Data_Rate = StringValue("DsssRate1Mbps");
            datarate = 1;
            break;
            
        case 2:
            Data_Rate = StringValue("DsssRate2Mbps");
            datarate = 2;
            break;
            
        case 3:
            Data_Rate = StringValue("DsssRate3Mbps");
            datarate = 3;
            break;
            
        case 4:
            Data_Rate = StringValue("DsssRate4Mbps");
            datarate = 4;
            break;
            
        case 5:
            Data_Rate = StringValue("DsssRate5Mbps");
            datarate = 5;
            break;
            
        case 6:
            Data_Rate = StringValue("DsssRate6Mbps");
            datarate = 6;
            break;
            
        case 7:
            Data_Rate = StringValue("DsssRate7Mbps");
            datarate = 7;
            break;
            
        case 8:
            Data_Rate = StringValue("DsssRate8Mbps");
            datarate = 8;
            break;
            
        case 9:
            Data_Rate = StringValue("DsssRate9Mbps");
            datarate = 9;
            break;
            
        case 10:
            Data_Rate = StringValue("DsssRate10Mbps");
            datarate = 10;
            break;
            
        case 11:
            Data_Rate = StringValue("DsssRate11Mbps");
            datarate = 11;
            break;
            
        case 12:
            Data_Rate = StringValue("DsssRate12Mbps");
            datarate = 12;
            break;
            
        case 13:
            Data_Rate = StringValue("DsssRate13Mbps");
            datarate = 13;
            break;
            
        case 14:
            Data_Rate = StringValue("DsssRate14Mbps");
            datarate = 14;
            break;
            
        case 15:
            Data_Rate = StringValue("DsssRate15Mbps");
            datarate = 15;
            break;
            
        case 16:
            Data_Rate = StringValue("DsssRate16Mbps");
            datarate = 16;
            break;
            
        case 17:
            Data_Rate = StringValue("DsssRate17Mbps");
            datarate = 17;
            break;
            
        case 18:
            Data_Rate = StringValue("DsssRate18Mbps");
            datarate = 18;
            break;
            
        case 19:
            Data_Rate = StringValue("DsssRate19Mbps");
            datarate = 19;
            break;
            
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
    address.Assign (apDev1);
    Ipv4InterfaceContainer staInterface1 = address.Assign (staDevs1);
    
    for (uint16_t i = 0; i < nSta; i++){
        ApplicationContainer cbrApps1;
        uint16_t cbrPort1;
        cbrPort1 = 12345 + i;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(staInterface1.GetAddress(i), cbrPort1));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
        onOffHelper.SetConstantRate(DataRate(DownDataRate));
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
        cbrApps1.Add (onOffHelper.Install (ApNode.Get (0)));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (staInterface1.GetAddress(i), cbrPort1));
        cbrApps1.Add (sink.Install(StaNodes.Get(i)));
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
        total_Tx = total_Tx + i->second.txBytes;
        total_Tput = total_Tput + i->second.rxBytes * 8.0 / sim_time / 1024/ 1024;
    }
    std::cout << "Downlink Channel Rate: " << datarate << " Mbps\n";
    total_Load = total_Tx * 8.0 / sim_time / 1024 / 1024;
    std::cout << "Downlink Traffic Load: " << total_Load << " Mbps\n";
    std::cout << "Downlink Throughput: " << total_Tput << " Mbps\n";
    std::cout << "Downlink Throughput/Traffic Load: " <<total_Tput / total_Load << "\n";
    
    //Clean up
    Simulator::Destroy ();
    
    return total_Tput / total_Load;
}

//Uplink Interface AP <---- Stations
double Uplink(uint16_t up_rate){
    
    uint32_t nSta = NSta; //number of stations
    double radius = 50; //distance between ap and stations
    uint32_t sim_time = Sim_time;
    double total_Tx = 0;
    double total_Tput = 0;
    double total_Load;
    uint16_t datarate;
    StringValue Data_Rate;
    
    switch (up_rate) {
        case 1:
            Data_Rate = StringValue("DsssRate1Mbps");
            datarate = 1;
            break;
            
        case 2:
            Data_Rate = StringValue("DsssRate2Mbps");
            datarate = 2;
            break;
            
        case 3:
            Data_Rate = StringValue("DsssRate3Mbps");
            datarate = 3;
            break;
            
        case 4:
            Data_Rate = StringValue("DsssRate4Mbps");
            datarate = 4;
            break;
            
        case 5:
            Data_Rate = StringValue("DsssRate5Mbps");
            datarate = 5;
            break;
            
        case 6:
            Data_Rate = StringValue("DsssRate6Mbps");
            datarate = 6;
            break;
            
        case 7:
            Data_Rate = StringValue("DsssRate7Mbps");
            datarate = 7;
            break;
            
        case 8:
            Data_Rate = StringValue("DsssRate8Mbps");
            datarate = 8;
            break;
            
        case 9:
            Data_Rate = StringValue("DsssRate9Mbps");
            datarate = 9;
            break;
            
        case 10:
            Data_Rate = StringValue("DsssRate10Mbps");
            datarate = 10;
            break;
            
        case 11:
            Data_Rate = StringValue("DsssRate11Mbps");
            datarate = 11;
            break;
            
        case 12:
            Data_Rate = StringValue("DsssRate12Mbps");
            datarate = 12;
            break;
            
        case 13:
            Data_Rate = StringValue("DsssRate13Mbps");
            datarate = 13;
            break;
            
        case 14:
            Data_Rate = StringValue("DsssRate14Mbps");
            datarate = 14;
            break;
            
        case 15:
            Data_Rate = StringValue("DsssRate15Mbps");
            datarate = 15;
            break;
            
        case 16:
            Data_Rate = StringValue("DsssRate16Mbps");
            datarate = 16;
            break;
            
        case 17:
            Data_Rate = StringValue("DsssRate17Mbps");
            datarate = 17;
            break;
            
        case 18:
            Data_Rate = StringValue("DsssRate18Mbps");
            datarate = 18;
            break;
            
        case 19:
            Data_Rate = StringValue("DsssRate19Mbps");
            datarate = 19;
            break;
            
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
    WifiHelper wifi2;
    wifi2.SetStandard(WIFI_PHY_STANDARD_80211b); //use 802.11b
    wifi2.SetRemoteStationManager ("ns3::ConstantRateWifiManager", "DataMode", Data_Rate, "ControlMode", Data_Rate);
    
    NqosWifiMacHelper wifiMac2 = NqosWifiMacHelper::Default ();
    YansWifiPhyHelper wifiPhy2 = YansWifiPhyHelper::Default ();
    YansWifiChannelHelper wifiChannel2 = YansWifiChannelHelper::Default ();
    wifiPhy2.SetChannel (wifiChannel2.Create ());
    Ssid ssid2 = Ssid ("wifi-default");
    
    wifiMac2.SetType ("ns3::StaWifiMac", "Ssid", SsidValue (ssid2), "ActiveProbing", BooleanValue (false));
    NetDeviceContainer staDevs2;
    staDevs2 = wifi2.Install (wifiPhy2, wifiMac2, StaNodes);
    
    wifiMac2.SetType ("ns3::ApWifiMac", "Ssid", SsidValue (ssid2));
    NetDeviceContainer apDev2;
    apDev2 = wifi2.Install (wifiPhy2, wifiMac2, ApNode);
    
    //Internet protocol stack
    InternetStackHelper stack;
    stack.Install (ApNode);
    stack.Install (StaNodes);
    Ipv4AddressHelper address;
    address.SetBase ("10.0.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface2 = address.Assign (apDev2);
    address.Assign (staDevs2);
    
    for (uint16_t i = 0; i < nSta; i++){
        ApplicationContainer cbrApps2;
        uint16_t cbrPort2 = 9;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(apInterface2.GetAddress(0), cbrPort2));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
        onOffHelper.SetConstantRate(DataRate(UpDataRate));
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
        cbrApps2.Add (onOffHelper.Install (StaNodes.Get (i)));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (apInterface2.GetAddress(0), cbrPort2));
        cbrApps2.Add (sink.Install(ApNode.Get(0)));
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
        total_Tx = total_Tx + i->second.txBytes;
        total_Tput = total_Tput + i->second.rxBytes * 8.0 / sim_time / 1024/ 1024;
    }
    std::cout << "Uplink Channel Rate: " << datarate << " Mbps\n";
    total_Load = total_Tx * 8.0 / sim_time / 1024 / 1024;
    std::cout << "Uplink Traffic Load: " << total_Load << " Mbps\n";
    std::cout << "Uplink Throughput: " << total_Tput << " Mbps\n";
    std::cout << "Uplink Throughput/Traffic Load: " <<total_Tput / total_Load << "\n";
    
    //Clean up
    Simulator::Destroy ();
    
    return total_Tput / total_Load;
}

int main (int argc, char **argv){
    
    uint16_t i, j;
    
    double down_ratio, up_ratio;
    double rat_dis;
    double min_dis = -1;
    
    int opt_bwdiv = 0;
    
    //Input arguments
    std::cout << "Input: Number of stations, Simulation time, Downlink data rate, Uplink data rate." << std::endl;
    std::cin >> NSta >> Sim_time >> DownDataRate >> UpDataRate;
    
    for (i = 1; i < 20; i++){
        j = 20 - i;
        std::cout << "Downlink traffic: " << i << std::endl;
        down_ratio = Downlink(i);
        std::cout << "------------------------------------------------\n";
        
        std::cout << "Uplink traffic: " << j << std::endl;
        up_ratio = Uplink(j);
        std::cout << "------------------------------------------------\n";
        
        rat_dis = down_ratio - up_ratio;
        if(rat_dis < 0)
            rat_dis = -rat_dis;
        if(rat_dis < min_dis){
            opt_bwdiv = i;
            min_dis = rat_dis;
        }
        else if(min_dis == -1){
            opt_bwdiv = i;
            min_dis = rat_dis;
        }
    }
    std::cout << "**********************************************" << std::endl;
    std::cout << "Number of stations:" << NSta << std::endl;
    std::cout << "Simulation time:" << Sim_time << std::endl;
    std::cout << "Downlink data rate:" << DownDataRate << std::endl;
    std::cout << "Uplink data rate:" << UpDataRate << std::endl;
    std::cout << "Traffic ratio is (DL/UL):" << DownDataRate/UpDataRate << std::endl;
    std::cout << "Optimal bandwidth division ratio is (DL/UL):" << opt_bwdiv << "/" << 20 - opt_bwdiv << std::endl;
    std::cout << "**********************************************" << std::endl;
    
    return 0;
}
