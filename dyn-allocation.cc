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
 */

 /* 
 *Dual WiFi Protocol
 *
 *by Yang
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

std::string DATARATE_PREFIX ("DsssRate");
std::string DATARATE_SUFFIX ("Mbps");

//packetCounts, used to allocate the bandwidth
std::vector<long> staCounts;
std::vector<long> apCounts;
std::vector<int> staRecvCounts;
std::vector<std::vector<int> > ap_counts_generated;
std::vector<std::vector<int> > sta_counts_generated;
long aprates[5];
long starates[5];
// assume the station number is 5, it is hard-coded here, it's bad I know
int apRecvCount;
uint32_t nSta = 5;
uint32_t packetSize = 1024; // bytes

void Uplink2(uint16_t up_rate, double Sim_time, double iterval);
void Downlink2(uint16_t up_rate, double Sim_time, double interval);

long getApToStaTotal()
{
  long temp_ap_count = 0;
  for(uint32_t i = 0; i < nSta;i++) {
    temp_ap_count += apCounts[i];
  }
  return temp_ap_count;
}

long getStaToApTotal(){
  long temp_sta_count = 0;
  for(uint32_t i = 0; i < nSta;i++) {
    temp_sta_count += staCounts[i];
  }
  return temp_sta_count;
}

long getDataRateLong(int tcount, int totalCounts, int datarate){
  long temprate = datarate * 1000 * 1000;  //Mbps -> bit per seconds
  long res = temprate * tcount / totalCounts;
  return res;
}

//caution: pay attention to the sum of all link rates, may be less than the total data rate
std::string getDataRate(int tcount, int totalCounts, int datarate){
  //NS_LOG_UNCOND("tcount is "<<tcount<< " and total is " << totalCounts);
  return std::to_string(getDataRateLong(tcount, totalCounts, datarate));
}

int updateStaCount(int id, int num) {
  if(staCounts[id] > num) {
    staCounts[id] -= num;
  } else {
    num = staCounts[id];
    staCounts[id] = 1;
  }
  return num;
}

int updateApCount(int id, int num) {
  if(apCounts[id] > num) {
    apCounts[id] -= num;
  } else {
    num = apCounts[id];
    apCounts[id] = 1;
  }
  return num;
}

void generateApCounts(int num){
  int size = ap_counts_generated[num].size();
  for(int i = 0; i < nSta; i++){
    if(i >= size)
      apCounts[i] += 0;
    else
      apCounts[i] += ap_counts_generated[num][i];
  }

  // log information
  // for(uint32_t i = 0; i < nSta;i++) {
  //   NS_LOG_UNCOND("current packets in ap need to be sent to sta " << i << " is " << apCounts[i]);
  // }
}

void generateStaCounts(int num){
  int size = sta_counts_generated[num].size();
  for(int i = 0; i< nSta; i++){
    if(size <= i)
      staCounts[i] += 0;
    else
      staCounts[i] += sta_counts_generated[num][i];
  }

  // log information
  // for(uint32_t i = 0; i < nSta;i++) {
  //   NS_LOG_UNCOND("current packets in sta " << i << " need to be sent to ap is " << staCounts[i]);
  //   //NS_LOG_UNCOND("ap to sta " << i << " is " << apCounts[i]);
  // }
}

std::string getDataRate2(int datarate){
  return std::to_string((long)datarate * 1000 * 1000 / nSta);
}

//Downlink Interface AP ----> Stations
double Downlink(uint16_t datarate, double sim_time, int counter, int totalTimes)
{
  double radius = 50; //distance between ap and stas
  double total_Tx = 0;
  double total_Tput = 0;
  double total_Load;
  std::stringstream Downlink_Rate;
  Downlink_Rate.str("");
  Downlink_Rate<<DATARATE_PREFIX<<datarate<<DATARATE_SUFFIX;

  StringValue Data_Rate = StringValue(Downlink_Rate.str());
  //NS_LOG_UNCOND ("send downlink at speed " << Downlink_Rate.str());
  //Disable rts/cts
  UintegerValue ctsThr = UintegerValue (100000);
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

  // Setup Nodes
  NodeContainer ApNode;
  ApNode.Create(1);
  NodeContainer StaNodes;
  StaNodes.Create(nSta);

  //Place nodes
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionalloc1 = CreateObject<ListPositionAllocator>();
  for (size_t i = 0; i < nSta; i++)
  {
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

  // Setup Wifi devices, channle, PHY, MAC
  WifiHelper wifi1;
  wifi1.SetStandard(WIFI_PHY_STANDARD_80211b);
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

  //Internet protoal stack
  InternetStackHelper stack;
  stack.Install (ApNode);
  stack.Install (StaNodes);
  Ipv4AddressHelper address;
  address.SetBase ("10.0.0.0", "255.255.255.0");
  address.Assign (apDev1);
  Ipv4InterfaceContainer staInterface1 = address.Assign (staDevs1);
  //used for search correct station, this is because the flow monitor is disordered for the stations.
  std::vector<Ipv4Address> myAddress;
  for(int i =0;i< nSta;i++){
    myAddress.push_back(staInterface1.GetAddress(i));
  }

  long apTotal = getApToStaTotal();

  if(counter==0){
    for (uint16_t i = 0; i < nSta; i++)
    { 
      ApplicationContainer cbrApps1;
      uint16_t cbrPort1;
      cbrPort1 = 12345 + i;
      OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(staInterface1.GetAddress(i), cbrPort1)); 
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
      onOffHelper.SetConstantRate(DataRate(getDataRate2(datarate)));
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.01*(i+1))));
      cbrApps1.Add (onOffHelper.Install (ApNode.Get (0)));
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (staInterface1.GetAddress(i), cbrPort1));
      cbrApps1.Add (sink.Install(StaNodes.Get(i)));
    }
  }else{
  for (uint16_t i = 0; i < nSta; i++)
    {
      //get the new data rate, and half the data rate if the new one is too small
      long thisDataRate = getDataRateLong(apCounts[i], apTotal, datarate);
      if(thisDataRate == 0){
        aprates[i] /= 2;
        thisDataRate = aprates[i];
        if(thisDataRate == 0)
          continue;
      } else {
        aprates[i] = thisDataRate;
      }

      ApplicationContainer cbrApps1;
      uint16_t cbrPort1;
      cbrPort1 = 12345 + i;
      OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(staInterface1.GetAddress(i), cbrPort1)); 
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
      onOffHelper.SetConstantRate(DataRate(std::to_string(thisDataRate)));
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
      cbrApps1.Add (onOffHelper.Install (ApNode.Get (0)));
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (staInterface1.GetAddress(i), cbrPort1));
      cbrApps1.Add (sink.Install(StaNodes.Get(i)));
    } 
  }
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  // Run simulation for sim_time seconds
  Simulator::Stop (Seconds (sim_time));
  Simulator::Run ();
  Simulator::Destroy ();
  //update the packets number with the packets generated in this interval time
  generateApCounts(counter);

  //update the packets number to be transmitted in next interval
  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    int recvPackets = 0;
    for (int x = 0; x < nSta ; x++)
    {
      if(t.destinationAddress == myAddress[x]){
        recvPackets = updateApCount(x, i->second.rxPackets);
        break;
      }
    }
    total_Tx = total_Tx + i->second.txBytes;
    // this 1052 is a fix value used to compare with the dual wifi.
    // in dual wifi, when the packet size is 1024, the rxPackets is 10, the rxBytes is 10 * 1052!!!
    // to keep the throughput the small criterion, I use 1052 here, actually it should be 540.
    total_Tput += recvPackets * 1052 * 8.0 / sim_time / 1024/ 1024;          
  }
   
  total_Load = total_Tx * 8.0 / sim_time / 1024 / 1024;
  return total_Tput;
}
 

//Stations ------> AP
double Uplink(uint16_t datarate, double sim_time, int counter, int totalTimes)
{
  double radius = 50; //distance between ap and stas
  double total_Tx = 0;
  double total_Tput = 0;
  double total_Load;

  // get the bandwidth
  std::stringstream Uplink_Rate;
  Uplink_Rate.str("");
  Uplink_Rate<<DATARATE_PREFIX<<datarate<<DATARATE_SUFFIX;

  StringValue Data_Rate = StringValue(Uplink_Rate.str());
  //NS_LOG_UNCOND ("send uplink at speed " << Uplink_Rate.str());
  
  //Disable rts/cts
  UintegerValue ctsThr = UintegerValue (100000);
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", ctsThr);

  // Setup Nodes
  NodeContainer ApNode;
  ApNode.Create(1);
  NodeContainer StaNodes;
  StaNodes.Create(nSta);

  //Place nodes
  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionalloc1 = CreateObject<ListPositionAllocator>();
  for (size_t i = 0; i < nSta; i++)
  {
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

  // Setup Wifi devices, channle, PHY, MAC
  WifiHelper wifi2;
  wifi2.SetStandard(WIFI_PHY_STANDARD_80211b);
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
  
  //Internet protoal stack
  InternetStackHelper stack;
  stack.Install (ApNode);
  stack.Install (StaNodes);
  Ipv4AddressHelper address; 
  address.SetBase ("10.0.1.0", "255.255.255.0");
  Ipv4InterfaceContainer apInterface2 = address.Assign (apDev2);
  Ipv4InterfaceContainer staInterface2 = address.Assign (staDevs2);
  //used for search correct station, this is because the flow monitor is disordered for the stations.
  std::vector<Ipv4Address> myAddress;
  for(int i =0;i< nSta;i++){
    myAddress.push_back(staInterface2.GetAddress(i));
  }

  long staTotal = getStaToApTotal();
  if(counter == 0){
    for (uint16_t i = 0; i < nSta; i++)
    {
      ApplicationContainer cbrApps2;
      uint16_t cbrPort2 = 9;
      OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(apInterface2.GetAddress(0), cbrPort2)); 
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
      onOffHelper.SetConstantRate(DataRate(getDataRate2(datarate))); // bits/s
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.002*(i+1))));
      cbrApps2.Add (onOffHelper.Install (StaNodes.Get (i)));
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (apInterface2.GetAddress(0), cbrPort2));
      cbrApps2.Add (sink.Install (ApNode.Get(0)));
    }
  }else{
    for (uint16_t i = 0; i < nSta; i++)
    {
      long thisDataRate = getDataRateLong(staCounts[i], staTotal, datarate);
      if(thisDataRate == 0){
        starates[i] /= 2;
        thisDataRate = starates[i];
        if(thisDataRate == 0)
          continue;
      } else {
        starates[i] = thisDataRate;
      }

      ApplicationContainer cbrApps2;
      uint16_t cbrPort2 = 9;
      OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(apInterface2.GetAddress(0), cbrPort2)); 
      onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
      onOffHelper.SetConstantRate(DataRate(std::to_string(thisDataRate))); // bits/s
      onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.002*(i+1))));
      cbrApps2.Add (onOffHelper.Install (StaNodes.Get (i)));
      PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (apInterface2.GetAddress(0), cbrPort2));
      cbrApps2.Add (sink.Install (ApNode.Get(0)));
    }
  }
  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
 
  Simulator::Stop (Seconds (sim_time));
  Simulator::Run ();
  Simulator::Destroy ();
  generateStaCounts(counter);

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
  {
    Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
    total_Tx = total_Tx + i->second.txBytes;
    int recvPackets = 0;
    for (int x = 0; x < nSta ; x++)
    {
      if(t.sourceAddress == myAddress[x]){
        recvPackets = updateStaCount(x, i->second.rxPackets);
        break;
      }
    }
          total_Tput += recvPackets * 1052 * 8.0 / sim_time / 1024/ 1024;
  }
   
  total_Load = total_Tx * 8.0 / sim_time / 1024 / 1024;
  return total_Tput;
}

// first run the dual wifi to make sure the traffic loads are the same.
void initializeCounts(double sim_time, double interval){
  for(uint32_t i = 0; i < nSta;i++) {
    staCounts.push_back(0);
    apCounts.push_back(0);
    staRecvCounts.push_back(0);
  }
  apRecvCount = 0;
  
  Downlink2(20, sim_time, interval);
  Uplink2(20, sim_time, interval);

  for(int i = ap_counts_generated.size() - 1; i > 0; i--){
    int size1 = ap_counts_generated[i].size();
    int size2 = ap_counts_generated[i-1].size();
    int size_min = size1 > size2? size2 : size1;
    for(int j = 0; j< size_min; j++){
      ap_counts_generated[i][j] -= ap_counts_generated[i-1][j];
    }
  }
  
  for(int i = sta_counts_generated.size() - 1; i > 0; i--){
    int size1 = sta_counts_generated[i].size();
    int size2 = sta_counts_generated[i-1].size();
    int size_min = size1 > size2?size2:size1;
    for(int j = 0; j< size_min;j++){
      sta_counts_generated[i][j] -= sta_counts_generated[i-1][j];
    }
  }
  //log
  // for(int i = 0;i<ap_counts_generated.size();i++){
  //   for(int j = 0; j< ap_counts_generated[i].size();j++){
  //     NS_LOG_UNCOND(ap_counts_generated[i][j]);
  //   }
  //   NS_LOG_UNCOND("---------------");
  // }
  // NS_LOG_UNCOND("===========");
  // for(int i = 0;i<sta_counts_generated.size();i++){
  //   for(int j = 0; j< sta_counts_generated[i].size();j++){
  //     NS_LOG_UNCOND(sta_counts_generated[i][j]);
  //   }
  //   NS_LOG_UNCOND("---------------");
  // }
}

int getDownlinkBandwidth(int total_Band, int lastD, int lastU)
{
  long temp_ap_count = 0;
  long temp_sta_count = 0;
  for(uint32_t i = 0; i < nSta;i++) {
    temp_sta_count += staCounts[i];
    temp_ap_count += apCounts[i];
  }
  // NS_LOG_UNCOND("sta count is " << temp_sta_count);
  // NS_LOG_UNCOND("ap count is " << temp_ap_count);
  if(temp_ap_count == temp_sta_count)
    return 10;

  if(temp_sta_count < 2 * nSta)
    return 20 - lastU / 2;
  if(temp_ap_count < 2 * nSta)
    return lastD / 2;

  return total_Band * temp_ap_count / (temp_sta_count + temp_ap_count);
}

int main (int argc, char **argv)
{ 
  Time interPacketInterval;
  double interval = 0.9; // seconds
  double sim_time = 10.0;

  CommandLine cmd;
  cmd.AddValue ("nodes", "nodes of the stations", nSta);
  cmd.AddValue ("packetSize", "size of application packet sent", packetSize);
  cmd.AddValue ("interval", "interval (seconds) between packets", interval);
  cmd.AddValue ("time", "interval (seconds) between packets", sim_time);
  cmd.Parse (argc, argv);

  initializeCounts(sim_time, interval);

  double maxTP = -1;
  double tempTP = 0;
  double avgDown = 0;
  double avgUp = 0;
  int times = sim_time / interval;
  
  // refer to the contention window half the bandwidth
  int lastD = 10, lastU = 10;
  for(int i = 0; i < times; i++)
  {
    int dlr = getDownlinkBandwidth(20, lastD, lastU);
    double downTP = 0;
    NS_LOG_UNCOND("------------------------------------------------");
    //NS_LOG_UNCOND("Downlink traffic: " << dlr);
    if(dlr == 0)
      generateApCounts(i);
    else
      downTP = Downlink(dlr, interval, i, times);
    NS_LOG_UNCOND("------------------------------------------------");

    int ulr = 20 - dlr;
    //NS_LOG_UNCOND("Uplink traffic: " << ulr);
    double upTP = 0;
    if(ulr == 0)
      generateStaCounts(i);
    else
      upTP = Uplink(ulr, interval, i, times);
    
    lastD = dlr;
    lastU = ulr;

    double tempT = downTP + upTP;
    NS_LOG_UNCOND("time "<<i<<" total tp is "<< tempT);
    
    tempTP += downTP + upTP;
    if(downTP + upTP > maxTP)
      maxTP = downTP+upTP;
    
    avgDown += downTP;
    avgUp += upTP;
    NS_LOG_UNCOND("------------------------------------------------");
    NS_LOG_UNCOND("------------------------------------------------");
  }
  tempTP/=(times);
  avgDown/=(times);
  avgUp/=(times);

  NS_LOG_UNCOND("best total_Tput is " << tempTP);
  NS_LOG_UNCOND("Avg down_Tput is " << avgDown<<" and Avg up_Tut is "<<avgUp);

  return 0;
}


//Dual wifi Downlink Interface AP ----> Stations
void Downlink2(uint16_t down_rate, double Sim_time, double interval){
    double radius = 50; //distance between ap and stations
    double sim_time = Sim_time;
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
    address.SetBase ("10.1.0.0", "255.255.255.0");
    address.Assign (apDev1);
    Ipv4InterfaceContainer staInterface1 = address.Assign (staDevs1);
    
    for (uint16_t i = 0; i < nSta; i++){
        ApplicationContainer cbrApps1;
        uint16_t cbrPort1;
        cbrPort1 = 12345 + i;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(staInterface1.GetAddress(i), cbrPort1));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
        
        switch (i) {
            case 0:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0798364]"));
                onOffHelper.SetAttribute("DataRate", StringValue("3.71753Mbps"));
                break;
            case 1:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.810696]"));
                onOffHelper.SetAttribute("DataRate", StringValue("9.08208Mbps"));
                break;
            case 2:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.198347]"));
                onOffHelper.SetAttribute("DataRate", StringValue("7.81866Mbps"));
                break;
            case 3:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.000208614]"));
                onOffHelper.SetAttribute("DataRate", StringValue("3.65143Mbps"));
                break;
            case 4:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.276027]"));
                onOffHelper.SetAttribute("DataRate", StringValue("4.89449Mbps"));
                break;
                
            default:
                break;
        }
        
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
        cbrApps1.Add (onOffHelper.Install (ApNode.Get (0)));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (staInterface1.GetAddress(i), cbrPort1));
        cbrApps1.Add (sink.Install(StaNodes.Get(i)));
    }
    
    //Install flow monitor on all nodes
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    
    //interval = sim_time;
    int times = sim_time / interval;
    for(int j = 0;j < times;j++){
      std::vector<int> v;

      //Run simulation
      Simulator::Stop (Seconds (interval));
      Simulator::Run ();
    
      //Print per flow statistics
      monitor->CheckForLostPackets ();
      Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
      std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
      for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i){
        v.push_back(i->second.rxPackets);
      }
      ap_counts_generated.push_back(v);
    }
    
    //Clean up
    Simulator::Destroy ();
}

//Dual wifi Uplink Interface AP <---- Stations
void Uplink2(uint16_t up_rate, double Sim_time, double interval){
    double radius = 50; //distance between ap and stations
    double sim_time = Sim_time;
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
    address.SetBase ("10.1.1.0", "255.255.255.0");
    Ipv4InterfaceContainer apInterface2 = address.Assign (apDev2);
    address.Assign (staDevs2);
    
    for (uint16_t i = 0; i < nSta; i++){
        ApplicationContainer cbrApps2;
        uint16_t cbrPort2 = 9;
        OnOffHelper onOffHelper("ns3::UdpSocketFactory", InetSocketAddress(apInterface2.GetAddress(0), cbrPort2));
        onOffHelper.SetAttribute ("PacketSize", UintegerValue (1024));
        
        switch (i) {
            case 0:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.372506]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.95607Mbps"));
                break;
            case 1:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.00545545]"));
                onOffHelper.SetAttribute("DataRate", StringValue("0.570262Mbps"));
                break;
            case 2:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0339597]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.26577Mbps"));
                break;
            case 3:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.0727692]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.01339Mbps"));
                break;
            case 4:
                onOffHelper.SetAttribute("OnTime", StringValue("ns3::ConstantRandomVariable[Constant=0.2]"));
                onOffHelper.SetAttribute("OffTime", StringValue("ns3::ConstantRandomVariable[Constant=0.00579667]"));
                onOffHelper.SetAttribute("DataRate", StringValue("1.17717Mbps"));
                break;
                
            default:
                break;
        }
        
        onOffHelper.SetAttribute ("StartTime", TimeValue (Seconds (0.001*(i+1))));
        cbrApps2.Add (onOffHelper.Install (StaNodes.Get (i)));
        
        PacketSinkHelper sink ("ns3::UdpSocketFactory", InetSocketAddress (apInterface2.GetAddress(0), cbrPort2));
        cbrApps2.Add (sink.Install(ApNode.Get(0)));
    }
    
    //Install flow monitor on all nodes
    FlowMonitorHelper flowmon;
    Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
    
    double total_Tx=0;
    double total_Load=0;
    int times = sim_time / interval;
    for(int j = 0;j < times;j++){
      std::vector<int> v;

      //Run simulation
      Simulator::Stop (Seconds (interval));
      Simulator::Run ();
    
      //Print per flow statistics
      monitor->CheckForLostPackets ();
      Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
      std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
      for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i){
          v.push_back(i->second.rxPackets);
          total_Tx = total_Tx + i->second.txBytes;
      }
      sta_counts_generated.push_back(v);
    }
    total_Load = total_Tx * 8.0 / sim_time / 1024 / 1024;
    std::cout << "Uplink Traffic Load: " << total_Load << " Mbps\n";
    
    double mysum=0;
    for(int i = 0; i < sta_counts_generated[sta_counts_generated.size()-1].size();i++){
        mysum += sta_counts_generated[sta_counts_generated.size()-1][i];
    }
    //NS_LOG_UNCOND("up link recved packets " << mysum);
    //Clean up
    Simulator::Destroy ();
    
}

