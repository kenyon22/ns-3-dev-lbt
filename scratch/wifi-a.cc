/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2012 The Boeing Company
 *
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
 * Author: Gary Pei <guangyu.pei@boeing.com>
 */

#include "ns3/core-module.h"
#include "ns3/mobility-module.h"
#include "ns3/gnuplot.h"
#include "ns3/gnuplot.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"

#include <iostream>
#include <sstream>

NS_LOG_COMPONENT_DEFINE ("WifiBianchiValidation");

using namespace ns3;

std::ofstream cwTraceFile;

void
CwTrace (uint32_t oldVal, uint32_t newVal)
{
  cwTraceFile << Simulator::Now ().GetSeconds () << " " << oldVal << " " << newVal << std::endl;
}

class Experiment
{
public:
  Experiment ();
  Experiment (std::string name);
  Gnuplot2dDataset Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                        const WifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel,
                        uint32_t pktSize, uint32_t netSize, double delta, uint32_t gridWidth, double duration);
private:
  void ReceivePacket (Ptr<Socket> socket);
  void SetPosition (Ptr<Node> node, Vector position);
  Ptr <Application> GetOnOff (Ptr<Node> node);
  Ptr<Socket> SetupPacketReceive (Ptr<Node> node);

  uint32_t m_bytesTotal;
  Gnuplot2dDataset m_output;
};

Experiment::Experiment ()
{}

Experiment::Experiment (std::string name)
  : m_output (name)
{
  m_output.SetStyle (Gnuplot2dDataset::LINES);
}

void
Experiment::SetPosition (Ptr<Node> node, Vector position)
{
  Ptr<MobilityModel> mobility = node->GetObject<MobilityModel> ();
  mobility->SetPosition (position);
}

void
Experiment::ReceivePacket (Ptr<Socket> socket)
{
  Ptr<Packet> packet;
  while (packet = socket->Recv ())
    {
      m_bytesTotal += packet->GetSize ();
    }
}

Ptr<Socket>
Experiment::SetupPacketReceive (Ptr<Node> node)
{
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<Socket> sink = Socket::CreateSocket (node, tid);
  sink->Bind (InetSocketAddress(Ipv4Address ("0.0.0.0"),80));
  sink->SetRecvCallback (MakeCallback (&Experiment::ReceivePacket, this));

  return sink;
}

Gnuplot2dDataset
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const WifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel,
                 uint32_t pktSize, uint32_t networkSize, double delta, uint32_t gridWidth, double duration)
{
  m_bytesTotal = 0;

  NodeContainer c;
  c.Create (networkSize);

  YansWifiPhyHelper phy = wifiPhy;
  phy.SetChannel (wifiChannel.Create ());

  WifiMacHelper mac = wifiMac;
  mac.SetType ("ns3::AdhocWifiMac");
  NetDeviceContainer devices = wifi.Install (phy, mac, c);

  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
    "MinX", DoubleValue (0.0),
    "MinY", DoubleValue (0.0),
    "DeltaX", DoubleValue (delta),
    "DeltaY", DoubleValue (delta),
    "GridWidth", UintegerValue (gridWidth),
    "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (c);

  InternetStackHelper internet;
  internet.Install (c);

  Ipv4AddressHelper ipv4;
  NS_LOG_INFO ("Assign IP Addresses.");
  ipv4.SetBase ("10.1.1.0", "255.255.255.0");
  Ipv4InterfaceContainer i = ipv4.Assign (devices);

  uint32_t nNodes = c.GetN ();
  std::vector<Ptr<Socket> > recvSinks;
  ApplicationContainer apps;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable> ();
  startTime->SetAttribute ("Max", DoubleValue (200.0));

  for (uint32_t i = 0; i < nNodes; ++i)
    {

       recvSinks.push_back(SetupPacketReceive (c.Get (i)));

       uint32_t j = (i+1) % nNodes;
       uint32_t interface = 
                c.Get (j)->GetObject<Ipv4> ()->GetInterfaceForDevice(c.Get (j)->GetDevice(0));
       InetSocketAddress remote =
        InetSocketAddress (c.Get (j)->GetObject<Ipv4> ()->GetAddress(interface,0).GetLocal(), 80);

       OnOffHelper onoff ("ns3::UdpSocketFactory", Address (remote));
       onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=250]"));

       onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
       onoff.SetAttribute ("DataRate", DataRateValue (DataRate (60000000)));
       onoff.SetAttribute ("PacketSize", UintegerValue (pktSize));

       ApplicationContainer app = onoff.Install (c.Get (i));
       app.Start(Seconds (0.5) + 
                 NanoSeconds(startTime->GetInteger ()));
       app.Stop (Seconds (duration + 0.5));

       apps.Add (app);
    } 

  // Trace CW evolution
  Config::ConnectWithoutContext ("/NodeList/0/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::RegularWifiMac/DcaTxop/DcfCwTrace", MakeCallback (&CwTrace));

  Simulator::Stop (Seconds (duration + 0.5));

  Simulator::Run ();
  Simulator::Destroy ();

  double mbs = ((m_bytesTotal * 8.0) / (1000000 * duration)); // Mb/s
  m_output.Add (networkSize, mbs);

  return m_output;
}

int main (int argc, char *argv[])
{
  uint32_t verbose = 0;
  double duration = 50;
  uint32_t netSize = 50;
  uint32_t pktSize = 1000;
  double delta = 0.1;
  uint32_t gridWidth = 10;
  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("2200"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("2200"));

  cwTraceFile.open ("wifi-a-cw-trace.out");

  // Align with OFDM standard values
  Config::SetDefault ("ns3::DcaTxop::MinCw", UintegerValue (15));
  Config::SetDefault ("ns3::DcaTxop::MaxCw", UintegerValue (1023));

  CommandLine cmd;
  cmd.AddValue ("verbose", "Show log output (default is 0: no log)", verbose);
  cmd.AddValue ("netSize", "The maximal Network Size", netSize);
  cmd.AddValue ("pktSize", "The frame size", pktSize);
  cmd.AddValue ("delta", "The delta offset in grid topology", delta);
  cmd.AddValue ("gridWidth", "The width of the grid", gridWidth);
  cmd.AddValue ("duration", "Time duration for each case (seconds)", duration);
  cmd.Parse (argc, argv);

  if (verbose == 1)
    {
      LogComponentEnable ("WifiBianchiValidation", LOG_LEVEL_ALL);
    }   
  else if (verbose == 2)
    {
      
      LogComponentEnable ("WifiBianchiValidation", LOG_LEVEL_ALL);
      LogComponentEnable ("DcfManager", LOG_LEVEL_ALL);
    }

  std::stringstream ss;
  ss << "wifi-a-"<<netSize<<"-p-"<<pktSize<<"-throughput.plt";
  std::ofstream netSizeThroughputPlot (ss.str().c_str());
  ss.str("");
  ss << "wifi-a-"<<netSize<<"-p-"<<pktSize<<"-throughput.eps";
  Gnuplot gnuplot = Gnuplot (ss.str());

  Experiment experiment;
  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");
  Gnuplot2dDataset dataset;

#if 0
  NS_LOG_DEBUG ("54");
  experiment = Experiment ("54mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate54Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);

  NS_LOG_DEBUG ("48");
  experiment = Experiment ("48mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate48Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);
 
  NS_LOG_DEBUG ("36");
  experiment = Experiment ("36mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate36Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);
 
  NS_LOG_DEBUG ("24");
  experiment = Experiment ("24mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate24Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);
 
  NS_LOG_DEBUG ("18");
  experiment = Experiment ("18mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate18Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);
 
  NS_LOG_DEBUG ("12");
  experiment = Experiment ("12mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate12Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);
#endif
 
  NS_LOG_DEBUG ("9");
  experiment = Experiment ("9mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate9Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      std::cout << "Running 9 Mb/s experiment for " << n << " nodes " << std::endl;
      cwTraceFile << "# 9 Mb/s rate; " << n << " nodes" << std::endl;
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);
 
  NS_LOG_DEBUG ("6");
  experiment = Experiment ("6mb");
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate6Mbps"));
  for (uint32_t n = 5; n <= netSize; n += 5)
    {
      std::cout << "Running 6 Mb/s experiment for " << n << " nodes " << std::endl;
      cwTraceFile << "# 6 Mb/s rate; " << n << " nodes" << std::endl;
      dataset = experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration);
    }
  gnuplot.AddDataset (dataset);

  gnuplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  gnuplot.SetLegend ("Number of competing stations", "Throughput (Mbps)");
  ss.str("");
  ss << "Frame size "<< pktSize << " bytes";
  gnuplot.SetTitle  (ss.str());
  gnuplot.SetExtra  ("#set xrange [0:50]\n\
#set yrange [0:54]\n\
set grid xtics ytics\n\
set mytics\n\
set style line 1 linewidth 5\n\
set style line 2 linewidth 5\n\
set style line 3 linewidth 5\n\
set style line 4 linewidth 5\n\
set style line 5 linewidth 5\n\
set style line 6 linewidth 5\n\
set style line 7 linewidth 5\n\
set style line 8 linewidth 5\n\
set style increment user");
  gnuplot.GenerateOutput (netSizeThroughputPlot);
  netSizeThroughputPlot.close ();

  cwTraceFile.close ();
  return 0;
}
