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
 *
 * Updated by Tom Henderson and Rohan Patidar
 */

#include "ns3/core-module.h"
#include "ns3/config-store-module.h"
#include "ns3/mobility-module.h"
#include "ns3/gnuplot.h"
#include "ns3/gnuplot.h"
#include "ns3/wifi-module.h"
#include "ns3/internet-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include "ns3/flow-monitor-module.h"
#include <iostream>
#include <sstream>

NS_LOG_COMPONENT_DEFINE ("WifiBianchi11aValidation");

using namespace ns3;

std::ofstream cwTraceFile;
std::ofstream backoffTraceFile;
std::ofstream phyTxTraceFile;

// Parse context strings of the form "/NodeList/3/DeviceList/1/Mac/Assoc"
// to extract the NodeId
uint32_t
ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);  // skip "/NodeList/"
  uint32_t pos = sub.find ("/Device");
  NS_LOG_DEBUG ("Found NodeId " << atoi (sub.substr (0, pos).c_str ()));
  return atoi (sub.substr (0,pos).c_str ());
}

void
CwTrace (std::string context, uint32_t oldVal, uint32_t newVal)
{
  cwTraceFile << Simulator::Now ().GetSeconds () << " " << ContextToNodeId (context) << " " << newVal << std::endl;
}

void
BackoffTrace (std::string context, uint32_t oldVal, uint32_t newVal)
{
  backoffTraceFile << Simulator::Now ().GetSeconds () << " " << ContextToNodeId (context) << " " << newVal << std::endl;
}

void
PhyTxTrace (std::string context, Ptr<const Packet> p)
{
  phyTxTraceFile << Simulator::Now ().GetSeconds () << " " << ContextToNodeId (context) << " " << p->GetSize () << std::endl;
}

class Experiment
{
public:
  Experiment ();
  int Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
           const WifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel,
           uint32_t pktSize, uint32_t netSize, double delta, uint32_t gridWidth, double duration, double &val_needed);
private:
  void ReceivePacket (Ptr<Socket> socket);
  void SetPosition (Ptr<Node> node, Vector position);
  Ptr <Application> GetOnOff (Ptr<Node> node);
  Ptr<Socket> SetupPacketReceive (Ptr<Node> node);

  uint32_t m_bytesTotal;
};

Experiment::Experiment ()
{
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
  sink->Bind (InetSocketAddress (Ipv4Address ("0.0.0.0"),80));
  sink->SetRecvCallback (MakeCallback (&Experiment::ReceivePacket, this));

  return sink;
}

int
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const WifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel,
                 uint32_t pktSize, uint32_t networkSize, double delta, uint32_t gridWidth, double duration,  double &thput)
{
  bool saveAttributeConfig = false;
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
  Ipv4InterfaceContainer interface = ipv4.Assign (devices);

  uint32_t nNodes = c.GetN ();
  std::vector<Ptr<Socket> > recvSinks;
  ApplicationContainer apps;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable> ();
  startTime->SetAttribute ("Max", DoubleValue (200.0));
  //phy.EnablePcapAll("myfirst");
  for (uint32_t i = 0; i < nNodes; ++i)
    {
      recvSinks.push_back (SetupPacketReceive (c.Get (i)));

      uint32_t j = (i + 1) % nNodes;
      uint32_t interface =
        c.Get (j)->GetObject<Ipv4> ()->GetInterfaceForDevice (c.Get (j)->GetDevice (0));
      InetSocketAddress remote =
        InetSocketAddress (c.Get (j)->GetObject<Ipv4> ()->GetAddress (interface,0).GetLocal (), 80);

      OnOffHelper onoff ("ns3::UdpSocketFactory", Address (remote));
      onoff.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=250]"));

      onoff.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));
      onoff.SetAttribute ("DataRate", DataRateValue (DataRate (60000000)));
      onoff.SetAttribute ("PacketSize", UintegerValue (pktSize));

      ApplicationContainer app = onoff.Install (c.Get (i));
      app.Start (Seconds (0.5) +
                 NanoSeconds (startTime->GetInteger ()));
      app.Stop (Seconds (duration + 0.5));

      apps.Add (app);
    }

  // Trace CW evolution
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::AdhocWifiMac/DcaTxop/DcfCwTrace", MakeCallback (&CwTrace));

  // Trace backoff evolution
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::AdhocWifiMac/DcaTxop/DcfBackoffTrace", MakeCallback (&BackoffTrace));

  // Trace Phy Tx start events
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin", MakeCallback (&PhyTxTrace));

  FlowMonitorHelper flowmon;
  Ptr<FlowMonitor> monitor = flowmon.InstallAll ();
  Simulator::Stop (Seconds (duration + 0.5));

  if (saveAttributeConfig)
    {
      // Output config store to txt format
      Config::SetDefault ("ns3::ConfigStore::Filename", StringValue ("output-attributes.txt"));
      Config::SetDefault ("ns3::ConfigStore::FileFormat", StringValue ("RawText"));
      Config::SetDefault ("ns3::ConfigStore::Mode", StringValue ("Save"));
      ConfigStore outputConfig2;
      outputConfig2.ConfigureAttributes ();
    }

  Simulator::Run ();
  double totalbytes = 0;

  monitor->CheckForLostPackets ();
  Ptr<Ipv4FlowClassifier> classifier = DynamicCast<Ipv4FlowClassifier> (flowmon.GetClassifier ());
  std::map<FlowId, FlowMonitor::FlowStats> stats = monitor->GetFlowStats ();
  for (std::map<FlowId, FlowMonitor::FlowStats>::const_iterator i = stats.begin (); i != stats.end (); ++i)
    {
      //Ipv4FlowClassifier::FiveTuple t = classifier->FindFlow (i->first);
      // std::cout << "Flow " << i->first  << " (" << t.sourceAddress << " -> " << t.destinationAddress << ")\n";
      std::cout << "  Tx Bytes:   " << i->second.txBytes << "\n";
      std::cout << "  Rx Bytes:   " << i->second.rxBytes << "\n";
      //std::cout << i->second.rxBytes * 8.0 / (i->second.timeLastRxPacket.GetSeconds() - i->second.timeFirstTxPacket.GetSeconds())/1000/1000  << "\n";
      totalbytes +=  i->second.rxBytes;
    }

  Simulator::Destroy ();
  thput = ((totalbytes * 8.0) / (1000 * 1000 * duration)); // Mb/s
  return 0;
}

int main (int argc, char *argv[])
{
  uint32_t verbose = 0;
  double duration = 300;
  uint32_t netSize = 50;
  uint32_t pktSize = 1500;
  double delta = 0.001;
  uint32_t rmax = 20;
  uint32_t gridWidth = 10;

  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("22000"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("22000"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue (1000));
  cwTraceFile.open ("wifi-11a-cw-trace.out");
  backoffTraceFile.open ("wifi-11a-backoff-trace.out");
  phyTxTraceFile.open ("wifi-11a-phy-tx-trace.out");

  // Align with OFDM standard values
  Config::SetDefault ("ns3::DcaTxop::MinCw", UintegerValue (15));
  Config::SetDefault ("ns3::DcaTxop::MaxCw", UintegerValue (1023));

  CommandLine cmd;
  cmd.AddValue ("verbose", "Show log output (default is 0: no log)", verbose);
  cmd.AddValue ("netSize", "The maximal Network Size", netSize);
  cmd.AddValue ("pktSize", "The frame size", pktSize);
  cmd.AddValue ("rmax", "The maximal number of runs per network size", rmax);
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
  ss << "wifi-11a-" << netSize << "-p-" << pktSize << "-throughput.plt";
  std::ofstream netSizeThroughputPlot (ss.str ().c_str ());
  ss.str ("");
  ss << "wifi-11a-" << netSize << "-p-" << pktSize << "-throughput.eps";
  Gnuplot gnuplot = Gnuplot (ss.str ());


  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211a);
  WifiMacHelper wifiMac;
  wifiMac.SetType ("ns3::AdhocWifiMac");
  YansWifiPhyHelper wifiPhy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper wifiChannel;
  wifiChannel.SetPropagationDelay ("ns3::ConstantSpeedPropagationDelayModel");
  wifiChannel.AddPropagationLoss ("ns3::LogDistancePropagationLossModel");

  NS_LOG_DEBUG ("6");
  Experiment experiment;
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("OfdmRate6Mbps"));


  Gnuplot2dDataset dataset;
  Gnuplot2dDataset dataset_bianchi;
  dataset.SetErrorBars (Gnuplot2dDataset::Y);
  dataset.SetStyle (Gnuplot2dDataset::LINES_POINTS);
  dataset_bianchi.SetStyle (Gnuplot2dDataset::LINES_POINTS);

  double mean_t,thput,stDev,thput_vector[rmax];


  for (uint32_t n = 5; n <= netSize; n += 5)
    {

      mean_t = 0;
      thput = 0;

      for (uint32_t run_index = 1; run_index <= rmax; run_index++)
        {
          std::cout << "Running 6 Mb/s experiment for " << n << " nodes " << std::endl;
          cwTraceFile << "# 6 Mb/s rate; " << n << " nodes" << std::endl;
          backoffTraceFile << "# 6 Mb/s rate; " << n << " nodes" << std::endl;
          experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration, thput);
          mean_t += thput;
          thput_vector[run_index - 1] = thput;
        }

      mean_t = mean_t / rmax;
      stDev = 0;
      for (uint32_t i = 0; i < rmax; ++i)
        {
          stDev += pow (thput_vector[i] - mean_t, 2);
        }

      stDev = sqrt (stDev / (rmax - 1));
      dataset.Add (n, mean_t, stDev);

      std::cout << mean_t;
    }

  dataset_bianchi.Add (5, 4.7033);
  dataset_bianchi.Add (10,4.3185);
  dataset_bianchi.Add (15,4.1012);
  dataset_bianchi.Add (20,3.9482);
  dataset_bianchi.Add (25,3.8289);
  dataset_bianchi.Add (30,3.7304);
  dataset_bianchi.Add (35,3.6459);
  dataset_bianchi.Add (40,3.5718);
  dataset_bianchi.Add (45,3.5055);
  dataset_bianchi.Add (50,3.4454);

  gnuplot.AddDataset (dataset);
  gnuplot.SetTerminal ("postscript eps color enh \"Times-BoldItalic\"");
  gnuplot.SetLegend ("Number of competing stations", "Throughput (Mbps)");
  ss.str ("");
  ss << "Frame size " << pktSize << " bytes";
  gnuplot.SetTitle  (ss.str ());
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
  gnuplot.AddDataset (dataset_bianchi);
  gnuplot.GenerateOutput (netSizeThroughputPlot);
  netSizeThroughputPlot.close ();

  cwTraceFile.close ();
  backoffTraceFile.close ();
  phyTxTraceFile.close ();
  return 0;
}
