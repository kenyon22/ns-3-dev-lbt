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
#include "ns3/wifi-module.h"
#include "ns3/network-module.h"
#include "ns3/applications-module.h"
#include <iostream>
#include <sstream>

NS_LOG_COMPONENT_DEFINE ("WifiBianchi11aValidation");

using namespace ns3;

std::ofstream cwTraceFile;
std::ofstream backoffTraceFile;
std::ofstream phyTxTraceFile;
std::ofstream macTxTraceFile;
std::ofstream socketRecvTraceFile;
std::vector<uint32_t> packetsReceived (100);
std::vector<uint32_t> bytesReceived (100);

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

void
MacTxTrace (std::string context, Ptr<const Packet> p)
{
  macTxTraceFile << Simulator::Now ().GetSeconds () << " " << ContextToNodeId (context) << " " << p->GetSize () << std::endl;
}

void
SocketRecvTrace (std::string context, Ptr<const Packet> p, const Address &addr)
{
  socketRecvTraceFile << Simulator::Now ().GetSeconds () << " " << ContextToNodeId (context) << " " << p->GetSize () << std::endl;
}

void
SocketRecvStats (std::string context, Ptr<const Packet> p, const Address &addr)
{
  uint32_t nodeId = ContextToNodeId (context);
  bytesReceived[nodeId] += p->GetSize ();
  packetsReceived[nodeId]++;
}

class Experiment
{
public:
  Experiment ();
  int Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
           const WifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel,
           uint32_t pktSize, uint32_t netSize, double delta, uint32_t gridWidth,           double duration, double &val_needed, bool tracing);
private:
};

Experiment::Experiment ()
{
}

int
Experiment::Run (const WifiHelper &wifi, const YansWifiPhyHelper &wifiPhy,
                 const WifiMacHelper &wifiMac, const YansWifiChannelHelper &wifiChannel,
                 uint32_t pktSize, uint32_t networkSize, double delta, uint32_t gridWidth, double duration,  double &throughput, bool tracing)
{
  bool saveAttributeConfig = false;

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

  PacketSocketHelper packetSocket;
  packetSocket.Install (c);

  uint32_t nNodes = c.GetN ();
  ApplicationContainer apps;
  TypeId tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  Ptr<UniformRandomVariable> startTime = CreateObject<UniformRandomVariable> ();
  startTime->SetAttribute ("Max", DoubleValue (200.0));
  for (uint32_t i = 0; i < nNodes; ++i)
    {
      uint32_t j = (i + 1) % nNodes;
      PacketSocketAddress socketAddr;
      socketAddr.SetSingleDevice (devices.Get (i)->GetIfIndex ());
      socketAddr.SetPhysicalAddress (devices.Get (j)->GetAddress ());
      socketAddr.SetProtocol (1);

      Ptr<PacketSocketClient> client = CreateObject<PacketSocketClient> ();
      client->SetRemote (socketAddr);
      c.Get (i)->AddApplication (client);
      client->SetAttribute ("PacketSize", UintegerValue (pktSize));
      client->SetAttribute ("MaxPackets", UintegerValue (0));
      client->SetAttribute ("Interval", TimeValue (MilliSeconds (1)));

      Ptr<PacketSocketServer> server = CreateObject<PacketSocketServer> ();
      server->SetLocal (socketAddr);
      c.Get (j)->AddApplication (server);
    }

    // Log packet receptions
    Config::Connect ("/NodeList/*/$ns3::Node/ApplicationList/*/$ns3::PacketSocketServer/Rx", MakeCallback (&SocketRecvStats));

  if (tracing)
    {
      // Trace CW evolution
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::AdhocWifiMac/DcaTxop/DcfCwTrace", MakeCallback (&CwTrace));

      // Trace backoff evolution
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::AdhocWifiMac/DcaTxop/DcfBackoffTrace", MakeCallback (&BackoffTrace));

      // Trace Phy Tx start events
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyTxBegin", MakeCallback (&PhyTxTrace));

      // Trace packet arrivals to the Wifi device
      Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::AdhocWifiMac/MacTx", MakeCallback (&MacTxTrace));

      // Trace packet receptions
      Config::Connect ("/NodeList/*/$ns3::Node/ApplicationList/*/$ns3::PacketSocketServer/Rx", MakeCallback (&SocketRecvTrace));
    }

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
  Simulator::Destroy ();
  if (tracing)
    {
      cwTraceFile.flush ();
      backoffTraceFile.flush ();
      phyTxTraceFile.flush ();
      macTxTraceFile.flush ();
      socketRecvTraceFile.flush ();
    }
  return 0;
}

int main (int argc, char *argv[])
{
  uint32_t verbose = 0;
  bool tracing = false;
  double duration = 300;
  uint32_t netSize = 50;
  uint32_t pktSize = 1500;
  double delta = 0.001;
  uint32_t trials = 20;
  uint32_t gridWidth = 10;

  // disable fragmentation
  Config::SetDefault ("ns3::WifiRemoteStationManager::FragmentationThreshold", StringValue ("22000"));
  Config::SetDefault ("ns3::WifiRemoteStationManager::RtsCtsThreshold", StringValue ("22000"));
  // Disable short retransmission failure (make retransmissions persistent)
  Config::SetDefault ("ns3::WifiRemoteStationManager::MaxSlrc", UintegerValue (10000));
  
  // Align with OFDM standard values
  Config::SetDefault ("ns3::DcaTxop::MinCw", UintegerValue (15));
  Config::SetDefault ("ns3::DcaTxop::MaxCw", UintegerValue (1023));

  CommandLine cmd;
  cmd.AddValue ("verbose", "Show log output (default is 0: no log)", verbose);
  cmd.AddValue ("tracing", "Generate trace files", tracing);
  cmd.AddValue ("netSize", "The maximum network size", netSize);
  cmd.AddValue ("pktSize", "The frame size", pktSize);
  cmd.AddValue ("trials", "The maximal number of runs per network size", trials);
  cmd.AddValue ("delta", "The delta offset in grid topology", delta);
  cmd.AddValue ("gridWidth", "The width of the grid", gridWidth);
  cmd.AddValue ("duration", "Time duration for each trial (seconds)", duration);
  cmd.Parse (argc, argv);

  if (tracing)
    {
      cwTraceFile.open ("wifi-11a-cw-trace.out");
      backoffTraceFile.open ("wifi-11a-backoff-trace.out");
      phyTxTraceFile.open ("wifi-11a-phy-tx-trace.out");
      macTxTraceFile.open ("wifi-11a-mac-tx-trace.out");
      socketRecvTraceFile.open ("wifi-11a-socket-recv-trace.out");
    }

  if (verbose == 1)
    {
      LogComponentEnable ("WifiBianchi11aValidation", LOG_LEVEL_ALL);
    }
  else if (verbose == 2)
    {
      LogComponentEnable ("WifiBianchi11aValidation", LOG_LEVEL_ALL);
      LogComponentEnable ("DcfManager", LOG_LEVEL_ALL);
      LogComponentEnable ("DcaTxop", LOG_LEVEL_ALL);
      LogComponentEnable ("EdcaTxopN", LOG_LEVEL_ALL);
    }
  else if (verbose == 2)
    {
      LogComponentEnable ("WifiBianchi11aValidation", LOG_LEVEL_ALL);
      WifiHelper h;
      h.EnableLogComponents ();
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

  double mean_t, throughput, stDev, throughputVector[trials];

  for (uint32_t n = 5; n <= netSize; n += 5)
    {

      mean_t = 0;
      throughput = 0;

      for (uint32_t run_index = 1; run_index <= trials; run_index++)
        {
          std::cout << "Trial " << run_index << " of " << trials << "; 6 Mb/s for " << n << " nodes " << std::endl;
          if (tracing)
            {
              cwTraceFile << "# Trial " << run_index << " of " << trials << "; 6 Mb/s for " << n << " nodes" << std::endl;
              backoffTraceFile << "# Trial " << run_index << " of " << trials << "; 6 Mb/s for " << n << " nodes" << std::endl;
              phyTxTraceFile << "# Trial " << run_index << " of " << trials << "; 6 Mb/s for " << n << " nodes" << std::endl;
              macTxTraceFile << "# Trial " << run_index << " of " << trials << "; 6 Mb/s for " << n << " nodes" << std::endl;
              socketRecvTraceFile << "# Trial " << run_index << " of " << trials << "; 6 Mb/s for " << n << " nodes" << std::endl;
            }
          experiment.Run (wifi, wifiPhy, wifiMac, wifiChannel, pktSize, n, delta, gridWidth, duration, throughput, tracing);
          mean_t += throughput;
          throughputVector[run_index - 1] = throughput;
        }

      mean_t = mean_t / trials;
      stDev = 0;
      for (uint32_t i = 0; i < trials; ++i)
        {
          stDev += pow (throughputVector[i] - mean_t, 2);
        }

      stDev = sqrt (stDev / (trials - 1));
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

  if (tracing)
    {
      cwTraceFile.close ();
      backoffTraceFile.close ();
      phyTxTraceFile.close ();
      macTxTraceFile.close ();
      socketRecvTraceFile.close ();
    }
  return 0;
}
