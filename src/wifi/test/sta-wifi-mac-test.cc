/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2018
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
 * Authors: Muhammad Iqbal CR <muh.iqbal.cr@gmail.com>
 */

#include "ns3/boolean.h"
#include "ns3/config.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-helper.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/pointer.h"
#include "ns3/simulator.h"
#include "ns3/test.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/wifi-mac-helper.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE ("StaWifiMacTest");

//-----------------------------------------------------------------------------
/**
 * Make sure that Wifi STA is correctly associating to the nearest AP
 * even when the AP operates on different channel number.
 *
 * See \bugid{2399}
 */

class StaWifiMacTestCase : public TestCase
{
public:
  StaWifiMacTestCase ();
  virtual ~StaWifiMacTestCase ();
  virtual void DoRun (void);

private:
  /**
   * Callback function on STA assoc event
   * \param context context string
   * \param bssid the associated AP's bssid
   */
  void AssocCallback (std::string context, Mac48Address bssid);
  /**
   * Callback function on STA RX event
   * \param context context string
   * \param p the received packet
   */
  void RxCallback (std::string context, Ptr<const Packet> p);
  /**
   * Turn beacon generation on the AP node
   * \param apNode the AP node
   */
  void TurnBeaconGenerationOn (Ptr<Node> apNode);

  Mac48Address m_associatedApBssid;
};

StaWifiMacTestCase::StaWifiMacTestCase ()
  : TestCase ("Test case for StaWifiMac")
{
}

StaWifiMacTestCase::~StaWifiMacTestCase ()
{
}

void
StaWifiMacTestCase::AssocCallback (std::string context, Mac48Address bssid)
{
  NS_LOG_DEBUG ("STA has associated with " << bssid);
  m_associatedApBssid = bssid;
}

void
StaWifiMacTestCase::RxCallback (std::string context, Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy ();
  WifiMacHeader hdr;
  packet->RemoveHeader (hdr);
  if (hdr.IsBeacon ())
    {
      Mac48Address bssid = hdr.GetAddr3 ();
      NS_LOG_DEBUG ("Received beacon from " << bssid);
    }
}

void
StaWifiMacTestCase::TurnBeaconGenerationOn (Ptr<Node> apNode)
{
  NS_LOG_DEBUG ("Turning beacon generation on node " << apNode->GetId ());
  Config::Set ("/NodeList/" + std::to_string(apNode->GetId ()) + "/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::ApWifiMac/BeaconGeneration",
               BooleanValue (true));
}

void
StaWifiMacTestCase::DoRun (void)
{
  NodeContainer apNodes;
  apNodes.Create (2);

  Ptr<Node> apNodeNearest = CreateObject<Node> ();
  Ptr<Node> staNode = CreateObject<Node> ();

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager");

  WifiMacHelper mac;
  NetDeviceContainer apDevice, apDeviceNearest;
  mac.SetType ("ns3::ApWifiMac",
               "BeaconGeneration", BooleanValue (true));
  apDevice = wifi.Install (phy, mac, apNodes);
  mac.SetType ("ns3::ApWifiMac",
               "BeaconGeneration", BooleanValue (false));
  apDeviceNearest = wifi.Install (phy, mac, apNodeNearest);

  NetDeviceContainer staDevice;
  mac.SetType ("ns3::StaWifiMac");
  staDevice = wifi.Install (phy, mac, staNode);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (10.0, 0.0, 0.0));
  positionAlloc->Add (Vector (5.0, 5.0, 0.0));
  positionAlloc->Add (Vector (5.0, 6.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (apNodes);
  mobility.Install (apNodeNearest);
  mobility.Install (staNode);

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc", MakeCallback (&StaWifiMacTestCase::AssocCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxBegin", MakeCallback (&StaWifiMacTestCase::RxCallback, this));

  Simulator::Schedule (Seconds (0.05), &StaWifiMacTestCase::TurnBeaconGenerationOn, this, apNodeNearest);

  Simulator::Stop (Seconds (0.2));
  Simulator::Run ();
  Simulator::Destroy ();

  NS_TEST_ASSERT_MSG_EQ (m_associatedApBssid, Mac48Address ("00:00:00:00:00:03"), "STA associated to the wrong AP");
}

/**
 * \ingroup sta-wifi-mac-test
 * \ingroup tests
 *
 * \brief StaWifiMac Test Suite
 */
class StaWifiMacTestSuite : public TestSuite
{
public:
  StaWifiMacTestSuite ();
};

StaWifiMacTestSuite::StaWifiMacTestSuite ()
  : TestSuite ("sta-wifi-mac", UNIT)
{
  AddTestCase (new StaWifiMacTestCase, TestCase::QUICK);
}

static StaWifiMacTestSuite g_staWifiMacTestSuite; ///< the test suite
