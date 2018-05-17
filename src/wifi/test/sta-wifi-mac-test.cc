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

#include "ns3/config.h"
#include "ns3/constant-position-mobility-model.h"
#include "ns3/constant-velocity-mobility-model.h"
#include "ns3/mac48-address.h"
#include "ns3/mobility-model.h"
#include "ns3/log.h"
#include "ns3/node-container.h"
#include "ns3/object-factory.h"
#include "ns3/pointer.h"
#include "ns3/propagation-delay-model.h"
#include "ns3/propagation-loss-model.h"
#include "ns3/simulator.h"
#include "ns3/string.h"
#include "ns3/test.h"
#include "ns3/yans-error-rate-model.h"
#include "ns3/yans-wifi-channel.h"
#include "ns3/yans-wifi-phy.h"
#include "ns3/wifi-mac.h"
#include "ns3/wifi-net-device.h"
#include "ns3/wifi-remote-station-manager.h"
#include <string>
#include <tuple>
#include <vector>

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
   * A tuple of {STA node id, associated MAC address}
   */
  typedef std::tuple<uint32_t, Mac48Address> AssocTuple;
  std::vector<AssocTuple> m_assocLog; ///< vector containing assoc log on STA assoc
  Ptr<ConstantVelocityMobilityModel> m_mobileStaMobility; ///< variable to contain mobile sta mobility model

  /**
   * Function to get node ID from context string
   * \param context context string
   */
  uint32_t ContextToNodeId(std::string context);
  /**
   * Function to create node
   * \param wifiMacTypeId type ID string
   * \param channel channel model
   * \param mobility mobility model
   * \param channelNumber channel number
   */
  Ptr<Node> CreateNode(std::string wifiMacTypeId, Ptr<YansWifiChannel> channel, Ptr<MobilityModel> mobility, uint8_t channelNumber);
  /**
   * Callback function on STA assoc event
   * \param context context string
   * \param bssidAddr the associated AP's MAC address
   */
  void AssocCallback (std::string context, Mac48Address bssidAddr);
  /**
   * Function to run test with static STA
   */
  void RunStaticStaTest();
  /**
   * Function to run test with mobile STA
   */
  void RunMobileStaTest();
  /**
   * Function to change STA velocity on mobile STA test
   * \param vector speed vector
   */
  void SetStaVelocity(Vector vector);
};

StaWifiMacTestCase::StaWifiMacTestCase ()
  : TestCase ("Test case for StaWifiMac with static STA")
{
}

StaWifiMacTestCase::~StaWifiMacTestCase ()
{
}

uint32_t
StaWifiMacTestCase::ContextToNodeId (std::string context)
{
  std::string sub = context.substr (10);  // skip "/NodeList/"
  uint32_t pos = sub.find ("/Device");
  return atoi (sub.substr (0, pos).c_str ());
}

Ptr<Node>
StaWifiMacTestCase::CreateNode (std::string wifiMacTypeId, Ptr<YansWifiChannel> channel, Ptr<MobilityModel> mobility, uint8_t channelNumber)
{
	Ptr<Node> node = CreateObject<Node> ();
  Ptr<WifiNetDevice> dev = CreateObject<WifiNetDevice> ();
  ObjectFactory mac;
  mac.SetTypeId (wifiMacTypeId);
  Ptr<WifiMac> wifiMac = mac.Create<WifiMac> ();
  wifiMac->ConfigureStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);

  Ptr<ErrorRateModel> error = CreateObject<YansErrorRateModel> ();
  Ptr<YansWifiPhy> phy = CreateObject<YansWifiPhy> ();
  phy->SetErrorRateModel (error);
  phy->SetChannel (channel);
  phy->SetMobility (mobility);
  phy->SetDevice (dev);
  phy->ConfigureStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  phy->SetChannelNumber (channelNumber);
  phy->SetChannelWidth (20);

  wifiMac->SetAddress (Mac48Address::Allocate ());
  dev->SetMac (wifiMac);
  dev->SetPhy (phy);
  ObjectFactory manager;
  manager.SetTypeId ("ns3::ConstantRateWifiManager");
  dev->SetRemoteStationManager (manager.Create<WifiRemoteStationManager> ());
  node->AddDevice (dev);

  return node;
}

void
StaWifiMacTestCase::AssocCallback (std::string context, Mac48Address bssidAddr)
{
  uint32_t nodeId = ContextToNodeId (context);
  NS_LOG_DEBUG ("Node id " << nodeId << " has associated with " << bssidAddr);
  m_assocLog.push_back (std::make_tuple (nodeId, bssidAddr));
}

void
StaWifiMacTestCase::RunStaticStaTest ()
{
  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
  ObjectFactory propDelay;
  propDelay.SetTypeId ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<PropagationDelayModel> propagationDelay = propDelay.Create<PropagationDelayModel> ();
  Ptr<PropagationLossModel> propagationLoss = CreateObject<RandomPropagationLossModel> ();
  channel->SetPropagationDelayModel (propagationDelay);
  channel->SetPropagationLossModel (propagationLoss);

  NodeContainer apNodes = NodeContainer ();
  Ptr<ConstantPositionMobilityModel> apMobility = CreateObject<ConstantPositionMobilityModel> ();

  apMobility->SetPosition (Vector (0.0, 0.0, 0.0));
  apNodes.Add (CreateNode ("ns3::ApWifiMac", channel, apMobility, 1));
  apMobility->SetPosition (Vector (10.0, 0.0, 0.0));
  apNodes.Add (CreateNode ("ns3::ApWifiMac", channel, apMobility, 6));
  apMobility->SetPosition (Vector (5.0, 5.0, 0.0));
  apNodes.Add (CreateNode ("ns3::ApWifiMac", channel, apMobility, 11));

  NodeContainer staNodes = NodeContainer ();
  Ptr<ConstantPositionMobilityModel> staMobility = CreateObject<ConstantPositionMobilityModel> ();

  staMobility->SetPosition (Vector (1.0, 0.0, 0.0));
  staNodes.Add (CreateNode ("ns3::StaWifiMac", channel, staMobility, 1));
  staMobility->SetPosition (Vector (10.0, 1.0, 0.0));
  staNodes.Add (CreateNode ("ns3::StaWifiMac", channel, staMobility, 1));
  staMobility->SetPosition (Vector (4.0, 5.0, 0.0));
  staNodes.Add (CreateNode ("ns3::StaWifiMac", channel, staMobility, 1));

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc", MakeCallback (&StaWifiMacTestCase::AssocCallback, this));

  Simulator::Stop (Seconds (3.0));
  Simulator::Run ();
  Simulator::Destroy ();

  //\todo: Add test assert
}

void
StaWifiMacTestCase::SetStaVelocity (Vector vector)
{
  m_mobileStaMobility->SetVelocity (vector);
}

void
StaWifiMacTestCase::RunMobileStaTest ()
{
  Ptr<YansWifiChannel> channel = CreateObject<YansWifiChannel> ();
  ObjectFactory propDelay = ObjectFactory ("ns3::ConstantSpeedPropagationDelayModel");
  Ptr<PropagationDelayModel> propagationDelay = propDelay.Create<PropagationDelayModel> ();
  Ptr<PropagationLossModel> propagationLoss = CreateObject<RandomPropagationLossModel> ();
  channel->SetPropagationDelayModel (propagationDelay);
  channel->SetPropagationLossModel (propagationLoss);

  NodeContainer apNodes = NodeContainer ();
  Ptr<ConstantPositionMobilityModel> apMobility = CreateObject<ConstantPositionMobilityModel> ();

  apMobility->SetPosition (Vector (0.0, 0.0, 0.0));
  apNodes.Add (CreateNode ("ns3::ApWifiMac", channel, apMobility, 1));
  apMobility->SetPosition (Vector (10.0, 0.0, 0.0));
  apNodes.Add (CreateNode ("ns3::ApWifiMac", channel, apMobility, 1));

  m_mobileStaMobility = CreateObject<ConstantVelocityMobilityModel> ();
  m_mobileStaMobility->SetPosition (Vector (1.0, 0.0, 0.0));
  Ptr<Node> staNode = CreateNode ("ns3::StaWifiMac", channel, m_mobileStaMobility, 1);

  Simulator::Schedule (Seconds (1.0), &StaWifiMacTestCase::SetStaVelocity, this, Vector (2.0, 0.0, 0.0));
  Simulator::Schedule (Seconds (1.0), &StaWifiMacTestCase::SetStaVelocity, this, Vector (0.0, 0.0, 0.0));

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/$ns3::StaWifiMac/Assoc", MakeCallback (&StaWifiMacTestCase::AssocCallback, this));

  Simulator::Stop (Seconds (6.0));
  Simulator::Run ();
  Simulator::Destroy ();

  //\todo: Add test assert
}

void
StaWifiMacTestCase::DoRun (void)
{
  {
    RunStaticStaTest();
  }

  {
    RunMobileStaTest();
  }
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
