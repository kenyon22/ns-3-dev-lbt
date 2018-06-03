#include "ns3/test.h"
#include "ns3/pointer.h"
#include "ns3/config.h"
#include "ns3/string.h"
#include "ns3/log.h"
#include "ns3/yans-wifi-helper.h"
#include "ns3/mobility-helper.h"
#include "ns3/node-container.h"
#include "ns3/wifi-helper.h"
#include "ns3/wifi-mac-helper.h"
#include "ns3/net-device-container.h"
#include "ns3/mgt-headers.h"
#include "ns3/wifi-mac-header.h"
#include "ns3/error-model.h"

NS_LOG_COMPONENT_DEFINE ("AddBaTest");

using namespace ns3;

class Bug2470TestCase : public TestCase
{
public:
  Bug2470TestCase ();
  virtual ~Bug2470TestCase ();
  virtual void DoRun (void);

private:
  void RxCallback (std::string context, Ptr<const Packet> p);
  /**
   * Triggers the arrival of a burst of 1000 Byte-long packets in the source device
   * \param numPackets number of packets in burst (maximum: 255)
   * \param sourceDevice pointer to the source NetDevice
   * \param destination address of the destination device
   */
  void SendPacketBurst (uint8_t numPackets, Ptr<NetDevice> sourceDevice, Address& destination) const;
};

Bug2470TestCase::Bug2470TestCase ()
  : TestCase ("Test case for Bug 2470")
{
}

Bug2470TestCase::~Bug2470TestCase ()
{
}

void
Bug2470TestCase::RxCallback (std::string context, Ptr<const Packet> p)
{
  Ptr<Packet> packet = p->Copy ();
  WifiMacHeader hdr;
  packet->RemoveHeader (hdr);
  if (hdr.IsAction ())
    {
      WifiActionHeader actionHdr;
      packet->RemoveHeader (actionHdr);
      NS_LOG_DEBUG ("Packet UID " << packet->GetUid () << "; BA action " << actionHdr);
    }
}

void
Bug2470TestCase::SendPacketBurst (uint8_t numPackets, Ptr<NetDevice> sourceDevice,
                                  Address& destination) const
{
  for (uint8_t i = 0; i < numPackets; i++)
    {
      Ptr<Packet> pkt = Create<Packet> (1000);  // 1000 dummy bytes of data
      sourceDevice->Send (pkt, destination, 0);
    }
}

void
Bug2470TestCase::DoRun (void)
{
  NodeContainer wifiApNode, wifiStaNode;
  wifiApNode.Create (1);
  wifiStaNode.Create (1);

  YansWifiPhyHelper phy = YansWifiPhyHelper::Default ();
  YansWifiChannelHelper channel = YansWifiChannelHelper::Default ();
  phy.SetChannel (channel.Create ());

  WifiHelper wifi;
  wifi.SetStandard (WIFI_PHY_STANDARD_80211n_2_4GHZ);
  wifi.SetRemoteStationManager ("ns3::ConstantRateWifiManager",
                                "DataMode", StringValue ("HtMcs7"));

  WifiMacHelper mac;
  NetDeviceContainer apDevice, staDevice;
  mac.SetType ("ns3::ApWifiMac");
  apDevice = wifi.Install (phy, mac, wifiApNode);
  mac.SetType ("ns3::StaWifiMac");
  staDevice = wifi.Install (phy, mac, wifiStaNode);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc = CreateObject<ListPositionAllocator> ();
  positionAlloc->Add (Vector (0.0, 0.0, 0.0));
  positionAlloc->Add (Vector (1.0, 0.0, 0.0));
  mobility.SetPositionAllocator (positionAlloc);

  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (wifiApNode);
  mobility.Install (wifiStaNode);

  /* Unblock when ErrorModel is implemented for WifiNetDevice
  // Create ListErrorModel to corrupt ADDBA req packet
  Ptr<ListErrorModel> pem = CreateObject<ListErrorModel> ();
  // ADDBA req is uid 15
  std::list<uint32_t> sampleList;
  sampleList.push_back (15);
  pem->SetList (sampleList);
  staDevice.Get (0)->SetAttribute ("ReceiveErrorModel", PointerValue (pem));
  */

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxBegin", MakeCallback (&Bug2470TestCase::RxCallback, this));

  Simulator::Schedule (Seconds (0.5), &Bug2470TestCase::SendPacketBurst, this, 5, apDevice.Get (0), staDevice.Get (0)->GetAddress ());

  Simulator::Stop (Seconds (0.8));
  Simulator::Run ();

  Simulator::Destroy ();
}


/**
 * \ingroup wifi-test
 * \ingroup tests
 *
 * \brief ADDBA Test Suite
 */
class AddBaTestSuite : public TestSuite
{
public:
  AddBaTestSuite ();
};

AddBaTestSuite::AddBaTestSuite ()
  : TestSuite ("addba-test", UNIT)
{
  AddTestCase (new Bug2470TestCase, TestCase::QUICK); //Bug 2470
}

static AddBaTestSuite g_addbaTestSuite; ///< the test suite