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
#include "ns3/boolean.h"

NS_LOG_COMPONENT_DEFINE ("AddBaTest");

using namespace ns3;

//-----------------------------------------------------------------------------
/**
 * Make sure that the ADDBA handshake process is protected.
 *
 * The scenario considers an access point and a station and utilizes
 * ReceiveListErrorModel to filter out received ADDBA request on STA or ADDBA
 * response on AP. The AP sends 5 packets each 1000 bytes (thus generating BA
 * agreement), 2 times during the test at 0.5s and 0.8s. The filtering process
 * happens only at the first ADDBA request. We expect that the packets in queue
 * after BA agreement fail are still sent with normal MPDU, and packets queued
 * at 0.8s will be sent with AMPDU. This test consider 4 subtest scenario:
 *
 *   1. ADDBA request packets are blocked on receive at STA for 6 time (less
 *      than SSRC)
 *   2. ADDBA request packets are blocked on receive at STA for 7 time (equals
 *      SSRC, triggering transmission failure at AP)
 *   3. ADDBA response packets are blocked on receive at AP for 6 time (less
 *      than SSRC)
 *   4. ADDBA response packets are blocked on receive at STA for 7 time (equals
 *      SSRC, STA stops retransmission of ADDBA response)
 *
 * All subtests expects 10 data packets received at STA.
 *
 * See \bugid{2470}
 */

class Bug2470TestCase : public TestCase
{
public:
  Bug2470TestCase ();
  virtual ~Bug2470TestCase ();
  virtual void DoRun (void);

private:
  // Will be removed
  void RxDropCallback (std::string context, Ptr<const Packet> p);
  /**
   * Callback when packet received at MAC layer
   * \param context node context
   * \param p the received packet
   */
  void RxCallback (std::string context, Ptr<const Packet> p);
  /**
   * Triggers the arrival of a burst of 1000 Byte-long packets in the source device
   * \param numPackets number of packets in burst (maximum: 255)
   * \param sourceDevice pointer to the source NetDevice
   * \param destination address of the destination device
   */
  void SendPacketBurst (uint8_t numPackets, Ptr<NetDevice> sourceDevice, Address& destination) const;
  /**
   * Run subtest for this test suite
   * \param apErrorModel ErrorModel used for AP
   * \param staErrorModel ErrorModel used for STA
   */
  void RunSubtest (PointerValue apErrorModel, PointerValue staErrorModel);

  uint8_t m_receivedDataCount; ///< Count received data on STA
};

Bug2470TestCase::Bug2470TestCase ()
  : TestCase ("Test case for Bug 2470"),
    m_receivedDataCount (0)
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
  if (hdr.HasData ())
    {
      NS_LOG_DEBUG ("Receiving packet UID " << packet->GetUid () << " received");
      m_receivedDataCount++;
    }
}

void
Bug2470TestCase::RxDropCallback (std::string context, Ptr<const Packet> p)
{
  NS_LOG_DEBUG ("Packet UID " << p->GetUid () << " dropped");
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
Bug2470TestCase::RunSubtest (PointerValue apErrorModel, PointerValue staErrorModel)
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
  NetDeviceContainer apDevice;
  phy.Set ("ReceiveErrorModel", apErrorModel);
  mac.SetType ("ns3::ApWifiMac", "EnableBeaconJitter", BooleanValue (false));
  apDevice = wifi.Install (phy, mac, wifiApNode);

  NetDeviceContainer staDevice;
  phy.Set ("ReceiveErrorModel", staErrorModel);
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

  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Mac/MacRx", MakeCallback (&Bug2470TestCase::RxCallback, this));
  Config::Connect ("/NodeList/*/DeviceList/*/$ns3::WifiNetDevice/Phy/$ns3::WifiPhy/PhyRxDrop", MakeCallback (&Bug2470TestCase::RxDropCallback, this));

  Simulator::Schedule (Seconds (0.5), &Bug2470TestCase::SendPacketBurst, this, 5, apDevice.Get (0), staDevice.Get (0)->GetAddress ());
  Simulator::Schedule (Seconds (0.8), &Bug2470TestCase::SendPacketBurst, this, 5, apDevice.Get (0), staDevice.Get (0)->GetAddress ());

  Simulator::Stop (Seconds (1.0));
  Simulator::Run ();
  Simulator::Destroy ();
}

void
Bug2470TestCase::DoRun (void)
{
  // Create ReceiveListErrorModel to corrupt ADDBA req packet. We use ReceiveListErrorModel
  // instead of ListErrorModel since packet UID is incremented between simulations. But
  // problem may occur because of random stream, therefore we suppress usage of rng as
  // much as possible (i.e., removing beacon jitter).
  Ptr<ReceiveListErrorModel> staPem = CreateObject<ReceiveListErrorModel> ();
  // Block retransmission of ADDBA request 6 times (< SSRC)
  std::list<uint32_t> blackList;
  blackList.push_back (8);
  blackList.push_back (9);
  blackList.push_back (10);
  blackList.push_back (11);
  blackList.push_back (12);
  blackList.push_back (13);
  staPem->SetList (blackList);

  {
    RunSubtest (PointerValue (), PointerValue (staPem));
    NS_LOG_DEBUG ("num of received packet " << +m_receivedDataCount);
    NS_TEST_ASSERT_MSG_EQ (m_receivedDataCount, 10, "Packet reception unexpectedly stopped after failed BA agreement on subtest 1");
  }

  m_receivedDataCount = 0;
  Ptr<ReceiveListErrorModel> staPem2 = CreateObject<ReceiveListErrorModel> ();
  // Block retransmission of ADDBA request 7 times
  blackList.push_back (15);
  staPem2->SetList (blackList);

  {
    RunSubtest (PointerValue (), PointerValue (staPem2));
    NS_LOG_DEBUG ("num of received packet " << +m_receivedDataCount);
    NS_TEST_ASSERT_MSG_EQ (m_receivedDataCount, 10, "Packet reception unexpectedly stopped after failed BA agreement on subtest 2");
  }

  m_receivedDataCount = 0;
  Ptr<ReceiveListErrorModel> apPem = CreateObject<ReceiveListErrorModel> ();
  blackList.clear ();
  // Block retransmission of ADDBA response 6 times (< SSRC)
  blackList.push_back (4);
  blackList.push_back (5);
  blackList.push_back (6);
  blackList.push_back (7);
  blackList.push_back (8);
  blackList.push_back (9);
  apPem->SetList (blackList);

  {
    RunSubtest (PointerValue (apPem), PointerValue ());
    NS_LOG_DEBUG ("num of received packet " << +m_receivedDataCount);
    NS_TEST_ASSERT_MSG_EQ (m_receivedDataCount, 10, "Packet reception unexpectedly stopped after failed BA agreement on subtest 3");
  }

  m_receivedDataCount = 0;
  Ptr<ReceiveListErrorModel> apPem2 = CreateObject<ReceiveListErrorModel> ();
  // Block retransmission of ADDBA response 7 times
  blackList.push_back (10);
  apPem2->SetList (blackList);

  {
    RunSubtest (PointerValue (apPem2), PointerValue ());
    NS_LOG_DEBUG ("num of received packet " << +m_receivedDataCount);
    NS_TEST_ASSERT_MSG_EQ (m_receivedDataCount, 10, "Packet reception unexpectedly stopped after failed BA agreement on subtest 4");
  }
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