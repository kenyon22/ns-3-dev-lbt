/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2006 INRIA
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
 * Author: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 */

#include "ideal-wifi-manager.h"
#include "wifi-phy.h"
#include "ns3/log.h"

namespace ns3 {

/**
 * \brief hold per-remote-station state for Ideal Wifi manager.
 *
 * This struct extends from WifiRemoteStation struct to hold additional
 * information required by the Ideal Wifi manager
 */
struct IdealWifiRemoteStation : public WifiRemoteStation
{
  double m_lastSnrObserved;  //!< SNR of most recently reported packet sent to the remote station
  double m_lastSnrUsed;      //!< SNR most recently used to select a rate
  uint64_t m_lastRate;       //!< Last data rate used towards the station
  uint16_t m_guardInterval;
  uint8_t m_channelWidth;
  double m_nss;              //!< SNR most recently used to select a rate
  WifiMode m_lastMode;       //!< Mode most recently used to the remote station
};

// To avoid using the cache before a valid value has been cached
static const double CACHE_INITIAL_VALUE = -100;

NS_OBJECT_ENSURE_REGISTERED (IdealWifiManager);

NS_LOG_COMPONENT_DEFINE ("IdealWifiManager");

TypeId
IdealWifiManager::GetTypeId (void)
{
  static TypeId tid = TypeId ("ns3::IdealWifiManager")
    .SetParent<WifiRemoteStationManager> ()
    .SetGroupName ("Wifi")
    .AddConstructor<IdealWifiManager> ()
    .AddAttribute ("BerThreshold",
                   "The maximum Bit Error Rate acceptable at any transmission mode",
                   DoubleValue (1e-5),
                   MakeDoubleAccessor (&IdealWifiManager::m_ber),
                   MakeDoubleChecker<double> ())
    .AddTraceSource ("RateChange",
                     "The transmission rate has changed",
                     MakeTraceSourceAccessor (&IdealWifiManager::m_rateChange),
                     "ns3::IdealWifiManager::RateChangeTracedCallback")
  ;
  return tid;
}

IdealWifiManager::IdealWifiManager ()
{
}

IdealWifiManager::~IdealWifiManager ()
{
}

void
IdealWifiManager::SetupPhy (const Ptr<WifiPhy> phy)
{
  NS_LOG_FUNCTION (this << phy);
  WifiRemoteStationManager::SetupPhy (phy);
}

uint8_t
IdealWifiManager::GetChannelWidthForMode (WifiMode mode) const
{
  NS_ASSERT (mode.GetModulationClass () != WIFI_MOD_CLASS_HT
             && mode.GetModulationClass () != WIFI_MOD_CLASS_VHT
             && mode.GetModulationClass () != WIFI_MOD_CLASS_HE);
  if (mode.GetModulationClass () == WIFI_MOD_CLASS_DSSS
      || mode.GetModulationClass () == WIFI_MOD_CLASS_HR_DSSS)
    {
      return 22;
    }
  else
    {
      return 20;
    }
}

void
IdealWifiManager::DoInitialize ()
{
  NS_LOG_FUNCTION (this);
  WifiMode mode;
  WifiTxVector txVector;
  uint8_t nss = 1;
  uint32_t nModes = GetPhy ()->GetNModes ();
  for (uint32_t i = 0; i < nModes; i++)
    {
      mode = GetPhy ()->GetMode (i);
      txVector.SetChannelWidth (GetChannelWidthForMode (mode));
      txVector.SetNss (nss);
      txVector.SetMode (mode);
      NS_LOG_DEBUG ("Initialize, adding mode = " << mode.GetUniqueName ());
      AddSnrThreshold (txVector, GetPhy ()->CalculateSnr (txVector, m_ber));
    }
  // Add all Ht and Vht MCSes
  if (HasVhtSupported () == true || HasHtSupported () == true || HasHeSupported () == true)
    {
      nModes = GetPhy ()->GetNMcs ();
      for (uint32_t i = 0; i < nModes; i++)
        {
          for (uint16_t j = 20; j <= GetPhy ()->GetChannelWidth (); j*=2)
            {
              txVector.SetChannelWidth (j);
              mode = GetPhy ()->GetMcs (i);
              if (mode.GetModulationClass () == WIFI_MOD_CLASS_HT)
                {
                  uint16_t guardInterval = GetPhy ()->GetShortGuardInterval () ? 400 : 800;
                  txVector.SetGuardInterval (guardInterval);
                  //derive NSS from the MCS index
                  nss = (mode.GetMcsValue () / 8) + 1;
                  NS_LOG_DEBUG ("Initialize, adding mode = " << mode.GetUniqueName () <<
                                " channel width " << (uint16_t) j <<
                                " nss " << (uint16_t) nss <<
                                " GI " << guardInterval);
                  NS_LOG_DEBUG ("In SetupPhy, adding mode = " << mode.GetUniqueName ());
                  txVector.SetNss (nss);
                  txVector.SetMode (mode);
                  AddSnrThreshold (txVector, GetPhy ()->CalculateSnr (txVector, m_ber));
                }
              else //VHT or HE
                {
                  uint16_t guardInterval;
                  if (mode.GetModulationClass () == WIFI_MOD_CLASS_VHT)
                    {
                      guardInterval = GetPhy ()->GetShortGuardInterval () ? 400 : 800;
                    }
                  else
                    {
                      guardInterval = GetPhy ()->GetGuardInterval ().GetNanoSeconds ();
                    }
                  for (uint8_t i = 1; i <= GetPhy ()->GetMaxSupportedTxSpatialStreams (); i++)
                    {
                      NS_LOG_DEBUG ("Initialize, adding mode = " << mode.GetUniqueName () <<
                                    " channel width " << (uint16_t) j <<
                                    " nss " << (uint16_t) i <<
                                    " GI " << guardInterval);
                      NS_LOG_DEBUG ("In SetupPhy, adding mode = " << mode.GetUniqueName ());
                      txVector.SetNss (i);
                      txVector.SetMode (mode);
                      AddSnrThreshold (txVector, GetPhy ()->CalculateSnr (txVector, m_ber));
                    }
                }
            }
        }
    }
}

double
IdealWifiManager::GetSnrThreshold (WifiTxVector txVector) const
{
  NS_LOG_FUNCTION (this << txVector.GetMode ().GetUniqueName ());
  for (Thresholds::const_iterator i = m_thresholds.begin (); i != m_thresholds.end (); i++)
    {
      NS_LOG_DEBUG ("Checking " << i->second.GetMode ().GetUniqueName () <<
                    " nss " << (uint16_t) i->second.GetNss () <<
                    " GI " << i->second.GetGuardInterval () <<
                    " width " << (uint16_t) i->second.GetChannelWidth ());
      NS_LOG_DEBUG ("against TxVector " << txVector.GetMode ().GetUniqueName () <<
                    " nss " << (uint16_t) txVector.GetNss () <<
                    " GI " << txVector.GetGuardInterval () <<
                    " width " << (uint16_t) txVector.GetChannelWidth ());
      if (txVector.GetMode () == i->second.GetMode ()
          && txVector.GetNss () == i->second.GetNss ()
          && txVector.GetChannelWidth () == i->second.GetChannelWidth ())
        {
          return i->first;
        }
    }
  NS_ASSERT (false);
  return 0.0;
}

void
IdealWifiManager::AddSnrThreshold (WifiTxVector txVector, double snr)
{
  NS_LOG_FUNCTION (this << txVector.GetMode ().GetUniqueName () << snr);
  m_thresholds.push_back (std::make_pair (snr, txVector));
}

WifiRemoteStation *
IdealWifiManager::DoCreateStation (void) const
{
  NS_LOG_FUNCTION (this);
  IdealWifiRemoteStation *station = new IdealWifiRemoteStation ();
  station->m_lastSnrObserved = 0.0;
  station->m_lastSnrUsed = CACHE_INITIAL_VALUE;
  station->m_lastRate = 0;
  station->m_lastMode = GetDefaultMode ();
  station->m_nss = 1;
  return station;
}


void
IdealWifiManager::DoReportRxOk (WifiRemoteStation *station,
                                double rxSnr, WifiMode txMode)
{
}

void
IdealWifiManager::DoReportRtsFailed (WifiRemoteStation *station)
{
}

void
IdealWifiManager::DoReportDataFailed (WifiRemoteStation *station)
{
}

void
IdealWifiManager::DoReportRtsOk (WifiRemoteStation *st,
                                 double ctsSnr, WifiMode ctsMode, double rtsSnr)
{
  NS_LOG_FUNCTION (this << st << ctsSnr << ctsMode.GetUniqueName () << rtsSnr);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  station->m_lastSnrObserved = rtsSnr;
}

void
IdealWifiManager::DoReportDataOk (WifiRemoteStation *st,
                                  double ackSnr, WifiMode ackMode, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << ackSnr << ackMode.GetUniqueName () << dataSnr);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  if (dataSnr == 0)
    {
      NS_LOG_WARN ("DataSnr reported to be zero; not saving this report.");
      return;
    }
  station->m_lastSnrObserved = dataSnr;
}

void
IdealWifiManager::DoReportAmpduTxStatus (WifiRemoteStation *st, uint8_t nSuccessfulMpdus, uint8_t nFailedMpdus, double rxSnr, double dataSnr)
{
  NS_LOG_FUNCTION (this << st << (uint16_t)nSuccessfulMpdus << (uint16_t)nFailedMpdus << rxSnr << dataSnr);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  if (dataSnr == 0)
    {
      NS_LOG_WARN ("DataSnr reported to be zero; not saving this report.");
      return;
    }
  station->m_lastSnrObserved = dataSnr;
}


void
IdealWifiManager::DoReportFinalRtsFailed (WifiRemoteStation *station)
{
}

void
IdealWifiManager::DoReportFinalDataFailed (WifiRemoteStation *station)
{
}

// This check is encapsulated in case any more sophisticated checks
// for applicability of cached data are added in the future
bool
IdealWifiManager::UseCachedDataTxVector (WifiRemoteStation *st) const
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  if (station->m_lastSnrUsed != CACHE_INITIAL_VALUE && station->m_lastSnrObserved == station->m_lastSnrUsed)
    {
      return true;
    }
  return false;
}

void
IdealWifiManager::UpdateCachedDataTxVector (WifiRemoteStation *st, WifiMode mode, uint8_t nss, uint16_t guardInterval, uint8_t channelWidth)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  NS_LOG_DEBUG ("Mode found; updating cached values for station to " <<  mode.GetUniqueName () << " snr " << station->m_lastSnrObserved);
  station->m_lastSnrUsed = station->m_lastSnrObserved;
  station->m_lastMode = mode;
  station->m_nss = nss;
  station->m_guardInterval = guardInterval;
  station->m_channelWidth = channelWidth;
  uint64_t dataRate = mode.GetDataRate (channelWidth, guardInterval, nss);
  if (station->m_lastRate != dataRate)
    {
      NS_LOG_DEBUG ("Updated datarate: " << dataRate << " to station " << station->m_state->m_address);
      station->m_lastRate = dataRate;
      m_rateChange (dataRate, station->m_state->m_address);
    }
}

bool
IdealWifiManager::DoGetDataTxVectorHe (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  WifiMode mode;
  uint8_t selectedNss = 1;
  WifiTxVector txVector;
  // channel width and guard interval are not variable within here
  // txVector.SetChannelWidth (GetPhy ()->GetChannelWidth ());
  // txVector.SetShortGuardInterval (GetPhy ()->GetGuardInterval ());

  // Search within the supported rate set for the mode corresponding
  // to the highest rate with an SNR threshold smaller than the last
  // SNR reported from the remote station
  WifiMode maxMode = GetDefaultMode ();
  uint64_t bestRate = 0;
  uint16_t guardInterval = std::max (GetGuardInterval (station), static_cast<uint16_t> (GetPhy ()->GetGuardInterval ().GetNanoSeconds ()));
  txVector.SetGuardInterval (guardInterval);
  uint8_t channelWidth = std::min (GetChannelWidth (station), GetPhy ()->GetChannelWidth ());
  txVector.SetChannelWidth (channelWidth);

  bool found = false;
  for (uint32_t i = 0; i < GetNMcsSupported (station); i++)
    {
      mode = GetMcsSupported (station, i);
      txVector.SetMode (mode);
      // If the node and peer are not both HE capable, only search (V)HT modes
      if (!HasHeSupported () || !GetHeSupported (station))
        {
          continue;
        }
      for (uint8_t nss = 1; nss <= GetNumberOfSupportedStreams (station); nss++)
        {
          txVector.SetNss (nss);
          if (WifiPhy::IsValidTxVector (txVector) == false)
            {
              NS_LOG_DEBUG ("Skipping mode " << mode.GetUniqueName () <<
                            " nss " << (uint16_t) nss << " width " <<
                            (uint16_t) txVector.GetChannelWidth ());
              continue;
            }
          double threshold = GetSnrThreshold (txVector);
          uint64_t dataRate = mode.GetDataRate (txVector.GetChannelWidth (), txVector.GetGuardInterval (), nss);
          NS_LOG_DEBUG ("Testing mode = " << mode.GetUniqueName () <<
                        " data rate " << dataRate <<
                        " threshold " << threshold  << " last snr observed " <<
                        station->m_lastSnrObserved << " cached " <<
                        station->m_lastSnrUsed);
          if (dataRate > bestRate && threshold < station->m_lastSnrObserved)
            {
              NS_LOG_DEBUG ("Candidate mode = " << mode.GetUniqueName () <<
                            " data rate " << dataRate <<
                            " threshold " << threshold  <<
                            " last snr observed " <<
                            station->m_lastSnrObserved);
              bestRate = dataRate;
              maxMode = mode;
              selectedNss = nss;
              found = true;
            }
        }
    }

  if (found)
    {
      UpdateCachedDataTxVector (st, maxMode, selectedNss, guardInterval, channelWidth);
    }
  return found;
}

bool
IdealWifiManager::DoGetDataTxVectorVht (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  WifiMode mode;
  uint8_t selectedNss = 1;
  WifiTxVector txVector;
  // channel width and guard interval are not variable within here
  // txVector.SetChannelWidth (GetPhy ()->GetChannelWidth ());
  // txVector.SetShortGuardInterval (GetPhy ()->GetGuardInterval ());

  // Search within the supported rate set for the mode corresponding
  // to the highest rate with an SNR threshold smaller than the last
  // SNR reported from the remote station
  WifiMode maxMode = GetDefaultMode ();
  uint64_t bestRate = 0;
  uint16_t guardInterval = std::max (GetShortGuardInterval (station) ? 400 : 800, GetPhy ()->GetShortGuardInterval () ? 400 : 800);
  txVector.SetGuardInterval (guardInterval);
  uint8_t channelWidth = std::min (GetChannelWidth (station), GetPhy ()->GetChannelWidth ());
  txVector.SetChannelWidth (channelWidth);

  bool found = false;
  for (uint32_t i = 0; i < GetNMcsSupported (station); i++)
    {
      mode = GetMcsSupported (station, i);
      txVector.SetMode (mode);
      // If the node and peer are both HE capable, only search HE modes
      if (HasHeSupported () && GetHeSupported (station))
        {
          continue;
        }
      // If the node and peer are not both VHT capable, only search HT modes
      if (!HasVhtSupported () || !GetVhtSupported (station))
        {
          continue;
        }
      for (uint8_t nss = 1; nss <= GetNumberOfSupportedStreams (station); nss++)
        {
          txVector.SetNss (nss);
          if (WifiPhy::IsValidTxVector (txVector) == false)
            {
              NS_LOG_DEBUG ("Skipping mode " << mode.GetUniqueName () <<
                            " nss " << (uint16_t) nss << " width " <<
                            (uint16_t) txVector.GetChannelWidth ());
              continue;
            }
          double threshold = GetSnrThreshold (txVector);
          uint64_t dataRate = mode.GetDataRate (txVector.GetChannelWidth (), txVector.GetGuardInterval (), nss);
          NS_LOG_DEBUG ("Testing mode = " << mode.GetUniqueName () <<
                        " data rate " << dataRate <<
                        " threshold " << threshold << " last snr observed " <<
                        station->m_lastSnrObserved << " cached " <<
                        station->m_lastSnrUsed);
          if (dataRate > bestRate && threshold < station->m_lastSnrObserved)
            {
              NS_LOG_DEBUG ("Candidate mode = " << mode.GetUniqueName () <<
                            " data rate " << dataRate <<
                            " threshold " << threshold  <<
                            " last snr observed " <<
                            station->m_lastSnrObserved);
              bestRate = dataRate;
              maxMode = mode;
              selectedNss = nss;
              found = true;
            }
        }
    }

  if (found)
    {
      UpdateCachedDataTxVector (st, maxMode, selectedNss, guardInterval, channelWidth);
    }
  return found;
}

bool
IdealWifiManager::DoGetDataTxVectorHt (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  WifiMode mode;
  uint8_t selectedNss = 1;
  WifiTxVector txVector;
  // // channel width and guard interval are not variable within here
  // txVector.SetChannelWidth (GetPhy ()->GetChannelWidth ());
  // txVector.SetShortGuardInterval (GetPhy ()->GetGuardInterval ());

  // Search within the supported rate set for the mode corresponding
  // to the highest rate with an SNR threshold smaller than the last
  // SNR reported from the remote station
  WifiMode maxMode = GetDefaultMode ();
  uint64_t bestRate = 0;
  uint16_t guardInterval = std::max (GetShortGuardInterval (station) ? 400 : 800, GetPhy ()->GetShortGuardInterval () ? 400 : 800);
  txVector.SetGuardInterval (guardInterval);
  uint8_t channelWidth = std::min (GetChannelWidth (station), GetPhy ()->GetChannelWidth ());
  txVector.SetChannelWidth (channelWidth);

  bool found = false;
  for (uint32_t i = 0; i < GetNMcsSupported (station); i++)
    {
      mode = GetMcsSupported (station, i);
      txVector.SetMode (mode);
      // If the node and peer are both VHT capable, only search VHT modes
      if (HasVhtSupported () && GetVhtSupported (station))
        {
          continue;
        }
      // If the node and peer are both HE capable, only search HE modes
      if (HasHeSupported () && GetHeSupported (station))
        {
          continue;
        }
      // Derive NSS from the MCS index. There is a different mode for each possible NSS value.
      uint8_t nss = (mode.GetMcsValue () / 8) + 1;
      txVector.SetNss (nss);
      if (WifiPhy::IsValidTxVector (txVector) == false
          || nss > GetNumberOfSupportedStreams (st))
        {
          NS_LOG_DEBUG ("Skipping mode " << mode.GetUniqueName () <<
                        " nss " << (uint16_t) nss << " width " <<
                        (uint16_t) txVector.GetChannelWidth ());
          continue;
        }
      double threshold = GetSnrThreshold (txVector);
      uint64_t dataRate = mode.GetDataRate (txVector.GetChannelWidth (), txVector.GetGuardInterval (), nss);
      NS_LOG_DEBUG ("Testing mode " << mode.GetUniqueName () <<
                    " data rate " << dataRate <<
                    " threshold " << threshold  << " last snr observed " <<
                    station->m_lastSnrObserved << " cached " <<
                    station->m_lastSnrUsed);
      if (dataRate > bestRate && threshold < station->m_lastSnrObserved)
        {
          NS_LOG_DEBUG ("Candidate mode = " << mode.GetUniqueName () <<
                        " data rate " << dataRate <<
                        " threshold " << threshold  <<
                        " last snr observed " <<
                        station->m_lastSnrObserved);
          bestRate = dataRate;
          maxMode = mode;
          selectedNss = nss;
          found = true;
        }
    }

  if (found)
    {
      UpdateCachedDataTxVector (st, maxMode, selectedNss, guardInterval, channelWidth);
    }
  return found;
}

bool
IdealWifiManager::DoGetDataTxVectorLegacy (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  WifiMode mode;
  uint8_t selectedNss = 1;
  WifiTxVector txVector;
  // channel width and guard interval are not variable within here
  // txVector.SetChannelWidth (GetPhy ()->GetChannelWidth ());
  // txVector.SetShortGuardInterval (GetPhy ()->GetGuardInterval ());

  // Search within the supported rate set for the mode corresponding
  // to the highest rate with an SNR threshold smaller than the last
  // SNR reported from the remote station
  WifiMode maxMode = GetDefaultMode ();
  uint64_t bestRate = 0;
  uint16_t guardInterval = std::max (GetShortGuardInterval (station) ? 400 : 800, GetPhy ()->GetShortGuardInterval () ? 400 : 800);
  txVector.SetGuardInterval (guardInterval);
  uint8_t selectedChannelWidth;

  bool found = false;
  for (uint32_t i = 0; i < GetNSupported (station); i++)
    {
      mode = GetSupported (station, i);
      txVector.SetMode (mode);
      txVector.SetNss (selectedNss);
      txVector.SetChannelWidth (GetChannelWidthForMode (mode));
      double threshold = GetSnrThreshold (txVector);
      NS_LOG_DEBUG ("mode = " << mode.GetUniqueName () <<
                    " threshold " << threshold  <<
                    " last snr observed " <<
                    station->m_lastSnrObserved);
      // uint64_t dataRate = mode.GetDataRate (GetPhy ()->GetChannelWidth (), GetPhy ()->GetGuardInterval (), nss);
      uint64_t dataRate = mode.GetDataRate (txVector.GetChannelWidth (), txVector.GetGuardInterval (), txVector.GetNss ());
      // Prefer a mode if its data rate exceeds previous candidate
      if (dataRate > bestRate && threshold < station->m_lastSnrObserved)
        {
          NS_LOG_DEBUG ("Candidate mode = " << mode.GetUniqueName () <<
                        " data rate " << dataRate <<
                        " threshold " << threshold  <<
                        " last snr observed " <<
                        station->m_lastSnrObserved);
          bestRate = dataRate;
          maxMode = mode;
          selectedChannelWidth = txVector.GetChannelWidth ();
          found = true;
        }
    }

  if (found)
    {
      UpdateCachedDataTxVector (st, maxMode, selectedNss, guardInterval, selectedChannelWidth);
    }
  return found;
}

WifiTxVector
IdealWifiManager::DoGetDataTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  if (UseCachedDataTxVector (st))
    {
      NS_LOG_DEBUG ("Using cached WifiTxVector to station " << station->m_state->m_address);
      NS_LOG_DEBUG ("Returning cached mode: " <<
                    station->m_lastMode.GetUniqueName () <<
                    " channelWidth: " << GetChannelWidth (station) <<
                    " nss " << (uint16_t) station->m_nss <<
                    " dataRate: " << station->m_lastRate);
      return WifiTxVector (station->m_lastMode, GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetPreambleForTransmission (station->m_lastMode, GetAddress (station)), station->m_guardInterval, GetNumberOfAntennas (), station->m_nss, 0, station->m_channelWidth, GetAggregation (station), false);

    }
  bool found = false;
  if (HasHeSupported () == true && GetHeSupported (st))
    {
      NS_LOG_DEBUG ("Searching HE modes to station " << station->m_state->m_address);
      found = DoGetDataTxVectorHe (st);
    }
  if (!found && HasVhtSupported () == true && GetVhtSupported (st))
    {
      NS_LOG_DEBUG ("Searching VHT modes to station " << station->m_state->m_address);
      found = DoGetDataTxVectorVht (st);
    }
  if (!found && HasHtSupported () == true && GetHtSupported (st))
    {
      NS_LOG_DEBUG ("Searching HT modes to station " << station->m_state->m_address);
      found = DoGetDataTxVectorHt (st);
    }
  if (!found)
    {
      NS_LOG_DEBUG ("Searching legacy modes to station " << station->m_state->m_address);
      found = DoGetDataTxVectorLegacy (st);
    }
  if (found)
    {
      return WifiTxVector (station->m_lastMode, GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetPreambleForTransmission (station->m_lastMode, GetAddress (station)), station->m_guardInterval, GetNumberOfAntennas (), station->m_nss, 0, station->m_channelWidth, GetAggregation (station), false);
    }
  else
    {
      NS_LOG_DEBUG ("Suitable mode not found; returning default mode");
      return WifiTxVector (GetDefaultMode (), GetDefaultTxPowerLevel (), GetLongRetryCount (station), GetPreambleForTransmission (GetDefaultMode (), GetAddress (station)), 800, GetNumberOfAntennas (), 1, 0, GetChannelWidth (station), GetAggregation (station), false);
    }
}

WifiTxVector
IdealWifiManager::DoGetRtsTxVector (WifiRemoteStation *st)
{
  NS_LOG_FUNCTION (this << st);
  IdealWifiRemoteStation *station = (IdealWifiRemoteStation *)st;
  //We search within the Basic rate set the mode with the highest
  //snr threshold possible which is smaller than m_lastSnr to
  //ensure correct packet delivery.
  double maxThreshold = 0.0;
  WifiTxVector txVector;
  WifiMode mode;
  uint8_t nss = 1;
  WifiMode maxMode = GetDefaultMode ();
  //avoid to use legacy rate adaptation algorithms for IEEE 802.11n/ac/ax
  //RTS is sent in a legacy frame; RTS with HT/VHT/HE is not yet supported
  for (uint32_t i = 0; i < GetNBasicModes (); i++)
    {
      mode = GetBasicMode (i);
      txVector.SetMode (mode);
      txVector.SetNss (nss);
      txVector.SetChannelWidth (GetChannelWidthForMode (mode));
      double threshold = GetSnrThreshold (txVector);
      if (threshold > maxThreshold && threshold < station->m_lastSnrObserved)
        {
          maxThreshold = threshold;
          maxMode = mode;
        }
    }
  return WifiTxVector (maxMode, GetDefaultTxPowerLevel (), GetShortRetryCount (station), GetPreambleForTransmission (maxMode, GetAddress (station)), 800, GetNumberOfAntennas (), nss, 0, GetChannelWidthForMode (maxMode), GetAggregation (station), false);
}

bool
IdealWifiManager::IsLowLatency (void) const
{
  return true;
}

} //namespace ns3
