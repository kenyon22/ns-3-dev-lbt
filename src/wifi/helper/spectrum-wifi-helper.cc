/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2008 INRIA
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
 * Authors: Mathieu Lacage <mathieu.lacage@sophia.inria.fr>
 *          Sébastien Deronne <sebastien.deronne@gmail.com>
 */

#include "spectrum-wifi-helper.h"
#include "ns3/spectrum-wifi-phy.h"
#include "ns3/wifi-spectrum-helper.h"
#include "ns3/names.h"
#include "ns3/log.h"

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("SpectrumWifiHelper");

SpectrumWifiPhyHelper::SpectrumWifiPhyHelper ()
  : m_channel (0),
    m_channelNumber (1)
{
  m_phy.SetTypeId ("ns3::SpectrumWifiPhy");
}

SpectrumWifiPhyHelper
SpectrumWifiPhyHelper::Default (void)
{
  SpectrumWifiPhyHelper helper;
  helper.SetErrorRateModel ("ns3::NistErrorRateModel");
  return helper;
}

void
SpectrumWifiPhyHelper::SetChannel (Ptr<SpectrumChannel> channel)
{
  m_channel = channel;
}

void
SpectrumWifiPhyHelper::SetChannel (std::string channelName)
{
  Ptr<SpectrumChannel> channel = Names::Find<SpectrumChannel> (channelName);
  m_channel = channel;
}

void
SpectrumWifiPhyHelper::SetChannelNumber (uint16_t nch)
{
  m_channelNumber = nch;
}

Ptr<WifiPhy>
SpectrumWifiPhyHelper::Create (Ptr<Node> node, Ptr<NetDevice> device) const
{
  Ptr<SpectrumWifiPhy> phy = m_phy.Create<SpectrumWifiPhy> ();
  phy->CreateWifiSpectrumPhyInterface (device);
  Ptr<ErrorRateModel> error = m_errorRateModel.Create<ErrorRateModel> ();
  phy->SetErrorRateModel (error);
  phy->SetChannel (m_channel);
  phy->SetChannelNumber (m_channelNumber);
  phy->SetChannelModel (m_channelModel);
  phy->SetDevice (device);
  phy->SetMobility (node->GetObject<MobilityModel> ());
  phy->SetNoisePowerSpectralDensity (WifiSpectrumHelper::CreateNoisePowerSpectralDensity (0, m_channelNumber));
  m_channel->AddRx (phy->GetSpectrumPhy());
  return phy;
}

} //namespace ns3
