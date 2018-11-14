/*
 * aqua-sim-phy-multi.h
 *
 *  Created on: Oct 11, 2018
 *      Author: dmitry
 */

#include "ns3/log.h"

#include "aqua-sim-phy-multi.h"
#include "aqua-sim-phy-cmn.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimPhyMulti");
NS_OBJECT_ENSURE_REGISTERED(AquaSimPhyMulti);

AquaSimPhyMulti::AquaSimPhyMulti (void)
{
	NS_LOG_FUNCTION(this);
	// Default number of subchannels
    m_nsubchannels = 10;
    // Default bandwidth of each subchannel
    m_subchannel_bandwidth = 3000; // hz (we currently assume that 1 hz ~ 1 bps in the modulation)
    m_central_freq_start = 10;
    m_central_freq_step = 3;

    // Allocate map //
	// Store current central frequency value
	double current_freq = m_central_freq_start;
	for (int i = 0; i < m_nsubchannels; i++)
	{
		// Create PhyCmn object
		Ptr<AquaSimPhyCmn> phy_cmn = CreateObject<AquaSimPhyCmn> ();
//		// Set NetDevice, Channel and SignalCache
//		phy_cmn->SetNetDevice(GetNetDevice());
//		phy_cmn->SetChannel(GetChannel());
//		phy_cmn->SetSignalCache(GetSignalCache());
		// Set central frequency
		phy_cmn->SetAttribute("Frequency", DoubleValue(current_freq));
//		std::cout << "SUBCHANNEL BW: " << m_subchannel_bandwidth << "\n";
//		phy_cmn->SetBandwidth(m_subchannel_bandwidth);

		// Store PhyCmn in map
		m_channel_phy_map.insert(std::make_pair(i + 1, phy_cmn));

		// Store freq-channel_id map
		m_freq_channel_map.insert(std::make_pair(current_freq, i + 1));


		current_freq += m_central_freq_step;
	}
}

AquaSimPhyMulti::~AquaSimPhyMulti(void)
{
  Dispose();
}

TypeId
AquaSimPhyMulti::GetTypeId ()
{
  static TypeId tid = TypeId("ns3::AquaSimPhyMulti")
	.SetParent<AquaSimPhyCmn>()
	.AddConstructor<AquaSimPhyMulti>()

    .AddAttribute("subchannel_bandwidth", "Subchannel bandwidth",
	DoubleValue(3000),
	MakeDoubleAccessor (&AquaSimPhyMulti::m_subchannel_bandwidth),
	MakeDoubleChecker<double> ())

  ;
  return tid;
}

void
AquaSimPhyMulti::AllocateSubchannels ()
{
	for (int i = 1; i <= m_nsubchannels; i++)
	{
		// Set Bandwidth
//		std::cout << "SUBCHANNEL BW: " << m_subchannel_bandwidth << "\n";
		m_channel_phy_map.at(i)->SetBandwidth(m_subchannel_bandwidth);

		// Set NetDevice, Channel and SignalCache
		m_channel_phy_map.at(i)->SetNetDevice(GetNetDevice());
		m_channel_phy_map.at(i)->SetChannel(GetChannel());
		m_channel_phy_map.at(i)->SetSignalCache(GetSignalCache());
	}
}

void
AquaSimPhyMulti::SetSubchannelId (int channel_id)
{
	m_current_subchannel_id = channel_id - 1;
}

//int
//AquaSimPhyMulti::GetSubchannelId ()
//{
//	return m_current_subchannel_id;
//}


bool
AquaSimPhyMulti::Recv (Ptr<Packet> p)
{
	AquaSimHeader asHeader;
	AquaSimPacketStamp pstamp;
	  p->RemoveHeader(pstamp);
	  p->PeekHeader(asHeader);
	  p->AddHeader(pstamp);

	if (asHeader.GetDirection() == AquaSimHeader::UP) {
		m_current_subchannel_id = m_freq_channel_map.find(pstamp.GetFreq())->second;
	}
	//	m_channel_phy_map.at(m_current_subchannel_id)->SetChannelId(m_current_subchannel_id);
		return m_channel_phy_map.at(m_freq_channel_map.find(pstamp.GetFreq())->second)->Recv(p);
	//	return m_channel_phy_map.at(0)->Recv(p);

}

void
AquaSimPhyMulti::SetSubchannelNumber (int n_subchannels)
{
	m_nsubchannels = n_subchannels;
}

void
AquaSimPhyMulti::SetSubchannelBandwidth (double bandwidth)
{
	m_subchannel_bandwidth = bandwidth;
}

int
AquaSimPhyMulti::PktRecvCount()
{
	for (std::map<int, Ptr<AquaSimPhyCmn>>::iterator it=m_channel_phy_map.begin(); it!=m_channel_phy_map.end(); ++it)
	{
		m_recvd_frames_counter += it->second->PktRecvCount();
	}
  return m_recvd_frames_counter;
}

void
AquaSimPhyMulti::SetSinrChecker(Ptr<AquaSimSinrChecker> sinrChecker)
{
	for (std::map<int, Ptr<AquaSimPhyCmn>>::iterator it=m_channel_phy_map.begin(); it!=m_channel_phy_map.end(); ++it)
	{
		it->second->SetSinrChecker(sinrChecker);
	}
}

void AquaSimPhyMulti::SetTransRange(double range)
{
  NS_LOG_FUNCTION(this);

  for (std::map<int, Ptr<AquaSimPhyCmn>>::iterator it=m_channel_phy_map.begin(); it!=m_channel_phy_map.end(); ++it)
	{
		it->second->SetTransRange(range);
	}
}

////////////// legacy stuff from aqua-sim-phy
/**
 * calculate transmission time of a packet of size pktsize
 * we consider the preamble
*/
Time
AquaSimPhyCmn::CalcTxTime (uint32_t pktSize, std::string * modName)
{
  //NS_ASSERT(modName == NULL);
//  return Time::FromDouble(m_modulations.find(m_modulationName)->second->TxTime(pktSize*8), Time::S)
//      + Time::FromInteger(Preamble(), Time::S);
	return Time::FromInteger(1, Time::S);
}

double
AquaSimPhyCmn::CalcPktSize (double txTime, std::string * modName)
{
//  return Modulation(modName)->PktSize (txTime - Preamble());
	return 1;
}

bool AquaSimPhyCmn::MatchFreq(double freq)
{
//  double epsilon = 1e-6;	//accuracy for float comparison

//  return std::fabs(freq - m_freq) < epsilon;
	return true;
}

int
AquaSimPhyMulti::GetSubchannelId()
{
	return m_current_subchannel_id;
}

//bool
//AquaSimPhyMulti::Decodable(double noise, double ps) {
////  double epsilon = 1e-6;	//accuracy for float comparison
////
////  if (ps < m_RXThresh) //signal is too weak
////    return false;
////
////  if (fabs(noise) <  epsilon) {
////    //avoid being divided by 0 when calculating SINR
////    return true;
////  }
////
////  return m_sinrChecker->Decodable(ps / noise);
//
//
//	return true;
//}
