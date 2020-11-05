/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Dmitrii Dugaev <ddugaev@gradcenter.cuny.edu>
 */

#include "aqua-sim-mac-routing.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-mac.h"
#include "aqua-sim-address.h"

#include "ns3/log.h"
#include "ns3/integer.h"
#include "ns3/simulator.h"

#include <math.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimRoutingMac");
NS_OBJECT_ENSURE_REGISTERED(AquaSimRoutingMac);


/* ======================================================================
Adaptive forwarding (routing) MAC for underwater sensor networks
====================================================================== */

AquaSimRoutingMac::AquaSimRoutingMac()
{
//  m_rand = CreateObject<UniformRandomVariable> ();
  m_max_range = 150;
  m_max_tx_power = 20; // Watts

  m_status = IDLE;
}

TypeId
AquaSimRoutingMac::GetTypeId()
{
  static TypeId tid = TypeId("ns3::AquaSimRoutingMac")
      .SetParent<AquaSimMac>()
      .AddConstructor<AquaSimRoutingMac>()
      .AddAttribute("max_range", "Maximum transmission range",
        DoubleValue(150),
        MakeDoubleAccessor (&AquaSimRoutingMac::m_max_range),
        MakeDoubleChecker<double> ())
	  .AddAttribute("max_tx_power", "Maximum transmission power",
		DoubleValue(20),
		MakeDoubleAccessor (&AquaSimRoutingMac::m_max_tx_power),
		MakeDoubleChecker<double> ())
	  .AddAttribute("optimal_metric", "Optimal Distance metric, m",
		DoubleValue(50),
		MakeDoubleAccessor (&AquaSimRoutingMac::m_optimal_metric),
		MakeDoubleChecker<double> ())
     ;
  return tid;
}

int64_t
AquaSimRoutingMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rand->SetStream(stream);
  return 1;
}

/*
this program is used to handle the received packet,
it should be virtual function, different class may have
different versions.
*/
bool
AquaSimRoutingMac::RecvProcess (Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION(this);
  /*std::cout << "\nBMac @RecvProcess check:\n";
  pkt->Print(std::cout);
  std::cout << "\n";*/

	AquaSimHeader ash;
  MacHeader mach;
  MacRoutingHeader mac_routing_h;

  pkt->RemoveHeader(ash);
  pkt->RemoveHeader(mach);
  pkt->RemoveHeader(mac_routing_h);

	AquaSimAddress dst = mac_routing_h.GetDstAddr();
	AquaSimAddress sender_addr = mac_routing_h.GetSenderAddr();
//	AquaSimAddress dst = mach.GetDA();

	//get a packet from modem, remove the sync hdr from txtime first
	//cmh->txtime() -= getSyncHdrLen();

//	std::cout << "Received mac-routing packet type: " << mac_routing_h.GetPType() << "\n";

	if (ash.GetErrorFlag())
	{
		NS_LOG_DEBUG("RoutingMac:RecvProcess: received corrupt packet.");
//		std::cout << "RoutingMac:RecvProcess: received corrupt packet.\n";
		pkt=0;
		return false;
	}

	// Parse the incoming frames according to their types (DATA, RREP, RREQ, REWARD, ACK, INIT, RTS, CTS)
//	if (dst == AquaSimAddress::GetBroadcast() || dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
//	if (dst == AquaSimAddress::GetBroadcast() || dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))

//	{

//		std::cout << "\n-------------------\nPTYPE: " << mac_routing_h.GetPType() << "\n";
//		std::cout << "NODE: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "\n";
//		std::cout << "TS: " << Simulator::Now() << "\n";
//		std::cout << "SRC: " << mac_routing_h.GetSrcAddr() << "\n";
//		std::cout << "DST: " << mac_routing_h.GetDstAddr() << "\n";
//		std::cout << "HOP COUNT: " << mac_routing_h.GetHopCount() << "\n";
//		std::cout << "TX Power: " << mac_routing_h.GetTxPower() << "\n";
//		std::cout << "RX Power: " << mac_routing_h.GetRxPower() << "\n";

		NS_LOG_DEBUG("\n-------------------\nPTYPE: " << mac_routing_h.GetPType());
		NS_LOG_DEBUG("TS: " << Simulator::Now());
		NS_LOG_DEBUG("SRC: " << mac_routing_h.GetSrcAddr());
		NS_LOG_DEBUG("DST: " << mac_routing_h.GetDstAddr());
		NS_LOG_DEBUG("HOP COUNT: " << mac_routing_h.GetHopCount());


		if (m_packetSize == 0)
		{
			ash.SetSize(ash.GetSize() - m_packetHeaderSize);
		}

		// Filter out the packets if their hop_count exceeds the maximum (to avoid loops)
		if (mac_routing_h.GetHopCount() > m_max_hop_count)
		{
//			std::cout << "Warning! The Hop Count exceeds the maximum!\n";
			NS_LOG_DEBUG("Warning! The Hop Count exceeds the maximum!");
			pkt = 0;
			return false;
		}

		// DATA PACKET TYPE
		////////////////////
		////////////////////

		// If the DATA packet comes from the network (i.e. PHY), then process it regardless of the current RTS/CTS status
		if (mac_routing_h.GetPType() == 0)
		{
			// The DATA packet must be destined to the node
			if (dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// Disable reward delays => no negative rewards if the data packet has been lost
				//// TODO: Maybe enable negative rewards in the future
	//			std::map<AquaSimAddress, AquaSimAddress> m; // Store dst_addr : sender_addr pair
	//			m.insert(std::make_pair(mach.GetDA(), mac_routing_h.GetSrcAddr()));
	//			// Send reward / ACK back
	//			if (m_reward_delays.count(m) == 0)
	//			{
	//				m_reward_delays.insert(std::make_pair(m, Simulator::Now()));
	//				// Delay and send back a reward message
	////				SendReward(mac_routing_h.GetSrcAddr(), GenerateReward(mach.GetDA()), pkt);
	////				Simulator::Schedule(m_reward_delay, &AquaSimRoutingMac::SendRewardHandler,
	////						this, m, GenerateReward(mach.GetDA()), pkt->Copy());
	//
	//				Simulator::Schedule(m_reward_delay, &AquaSimRoutingMac::SendRewardHandler,
	//						this, m, GenerateReward(mach.GetDA()), pkt);
	//			}
				////

//				std::cout << "GOT DATA MESSAGE. SRC: " << mac_routing_h.GetSrcAddr() << " DST: " << mac_routing_h.GetDstAddr() << "\n";

				// If dst_addr is its own, send up
				// If not, forward packet further
				double reward = 0;
				if (mach.GetDA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
				{
					// The data packet is for this node, send it up
					pkt->AddHeader(ash);  //leave MacHeader off since sending to upper layers
					SendUp(pkt);

					// Generate a single direct reward message back to the sender
					// Set mac_routing_header parameters
					Ptr<Packet> reward_msg = Create<Packet>();
					MacRoutingHeader reward_h;
					reward_h.SetPType(7); // 7 - DIRECT REWARD MESSAGE
					reward_h.SetId(0);
					reward_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
					reward_h.SetDstAddr(mac_routing_h.GetSrcAddr());

					// Set aqua-sim-header parameters for correct handling on Phy layer
					ash.SetSize(reward_h.GetSerializedSize() + mach.GetSerializedSize());
					ash.SetTxTime(Phy()->CalcTxTime(reward_h.GetSerializedSize() + mach.GetSerializedSize()));
					ash.SetErrorFlag(false);
					ash.SetDirection(AquaSimHeader::DOWN);

					// Set Tx power for the frame
					reward_h.SetTxPower(CalculateTxPower(CalculateDistance(mac_routing_h.GetTxPower(),
							mac_routing_h.GetRxPower()))); // Send direct reward message considering the distance

					// Set reward value
					reward = CalculateWeight(mac_routing_h.GetDirectDistance() - 0);
//					std::cout << "DIRECT DISTANCE: " << mac_routing_h.GetDirectDistance() << "\n";
					// Since the node is the destination itself, the residual distance is zero
//					std::cout << "RESIDUAL DISTANCE: " << 0 << "\n";

					reward_h.SetReward(reward);

					reward_msg->AddHeader(reward_h);
					reward_msg->AddHeader(mach);
					reward_msg->AddHeader(ash);

					// Put the INIT into init_list to further filter duplicate ones out
					FilterDuplicateInit(mach.GetSA());
					SendDownFrame(reward_msg);  // Send down the INIT message using the max tx_power

				}
				else
				{
					// Otherwise, insert the reward and forward the packet further
					// Generate reward from the given optimal metric and delta-distance to destination
					// I.e. how much the total distance has been reduced comparing to the optimal one
					reward = CalculateWeight(mac_routing_h.GetDirectDistance() -
							m_distances.find(mach.GetDA())->second);

//					std::cout << "DIRECT DISTANCE: " << mac_routing_h.GetDirectDistance() << "\n";
//					std::cout << "RESIDUAL DISTANCE: " << m_distances.find(mach.GetDA())->second << "\n";

					pkt->AddHeader(mach);
					pkt->AddHeader(ash);
					// Increment hop_count, set reward
					ForwardPacket(pkt, mac_routing_h.GetSrcAddr(), mac_routing_h.GetHopCount() + 1, reward);
				}
				return true;
			}

			// Update the weight value by the reward, if the sender_addr is for this node
			if (sender_addr == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{

//				std::cout << "REWARD PACKET\n";

				// Check the reward
				// If the reward = 0, then the packet contains no rewards, i.e. the packet has come from the initial source node
				// In that case, do not update the forwarding table with reward
				double reward = mac_routing_h.GetReward();
				if (reward != 0)
				{
					// Update forwarding table
					UpdateWeight(mach.GetDA(), mac_routing_h.GetSrcAddr(), reward);
				}

			}

			return true;
		}

		////////////////////
		////////////////////

		// Check the frame type
		// INIT
		if (mac_routing_h.GetPType() == 4)
		{
			// Update distances list
			UpdateDistance(mac_routing_h.GetTxPower(), mac_routing_h.GetRxPower(), mac_routing_h.GetSrcAddr());

			// Filter duplicate INIT reception
			if (mach.GetSA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				return true;
			}
			if (!FilterDuplicateInit(mach.GetSA()))
			{
				// Filter out the frame
//				std::cout << "INIT PACKET FILTERED\n";
//				NS_LOG_DEBUG("INIT PACKET FILTERED");
				pkt = 0;
				return true;
			}

//			std::cout << "GOT INIT MESSAGE. SRC: " << mac_routing_h.GetSrcAddr() << " DST: " << mac_routing_h.GetDstAddr() << "\n";

			// Send INIT back
			// Set mac_routing_header parameters
			Ptr<Packet> init = Create<Packet>();
			MacRoutingHeader init_h;
			init_h.SetPType(4); // 4 - INIT
			init_h.SetId(0);
			init_h.SetHopCount(0);
			init_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
			init_h.SetDstAddr(mac_routing_h.GetSrcAddr());

			// Set aqua-sim-header parameters for correct handling on Phy layer
			ash.SetSize(init_h.GetSerializedSize() + mach.GetSerializedSize());
			ash.SetTxTime(Phy()->CalcTxTime(init_h.GetSerializedSize() + mach.GetSerializedSize()));
			ash.SetErrorFlag(false);
			ash.SetDirection(AquaSimHeader::DOWN);

			// Set Tx power for the frame
			init_h.SetTxPower(m_max_tx_power); // Broadcast INIT message using max power

			init->AddHeader(init_h);
			init->AddHeader(mach);
			init->AddHeader(ash);

			// Put the INIT into init_list to further filter duplicate ones out
			FilterDuplicateInit(mach.GetSA());
			SendDownFrame(init);  // Send down the INIT message using the max tx_power

			return true;
		}

		// RTS
		if (mac_routing_h.GetPType() == 5) // 5 - RTS
		{
			// Update distances list
			UpdateDistance(mac_routing_h.GetTxPower(), mac_routing_h.GetRxPower(), mac_routing_h.GetSrcAddr());

			// Filter duplicate RTS reception
			if (!FilterDuplicateRts(mach.GetSA()))
			{
				// Filter out the frame
//				std::cout << "RTS PACKET FILTERED\n";
//				NS_LOG_DEBUG("RTS PACKET FILTERED");
				pkt = 0;
				return true;
			}

//			std::cout << "GOT RTS MESSAGE. SRC: " << mac_routing_h.GetSrcAddr() << " DST: " << mac_routing_h.GetDstAddr() << "\n";

			// Check if this RTS for this node or not
			// If yes, then generate the CTS back, otherwise, set the status to RTS_TO
			if (mac_routing_h.GetDstAddr() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// Set state to receive data
				m_status = DATA_RX;

				// Send CTS back
				// Set mac_routing_header parameters
				MacRoutingHeader cts_h;
				Ptr<Packet> cts = Create<Packet>();
				cts_h.SetPType(6); // 6 - CTS
				cts_h.SetId(0);
				cts_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
				cts_h.SetDstAddr(mac_routing_h.GetSrcAddr());

				// Set aqua-sim-header parameters for correct handling on Phy layer
				ash.SetSize(cts_h.GetSerializedSize() + mach.GetSerializedSize());
				ash.SetTxTime(Phy()->CalcTxTime(cts_h.GetSerializedSize() + mach.GetSerializedSize()));
				ash.SetErrorFlag(false);
				ash.SetDirection(AquaSimHeader::DOWN);

				// Set Tx power for the frame
				cts_h.SetTxPower(m_max_tx_power); // Broadcast CTS message using max power

				cts->AddHeader(cts_h);
				cts->AddHeader(mach);
				cts->AddHeader(ash);

				// Put the CTS into cts_list to further filter duplicate ones out
				FilterDuplicateCts(mach.GetDA());

				SendDownFrame(cts);  // Send down the CTS message using the max tx_power

				// State timeout scheduler
//				StateTimeoutHandler(m_rts_timeout + m_cts_timeout); // ???
				StateTimeoutHandler(m_data_timeout); // ???

			}
			else
			{
				m_status = CTS_TO;
				// State timeout scheduler
				StateTimeoutHandler(m_cts_timeout);

			}
			return true;
		}

		// CTS
		if (mac_routing_h.GetPType() == 6) // 6 - CTS
		{
			// Update distances list
			UpdateDistance(mac_routing_h.GetTxPower(), mac_routing_h.GetRxPower(), mac_routing_h.GetSrcAddr());

			// Filter duplicate CTS reception
			if (!FilterDuplicateCts(mach.GetSA()))
			{
				// Filter out the frame
//				std::cout << "CTS PACKET FILTERED\n";
//				NS_LOG_DEBUG("CTS PACKET FILTERED");
				pkt = 0;
				return true;
			}

//			std::cout << "GOT CTS MESSAGE. SRC: " << mac_routing_h.GetSrcAddr() << " DST: " << mac_routing_h.GetDstAddr() << "\n";

			// Check if this CTS for this node or not
			// If yes, then start data transmission
			if (mac_routing_h.GetDstAddr() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// Update last received TS
				m_rts_expirations.find(mach.GetDA())->second = Simulator::Now();

				m_status = DATA_TX;
//				StateTimeoutHandler(m_rts_timeout + m_cts_timeout); // ???
				StateTimeoutHandler(m_data_timeout); // ???

				// Send all packets which are in send buffer
				while ((!m_send_buffer.empty()) && (m_status == DATA_TX))
				{
//					std::cout << "Sending packet from send buffer... " << "\n";
//				NS_LOG_DEBUG("Sending packet from send buffer... ");

				Ptr<Packet> p = m_send_buffer.front();

				SendDownFrame(p);
				m_send_buffer.pop();

				}
				// Clear send buffer
				while(!m_send_buffer.empty()) m_send_buffer.pop();

			}
			else
			{
				m_status = CTS_TO;
				// State timeout scheduler
				StateTimeoutHandler(m_cts_timeout);
			}
			return true;
		}

		// Direct reward
		if (mac_routing_h.GetPType() == 7) // 7 - DIRECT REWARD
		{
			// If this reward for this node, update weight
			// Otherwise, discard
			if (mac_routing_h.GetDstAddr() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				UpdateWeight(mach.GetDA(), mac_routing_h.GetSrcAddr(), mac_routing_h.GetReward());
			}
			return true;
		}

		else
		{
//			std::cout << "Unknown packet type is received: " << mac_routing_h.GetPType() << "\n";
		}

//	printf("underwaterAquaSimRoutingMac: this is neither broadcast nor my packet, just drop it\n");
	//pkt=0;
	return false;
}


void
AquaSimRoutingMac::DropPacket(Ptr<Packet> pkt)
{
  //this is not necessary... only kept for current legacy issues
  pkt=0;
  return;
}


/*
this program is used to handle the transmitted packet,
it should be virtual function, different class may have
different versions.
*/
bool
AquaSimRoutingMac::TxProcess(Ptr<Packet> pkt)
{
	//  NS_LOG_FUNCTION(this << pkt);

	AquaSimHeader ash;
	MacHeader mach;
	pkt->RemoveHeader(ash);

	mach.SetDA(ash.GetDAddr());
	mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

	if( m_packetSize != 0 )
	  ash.SetSize(m_packetSize);
	else
	  ash.SetSize(m_packetHeaderSize + ash.GetSize());

	ash.SetTxTime(GetTxTime(pkt));

	pkt->AddHeader(mach);
	pkt->AddHeader(ash);

	ForwardPacket(pkt, AquaSimAddress::ConvertFrom(m_device->GetAddress()), 0, 0); // hop_count = 0, reward = 0

  return true;  //may be bug due to Sleep/default cases
}

bool
AquaSimRoutingMac::SendDownFrame (Ptr<Packet> pkt)
{
	  NS_LOG_FUNCTION(this << pkt);
	  AquaSimHeader ash;
	  MacHeader mach;
	  MacRoutingHeader mac_routing_h;

	  pkt->RemoveHeader(ash);
	  pkt->RemoveHeader(mach);
	  pkt->RemoveHeader(mac_routing_h);

	  // Set mac type to mac header
	  mach.SetDemuxPType(MacHeader::UWPTYPE_MAC_ROUTING);

      ash.SetDirection(AquaSimHeader::DOWN);
      pkt->AddHeader(mac_routing_h);
      pkt->AddHeader(mach);
      pkt->AddHeader(ash);

//      std::cout << "NODE: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << " STATE: " << m_status <<
//    		  " TS: " << Simulator::Now() << "\n";
//      std::cout << "PTYPE: " << mac_routing_h.GetPType() << "\n";

	  if (m_status == IDLE)
	  {
		  // If the packet type is not a data packet, but some MAC message (i.e. RTS, CTS or INIT), just send the message
		  if (mac_routing_h.GetPType() != 0)
		  {
		      SendDown(pkt);
		  }
		  // If the packet is DATA, send RTS, change the state
		  else
		  {
			  // Check if the DATA packet has come from the App or the Net interface, by checking mach Src address
			  // If the DATA packet has arrived from the APP, initiate RTS / CTS handshake
			  // If the DATA packet has arrived from the net interface, send it down, since the frame's supposed to be delivered via
			  // multiple hops
			  if (mach.GetSA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			  {
				  // Generate RTS
				// Set mac_routing_header parameters
				  MacRoutingHeader rts_h;
				Ptr<Packet> rts = Create<Packet>();
				rts_h.SetPType(5); // 5 - RTS
				rts_h.SetId(0);
				rts_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
				rts_h.SetDstAddr(mach.GetDA());
				// Set max tx_power
				rts_h.SetTxPower(m_max_tx_power);

				// Set aqua-sim-header parameters for correct handling on Phy layer
				ash.SetSize(rts_h.GetSerializedSize() + mach.GetSerializedSize());
				ash.SetTxTime(Phy()->CalcTxTime(rts_h.GetSerializedSize() + mach.GetSerializedSize()));
				ash.SetErrorFlag(false);
				ash.SetDirection(AquaSimHeader::DOWN);

	//			std::cout << "SENDING RTS\n";

				rts->AddHeader(rts_h);
				rts->AddHeader(mach);
				rts->AddHeader(ash);

				// Filter duplicate RTSs out
				FilterDuplicateRts(mach.GetDA());

				// Schedule the RTS expiration event
				// Set RTS timer
				if (m_rts_expirations.count(mach.GetDA()) == 0)
				{
					m_rts_expirations.insert(std::make_pair(mach.GetDA(), Time::FromInteger(0, Time::S)));
					Simulator::Schedule(m_rts_timeout, &AquaSimRoutingMac::RtsExpirationHandler, this, mach.GetDA());
				}

				// Put the DATA packet into the buffer
				m_send_buffer.push(pkt->Copy());

				// Send RTS
				SendDown(rts);

				m_status = CTS_WAIT;
			  }
			  else
			  {
				  SendDown(pkt);
			  }

		  }
		  return true;
	  }

	  if ((m_status == RTS_TO) || (m_status == CTS_TO) || (m_status == CTS_WAIT))
	  {
		  // If the node in RTS timeout state, put the packet into the buffer, if the packet is DATA
		  // Otherwise, discard the frame
		  if (mac_routing_h.GetPType() == 0)
		  {
			  // Check if the DATA packet has come from the App or the Net interface, by checking mach Src address
			  // If the DATA packet has arrived from the APP, put it to the buffer, since in TO state
			  // If the DATA packet has arrived from the net interface, send it down, since the frame's supposed to be delivered via
			  // multiple hops
			  if (mach.GetSA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			  {
				  m_send_buffer.push(pkt->Copy());
			  }
			  else
			  {
				  SendDown(pkt);
			  }
		  }
		  return true;
	  }

	  if (m_status == DATA_RX)
	  {
		  // If CTS or direct reward message, then send it out
		  if ((mac_routing_h.GetPType() == 6) || (mac_routing_h.GetPType() == 7))
		  {
			  SendDown(pkt);
			  return true;
		  }
		  // If data packet, which has arrived from the net, forward it further as well
		  // Check if the DATA packet has come from the App or the Net interface, by checking mach Src address
		  // If the DATA packet has arrived from the APP, put it to the buffer, since in DATA_RX state
		  // If the DATA packet has arrived from the net interface, send it down, since the frame's supposed to be delivered via
		  // multiple hops
		  if (mac_routing_h.GetPType() == 0)
		  {
			  if (mach.GetSA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			  {
				  m_send_buffer.push(pkt->Copy());
			  }
			  else
			  {
				  SendDown(pkt);
			  }
			  return true;
		  }
		  return true;
	  }

	  if (m_status == DATA_TX)
	  {
		  // If the packet type is a data packet, but not some MAC message (i.e. RTS, CTS or INIT), just send the data
		  if (mac_routing_h.GetPType() == 0)
		  {
//			  std::cout << "SEND DOWN DATA\n";
//			  std::cout << "SRC: " << mac_routing_h.GetSrcAddr() << " DST: " << mac_routing_h.GetDstAddr() << "\n";
		      SendDown(pkt);
		  }
		  return true;
	  }

	  return true;
}

// Forward packet / frame coming from the application or the network (DATA type)
bool
AquaSimRoutingMac::ForwardPacket(Ptr<Packet> p, AquaSimAddress sender_addr, int hop_count, double reward)
{
	AquaSimHeader ash;
	MacHeader mach;
	MacRoutingHeader mac_routing_h;

	p->RemoveHeader(ash);
	p->RemoveHeader(mach);

//	std::cout << "DST_ADDR: " << mach.GetDA() << "\n";

	// Store destination address as integer
	AquaSimAddress dst_addr = mach.GetDA();

	// Reset the direction if the packet is forwarded by intermediate node
	ash.SetDirection(AquaSimHeader::DOWN);

//	// Check if dst_addr is in the distances list
	if (m_distances.count(dst_addr) != 0)
	{

		//TODO://		// Store the optimal distance metric for the current packet towards given destination
//		double optimal_metric = CalculateOptimalMetric(m_distances.find(dst_addr)->second);

		// Update the forwarding table according to the known distances
		if (m_forwarding_table.count(dst_addr) == 0)
		{
			// Create new entry with initial weight
			std::map<AquaSimAddress, double> m; // {next_hop : weight}

			// For each possible destination / distance - calculate the initial weight based on the optimal distance metric
			for (auto const& x : m_distances)
			{
				m.insert(std::make_pair(x.first, CalculateWeight(x.second)));
		//		std::cout << "Creating new table entry with REWARD: " << reward / 2 << "\n";
				NS_LOG_DEBUG("Creating new table entry with WEIGHT: " << CalculateWeight(x.second));
			}

			m_forwarding_table.insert(std::make_pair(dst_addr, m));
		}

		// Select next_hop neighbor and send down the packet
		// If the hop_count exceeds the threshold, then send the packet directly to the destination
		AquaSimAddress next_hop_addr;
		if (hop_count >= m_hop_count_threshold)
		{
//			std::cout << "HOP COUNT EXCEEDED THRESHOLD. Sending to DST.\n";
			next_hop_addr = dst_addr;
		}
		else
		{
			next_hop_addr = SelectNextHop(dst_addr);
		}

		// Create mac_routing_header (DATA type)
		mac_routing_h.SetPType(0);
		mac_routing_h.SetId(0);
		mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
//		std::cout << "Next hop addr: " << next_hop_addr << "\n";
		mac_routing_h.SetDstAddr(next_hop_addr);
		mac_routing_h.SetHopCount(hop_count);

		// Set the direct distance from this node to the destination
		mac_routing_h.SetDirectDistance(m_distances.find(dst_addr)->second);
//		std::cout << "DIRECT DISTANCE FROM LIST: " << m_distances.find(dst_addr)->second << "\n";

		// Set reward
		mac_routing_h.SetReward(reward);

		// Set sender address for the reward
		mac_routing_h.SetSenderAddr(sender_addr);

		// Set tx time and size
		ash.SetSize(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize() + p->GetSize());
		ash.SetTxTime(Phy()->CalcTxTime(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize() + p->GetSize()));
		ash.SetErrorFlag(false);
		ash.SetDirection(AquaSimHeader::DOWN);

		// Set tx power for the given packet. The tx_power must be enough to reach the initial sender node.
		if (AquaSimAddress::ConvertFrom(m_device->GetAddress()) == sender_addr)
		{
			// Set tx_power sufficient to reach the destination
			mac_routing_h.SetTxPower(CalculateTxPower(m_distances.find(next_hop_addr)->second));
		}
		else
		{
			// Else, consider the tx_power to reach the sender node as well
			if ((m_distances.find(next_hop_addr)->second) <= (m_distances.find(sender_addr)->second))
			{
				// Set tx_power to reach the sender
				mac_routing_h.SetTxPower(CalculateTxPower(m_distances.find(sender_addr)->second));
			}
			else
			{
				// Set tx_power sufficient to reach the destination
				mac_routing_h.SetTxPower(CalculateTxPower(m_distances.find(next_hop_addr)->second));
			}
		}

		p->AddHeader(mac_routing_h);
		p->AddHeader(mach);
		p->AddHeader(ash);

		///
		// Disable reward timer expiration
//		std::map<AquaSimAddress, AquaSimAddress> m;
//		m.insert(std::make_pair(dst_addr, next_hop_addr));
//		// Set Reward timer
//		if (m_reward_expirations.count(m) == 0)
//		{
//			// Trigger new reward expiration event for this address
//			m_reward_expirations.insert(std::make_pair(m, Simulator::Now()- Seconds(0.1)));
////			std::cout << "New reward timeout scheduled: " << Simulator::Now() << "\n";
//			NS_LOG_DEBUG("New reward timeout scheduled: " << Simulator::Now());
//
//			Simulator::Schedule(m_reward_timeout, &AquaSimRoutingMac::RewardExpirationHandler, this, m);
//		}
		///

//		// Send down the packet to the next hop
////		std::cout << "Sending down data packet " << Simulator::Now() << "\n";
//		NS_LOG_DEBUG("Sending down data packet " << Simulator::Now());
//
////		std::cout << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << " --> " << mac_routing_h.GetDstAddr() << "\n";
//		NS_LOG_DEBUG(AquaSimAddress::ConvertFrom(m_device->GetAddress()) << " --> " << mac_routing_h.GetDstAddr());
//
////		std::cout << "Transmission Status: " << m_device->GetTransmissionStatus() << "\n";
//		NS_LOG_DEBUG("Transmission Status: " << m_device->GetTransmissionStatus());


		// RTS / CTS exchange
		SendDownFrame(p);

		return true;
	}
	else
	{
		// Add initial headers to the packet before buffering it
		p->AddHeader(mach);
		p->AddHeader(ash);

		// If the queue already exists, put packet in send_queue, finish
		if (m_send_queue.count(dst_addr) != 0)
		{
//			m_send_queue.find(dst_addr)->second.push(p->Copy());
			m_send_queue.find(dst_addr)->second.push(p);
		}
		// If there is no entry for the given destination address, create a queue and trigger INIT message
		else
		{
			// Create new sub-queue for given dst_addr
			std::queue<Ptr<Packet>> q;
			// Create new entry in send_queue
			m_send_queue.insert(std::make_pair(dst_addr, q));
			// Put packet in queue
//			m_send_queue.find(dst_addr)->second.push(p->Copy());
			m_send_queue.find(dst_addr)->second.push(p);

			// Generate and send INIT message
			Ptr<Packet> init = Create<Packet>();
			MacRoutingHeader init_h;
			init_h.SetPType(4);	// 4 - INIT message type
			init_h.SetId(0);
			init_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

			// Set frame to broadcast address
			init_h.SetDstAddr(dst_addr);
			init_h.SetHopCount(0);

			// Set aqua-sim-header parameters for correct handling on Phy layer
			ash.SetSize(init_h.GetSerializedSize() + mach.GetSerializedSize());
			ash.SetTxTime(Phy()->CalcTxTime(init_h.GetSerializedSize() + mach.GetSerializedSize()));
			ash.SetErrorFlag(false);
			ash.SetDirection(AquaSimHeader::DOWN);

			// Set Tx power for the frame
			init_h.SetTxPower(m_max_tx_power); // Broadcast INIT message using max power

			init->AddHeader(init_h);
			init->AddHeader(mach);
			init->AddHeader(ash);

			// Listen for backward INIT messages
			// Set INIT timer
			if (m_init_expirations.count(dst_addr) == 0)
			{
				m_init_expirations.insert(std::make_pair(dst_addr, Time::FromInteger(0, Time::S)));
				Simulator::Schedule(m_init_timeout, &AquaSimRoutingMac::InitExpirationHandler, this, dst_addr);
			}


//			std::cout << "Sending RREQ " << Simulator::Now() << "\n";
//			NS_LOG_DEBUG("Sending RREQ " << Simulator::Now());
			NS_LOG_DEBUG("Sending INIT " << Simulator::Now());

//			// Put the INIT into init_list to further filter duplicate ones out
//			FilterDuplicateInit(mach.GetSA());

			// Send frame
			SendDownFrame(init);

			return true;
		}

		return true;
	}

	return true;
}

// TODO: Maybe consider in the future the impact of the average weight towards given destination
//double
//AquaSimRoutingMac::GenerateReward(AquaSimAddress dst_addr)
//{
//	// If no entries in forwarding table, return 0
//	if (m_forwarding_table.count(dst_addr) == 0)
//	{
//		if (dst_addr == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
//		{
//			// If the dst_adress is this node itself, update the forwarding table and return reward
//			UpdateWeight(dst_addr, dst_addr, 100*2); // Needs to multiply by two since the UpdateMethods calculates the average
//			return 100; // Corresponds to the weight to itself
//		}
//
//		// Else, there this node has no path to the given destination, return 0
//		return 0;
//	}
//
//	// Calculate and return an average weight towards a given destination from the node's forwarding table
//	double weight_sum = 0;
//	int n = 0;
//
//	// Iterate through the weights to find average
//	for (auto const& x : m_forwarding_table.find(dst_addr)->second)
//	{
//		weight_sum = weight_sum + x.second;
//		n++;
//	}
//
//	return weight_sum / n;
//}


// ONLY GREEDY FOR NOW
AquaSimAddress
AquaSimRoutingMac::SelectNextHop(AquaSimAddress dst_addr)
{
	// Forwarding table has the following format: {dst_adddr: {next_hop1 : w1, ... next_hopN : wN}}
	// For the given dst_addr, select a next_hop_addr with max weight (greedy method for now)
	double max_value = 0;
	AquaSimAddress next_hop_addr = 0;
	// Iterate through the weights to find the max value
	for (auto const& x : m_forwarding_table.find(dst_addr)->second)
	{
		if (max_value <= x.second)
		{
			max_value = x.second;
			next_hop_addr = x.first;
		}
//		std::cout << x.first << " " << x.second << "\n";
	}

//	std::cout << "MAX VALUE: " << max_value << "\n";
	NS_LOG_DEBUG("MAX VALUE: " << max_value);

//	std::cout << "NEXT HOP ADDR: " << next_hop_addr << "\n";
	NS_LOG_DEBUG("NEXT HOP ADDR: " << next_hop_addr);

	return next_hop_addr;
}

// Update weight in forwarding table
bool
AquaSimRoutingMac::UpdateWeight(AquaSimAddress dst_addr, AquaSimAddress next_hop_addr, double reward)
{
//	std::cout << "UPDATED REWARD: " << reward << "\n";
	NS_LOG_DEBUG("UPDATED REWARD: " << reward);

	if (m_forwarding_table.count(dst_addr) == 0)
	{
		// Create new entry with initial weight according to given reward
		std::map<AquaSimAddress, double> m; // {next_hop : weight}

//		m.insert(std::make_pair(next_hop_addr, reward / 2));
		m.insert(std::make_pair(next_hop_addr, reward));
//		std::cout << "Creating new table entry with REWARD: " << reward << "\n";
//		NS_LOG_DEBUG("Creating new table entry with REWARD: " << reward / 2);

		m_forwarding_table.insert(std::make_pair(dst_addr, m));
	}
	else
	{
		// Check if the next_hop_entry exist, if not - create one and return
		if (m_forwarding_table.find(dst_addr)->second.count(next_hop_addr) == 0)
		{
//			m_forwarding_table.find(dst_addr)->second.insert(std::make_pair(next_hop_addr, reward / 2));
			m_forwarding_table.find(dst_addr)->second.insert(std::make_pair(next_hop_addr, reward));
		}

		double current_weight = m_forwarding_table.find(dst_addr)->second.find(next_hop_addr)->second;
//		std::cout << "CURRENT WEIGHT: " << current_weight << " Node Address: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "\n";
//		NS_LOG_DEBUG("CURRENT WEIGHT: " << current_weight << " Node Address: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()));

//		std::cout << "REWARD: " << reward << "\n";
//		NS_LOG_DEBUG("REWARD: " << reward);

//		std::cout << "TABLE SIZE: " << m_forwarding_table.size() << "\n";
//		NS_LOG_DEBUG("TABLE SIZE: " << m_forwarding_table.size());

		// If the current weight is too small (due to lack of rewards), just remove the entry from the table
		if ((current_weight + reward) <= 0)
//		if (current_weight < 0.01)
		{
//			std::cout << "Warning! Weight dropped below the threshold! Deleting table entry!\n";
//			NS_LOG_DEBUG("Warning! Weight dropped below the threshold! Deleting table entry!");

			m_forwarding_table.find(dst_addr)->second.erase(next_hop_addr);
			if (m_forwarding_table.find(dst_addr)->second.size() == 0)
			{
//				std::cout << "Deleting the entire entry\n";
				m_forwarding_table.erase(dst_addr);
			}
			return 0;

		}

		// Calculate sample average and update the weight
		m_forwarding_table.find(dst_addr)->second.at(next_hop_addr) = (current_weight + reward) / 2;
	}
	return 0;
}

double
AquaSimRoutingMac::CalculateWeight(double distance)
{
	// Calculate weight based on the difference between optimal_distance and D(Tx,Rx)
	if (distance <= 0)
	{
		// Return minimum possible reward
		return m_min_reward;
	}

	// TODO: Think out how to give more weight to a closer node, then to a more distant one.
	if (m_optimal_metric <= abs(distance))
	{
		return 100 * (m_optimal_metric / distance);
	}
	else
	{
		return 100 * (distance / m_optimal_metric);
	}
}

// Not used for now
void
AquaSimRoutingMac::RewardExpirationHandler(std::map<AquaSimAddress, AquaSimAddress> dst_to_next_hop_map)
{
	// Check the last received reward timestamp for a given address
	// If the time difference is greater than the reward timeout value, then generate a negative reward
	if ((Simulator::Now() - m_reward_expirations.find(dst_to_next_hop_map)->second) > m_reward_timeout)
	{
//		std::cout << "Reward wait timeout! " << Simulator::Now() - m_reward_expirations.find(dst_to_next_hop_map)->second << "\n";
		NS_LOG_DEBUG("Reward wait timeout! " << Simulator::Now() - m_reward_expirations.find(dst_to_next_hop_map)->second);

		// Generate negative reward
		UpdateWeight(dst_to_next_hop_map.begin()->first, dst_to_next_hop_map.begin()->second, m_negative_reward);
	}
//	// Update timestamp
//	m_reward_expirations.at(dst_to_next_hop_map) = Simulator::Now() - Seconds(0.1);
//	Simulator::Schedule(m_reward_timeout, &AquaSimRoutingMac::RewardExpirationHandler, this, dst_to_next_hop_map);
	// Delete entry
	m_reward_expirations.erase(dst_to_next_hop_map);
}

void
AquaSimRoutingMac::InitExpirationHandler(AquaSimAddress node_address)
{
	// Check the last received INIT timestamp for a given address
	// If the time difference is greater than the INIT timeout value, then clear the corresponding queue
	if ((Simulator::Now() - m_init_expirations.find(node_address)->second) > m_init_timeout)
	{
//		std::cout << "INIT wait timeout!\n";
		NS_LOG_DEBUG("INIT wait timeout!");

//		std::cout << "DST_ADDR: " << node_address << "\n";
		NS_LOG_DEBUG("DST_ADDR: " << node_address);

//		std::cout << "TS: " << m_rrep_expirations.find(node_address)->second << "\n";
		NS_LOG_DEBUG("TS: " << m_init_expirations.find(node_address)->second);

//		std::cout << "Current Time: " << Simulator::Now() << "\n";
		NS_LOG_DEBUG("Current Time: " << Simulator::Now());

		// Delete the entry and the queue
		m_init_expirations.erase(node_address);
		m_send_queue.erase(node_address);
	}
}

void
AquaSimRoutingMac::RtsExpirationHandler(AquaSimAddress node_address)
{
	// Check the last received RTS timestamp for a given address
	// If the time difference is greater than the RTS timeout value, then clear the corresponding queue
	if ((Simulator::Now() - m_rts_expirations.find(node_address)->second) > m_rts_timeout)
	{
//		std::cout << "RTS wait timeout!\n";
		NS_LOG_DEBUG("RTS wait timeout!");

//		std::cout << "DST_ADDR: " << node_address << "\n";
//		NS_LOG_DEBUG("DST_ADDR: " << node_address);

//		std::cout << "TS: " << m_rrep_expirations.find(node_address)->second << "\n";
//		NS_LOG_DEBUG("TS: " << m_rts_expirations.find(node_address)->second);

//		std::cout << "Current Time: " << Simulator::Now() << "\n";
//		NS_LOG_DEBUG("Current Time: " << Simulator::Now());

		// Delete the entry
		m_rts_expirations.erase(node_address);
//		m_send_queue.erase(node_address);
		// Set status to IDLE
		m_status = IDLE;
	}
}

void
AquaSimRoutingMac::CtsExpirationHandler(AquaSimAddress node_address)
{
	// Check the last received CTS timestamp for a given address
	// If the time difference is greater than the CTS timeout value, then clear the corresponding queue
	if ((Simulator::Now() - m_cts_expirations.find(node_address)->second) > m_cts_timeout)
	{
//		std::cout << "CTS wait timeout!\n";
		NS_LOG_DEBUG("CTS wait timeout!");

//		std::cout << "DST_ADDR: " << node_address << "\n";
		NS_LOG_DEBUG("DST_ADDR: " << node_address);

//		std::cout << "TS: " << m_rrep_expirations.find(node_address)->second << "\n";
		NS_LOG_DEBUG("TS: " << m_cts_expirations.find(node_address)->second);

//		std::cout << "Current Time: " << Simulator::Now() << "\n";
		NS_LOG_DEBUG("Current Time: " << Simulator::Now());

		// Delete the entry and the queue
		m_cts_expirations.erase(node_address);
		m_send_queue.erase(node_address);
	}
}

void
AquaSimRoutingMac::StateTimeoutHandler(Time timeout)
{
	if (m_state_timeout < (Simulator::Now() + timeout))
	{
		m_state_timeout = Simulator::Now() + timeout;
	}
	// Schedule the IDLE event
	Simulator::Schedule(timeout, &AquaSimRoutingMac::SetToIdle, this);
}

void
AquaSimRoutingMac::SetToIdle()
{
	// Change state to IDLE, since the current state timeout hasn't been changed, therefore, no other timeouts were introduced
	if (m_state_timeout <= Simulator::Now())
	{
//		std::cout << "SETTING TO IDLE: " << Simulator::Now() << "\n";
		m_status = IDLE;
	}
}

bool
AquaSimRoutingMac::FilterDuplicateInit(AquaSimAddress src_addr)
{
	// Check if this address already exists in the list
	if (m_init_list.count(src_addr) == 0)
	{
		// If no, then add the address with the current timestamp, return True -> packet is not filtered
		m_init_list.insert(std::make_pair(src_addr, Simulator::Now()));
		return true;
	}
	else
	{
		// Check the last received timestamp, if it's larger than the INIT timeout, then update the time, return true
		if ((Simulator::Now() - m_init_list.find(src_addr)->second) > m_init_timeout + Seconds(0.1))
		{
//			std::cout << "RREQ Current Time: " << Simulator::Now() << "\n";
			NS_LOG_DEBUG("INIT Current Time: " << Simulator::Now());

//			std::cout << "RREQ LIST LAST TS: " << m_rreq_list.find(src_addr)->second << "\n";
			NS_LOG_DEBUG("INIT LIST LAST TS: " << m_init_list.find(src_addr)->second);

			m_init_list.at(src_addr) = Simulator::Now();
			return true;
		}
		else
		{
			// Otherwise, filter out the frame
			return false;
		}
	}
}

bool
AquaSimRoutingMac::FilterDuplicateRts(AquaSimAddress src_addr)
{
	// Check if this address already exists in the list
	if (m_rts_list.count(src_addr) == 0)
	{
		// If no, then add the address with the current timestamp, return True -> packet is not filtered
		m_rts_list.insert(std::make_pair(src_addr, Simulator::Now()));
		return true;
	}
	else
	{
		// Check the last received timestamp, if it's larger than the RTS timeout, then update the time, return true
		if ((Simulator::Now() - m_rts_list.find(src_addr)->second) > m_rts_timeout + Seconds(0.1))
		{
//			std::cout << "RREQ Current Time: " << Simulator::Now() << "\n";
			NS_LOG_DEBUG("RTS Current Time: " << Simulator::Now());

//			std::cout << "RREQ LIST LAST TS: " << m_rreq_list.find(src_addr)->second << "\n";
			NS_LOG_DEBUG("RTS LIST LAST TS: " << m_rts_list.find(src_addr)->second);

			m_rts_list.at(src_addr) = Simulator::Now();
			return true;
		}
		else
		{
			// Otherwise, filter out the frame
			return false;
		}
	}
}

bool
AquaSimRoutingMac::FilterDuplicateCts(AquaSimAddress src_addr)
{
	// Check if this address already exists in the list
	if (m_cts_list.count(src_addr) == 0)
	{
		// If no, then add the address with the current timestamp, return True -> packet is not filtered
		m_cts_list.insert(std::make_pair(src_addr, Simulator::Now()));
		return true;
	}
	else
	{
		// Check the last received timestamp, if it's larger than the CTS timeout, then update the time, return true
		if ((Simulator::Now() - m_cts_list.find(src_addr)->second) > m_cts_timeout + Seconds(0.1))
		{
//			std::cout << "RREQ Current Time: " << Simulator::Now() << "\n";
			NS_LOG_DEBUG("CTS Current Time: " << Simulator::Now());

//			std::cout << "RREQ LIST LAST TS: " << m_rreq_list.find(src_addr)->second << "\n";
			NS_LOG_DEBUG("CTS LIST LAST TS: " << m_cts_list.find(src_addr)->second);

			m_cts_list.at(src_addr) = Simulator::Now();
			return true;
		}
		else
		{
			// Otherwise, filter out the frame
			return false;
		}
	}
}

void
AquaSimRoutingMac::UpdateDistance(double tx_power, double rx_power, AquaSimAddress dst_addr)
{
	if (m_distances.count(dst_addr) != 0)
	{
		m_distances.at(dst_addr) = CalculateDistance(tx_power, rx_power);
	}
	else
	{
		m_distances.insert(std::make_pair(dst_addr, CalculateDistance(tx_power, rx_power)));
	}
}

double
AquaSimRoutingMac::CalculateDistance(double tx_power, double rx_power)
{
	// A very rough approximation of rayleigh model, used in the propagation module:
	// rx_power = tx_power / d^k * alpha^(d/1000), k = 2
	// This approximation works if freq=25kHz, i.e. alpha ~ 4
	return sqrt(tx_power / rx_power);
}

double
AquaSimRoutingMac::CalculateTxPower(double d)
{
	  // Calculate Tx power given the distance and expected Rx_threshold
	  // The calculation is based on Rayleight model, used in the aqua-sim-propagation module:
	  // Rx = Tx / (d^k * alpha^(d/1000)), k = 2, alpha = 4.07831, (f = 25kHz)
	double tx_power = pow(d, 2) * pow(4.07831, (d / 1000)) * (m_rx_threshold + 0.0003); // 0.0003 to adjust the model

	if (tx_power > m_max_tx_power)
	{
		return m_max_tx_power;
	}
	else
	{
		return tx_power;
	}
}

double
AquaSimRoutingMac::CalculateOptimalMetric(double distance)
{
	// Optimal metric is a sub-distance of the given distance
	// This metric should be calculated considering max tx_power, rx_power threshold, processing power
	// Currently, use the constant number of intermediate nodes
	return distance / (3 + 1); // 3 intermediate nodes => 4 chunks of sub-distances
}

void AquaSimRoutingMac::DoDispose()
{
  NS_LOG_FUNCTION(this);
  AquaSimMac::DoDispose();
}
