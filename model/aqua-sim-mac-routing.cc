/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Dmitrii Dugaev <ddugaev@gradcenter.cuny.edu>
 * based on aqua-sim-mac-broadcast model
 */

#include "aqua-sim-mac-routing.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-mac.h"
#include "aqua-sim-address.h"

#include "ns3/log.h"
#include "ns3/integer.h"
#include "ns3/simulator.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimRoutingMac");
NS_OBJECT_ENSURE_REGISTERED(AquaSimRoutingMac);


/* ======================================================================
Adaptive forwarding (routing) MAC for  underwater sensor networks (based on broadcast-mac)
====================================================================== */

AquaSimRoutingMac::AquaSimRoutingMac()
{
  m_backoffCounter=0;
  m_rand = CreateObject<UniformRandomVariable> ();
}

TypeId
AquaSimRoutingMac::GetTypeId()
{
  static TypeId tid = TypeId("ns3::AquaSimRoutingMac")
      .SetParent<AquaSimMac>()
      .AddConstructor<AquaSimRoutingMac>()
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

	// Parse the incoming frames according to their types (DATA, RREP, RREQ, REWARD, ACK)
	if (dst == AquaSimAddress::GetBroadcast() || dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
	{

//		std::cout << "\n-------------------\nPTYPE: " << mac_routing_h.GetPType() << "\n";
//		std::cout << "TS: " << Simulator::Now() << "\n";
//		std::cout << "SRC: " << mac_routing_h.GetSrcAddr() << "\n";
//		std::cout << "DST: " << mac_routing_h.GetDstAddr() << "\n";
//		std::cout << "HOP COUNT: " << mac_routing_h.GetHopCount() << "\n";

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

		// Check the frame type
		// DATA
		if (mac_routing_h.GetPType() == 0)
		{
//			std::cout << "DATA PACKET\n";

			std::map<AquaSimAddress, AquaSimAddress> m; // Store dst_addr : sender_addr pair
			m.insert(std::make_pair(mach.GetDA(), mac_routing_h.GetSrcAddr()));
			// Send reward / ACK back
			if (m_reward_delays.count(m) == 0)
			{
				m_reward_delays.insert(std::make_pair(m, Simulator::Now()));
				// Delay and send back a reward message
//				SendReward(mac_routing_h.GetSrcAddr(), GenerateReward(mach.GetDA()), pkt);
//				Simulator::Schedule(m_reward_delay, &AquaSimRoutingMac::SendRewardHandler,
//						this, m, GenerateReward(mach.GetDA()), pkt->Copy());

				Simulator::Schedule(m_reward_delay, &AquaSimRoutingMac::SendRewardHandler,
						this, m, GenerateReward(mach.GetDA()), pkt);

			}

			// If dst_addr is its own, send up
			// If not, forward packet further
			if (mach.GetDA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// The data packet is for this node, send it up
			    pkt->AddHeader(ash);  //leave MacHeader off since sending to upper layers
				SendUp(pkt);
				return true;
			}

			// Otherwise, forward the packet further
			pkt->AddHeader(mach);
			pkt->AddHeader(ash);
			ForwardPacket (pkt);
//			pkt=0;
			return true;
		}
		// RREQ
		if (mac_routing_h.GetPType() == 1)
		{
			// Filter duplicate RREQ reception
//			std::cout << "RREQ SRC ADDR: " << mach.GetSA() << " TS: " << Simulator::Now() <<"\n";
			if (!FilterDuplicateRreq(mach.GetSA()))
			{
				// Filter out the frame
//				std::cout << "RREQ PACKET FILTERED\n";
				NS_LOG_DEBUG("RREQ PACKET FILTERED");
				pkt = 0;
				return true;
			}

//			std::cout << "Recevied RREQ src, dst: " << mac_routing_h.GetSrcAddr()
//					<< ", " << mac_routing_h.GetDstAddr() << " Hop Count: " << mac_routing_h.GetHopCount() << "\n";

			// Update forwarding table
			pkt->AddHeader(mac_routing_h);
			UpdateWeight(mach.GetSA(), mac_routing_h.GetSrcAddr(), CalculateWeight(pkt));

			// Check whether this RREQ is for this node or not
			if (mach.GetDA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// If yes, send out RREP back
				// Set mac_routing_header parameters
				Ptr<Packet> rrep = Create<Packet>();
				mac_routing_h.SetPType(2); // 2 - RREP
				mac_routing_h.SetId(0);
				mac_routing_h.SetHopCount(0);
				mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
				mac_routing_h.SetDstAddr(AquaSimAddress::GetBroadcast());

				// Set aqua-sim-header parameters for correct handling on Phy layer
				ash.SetSize(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize());
				ash.SetTxTime(Phy()->CalcTxTime(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize()));
				ash.SetErrorFlag(false);
				ash.SetDirection(AquaSimHeader::DOWN);

				rrep->AddHeader(mac_routing_h);
				rrep->AddHeader(mach);
				rrep->AddHeader(ash);

				// Put the RREP into rreq_list to further filter duplicate ones out
				FilterDuplicateRrep(mach.GetDA());
				SendDownFrame(rrep);

				return true;
			}
			else
			{
				// Otherwise, increment hop_count, broadcast RREQ further
				mac_routing_h.IncrementHopCount();
				mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
				mac_routing_h.SetDstAddr(AquaSimAddress::GetBroadcast());
				ash.SetDirection(AquaSimHeader::DOWN);

				pkt->AddHeader(mac_routing_h);
				pkt->AddHeader(mach);
				pkt->AddHeader(ash);

				SendDownFrame(pkt);
//				pkt=0;
			}

			return true;
		}
		// RREP
		if (mac_routing_h.GetPType() == 2)
		{
			// Filter duplicate RREP reception
			if (!FilterDuplicateRrep(mach.GetDA()))
			{
				// Filter out the frame
//				std::cout << "RREP PACKET FILTERED\n";
				NS_LOG_DEBUG("RREP PACKET FILTERED");
				pkt = 0;
				return true;
			}

//			std::cout << "Recevied RREP src, dst: " << mac_routing_h.GetSrcAddr()
//					<< ", " << mac_routing_h.GetDstAddr() << " Hop Count: " << mac_routing_h.GetHopCount() << "\n";

			// Update forwarding table
			pkt->AddHeader(mac_routing_h);
//			std::cout << "MAC_ROUTING src addr: " << mac_routing_h.GetSrcAddr() << "\n";
			UpdateWeight(mach.GetDA(), mac_routing_h.GetSrcAddr(), CalculateWeight(pkt));

			// Check whether this RREP is for this node or not
			if (mach.GetSA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// Update received timestamp and clear the send queue by sending out all buffered packets, and delete queue
				m_rrep_expirations.find(mach.GetDA())->second = Simulator::Now();
				while (!m_send_queue.find(mach.GetDA())->second.empty())
				{
					// Forward packet
//					std::cout << "Forwarding packet from send queue... " << "\n";
					NS_LOG_DEBUG("Forwarding packet from send queue... ");

					if (m_send_queue.count(mach.GetDA()) == 0)
					{
						return true;
					}

					Ptr<Packet> p = m_send_queue.find(mach.GetDA())->second.front();

					ForwardPacket(p);
					m_send_queue.find(mach.GetDA())->second.pop();

				}
				// Delete queue
				m_send_queue.erase(mach.GetDA());

				return true;
			}
			else
			{
				// Otherwise, increment hop_count, broadcast RREP further
				mac_routing_h.IncrementHopCount();
				mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
				mac_routing_h.SetDstAddr(AquaSimAddress::GetBroadcast());
				ash.SetDirection(AquaSimHeader::DOWN);

				pkt->AddHeader(mac_routing_h);
				pkt->AddHeader(mach);
				pkt->AddHeader(ash);

				SendDownFrame(pkt);
//				pkt=0;
			}

			return true;
		}
		// ACK / Reward
		if (mac_routing_h.GetPType() == 3)
		{
			// Check if the received reward message's dst_addr exists in the reward_expiration list
			std::map<AquaSimAddress, AquaSimAddress> m;
			m.insert(std::make_pair(mach.GetDA(), mac_routing_h.GetSrcAddr()));
//			std::cout << mach.GetDA() << "\n";
//			std::cout << mac_routing_h.GetSrcAddr() << "\n";

			if (m_reward_expirations.count(m) != 0)
			{
				// Check if the reward is received within the reward_timeout or not
//				std::cout << "Time difference: " << (Simulator::Now() - m_reward_expirations.find(m)->second) << "\n";
//				std::cout << "Reward timeout: " << m_reward_timeout << "\n";

				if ((Simulator::Now() - m_reward_expirations.find(m)->second) <= m_reward_timeout)
				{
					// Update reward value in the forwarding table
					UpdateWeight(m.begin()->first, m.begin()->second, mac_routing_h.GetReward());
//					std::cout << "REWARD FROM MESSAGE: " << mac_routing_h.GetReward() << "\n";

					// Update the timestamp value in the reward_expiration list
					m_reward_expirations.at(m) = Simulator::Now();

				}
				else
				{
					// Raise warning
//					std::cout << "Warning! Received reward is out of timeout: " <<
//							Simulator::Now() - m_reward_expirations.find(m)->second << "\n";
					NS_LOG_DEBUG("Warning! Received reward is out of timeout: " <<
							Simulator::Now() - m_reward_expirations.find(m)->second);
				}

			}
//			else
//			{
//				// This cannot happen
//				std::cout << "Warning! Reward is received from unexpected sender address: " << mac_routing_h.GetSrcAddr() << "\n";
////				std::cout << mach.GetDA() << "\n";
////				std::cout << mac_routing_h.GetSrcAddr() << "\n";
//			}

			return true;
		}
		// Some unknown packet type id
		else
		{
//			std::cout << "Unknown packet type is received: " << mac_routing_h.GetPType() << "\n";
			NS_LOG_DEBUG("Unknown packet type is received: " << mac_routing_h.GetPType());
		}

//    pkt->AddHeader(ash);  //leave MacHeader off since sending to upper layers
//	return SendUp(pkt);
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

	ForwardPacket(pkt);

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

//	  mach.SetDA(ash.GetDAddr());
//	  mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

//	  if( m_packetSize != 0 )
//	    ash.SetSize(m_packetSize);
//	  else
//	    ash.SetSize(m_packetHeaderSize + ash.GetSize());
//
//	  ash.SetTxTime(GetTxTime(pkt));
//
//	  switch( m_device->GetTransmissionStatus() )
//	  {
//	  case SLEEP:
//	      PowerOn();
//	      break;
//	  case NIDLE:
	      ash.SetDirection(AquaSimHeader::DOWN);
	      //ash->addr_type()=NS_AF_ILINK;
	      //add the sync hdr
//	      pkt->AddHeader(mach);
	      pkt->AddHeader(ash);
	      //Phy()->SetPhyStatus(PHY_SEND);

	      SendDown(pkt);

//	      m_backoffCounter=0;
//	      return true;
//	  case RECV:
//	    {
//	      double backoff=m_rand->GetValue()*BC_BACKOFF;
//	      NS_LOG_DEBUG("BACKOFF time:" << backoff << " on node:" << m_device->GetAddress() << "\n");
////	      std::cout << "BACKOFF time:" << backoff << " on node:" << m_device->GetAddress() << "\n";
//	      //pkt->AddHeader(mach);
//	      pkt->AddHeader(ash);
//	      Simulator::Schedule(Seconds(backoff),&AquaSimRoutingMac::BackoffHandler,this,pkt);
//	    }
//	      return true;
//	  case SEND:
//	    {
//	      double backoff=m_rand->GetValue()*BC_BACKOFF;
//	      NS_LOG_DEBUG("BACKOFF time:" << backoff << " on node:" << m_device->GetAddress() << "\n");
//	      //pkt->AddHeader(mach);
//	      pkt->AddHeader(ash);
//	      Simulator::Schedule(Seconds(backoff),&AquaSimRoutingMac::BackoffHandler,this,pkt);
//	    }
//	    return true;
//	      /*pkt=0;*/
//	  default:
//	      /*
//	      * all cases have been processed above, so simply return
//	      */
//	    break;
//	  }
	  return true;  //may be bug due to Sleep/default cases
}

// Forward packet / frame coming from the application or the network (DATA type)
bool
AquaSimRoutingMac::ForwardPacket(Ptr<Packet> p)
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

	// Check if dst_addr is in the forwarding table
	if (m_forwarding_table.count(dst_addr) != 0)
	{
		// The dst_addr exists in the table
		// Select next_hop neighbor and send down the packet
		AquaSimAddress next_hop_addr = SelectNextHop(dst_addr);
//		mach.SetDA(next_hop_addr);

		// Create mac_routing_header (DATA type)
		mac_routing_h.SetPType(0);
		mac_routing_h.SetId(0);
		mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
//		std::cout << "Next hop addr: " << next_hop_addr << "\n";
		mac_routing_h.SetDstAddr(next_hop_addr);
		mac_routing_h.IncrementHopCount(); // Move this to reception side !!!!!!

		p->AddHeader(mac_routing_h);
		p->AddHeader(mach);
		p->AddHeader(ash);

		std::map<AquaSimAddress, AquaSimAddress> m;
		m.insert(std::make_pair(dst_addr, next_hop_addr));
		// Set Reward timer
		if (m_reward_expirations.count(m) == 0)
		{
			// Trigger new reward expiration event for this address
			m_reward_expirations.insert(std::make_pair(m, Simulator::Now()- Seconds(0.1)));
//			std::cout << "New reward timeout scheduled: " << Simulator::Now() << "\n";
			NS_LOG_DEBUG("New reward timeout scheduled: " << Simulator::Now());

			Simulator::Schedule(m_reward_timeout, &AquaSimRoutingMac::RewardExpirationHandler, this, m);
		}

		// Send down the packet to the next hop
//		std::cout << "Sending down data packet " << Simulator::Now() << "\n";
		NS_LOG_DEBUG("Sending down data packet " << Simulator::Now());

//		std::cout << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << " --> " << mac_routing_h.GetDstAddr() << "\n";
		NS_LOG_DEBUG(AquaSimAddress::ConvertFrom(m_device->GetAddress()) << " --> " << mac_routing_h.GetDstAddr());

//		std::cout << "Transmission Status: " << m_device->GetTransmissionStatus() << "\n";
		NS_LOG_DEBUG("Transmission Status: " << m_device->GetTransmissionStatus());

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
		// If there is no entry for the given destination address, create a queue and trigger RREQ/RREP procedure
		else
		{
			// Create new sub-queue for given dst_addr
			std::queue<Ptr<Packet>> q;
			// Create new entry in send_queue
			m_send_queue.insert(std::make_pair(dst_addr, q));
			// Put packet in queue
//			m_send_queue.find(dst_addr)->second.push(p->Copy());
			m_send_queue.find(dst_addr)->second.push(p);

			// Generate and broadcast RREQ message
			Ptr<Packet> rreq = Create<Packet>();
			mac_routing_h.SetPType(1);	// 1 - RREQ message type
//			std::cout << mac_routing_h.GetPType() << "\n";
			mac_routing_h.SetId(0);
			mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
			// Set frame to broadcast address
			mac_routing_h.SetDstAddr(AquaSimAddress::GetBroadcast());
			mac_routing_h.SetHopCount(0);

			// Set aqua-sim-header parameters for correct handling on Phy layer
			ash.SetSize(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize());
			ash.SetTxTime(Phy()->CalcTxTime(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize()));
			ash.SetErrorFlag(false);
			ash.SetDirection(AquaSimHeader::DOWN);

			rreq->AddHeader(mac_routing_h);
			rreq->AddHeader(mach);
			rreq->AddHeader(ash);

			// Wait for RREP - set RREP timeout event in scheduler
			// Set RREP timer
//			std::cout << "FORWARD PACKET DST_ADDR: " << dst_addr << "\n";
			if (m_rrep_expirations.count(dst_addr) == 0)
			{
				m_rrep_expirations.insert(std::make_pair(dst_addr, Time::FromInteger(0, Time::S)));
				Simulator::Schedule(m_rrep_timeout, &AquaSimRoutingMac::RrepExpirationHandler, this, dst_addr);
			}

//			std::cout << "Sending RREQ " << Simulator::Now() << "\n";
			NS_LOG_DEBUG("Sending RREQ " << Simulator::Now());

			// Put the RREQ into rreq_list to further filter duplicate ones out
			FilterDuplicateRreq(mach.GetSA());
			// Send frame
			SendDownFrame(rreq);

			return true;
		}

		return true;
	}

	return true;
}

double
AquaSimRoutingMac::GenerateReward(AquaSimAddress dst_addr)
{
	// If no entries in forwarding table, return 0
	if (m_forwarding_table.count(dst_addr) == 0)
	{
		if (dst_addr == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
		{
			// If the dst_adress is this node itself, update the forwarding table and return reward
			UpdateWeight(dst_addr, dst_addr, 100*2); // Needs to multiply by two since the UpdateMethods calculates the average
			return 100; // Corresponds to the weight to itself
		}

		// Else, there this node has no path to the given destination, return 0
		return 0;
	}

	// Calculate and return an average weight towards a given destination from the node's forwarding table
	double weight_sum = 0;
	int n = 0;

	// Iterate through the weights to find average
	for (auto const& x : m_forwarding_table.find(dst_addr)->second)
	{
		weight_sum = weight_sum + x.second;
		n++;
	}

	return weight_sum / n;
}

void
AquaSimRoutingMac::SendRewardHandler (std::map<AquaSimAddress, AquaSimAddress> dst_sender_map, double reward, Ptr<Packet> p)
{
	// Send reward
	SendReward(dst_sender_map.begin()->second, reward, p);
	// Remove the corresponding map entry
	m_reward_delays.erase(dst_sender_map);
}

void
AquaSimRoutingMac::SendReward(AquaSimAddress dst_addr, double reward, Ptr<Packet> p)
{
	AquaSimHeader ash;
	MacHeader mach;
	MacRoutingHeader mac_routing_h;

	p->RemoveHeader(ash);
	p->RemoveHeader(mach);
	p->RemoveHeader(mac_routing_h);

	Ptr<Packet> reward_frame = Create<Packet>();

	mach.SetSA(dst_addr);
	mach.SetDA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));

	mac_routing_h.SetPType(3);	// 3 - Reward / ACK message type
	mac_routing_h.SetId(0);
	mac_routing_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	// Set frame to broadcast address
	mac_routing_h.SetDstAddr(dst_addr);
	mac_routing_h.SetReward(reward);

	// Set aqua-sim-header parameters for correct handling on Phy layer
	ash.SetSize(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize());
	ash.SetTxTime(Phy()->CalcTxTime(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize()));
	ash.SetErrorFlag(false);
	ash.SetDirection(AquaSimHeader::DOWN);

	// Send frame
	reward_frame->AddHeader(mac_routing_h);
	reward_frame->AddHeader(mach);
	reward_frame->AddHeader(ash);

//	std::cout << "SENDING REWARD: " << dst_addr << " " << Simulator::Now() << "\n";
	NS_LOG_DEBUG("SENDING REWARD: " << dst_addr << " " << Simulator::Now());

//	std::cout << "MACH DST ADDR: " << mach.GetDA() << "\n";
	SendDownFrame(reward_frame);
//	p = 0;
}

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

		m.insert(std::make_pair(next_hop_addr, reward / 2));
//		std::cout << "Creating new table entry with REWARD: " << reward / 2 << "\n";
		NS_LOG_DEBUG("Creating new table entry with REWARD: " << reward / 2);

		m_forwarding_table.insert(std::make_pair(dst_addr, m));
	}
	else
	{
		// Check if the next_hop_entry exist, if not - create one and return
		if (m_forwarding_table.find(dst_addr)->second.count(next_hop_addr) == 0)
		{
			m_forwarding_table.find(dst_addr)->second.insert(std::make_pair(next_hop_addr, reward / 2));
		}

		double current_weight = m_forwarding_table.find(dst_addr)->second.find(next_hop_addr)->second;
//		std::cout << "CURRENT WEIGHT: " << current_weight << " Node Address: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()) << "\n";
		NS_LOG_DEBUG("CURRENT WEIGHT: " << current_weight << " Node Address: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()));

//		std::cout << "REWARD: " << reward << "\n";
		NS_LOG_DEBUG("REWARD: " << reward);

//		std::cout << "TABLE SIZE: " << m_forwarding_table.size() << "\n";
		NS_LOG_DEBUG("TABLE SIZE: " << m_forwarding_table.size());

		// If the current weight is too small (due to lack of rewards), just remove the entry from the table
		if ((current_weight + reward) <= 0)
//		if (current_weight < 0.01)
		{
//			std::cout << "Warning! Weight dropped below the threshold! Deleting table entry!\n";
			NS_LOG_DEBUG("Warning! Weight dropped below the threshold! Deleting table entry!");

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
AquaSimRoutingMac::CalculateWeight(Ptr<Packet> frame)
{
	MacRoutingHeader mac_routing_h;
	frame->RemoveHeader(mac_routing_h);

	// Calculate initial weight based on hop count value
	// in future, change it to also consider the energy loss factor, i.e. (Tx - Rx) power difference
	return 100.0 / (mac_routing_h.GetHopCount() + 1);
}

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
AquaSimRoutingMac::RrepExpirationHandler(AquaSimAddress node_address)
{
	// Check the last received RREP timestamp for a given address
	// If the time difference is greater than the RREP timeout value, then clear the corresponding queue
	if ((Simulator::Now() - m_rrep_expirations.find(node_address)->second) > m_rrep_timeout)
	{
//		std::cout << "RREP wait timeout!\n";
		NS_LOG_DEBUG("RREP wait timeout!");

//		std::cout << "DST_ADDR: " << node_address << "\n";
		NS_LOG_DEBUG("DST_ADDR: " << node_address);

//		std::cout << "TS: " << m_rrep_expirations.find(node_address)->second << "\n";
		NS_LOG_DEBUG("TS: " << m_rrep_expirations.find(node_address)->second);

//		std::cout << "Current Time: " << Simulator::Now() << "\n";
		NS_LOG_DEBUG("Current Time: " << Simulator::Now());

		// Delete the entry and the queue
		m_rrep_expirations.erase(node_address);
		m_send_queue.erase(node_address);
	}
}

// Filter duplicate broadcast frames reception
bool
AquaSimRoutingMac::FilterDuplicateRreq(AquaSimAddress src_addr)
{
	// Check if this address already exists in the list
	if (m_rreq_list.count(src_addr) == 0)
	{
		// If no, then add the address with the current timestamp, return True -> packet is not filtered
		m_rreq_list.insert(std::make_pair(src_addr, Simulator::Now()));
		return true;
	}
	else
	{
		// Check the last received timestamp, if it's larger than the RREP timeout, then update the time, return true
		if ((Simulator::Now() - m_rreq_list.find(src_addr)->second) > m_rrep_timeout + Seconds(0.1))
		{
//			std::cout << "RREQ Current Time: " << Simulator::Now() << "\n";
			NS_LOG_DEBUG("RREQ Current Time: " << Simulator::Now());

//			std::cout << "RREQ LIST LAST TS: " << m_rreq_list.find(src_addr)->second << "\n";
			NS_LOG_DEBUG("RREQ LIST LAST TS: " << m_rreq_list.find(src_addr)->second);

			m_rreq_list.at(src_addr) = Simulator::Now();
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
AquaSimRoutingMac::FilterDuplicateRrep(AquaSimAddress src_addr)
{
	// Check if this address already exists in the list
	if (m_rrep_list.count(src_addr) == 0)
	{
		// If no, then add the address with the current timestamp, return True -> packet is not filtered
		m_rrep_list.insert(std::make_pair(src_addr, Simulator::Now()));
		return true;
	}
	else
	{
		// Check the last received timestamp, if it's larger than the RREP timeout, then update the time, return true
		if ((Simulator::Now() - m_rrep_list.find(src_addr)->second) >= m_rrep_timeout)
		{
			m_rrep_list.at(src_addr) = Simulator::Now();
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
AquaSimRoutingMac::BackoffHandler(Ptr<Packet> pkt)
{
  m_backoffCounter++;
  if (m_backoffCounter<BC_MAC_ROUTING_MAXIMUMCOUNTER)
//    TxProcess(pkt);
	  SendDownFrame(pkt);
  else
    {
//      NS_LOG_INFO("BackoffHandler: too many backoffs");
      NS_LOG_DEBUG("BackoffHandler: too many backoffs");
      m_backoffCounter=0;
      DropPacket(pkt);
    }
}

void AquaSimRoutingMac::DoDispose()
{
  NS_LOG_FUNCTION(this);
  AquaSimMac::DoDispose();
}
