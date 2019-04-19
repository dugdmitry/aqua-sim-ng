/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Dmitrii Dugaev <ddugaev@gradcenter.cuny.edu>
 */

#include "aqua-sim-mac-routing-aloha.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-mac.h"
#include "aqua-sim-address.h"

#include "ns3/log.h"
#include "ns3/integer.h"
#include "ns3/simulator.h"

#include <math.h>

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimRoutingMacAloha");
NS_OBJECT_ENSURE_REGISTERED(AquaSimRoutingMacAloha);


/* ======================================================================
Adaptive forwarding (routing) MAC for underwater sensor networks
====================================================================== */

AquaSimRoutingMacAloha::AquaSimRoutingMacAloha()
{
  m_rand = CreateObject<UniformRandomVariable> ();
  m_max_range = 150;
  m_max_tx_power = 20; // Watts

  m_status = IDLE;
}

TypeId
AquaSimRoutingMacAloha::GetTypeId()
{
  static TypeId tid = TypeId("ns3::AquaSimRoutingMacAloha")
      .SetParent<AquaSimMac>()
      .AddConstructor<AquaSimRoutingMacAloha>()
      .AddAttribute("max_range", "Maximum transmission range",
        DoubleValue(150),
        MakeDoubleAccessor (&AquaSimRoutingMacAloha::m_max_range),
        MakeDoubleChecker<double> ())
	  .AddAttribute("max_tx_power", "Maximum transmission power",
		DoubleValue(20),
		MakeDoubleAccessor (&AquaSimRoutingMacAloha::m_max_tx_power),
		MakeDoubleChecker<double> ())
	  .AddAttribute("optimal_metric", "Optimal Distance metric, m",
		DoubleValue(50),
		MakeDoubleAccessor (&AquaSimRoutingMacAloha::m_optimal_metric),
		MakeDoubleChecker<double> ())
     ;
  return tid;
}

int64_t
AquaSimRoutingMacAloha::AssignStreams (int64_t stream)
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
AquaSimRoutingMacAloha::RecvProcess (Ptr<Packet> pkt)
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

	if (ash.GetErrorFlag())
	{
		NS_LOG_DEBUG("RoutingMac:RecvProcess: received corrupt packet.");
//		std::cout << "RoutingMac:RecvProcess: received corrupt packet.\n";
		pkt=0;
		return false;
	}

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
			NS_LOG_DEBUG("Warning! The Hop Count exceeds the maximum!");
			pkt = 0;
			return false;
		}

		// DATA PACKET TYPE
		////////////////////
		////////////////////

		// If the DATA packet comes from the network (i.e. PHY), then process it
		if (mac_routing_h.GetPType() == 0)
		{
			// TODO: maybe to include some overhearing here. I.e., update the local distances table from all received packets
			// The DATA packet must be destined to the node
			if (dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
			{
				// Update the distance to the source node, if this information is missing in the local distances table.
				// This might happen when INIT messages are lost.
				if (m_distances.count(mac_routing_h.GetSrcAddr()) == 0)
				{
					m_distances.insert(std::make_pair(mac_routing_h.GetSrcAddr(), mac_routing_h.GetNextHopDistance() - m_dist_error));
				}

				// If dst_addr is its own, send up
				// If not, forward packet further
				double reward = 0;
				if (mach.GetDA() == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
				{
					// The data packet is for this node, send it up
					pkt->AddHeader(ash);  //leave MacHeader off since sending to upper layers
//					NS_LOG_DEBUG("SENDING UP HOP COUNT: " << mac_routing_h.GetHopCount());
					SendUp(pkt);

					// Generate a single direct reward message back to the sender
					// Set mac_routing_header parameters
					Ptr<Packet> reward_msg = Create<Packet>();
					MacRoutingHeader reward_h;
					reward_h.SetPType(7); // 7 - DIRECT REWARD MESSAGE
					reward_h.SetId(0);
					reward_h.SetSrcAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
					reward_h.SetDstAddr(mac_routing_h.GetSrcAddr());

//					std::cout << "REWARD SRC ADDR: " << reward_h.GetSrcAddr() << "\n";
//					std::cout << "REWARD DST ADDR: " << reward_h.GetDstAddr() << "\n";

					// Set aqua-sim-header parameters for correct handling on Phy layer
					ash.SetSize(reward_h.GetSerializedSize() + mach.GetSerializedSize());
					ash.SetTxTime(Phy()->CalcTxTime(reward_h.GetSerializedSize() + mach.GetSerializedSize()));
					ash.SetErrorFlag(false);
					ash.SetDirection(AquaSimHeader::DOWN);

					// Set Tx power for the frame
//					reward_h.SetTxPower(CalculateTxPower(CalculateDistance(mac_routing_h.GetTxPower(),
//							mac_routing_h.GetRxPower()))); // Send direct reward message considering the distance

					reward_h.SetTxPower(CalculateTxPower(CalculateDistance(mac_routing_h.GetSrcAddr()))); // Send direct reward message considering the distance


					// Set reward value
					reward = CalculateWeight(mac_routing_h.GetDirectDistance() - 0);
//					std::cout << "DIRECT DISTANCE: " << mac_routing_h.GetDirectDistance() << "\n";
					// Since the node is the destination itself, the residual distance is zero
//					std::cout << "RESIDUAL DISTANCE: " << 0 << "\n";

					reward_h.SetReward(reward);

					reward_msg->AddHeader(reward_h);
					reward_msg->AddHeader(mach);
					reward_msg->AddHeader(ash);

					SendDownFrame(reward_msg);  // Send down the direct reward message

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
AquaSimRoutingMacAloha::DropPacket(Ptr<Packet> pkt)
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
AquaSimRoutingMacAloha::TxProcess(Ptr<Packet> pkt)
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
AquaSimRoutingMacAloha::SendDownFrame (Ptr<Packet> pkt)
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

	  // Insert the next_hop distance to each sent packet. Get next hop distance from the distances list
	  // Add some "guard distance", just to mitigate the initial error in distance calculation from Tx/Rx
	  mac_routing_h.SetNextHopDistance(m_distances.at(mac_routing_h.GetDstAddr()) + m_dist_error);

	  ash.SetTimeStamp(Simulator::Now());
      ash.SetDirection(AquaSimHeader::DOWN);
      pkt->AddHeader(mac_routing_h);
      pkt->AddHeader(mach);
      pkt->AddHeader(ash);

      // Pre-queue the packet. If the device status will not be idle, then re-transmit the packet after some interval
      m_send_buffer.push(pkt->Copy());

      // Send down frames, considering the device status
      Send(pkt);

      return true;
}

void
AquaSimRoutingMacAloha::ResendFrame()
{
  //m_device->SetTransmissionStatus(NIDLE);

    if (!m_send_buffer.empty() ) {
    	Ptr<Packet> tmp = m_send_buffer.front();

			Send(tmp->Copy());
		}
}

bool
AquaSimRoutingMacAloha::Send (Ptr<Packet> pkt)
{
	NS_LOG_FUNCTION(this << pkt);
  AquaSimHeader asHeader;
  pkt->PeekHeader(asHeader);


  //compute estimated RTT
//  Time txtime = asHeader.GetTxTime();
  Time txtime = asHeader.GetTxTime();
//  Time txtime = Seconds(0.5);


//  std::cout << "TX TIME: " << txtime << "\n";
//  Time ertt = txtime + GetTxTime(alohaH.GetSerializedSize()) + Seconds(m_waitACKTimeOffset);

  switch( m_device->GetTransmissionStatus() ) {
    case SLEEP:
      PowerOn();

    case NIDLE: {
      //m_device->SetTransmissionStatus(SEND);

		if (!m_send_buffer.empty()) {
			m_send_buffer.front()=0;
			m_send_buffer.pop();
		}

      SendDown(pkt);

//      Simulator::Schedule(txtime + Seconds(0.01), &AquaSimRoutingMacAloha::StatusProcess, this);
      Simulator::Schedule(txtime + Seconds(0.1), &AquaSimRoutingMacAloha::ResendFrame, this);

      break;
		}
    case RECV:
      NS_LOG_INFO("SendPkt: RECV-SEND collision!!!");
	pkt=0;
      break;

    default:
    //status is SEND
      NS_LOG_INFO("SendPkt: node " << m_device->GetNode() << " send data too fast");
	pkt=0;
  }
  return true;
}

// Forward packet / frame coming from the application or the network (DATA type)
bool
AquaSimRoutingMacAloha::ForwardPacket(Ptr<Packet> p, AquaSimAddress sender_addr, int hop_count, double reward)
{
	AquaSimHeader ash;
	MacHeader mach;
	MacRoutingHeader mac_routing_h;

	p->RemoveHeader(ash);
	p->RemoveHeader(mach);

	// Store destination address as integer
	AquaSimAddress dst_addr = mach.GetDA();

	// Reset the direction if the packet is forwarded by intermediate node
	ash.SetDirection(AquaSimHeader::DOWN);

	if (m_distances.count(dst_addr) == 0)
	{
		m_distances.insert(std::make_pair(dst_addr, CalculateDistance(dst_addr)));
	}

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
//			std::cout << "TABLE SIZE: " << m_forwarding_table.size() << "\n";
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
		// DO NOT CONSIDER HEADER OVERHEAD IN THE TESTS
//		ash.SetTxTime(Phy()->CalcTxTime(mac_routing_h.GetSerializedSize() + mach.GetSerializedSize() + p->GetSize()));
//		ash.SetTxTime(Phy()->CalcTxTime(p->GetSize()));

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

		// Send Frame
		NS_LOG_DEBUG("TX power data: " << mac_routing_h.GetTxPower());

		SendDownFrame(p);

		return true;

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
AquaSimRoutingMacAloha::SelectNextHop(AquaSimAddress dst_addr)
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
AquaSimRoutingMacAloha::UpdateWeight(AquaSimAddress dst_addr, AquaSimAddress next_hop_addr, double reward)
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
//		NS_LOG_DEBUG("CURRENT WEIGHT: " << current_weight << " Node Address: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()));
//		NS_LOG_DEBUG("REWARD: " << reward);
//		NS_LOG_DEBUG("TABLE SIZE: " << m_forwarding_table.size());

		// If the current weight is too small (due to lack of rewards), just remove the entry from the table
		if ((current_weight + reward) <= 0)
		{
			NS_LOG_DEBUG("Warning! Weight dropped below the threshold! Deleting table entry!");

			m_forwarding_table.find(dst_addr)->second.erase(next_hop_addr);
			if (m_forwarding_table.find(dst_addr)->second.size() == 0)
			{
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
AquaSimRoutingMacAloha::CalculateWeight(double distance)
{
	// Calculate weight based on the difference between optimal_distance and D(Tx,Rx)
	if (distance <= 0)
	{
		// Return minimum possible reward
		return m_min_reward;
	}

	// TODO: Think out how to give more weight to a closer node, then to a more distant one.
	if (m_optimal_metric <= distance)
	{
		return 100 * (m_optimal_metric / distance);
	}
	else
	{
		return 100 * (distance / m_optimal_metric);
	}
}

//// Not used for now
//void
//AquaSimRoutingMacAloha::RewardExpirationHandler(std::map<AquaSimAddress, AquaSimAddress> dst_to_next_hop_map)
//{
//	// Check the last received reward timestamp for a given address
//	// If the time difference is greater than the reward timeout value, then generate a negative reward
//	if ((Simulator::Now() - m_reward_expirations.find(dst_to_next_hop_map)->second) > m_reward_timeout)
//	{
////		std::cout << "Reward wait timeout! " << Simulator::Now() - m_reward_expirations.find(dst_to_next_hop_map)->second << "\n";
//		NS_LOG_DEBUG("Reward wait timeout! " << Simulator::Now() - m_reward_expirations.find(dst_to_next_hop_map)->second);
//
//		// Generate negative reward
//		UpdateWeight(dst_to_next_hop_map.begin()->first, dst_to_next_hop_map.begin()->second, m_negative_reward);
//	}
////	// Update timestamp
////	m_reward_expirations.at(dst_to_next_hop_map) = Simulator::Now() - Seconds(0.1);
////	Simulator::Schedule(m_reward_timeout, &AquaSimRoutingMac::RewardExpirationHandler, this, dst_to_next_hop_map);
//	// Delete entry
//	m_reward_expirations.erase(dst_to_next_hop_map);
//}

void
AquaSimRoutingMacAloha::SetToIdle()
{
	// Change state to IDLE, since the current state timeout hasn't been changed, therefore, no other timeouts were introduced
	if (m_state_timeout <= Simulator::Now())
	{
//		std::cout << "SETTING TO IDLE: " << Simulator::Now() << "\n";
		m_status = IDLE;
	}
}

void
AquaSimRoutingMacAloha::UpdateDistance(double tx_power, double rx_power, AquaSimAddress dst_addr)
{
	if (m_distances.count(dst_addr) != 0)
	{
//		m_distances.at(dst_addr) = CalculateDistance(tx_power, rx_power);
		m_distances.at(dst_addr) = CalculateDistance(dst_addr);
	}
	else
	{
//		m_distances.insert(std::make_pair(dst_addr, CalculateDistance(tx_power, rx_power)));
		m_distances.insert(std::make_pair(dst_addr, CalculateDistance(dst_addr)));
	}
}

double
AquaSimRoutingMacAloha::CalculateDistance(double tx_power, double rx_power)
{
	// A very rough approximation of rayleigh model, used in the propagation module:
	// rx_power = tx_power / d^k * alpha^(d/1000), k = 2
	// This approximation works if freq=25kHz, i.e. alpha ~ 4
	NS_LOG_DEBUG("TX_POWER_CALC_DISTANCE: " << tx_power);
	NS_LOG_DEBUG("RX_POWER_CALC_DISTANCE: " << rx_power);
	return sqrt(tx_power / rx_power);
}

double
AquaSimRoutingMacAloha::CalculateDistance(AquaSimAddress dst_addr)
{
	double dist = 0;
	// Get the distance from the mobility model

    Ptr<Object> sObject = m_device->GetNode();
    Ptr<MobilityModel> senderModel = sObject->GetObject<MobilityModel> ();

    Ptr<Object> rObject = m_device->GetChannel()->GetDevice(dst_addr.GetAsInt() - 1)->GetNode();
    Ptr<MobilityModel> recvModel = rObject->GetObject<MobilityModel> ();

    dist = senderModel->GetDistanceFrom(recvModel);

//    std::cout << "MOBILITY DISTANCE: " << dist << "\n";

    return dist;
}

double
AquaSimRoutingMacAloha::CalculateTxPower(double d)
{
	  // Calculate Tx power given the distance and expected Rx_threshold
	  // The calculation is based on Rayleight model, used in the aqua-sim-propagation module:
	  // Rx = Tx / (d^k * alpha^(d/1000)), k = 2, alpha = 4.07831, (f = 25kHz)
//	double tx_power = pow(d, 2) * pow(4.07831, (d / 1000)) * (m_rx_threshold + 0.0003); // 0.0003 to adjust the model
	double tx_power = pow(d, 2) * pow(4.07831, (d / 1000)) * m_rx_threshold + 0.0001;
	NS_LOG_DEBUG("GIVEN DISTANCE: " << d);
	NS_LOG_DEBUG("CALCULATED TX POWER: " << tx_power);

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
AquaSimRoutingMacAloha::CalculateOptimalMetric(double distance)
{
	// Optimal metric is a sub-distance of the given distance
	// This metric should be calculated considering max tx_power, rx_power threshold, processing power
	// Currently, use the constant number of intermediate nodes
	return distance / (3 + 1); // 3 intermediate nodes => 4 chunks of sub-distances
}

void AquaSimRoutingMacAloha::DoDispose()
{
  NS_LOG_FUNCTION(this);
  AquaSimMac::DoDispose();
}
