/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2020 The City University of New York
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
 * Author: Dmitrii Dugaev <ddugaev@gradcenter.cuny.edu>
 */

#include "aqua-sim-mac-jamming.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-mac.h"
#include "aqua-sim-address.h"

#include "ns3/log.h"
#include "ns3/integer.h"
#include "ns3/simulator.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimJammingMac");
NS_OBJECT_ENSURE_REGISTERED(AquaSimJammingMac);


/* =========
Jamming MAC
============ */

AquaSimJammingMac::AquaSimJammingMac()
{
  m_rand = CreateObject<UniformRandomVariable> ();
  m_packetSize = 800; // bytes
  m_cc_accumulation_time = Seconds(2);
  m_epoch_time = Seconds(10);
  m_guard_time = MilliSeconds(1);
}

TypeId
AquaSimJammingMac::GetTypeId()
{
  static TypeId tid = TypeId("ns3::AquaSimJammingMac")
      .SetParent<AquaSimMac>()
      .AddConstructor<AquaSimJammingMac>()
      .AddAttribute("PacketSize", "Size of packet",
        IntegerValue(800),
        MakeIntegerAccessor (&AquaSimJammingMac::m_packetSize),
        MakeIntegerChecker<uint16_t> ())
      .AddAttribute ("AccumulationTime", "Interval to accumulate all incoming cc-requests",
        TimeValue(Seconds(2)),
        MakeTimeAccessor(&AquaSimJammingMac::m_cc_accumulation_time),
        MakeTimeChecker())
      .AddAttribute ("EpochTime", "Interval between cc-request generations",
        TimeValue(Seconds(10)),
        MakeTimeAccessor(&AquaSimJammingMac::m_epoch_time),
        MakeTimeChecker())
      .AddAttribute ("GuardTime", "Interval in-between the transmissions in a train",
        TimeValue(MilliSeconds(1)),
        MakeTimeAccessor(&AquaSimJammingMac::m_guard_time),
        MakeTimeChecker())
    ;
  return tid;
}

int64_t
AquaSimJammingMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rand->SetStream(stream);
  return 1;
}

void
AquaSimJammingMac::StartCC()
{
  if (m_sendQueue.size() != 0)
  {
    // Log epoch number (amount of cc --> cs --> data handshakes)
    std::cout << "EPOCH No: " << m_epoch_no << "\n";
    // Start cc phase
    // Read aqua-sim header info from a data packet
    AquaSimHeader ash;
    m_sendQueue.front()->PeekHeader(ash);

    // Create a CC-request
    Ptr<Packet> cc_request = Create<Packet>();
    MacHeader mach;
    mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()) );
    mach.SetDA(ash.GetDAddr());
    mach.SetDemuxPType(MacHeader::UWPTYPE_OTHER);
    JammingMacHeader jamh;
    jamh.SetPType(1);           // 1 - cc-request
    jamh.SetNodeId(m_device->GetNode()->GetId());
    cc_request->AddHeader(jamh);
    cc_request->AddHeader(mach);
    cc_request->AddHeader(ash);

    // Send CC-request towards the sink, using pure-ALOHA
    Simulator::Schedule(Seconds(GetBackoff()), &AquaSimJammingMac::SendPacketAloha, this, cc_request->Copy());
    m_epoch_no++;
  }

  // Schedule next CC-request after some time
  Simulator::Schedule(m_epoch_time, &AquaSimJammingMac::StartCC, this);
}

void
AquaSimJammingMac::SendPacketAloha(Ptr<Packet> pkt)
{
  // Check net_device status
  // If it is IDLE, send packet down to PHY immediately
  // otherwise, do random backoff again
  if (m_device->GetTransmissionStatus() == TransStatus::NIDLE)
  {
    // Call parent method to send packet down to PHY
    SendDown(pkt);
  }
  else
  {
    // Do another backoff
    Simulator::Schedule(Seconds(GetBackoff()), &AquaSimJammingMac::SendPacketAloha, this, pkt->Copy());
  }
}

// Return a random number in range
double AquaSimJammingMac::GetBackoff()
{
  return m_rand->GetValue(0.1, 1.0);
}

void
AquaSimJammingMac::TriggerCsReply()
{
  // Create a schedule based on a list of the requesting nodes
  std::map<uint32_t, uint16_t> schedule = CreateSchedule(TRAIN_SCHEDULE);

  // Generate and broadcats the schedule (CS-reply) back
  Ptr<Packet> cs_reply = GenerateCsReply(schedule);
  // Broadcast the packet using Aloha
  Simulator::Schedule(Seconds(GetBackoff()), &AquaSimJammingMac::SendPacketAloha, this, cs_reply->Copy());

  // Clear the cc-request list
  m_request_list.clear();
}

std::map<uint32_t, uint16_t>
AquaSimJammingMac::CreateSchedule(JammingMacScheduleType schedule_type)
{
  std::map<uint32_t, uint16_t> schedule;
  if (schedule_type == TRAIN_SCHEDULE)
  {
    // Iterate over all the cc-requests and schedule their transmissions side-by-side
    uint16_t delay_ms = 0;
    uint16_t pkt_tx_time_ms = GetTxTime(m_packetSize).GetMilliSeconds();
    for (auto const& x : m_request_list)
    {
      schedule.insert(std::make_pair(x.first, delay_ms));
      delay_ms += (pkt_tx_time_ms + m_guard_time.GetMilliSeconds());
    }
  }
  else if (schedule_type == RANDOM_SCHEDULE)
  {
    NS_FATAL_ERROR ("Not implemented!");
  }
  else if (schedule_type == AUCTION_SCHEDULE)
  {
    NS_FATAL_ERROR ("Not implemented!");
  }
  else
  {
    NS_FATAL_ERROR ("Invalid schedule type!");
  }
  return schedule;
}

Ptr<Packet>
AquaSimJammingMac::GenerateCsReply(std::map<uint32_t, uint16_t> schedule)
{
  // Create a packet and allocate the corresponding headers
  AquaSimHeader ash;
  MacHeader mach;
  JammingMacHeader jamh;

  // Src/dst addresses
  AquaSimAddress src = AquaSimAddress::ConvertFrom(GetAddress());   // sink-node address
  AquaSimAddress dst = AquaSimAddress::GetBroadcast();              // broadcast cs-reply to all the nodes

  Ptr<Packet> cs_reply = Create<Packet>();
  ash.SetSAddr(src);
  ash.SetDAddr(dst);
  ash.SetDirection(AquaSimHeader::DOWN);
  mach.SetSA(src);
  mach.SetDA(dst);
  mach.SetDemuxPType(MacHeader::UWPTYPE_OTHER);
  // Set schedule
  JammingMacHeader cs_reply_h;
  cs_reply_h.SetPType(2);           // 2 - cs-reply
  cs_reply_h.SetNodeId(m_device->GetNode()->GetId());
  // Allocate the schedule
  for (auto const& x : schedule)
  {
    cs_reply_h.SetSchedule(x.first, x.second);  
  }
  cs_reply->AddHeader(cs_reply_h);
  cs_reply->AddHeader(mach);
  cs_reply->AddHeader(ash);
  return cs_reply;
}

void
AquaSimJammingMac::ProcessCcRequest(uint32_t node_id, Vector coords)
{
  // Store the node info in a request list
  if (m_request_list.find(node_id) == m_request_list.end())
  {
    // Insert a new entry in a map
    m_request_list.insert(std::make_pair(node_id, coords));
  }
}

bool
AquaSimJammingMac::RecvProcess (Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION(this);

	AquaSimHeader ash;
  MacHeader mach;
  JammingMacHeader jamh;
  pkt->RemoveHeader(ash);
  pkt->RemoveHeader(mach);
  pkt->RemoveHeader(jamh);
	AquaSimAddress dst = mach.GetDA();
	AquaSimAddress src = mach.GetSA();


	if (ash.GetErrorFlag())
	{
		NS_LOG_DEBUG("JammingMac:RecvProcess: received corrupt packet.");
		pkt=0;
		return false;
	}

  // std::cout << "Node ID: " << m_device->GetNode()->GetId() << "\n";
  // std::cout << "SRC ADDR: " << mach.GetSA() << "\n";
  // std::cout << "DST ADDR: " << mach.GetDA() << "\n";

  // If the destination unicast, then process either cc-request or data-packet
  // TODO: assert that this is actually a sink
	if (dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
	{
    // Check the received packet type
    // If the packet is CC-request (type == 1)
    if (jamh.GetPType() == 1)
    {
      std::cout << "Time_ms: " << Simulator::Now().GetMilliSeconds() << "\tSink:" << m_device->GetNode()->GetId() << "\tCC-request received from Node " << +jamh.GetNodeId() << "\n";
      // Handle CC-request on a sink
      if (!m_sink_timer.IsRunning())
      {
        // If the timer is not running, start it and set the expiration time
        m_sink_timer.SetFunction(&AquaSimJammingMac::TriggerCsReply, this);
        m_sink_timer.SetDelay(m_cc_accumulation_time);
        m_sink_timer.Schedule();
      }

      // Accumulate all incoming cc-requests while the timer runs
      Vector coords = Vector(0, 0, 0);
      ProcessCcRequest(jamh.GetNodeId(), coords);
    }
    else if (jamh.GetPType() == 0)
    {
      std::cout << "Time_ms: " << Simulator::Now().GetMilliSeconds() << "\tSink:" << m_device->GetNode()->GetId() << "\tData-packet received from Node " << +jamh.GetNodeId() << "\n";
      // Attach aqua-sim-header and send packet up
      pkt->AddHeader(ash);
      SendUp(pkt);
    }
	}
  // If the destination is broadcast, then the packet is cs-reply, process it accordingly
  else if (dst == AquaSimAddress::GetBroadcast())
  {
    if (jamh.GetPType() == 2)
    {
      std::cout << "Time_ms: " << Simulator::Now().GetMilliSeconds() << "\tNode:" << m_device->GetNode()->GetId() << "\tCS-reply received from Node " << +jamh.GetNodeId() << "\n";
      // Schedule packet transmissions based on the schedule
      if (jamh.GetNodeBit(m_device->GetNode()->GetId()) == 1)
      {
        Time tx_delay = MilliSeconds(jamh.GetSchedule(m_device->GetNode()->GetId()));
        // Get data packet from queue
        Ptr<Packet> data_packet = m_sendQueue.front();
        m_sendQueue.pop_front();
        // Attach mac-headers to packet
        data_packet->RemoveHeader(ash);
        mach.SetSA(ash.GetSAddr());
        mach.SetDA(ash.GetDAddr());
        mach.SetDemuxPType(MacHeader::UWPTYPE_OTHER);
        JammingMacHeader data_h;
        data_h.SetPType(0);       // 0 - data packet
        data_h.SetNodeId(m_device->GetNode()->GetId());
        data_packet->AddHeader(data_h);
        data_packet->AddHeader(mach);
        data_packet->AddHeader(ash);
        
        Simulator::Schedule(tx_delay, &AquaSimJammingMac::SendDown, this, data_packet, ns3::NIDLE);
      }
    }
    else
    {
      NS_FATAL_ERROR ("The broadcast-packet is not CS-reply!");
    }
  }
  else
  {
    // TODO:
    // log the transmission, overheard from the other nodes to another destination
  }

	pkt=0;
	return false;
}

bool
AquaSimJammingMac::TxProcess(Ptr<Packet> pkt)
{
  NS_LOG_FUNCTION(this << pkt);

  // Put incoming data packets to send-queue
  m_sendQueue.push_back(pkt);

  // Start the very first Channel Competition (CC) phase
  if (!m_firstCcInit)
  {
    StartCC();
    m_firstCcInit = true;
  }

  return true;
}

void AquaSimJammingMac::DoDispose()
{
  NS_LOG_FUNCTION(this);
  AquaSimMac::DoDispose();
}
