/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
* Author: dmitrii <dugdmitry@gmail.com>
*/

#include "aqua-sim-routing-efrlr.h"
#include "aqua-sim-address.h"
#include "aqua-sim-header-routing.h"
#include "aqua-sim-header.h"
#include "ns3/log.h"

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimRoutingEfrlr");
NS_OBJECT_ENSURE_REGISTERED(AquaSimRoutingEfrlr);


AquaSimRoutingEfrlr::AquaSimRoutingEfrlr()
{
  m_rand = CreateObject<UniformRandomVariable> ();
  Simulator::Schedule(Seconds(1), &AquaSimRoutingEfrlr::PrintGlobalTopology, this);
}

TypeId
AquaSimRoutingEfrlr::GetTypeId()
{
  static TypeId tid = TypeId ("ns3::AquaSimRoutingEfrlr")
    .SetParent<AquaSimRouting> ()
    .AddConstructor<AquaSimRoutingEfrlr> ()
  ;
  return tid;
}

int64_t
AquaSimRoutingEfrlr::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  return 0;
}

// Method to extract global network topology, i.e. positions of all the nodes
void
AquaSimRoutingEfrlr::PrintGlobalTopology() {
  uint32_t nDev = GetNetDevice()->GetChannel()->GetNDevices();
  std::cout << "NODE " << GetNetDevice()->GetNode()->GetId() << ":\n";
  for (uint32_t i=0; i<nDev; i++) {
    AquaSimAddress addr = AquaSimAddress::ConvertFrom(GetNetDevice()->GetChannel()->GetDevice(i)->GetAddress());
    Ptr<MobilityModel> mobility = GetNetDevice()->GetChannel()->GetDevice(i)->GetNode()->GetObject<MobilityModel> ();
    std::cout << "Node Address: " << addr << "\n";
    std::cout << "Node Position (x,y,z): " << mobility->GetPosition() << "\n";
  }
}

// Entry point - a packet is either received from the App, or from the MAC
bool
AquaSimRoutingEfrlr::Recv(Ptr<Packet> packet, const Address &dest, uint16_t protocolNumber)
{
  NS_LOG_FUNCTION(this);

  AquaSimHeader ash;
  AquaSimAddress myAddr = AquaSimAddress::ConvertFrom(GetNetDevice()->GetAddress());
  packet->RemoveHeader(ash);

  // packet is originated at this node
  if (ash.GetNumForwards()==0)
  {
    ash.SetDirection(AquaSimHeader::DOWN);
    ash.SetNumForwards(0);
    ash.SetDAddr(AquaSimAddress::ConvertFrom(dest));
    ash.SetErrorFlag(false);
    ash.SetUId(packet->GetUid());
  }
  else  // packet is not originated at this node, i.e. received from MAC
  {
    AquaSimAddress daddr = ash.GetDAddr();
    if (daddr == myAddr)
    {
      packet->AddHeader(ash);
      DataForSink(packet);
      return true;
    }

    if (IsDeadLoop(packet))
    {
      NS_LOG_INFO("Deadloop detected. Dropping pkt.");
      return true;
    }

    // if packet is broadcast, send it to the app
    if(daddr == AquaSimAddress::GetBroadcast())
    {
      Ptr<Packet> cpkt = packet->Copy();
      cpkt->AddHeader(ash);
      DataForSink(cpkt);
    }
  }

  // select next hop
  // TODO: insert the RL logic here
  ash.SetNextHop(AquaSimAddress::GetBroadcast());
  // forward packet further
  packet->AddHeader(ash);
  ash.SetSAddr(myAddr);
  ash.SetNumForwards(ash.GetNumForwards() + 1);
  packet->AddHeader(ash);
  // send to MAC
  MACsend(packet);
  return true;
}

// Interface to send packet down to MAC
void
AquaSimRoutingEfrlr::MACsend(Ptr<Packet> pkt, Time delay)
{
  NS_LOG_FUNCTION(this);
  AquaSimHeader ash;
  pkt->PeekHeader(ash);
  Simulator::Schedule(delay, &AquaSimRouting::SendDown,this,
                        pkt,ash.GetNextHop(),Seconds(0));
}

// Interface to delivery incoming packet to the App
void
AquaSimRoutingEfrlr::DataForSink(Ptr<Packet> pkt)
{
	//  printf("DataforSink: the packet is send to demux\n");
	NS_LOG_FUNCTION(this << pkt << "Sending up to dmux.");
	if (!SendUp(pkt))
		NS_LOG_WARN("DataForSink: Something went wrong when passing packet up to dmux.");
}

void
AquaSimRoutingEfrlr::DoDispose()
{
  NS_LOG_FUNCTION(this);
  AquaSimRouting::DoDispose();
}
