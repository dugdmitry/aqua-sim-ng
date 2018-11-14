/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Connecticut
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
 * Author: Robert Martin <robert.martin@engr.uconn.edu>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/v4ping-helper.h"

#include "ns3/mobility-module.h"
#include "ns3/energy-module.h"  //may not be needed here...
#include "ns3/aqua-sim-ng-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/callback.h"


/*
 * VBF Test
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("VBF");

int
main (int argc, char *argv[])
{
  double simStop = 10; //seconds
  int nodes = 1;
  int sinks = 1;
  uint32_t m_dataRate = 80000;
//  uint32_t m_dataRate = 10000;
  uint32_t m_packetSize = 22;
//  uint32_t m_packetSize = 40;
  double range = 100;
  double distance = 10;
  //int m_maxBurst =10;
  // Define channel ID
  int channelId = 1;

//  LogComponentEnable ("VBF", LOG_LEVEL_INFO);

  //to change on the fly
  CommandLine cmd;
  cmd.AddValue ("simStop", "Length of simulation", simStop);
  cmd.AddValue ("nodes", "Amount of regular underwater nodes", nodes);
  cmd.AddValue ("sinks", "Amount of underwater sinks", sinks);
  cmd.AddValue ("packetSize", "Size of packet", m_packetSize);
  cmd.AddValue ("channelId", "Channel ID", channelId);
  cmd.Parse(argc,argv);

  std::cout << "-----------Initializing simulation-----------\n";

  std::cout << "Packet size: " << m_packetSize << "\n";
  std::cout << "Channel ID: " <<   channelId << "\n";

  NodeContainer nodesCon;
  NodeContainer sinksCon;

  nodesCon.Create(nodes);
  sinksCon.Create(sinks);

  PacketSocketHelper socketHelper;
  socketHelper.Install(nodesCon);
  socketHelper.Install(sinksCon);

  //establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();
  //AquaSimEnergyHelper energy;	//******this could instead be handled by node helper. ****/
  asHelper.SetChannel(channel.Create());
//  asHelper.SetMac("ns3::AquaSimBroadcastMac");
//  asHelper.SetRouting("ns3::AquaSimVBF");
  asHelper.SetRouting("ns3::AquaSimRoutingDummy");
//  asHelper.SetMac("ns3::AquaSimMultichannelMac", "ChannelId", IntegerValue (channelId));
  asHelper.SetMac("ns3::AquaSimMultichannelMac", "channel_hopping", BooleanValue (true));
//  asHelper.SetMac("ns3::AquaSimMultichannelMac", "random_channel_hopping", BooleanValue (true));
  asHelper.SetPhy("ns3::AquaSimPhyCmn");
//  asHelper.SetPhy("ns3::AquaSimPhyCmn", "Bandwidth", UintegerValue (30000));

//    asHelper.SetPhy("ns3::AquaSimPhyMultichannel");
//  asHelper.SetRouting("ns3::AquaSimStaticRouting");
//  asHelper.SetRouting("ns3::AquaSimStaticRouting", "fileName", char("/home/dmitry/ns-aqua-sim-3.27/ns-3.27/route_table.txt"));
//  AquaSimStaticRouting static_routing;
//  static_routing.SetRouteTable((char*)"/home/dmitry/ns-aqua-sim-3.27/ns-3.27/route_table.txt");

  NetDeviceContainer devices;

//  std::cout << "Creating Nodes\n";
  for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End(); i++)
    {
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      devices.Add(asHelper.Create(*i, newDevice));
      newDevice->GetPhy()->SetTransRange(range);

      NS_LOG_DEBUG("Node: " << *i << " newDevice: " << newDevice <<
    		  " freq:" << newDevice->GetPhy()->GetFrequency() << " addr:" <<
               AquaSimAddress::ConvertFrom(newDevice->GetAddress()).GetAsInt() );
    }

//  std::cout << "Creating Sinks\n";
  for (NodeContainer::Iterator i = sinksCon.Begin(); i != sinksCon.End(); i++)
    {
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      devices.Add(asHelper.Create(*i, newDevice));
      newDevice->GetPhy()->SetTransRange(range);

      NS_LOG_DEBUG("Node: " << *i << " newDevice: " << newDevice <<
    		  " freq:" << newDevice->GetPhy()->GetFrequency() << " addr:" <<
               AquaSimAddress::ConvertFrom(newDevice->GetAddress()).GetAsInt() );
    }

  // Create static grid
  MobilityHelper mobility;
  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
								 "MinX", DoubleValue (0.0),
								 "MinY", DoubleValue (0.0),
								 "DeltaX", DoubleValue (distance),
								 "DeltaY", DoubleValue (0),
								 "GridWidth", UintegerValue (nodes + sinks),
								 "LayoutType", StringValue ("RowFirst"));
  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
  mobility.Install (nodesCon);
  mobility.Install (sinksCon);

  PacketSocketAddress socket;
  socket.SetAllDevices();
  // socket.SetSingleDevice (devices.Get(0)->GetIfIndex());
  socket.SetPhysicalAddress (devices.Get(nodes)->GetAddress());
  socket.SetProtocol (0);

  //std::cout << devices.Get(nodes)->GetAddress() << " &&& " << devices.Get(0)->GetIfIndex() << "\n";
  //std::cout << devices.Get(0)->GetAddress() << " &&& " << devices.Get(1)->GetIfIndex() << "\n";

  OnOffHelper app ("ns3::PacketSocketFactory", Address (socket));
  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1.0]"));
  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0]"));
//  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.0066]"));
//  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.9934]"));
//  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.026]"));
//  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.974]"));
  app.SetAttribute ("DataRate", DataRateValue (m_dataRate));
  app.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  ApplicationContainer apps = app.Install (nodesCon);
  apps.Start (Seconds (0.5));
  apps.Stop (Seconds (simStop));

  Ptr<Node> sinkNode = sinksCon.Get(0);
  TypeId psfid = TypeId::LookupByName ("ns3::PacketSocketFactory");

  Ptr<Socket> sinkSocket = Socket::CreateSocket (sinkNode, psfid);
  sinkSocket->Bind (socket);


  Packet::EnablePrinting ();  //for debugging purposes
  std::cout << "-----------Running Simulation-----------\n";
  Simulator::Stop(Seconds(simStop));
  Simulator::Run();
  asHelper.GetChannel()->PrintCounters();
  Simulator::Destroy();

  std::cout << "fin.\n";
  return 0;
}
