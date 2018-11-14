/*
 * mac_collision_test.cc
 *
 *  Created on: Sep 21, 2018
 *      Author: dmitry
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


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("MAC_COLLISION_TEST");

int
main (int argc, char *argv[])
{
  double simStop = 40; //seconds
  int nodes = 2;
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
  asHelper.SetRouting("ns3::AquaSimRoutingDummy");

//  asHelper.SetMac("ns3::AquaSimBroadcastMac");
  asHelper.SetMac("ns3::AquaSimMultichannelMac", "channel_hopping", BooleanValue (true));

//  asHelper.SetPhy("ns3::AquaSimPhyCmn");
  asHelper.SetPhy("ns3::AquaSimPhyMulti");

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
      newDevice->GetPhy()->AllocateSubchannels();

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
      newDevice->GetPhy()->AllocateSubchannels();

      NS_LOG_DEBUG("Node: " << *i << " newDevice: " << newDevice <<
    		  " freq:" << newDevice->GetPhy()->GetFrequency() << " addr:" <<
               AquaSimAddress::ConvertFrom(newDevice->GetAddress()).GetAsInt() );
    }

//  // Create static grid for nodes
//  MobilityHelper mobility;
//  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//								 "MinX", DoubleValue (0.0),
//								 "MinY", DoubleValue (0.0),
//								 "DeltaX", DoubleValue (distance * 2),
//								 "DeltaY", DoubleValue (0),
////								 "GridWidth", UintegerValue (nodes + sinks),
//
//								 "GridWidth", UintegerValue (nodes),
//
//								 "LayoutType", StringValue ("RowFirst"));
//  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//  mobility.Install (nodesCon);
////  mobility.Install (sinksCon);
//
//  // Create static grid for sinks
//  MobilityHelper mobility2;
//  mobility.SetPositionAllocator ("ns3::GridPositionAllocator",
//								 "MinX", DoubleValue (distance),
//								 "MinY", DoubleValue (0),
//								 "DeltaX", DoubleValue (distance),
//								 "DeltaY", DoubleValue (),
//								 "GridWidth", UintegerValue (sinks),
//								 "LayoutType", StringValue ("RowFirst"));
//  mobility.SetMobilityModel ("ns3::ConstantPositionMobilityModel");
//  mobility2.Install (sinksCon);

  MobilityHelper mobility;
  Ptr<ListPositionAllocator> positionAlloc1 = CreateObject <ListPositionAllocator>();
  positionAlloc1 ->Add(Vector(0, 0, 0));
  positionAlloc1 ->Add(Vector(distance * 2, 0, 0));
  mobility.SetPositionAllocator(positionAlloc1);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodesCon);

  MobilityHelper mobility2;
  Ptr<ListPositionAllocator> positionAlloc2 = CreateObject <ListPositionAllocator>();
  positionAlloc2 ->Add(Vector(distance + 7, 0, 0));
  mobility2.SetPositionAllocator(positionAlloc2);
  mobility2.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility2.Install(sinksCon);


  PacketSocketAddress socket;
  socket.SetAllDevices();
  // socket.SetSingleDevice (devices.Get(0)->GetIfIndex());

  socket.SetPhysicalAddress (devices.Get(nodes)->GetAddress());
  socket.SetProtocol (0);

//  // Check simultaneous transmission
//  PacketSocketAddress socket2;
//  socket2.SetAllDevices();
//  socket2.SetPhysicalAddress (devices.Get(nodes)->GetAddress());
//  socket2.SetProtocol (0);

  //std::cout << devices.Get(nodes)->GetAddress() << " &&& " << devices.Get(0)->GetIfIndex() << "\n";
  //std::cout << devices.Get(0)->GetAddress() << " &&& " << devices.Get(1)->GetIfIndex() << "\n";

  OnOffHelper app ("ns3::PacketSocketFactory", Address (socket));
  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));

  // Check simultaneous transmission
//  OnOffHelper app2 ("ns3::PacketSocketFactory", Address (socket2));
  OnOffHelper app2 ("ns3::PacketSocketFactory", Address (socket));
  app2.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=1]"));
  app2.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0]"));


//  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.1]"));
//  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.9]"));

  app.SetAttribute ("DataRate", DataRateValue (m_dataRate));
  app.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  // Check simultaneous transmission
  app2.SetAttribute ("DataRate", DataRateValue (m_dataRate));
  app2.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  ApplicationContainer apps = app.Install (nodesCon.Get(0));
  apps.Start (Seconds (0.5));
  apps.Stop (Seconds (simStop));

  // Check simultaneous transmission
  ApplicationContainer apps2 = app2.Install (nodesCon.Get(1));
  apps2.Start (Seconds (0.5));
  apps2.Stop (Seconds (simStop));

  Ptr<Node> sinkNode = sinksCon.Get(0);
  TypeId psfid = TypeId::LookupByName ("ns3::PacketSocketFactory");

//  Ptr<Socket> sinkSocket = Socket::CreateSocket (sinkNode, psfid);
//  sinkSocket->Bind (socket);
//
//  Ptr<Socket> sinkSocket2 = Socket::CreateSocket (sinkNode, psfid);
//  sinkSocket->Bind (socket2);


  Packet::EnablePrinting ();  //for debugging purposes
  std::cout << "-----------Running Simulation-----------\n";
  Simulator::Stop(Seconds(simStop));
  Simulator::Run();
  asHelper.GetChannel()->PrintCounters();
  Simulator::Destroy();

  std::cout << "fin.\n";
  return 0;
}
