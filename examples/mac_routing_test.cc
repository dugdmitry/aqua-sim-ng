/*
 * mac_routing_test.c
 *
 *  Created on: Nov 23, 2018
 *      Author: dmitry
 */


#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aqua-sim-ng-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/callback.h"


/*
 * Mac_routing_test between two nodes:
 *
 * N -------> S
 *
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Mac_routing_test");

int
main (int argc, char *argv[])
{
  double simStop = 30; //seconds
  int nodes = 1;
  int sinks = 1;
  uint32_t m_dataRate = 80000; // bps
  uint32_t m_packetSize = 50; // bytes
  double range = 150;	// meters
  double distance = 10; // meters

//  LogComponentEnable ("ASBroadcastMac", LOG_LEVEL_INFO);

  //to change on the fly
  CommandLine cmd;
  cmd.AddValue ("simStop", "Length of simulation", simStop);
  cmd.AddValue ("nodes", "Amount of regular underwater nodes", nodes);
  cmd.AddValue ("sinks", "Amount of underwater sinks", sinks);
  cmd.AddValue ("range", "Transmission range", range);
  cmd.AddValue ("distance", "Distance between nodes", distance);
  cmd.AddValue ("data_rate", "On/Off app data rate, bps", m_dataRate);
  cmd.AddValue ("packet_size", "Packet size", m_packetSize);

  cmd.Parse(argc,argv);

  std::cout << "-----------Initializing simulation-----------\n";

  NodeContainer nodesCon;
  NodeContainer sinksCon;
  nodesCon.Create(nodes);
  sinksCon.Create(sinks);

  PacketSocketHelper socketHelper;
  socketHelper.Install(nodesCon);
  socketHelper.Install(sinksCon);

  //establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  // Set propagation to RangePropagation to control the transmission range
  channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();
  asHelper.SetChannel(channel.Create());
//  asHelper.SetMac("ns3::AquaSimBroadcastMac");
//  asHelper.SetMac("ns3::AquaSimRoutingMac", "max_range", DoubleValue(range));
  asHelper.SetMac("ns3::AquaSimSFama");

  asHelper.SetRouting("ns3::AquaSimRoutingDummy");

  asHelper.SetPhy("ns3::AquaSimPhyCmn");

  /*
   * Set up mobility model for nodes and sinks
   */
  MobilityHelper mobility;
  NetDeviceContainer devices;
  Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator> ();
  Vector boundry = Vector(0,0,0);

  std::cout << "Creating Nodes\n";

  for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End(); i++)
    {
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      position->Add(boundry);
      devices.Add(asHelper.Create(*i, newDevice));

      NS_LOG_DEBUG("Node:" << newDevice->GetAddress() << " position(x):" << boundry.x);
//      std::cout << "Node:" << newDevice->GetAddress() << " position(x):" << boundry.x << "\n";
//      boundry.x += 100;
      boundry.x += distance;
      newDevice->GetPhy()->SetTransRange(range);
//      newDevice->GetPhy()->SetBandwidth(3000);
    }

  for (NodeContainer::Iterator i = sinksCon.Begin(); i != sinksCon.End(); i++)
    {
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      position->Add(boundry);
      devices.Add(asHelper.Create(*i, newDevice));

      NS_LOG_DEBUG("Sink:" << newDevice->GetAddress() << " position(x):" << boundry.x);
//      std::cout << "Sink:" << newDevice->GetAddress() << " position(x):" << boundry.x << "\n";
//      boundry.x += 100;
      boundry.x += distance;
      newDevice->GetPhy()->SetTransRange(range);
//      newDevice->GetPhy()->SetBandwidth(3000);
    }

  mobility.SetPositionAllocator(position);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodesCon);
  mobility.Install(sinksCon);

  PacketSocketAddress socket;
  socket.SetAllDevices();
  socket.SetPhysicalAddress (devices.Get(nodes)->GetAddress()); //Set dest to first sink (nodes+1 device)
  socket.SetProtocol (0);

  OnOffHelper app ("ns3::PacketSocketFactory", Address (socket));
  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.001]"));
  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.999]"));
  app.SetAttribute ("DataRate", DataRateValue (m_dataRate));
  app.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  ApplicationContainer apps = app.Install (nodesCon);
  apps.Start (Seconds (1.5));
  apps.Stop (Seconds (simStop + 1));


  Ptr<Node> sinkNode = sinksCon.Get(0);
  TypeId psfid = TypeId::LookupByName ("ns3::PacketSocketFactory");

  Ptr<Socket> sinkSocket = Socket::CreateSocket (sinkNode, psfid);
  sinkSocket->Bind (socket);

/*
 *  For channel trace driven simulation
 */
/*
  AquaSimTraceReader tReader;
  tReader.SetChannel(asHelper.GetChannel());
  if (tReader.ReadFile("channelTrace.txt")) NS_LOG_DEBUG("Trace Reader Success");
  else NS_LOG_DEBUG("Trace Reader Failure");
*/

  Packet::EnablePrinting (); //for debugging purposes
  std::cout << "-----------Running Simulation-----------\n";
  Simulator::Stop(Seconds(simStop));
  Simulator::Run();

  asHelper.GetChannel()->PrintCounters();

  Simulator::Destroy();

  std::cout << "fin.\n";
  return 0;
}
