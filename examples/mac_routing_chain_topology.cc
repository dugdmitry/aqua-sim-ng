/*
 * mac_routing_test.c
 *
 *  Created on: Nov 24, 2018
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
 * Mac_routing chain topology test between two edge nodes:
 *
 * S<---><--->N1<---><--->N2<---><--->Nn<---><--->D
 *
 * <---> - transmission range
 *
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("Mac_routing_chain_topology");

int
main (int argc, char *argv[])
{
  double simStop = 100; //seconds
  int intermediate_nodes = 1;

  double m_dataRate = 80000; // bps
  double m_packetSize = 50; // bytes
  double range = 150;	// meters
  double distance = 150; // meters

  // Poisson traffic parameters
  double lambda = 0.1;

  // Max Tx power
  double max_tx_power = 20; // Watts

//  LogComponentEnable ("ASBroadcastMac", LOG_LEVEL_INFO);

  //to change on the fly
  CommandLine cmd;
  cmd.AddValue ("simStop", "Length of simulation", simStop);
  cmd.AddValue ("lambda", "Packet arrival rate", lambda);
  cmd.AddValue ("packet_size", "Packet size", m_packetSize);
  cmd.AddValue ("range", "Transmission range", range);
  cmd.AddValue ("distance", "Distance between nodes", distance);
  cmd.AddValue ("nodes", "Number of intermediate nodes", intermediate_nodes);
  cmd.AddValue ("tx_power", "Max transmission power", max_tx_power);

  cmd.Parse(argc,argv);

  // Total number of nodes, including source and destination
  int nodes = intermediate_nodes + 2;


  std::cout << "-----------Initializing simulation-----------\n";

  NodeContainer nodesCon;
  nodesCon.Create(nodes);

  PacketSocketHelper socketHelper;
  socketHelper.Install(nodesCon);

  //establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  // Set propagation to RangePropagation to control the transmission range
  channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();
  asHelper.SetChannel(channel.Create());
//  asHelper.SetMac("ns3::AquaSimBroadcastMac");

  asHelper.SetMac("ns3::AquaSimRoutingMacAloha", "max_range", DoubleValue(range), "max_tx_power", DoubleValue(max_tx_power),
		  "packet_size", IntegerValue(m_packetSize));

  asHelper.SetRouting("ns3::AquaSimRoutingDummy");

  asHelper.SetPhy("ns3::AquaSimPhyCmn", "PT", DoubleValue(max_tx_power));

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

//      NS_LOG_DEBUG("Node:" << newDevice->GetAddress() << " position(x):" << boundry.x);
      std::cout << "Node:" << newDevice->GetAddress() << " position(x):" << boundry.x << "\n";
//      boundry.x += 100;
      boundry.x += distance;
      newDevice->GetPhy()->SetTransRange(range);
//      newDevice->GetPhy()->SetBandwidth(3000);
    }

  mobility.SetPositionAllocator(position);
  mobility.SetMobilityModel("ns3::ConstantPositionMobilityModel");
  mobility.Install(nodesCon);

  PacketSocketAddress socket;
  socket.SetAllDevices();
  socket.SetPhysicalAddress (devices.Get(nodes-1)->GetAddress()); // Destination node is the last node in the container
  socket.SetProtocol (0);

  OnOffHelper app ("ns3::PacketSocketFactory", Address (socket));

  char duration_on[300];
  char duration_off[300];

  sprintf(duration_on, "ns3::ExponentialRandomVariable[Mean=%f]", (m_packetSize * 8) / m_dataRate);
  sprintf(duration_off, "ns3::ExponentialRandomVariable[Mean=%f]", 1 / lambda);

//  app.SetAttribute ("OnTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.001]"));
//  app.SetAttribute ("OffTime", StringValue ("ns3::ConstantRandomVariable[Constant=0.999]"));
  app.SetAttribute ("OnTime", StringValue (duration_on));
  app.SetAttribute ("OffTime", StringValue (duration_off));

  app.SetAttribute ("DataRate", DataRateValue (m_dataRate));
  app.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  ApplicationContainer apps = app.Install (nodesCon.Get(0)); // Source node is the first one in the container
  apps.Start (Seconds (0.5));
  apps.Stop (Seconds (simStop + 1));

//  Ptr<Node> dest_node = nodesCon.Get(nodes + 1);
//  TypeId psfid = TypeId::LookupByName ("ns3::PacketSocketFactory");
//
//  Ptr<Socket> sinkSocket = Socket::CreateSocket (dest_node, psfid);
//  sinkSocket->Bind (socket);

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

  // Enable ASCII traces
  std::string asciiTraceFile = "xmac-trace-chain.asc";
  std::ofstream ascii (asciiTraceFile.c_str());
  if (!ascii.is_open()) {
    NS_FATAL_ERROR("Could not open trace file.");
  }
  asHelper.EnableAsciiAll(ascii);


  Simulator::Run();

  asHelper.GetChannel()->PrintCounters();

  Simulator::Destroy();

  std::cout << "fin.\n";
  return 0;
}
