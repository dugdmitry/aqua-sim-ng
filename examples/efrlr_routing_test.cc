/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Environmental Friendly Reinforcement Learning based Routing (EFRLR) protocol tests
 *
 * Author: dmitrii <dugdmitry@gmail.com>
 */

#include "ns3/core-module.h"
#include "ns3/network-module.h"
#include "ns3/mobility-module.h"
#include "ns3/aqua-sim-ng-module.h"
#include "ns3/applications-module.h"
#include "ns3/log.h"
#include "ns3/callback.h"

#include <iomanip>

/*
 * EFRLR simulation test
 *
 */

using namespace ns3;

NS_LOG_COMPONENT_DEFINE("EfrlrRoutingTest");

// Return random coordinates in a circle with given radius and center, at given depth (y-coord)
Vector
getBottomCoords(double center_x, double center_z, double radius, double depth, Ptr<UniformRandomVariable> random_stream)
{
  Vector boundry = Vector(0,0,0);
  double x,z;
  do
  {
    x = random_stream->GetValue (0, 2*radius);
    z = random_stream->GetValue (0, 2*radius);
  }
  while (std::sqrt (x*x + z*z) > radius);

  boundry.x = x+center_x;
  boundry.z = z+center_z;
  boundry.y = depth; // the nodes are located at the bottom
  return boundry;
}

// tracebacks for the stats
uint32_t totalOrigPkts = 0;
uint32_t totalRecvDataPkts = 0;
uint32_t totalPhyTxPkts = 0;
uint32_t totalPhyRxPkts = 0;

void
traceRoutingRx(Ptr<const Packet> packet)
{
  totalRecvDataPkts += 1;
}

void
traceOrigPkts(Ptr<const Packet> packet)
{
  totalOrigPkts += 1;
}

void
tracePhyTx(Ptr<Packet> packet, double noise)
{
  totalPhyTxPkts += 1;
}

void
tracePhyRx(Ptr<Packet> packet, double noise)
{
  totalPhyRxPkts += 1;
}

int
main (int argc, char *argv[])
{
  double simStop = 1800; //seconds
  uint32_t seed_no;
  double lambda = 0.1;  // Poisson traffic, pkts/sec
  int nodes = 2;
  int sinks = 1;
  double m_dataRate = 24;
  uint32_t m_packetSize = 100;
  // location params, meters
  double radius = 100;
  double center_x = 100;
  double center_z = 100;
  double depth = 100;
  Time epochTime = Seconds(10);

  LogComponentEnable ("EfrlrRoutingTest", LOG_LEVEL_INFO);

  //to change on the fly
  CommandLine cmd;
  cmd.AddValue ("seed", "Seed for random generation", seed_no);
  cmd.AddValue ("simStop", "Length of simulation", simStop);
  cmd.AddValue ("lambda", "Packet arrival rate", lambda);
  cmd.AddValue ("nodes", "Amount of regular underwater nodes", nodes);
  cmd.AddValue ("sinks", "Amount of underwater sinks", sinks);
  cmd.AddValue ("psize", "Data packet size, bytes", m_packetSize);
  cmd.AddValue ("rate", "Data rate for CBR, bps", m_dataRate);
  cmd.AddValue ("radius", "Radius of random nodes at bottom, meters", radius);
  cmd.AddValue ("center_x", "Center of a circular bottom: x-coordinate, meters", center_x);
  cmd.AddValue ("center_z", "Center of a circular bottom: z-coordinate, meters", center_z);
  cmd.AddValue ("depth", "Depth of the bottom, meters", depth);
  cmd.Parse(argc,argv);

  std::cout << "-----------Initializing simulation-----------\n";

  // Initialize pseudo-random generator
  SeedManager::SetSeed (12345);
  SeedManager::SetRun (seed_no);
  Ptr<UniformRandomVariable> random_stream = CreateObject<UniformRandomVariable> ();

  NodeContainer nodesCon;
  NodeContainer sinksCon;
  nodesCon.Create(nodes);
  sinksCon.Create(sinks);

  PacketSocketHelper socketHelper;
  socketHelper.Install(nodesCon);
  socketHelper.Install(sinksCon);

  //establish layers using helper's pre-build settings
  AquaSimChannelHelper channel = AquaSimChannelHelper::Default();
  //channel.SetPropagation("ns3::AquaSimRangePropagation");
  AquaSimHelper asHelper = AquaSimHelper::Default();
  asHelper.SetChannel(channel.Create());

  // set mac and routing layers
  asHelper.SetMac("ns3::AquaSimAloha", "AckOn", IntegerValue(0));
  asHelper.SetRouting("ns3::AquaSimRoutingEfrlr");

  /*
   * Set up mobility model for nodes and sinks
   */
  MobilityHelper mobility;
  NetDeviceContainer devices;
  Ptr<ListPositionAllocator> position = CreateObject<ListPositionAllocator> ();
  Vector boundry = Vector(0,0,0);

  std::cout << "Creating Nodes\n";

  // Place nodes at the "bottom" - a random circle in (x,z)-plane; y - depth of the "bottom"
  for (NodeContainer::Iterator i = nodesCon.Begin(); i != nodesCon.End(); i++)
    {
      position->Add(getBottomCoords(center_x, center_z, radius, depth, random_stream));
      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      devices.Add(asHelper.Create(*i, newDevice));
      //newDevice->GetPhy()->SetTransRange(range);
    }

  // Place all sinks at the center of the circle, at 0-meter depth (y=0)
  // TODO: place multiple sinks at different positions
  for (NodeContainer::Iterator i = sinksCon.Begin(); i != sinksCon.End(); i++)
    {
      boundry.x = center_x;
      boundry.z = center_z;
      boundry.y = 0; // sink is located at the surface
      position->Add(boundry);

      Ptr<AquaSimNetDevice> newDevice = CreateObject<AquaSimNetDevice>();
      devices.Add(asHelper.Create(*i, newDevice));
      //newDevice->GetPhy()->SetTransRange(range);
    }

  mobility.SetPositionAllocator(position);
  mobility.Install(nodesCon);
  mobility.Install(sinksCon);

  // Print-debug node positions
  for (uint32_t i = 0; i < nodesCon.GetN(); i++)
  {    
    Ptr<MobilityModel> mob = nodesCon.Get(i)->GetObject<MobilityModel>();
    NS_LOG_DEBUG("Node " << i << " (x,y,z)-position: (" << mob->GetPosition().x << 
                  ",\t" << mob->GetPosition().y << ",\t" << mob->GetPosition().z << ")");
  }
  for (uint32_t i = 0; i < sinksCon.GetN(); i++)
  {    
    Ptr<MobilityModel> mob = sinksCon.Get(i)->GetObject<MobilityModel>();
    NS_LOG_DEBUG("Sink " << i << " (x,y,z)-position: (" << mob->GetPosition().x << 
                  ",\t" << mob->GetPosition().y << ",\t" << mob->GetPosition().z << ")");
  }

  // Application and sockets
  PacketSocketAddress socket;
  socket.SetAllDevices();
  socket.SetPhysicalAddress (devices.Get(nodes)->GetAddress()); //Set dest to first sink (nodes+1 device)
  socket.SetProtocol (0);

  OnOffHelper app ("ns3::PacketSocketFactory", Address (socket));

  char duration_on[300];
  char duration_off[300];

  sprintf(duration_on, "ns3::ExponentialRandomVariable[Mean=%f]", (m_packetSize * 8) / m_dataRate);
  sprintf(duration_off, "ns3::ExponentialRandomVariable[Mean=%f]", 1 / lambda);

  app.SetAttribute ("OnTime", StringValue (duration_on));
  app.SetAttribute ("OffTime", StringValue (duration_off));

  app.SetAttribute ("DataRate", DataRateValue (m_dataRate));
  app.SetAttribute ("PacketSize", UintegerValue (m_packetSize));

  ApplicationContainer apps = app.Install (nodesCon);
  apps.Start (Seconds (0.5));
  apps.Stop (Seconds (simStop + 1));

  Ptr<Node> sinkNode = sinksCon.Get(0);
  TypeId psfid = TypeId::LookupByName ("ns3::PacketSocketFactory");

  Ptr<Socket> sinkSocket = Socket::CreateSocket (sinkNode, psfid);
  sinkSocket->Bind (socket);

  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NetDevice/Mac/RoutingRx", MakeCallback (&traceRoutingRx));
  Config::ConnectWithoutContext ("/NodeList/*/ApplicationList/*/$ns3::Application/Tx", MakeCallback (&traceOrigPkts));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NetDevice/Phy/Tx", MakeCallback (&tracePhyTx));
  Config::ConnectWithoutContext ("/NodeList/*/DeviceList/*/$ns3::NetDevice/Phy/Rx", MakeCallback (&tracePhyRx));

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

  // // Enable ASCII trace files
  // Packet::EnablePrinting ();  //for debugging purposes
  // char buff[1000];
  // // Naming convention: lambda-number_of_nodes-n_intermediate_nodes-seed
  // std::stringstream stream;
  // stream << std::fixed << std::setprecision(2) << lambda;
  // std::string lambda_string = stream.str();
  // snprintf(buff, sizeof(buff), "efrlr-trace-%s-%d.asc", lambda_string.c_str(), (nodes + sinks));
  // std::string asciiTraceFile = buff;
  // // asciiTraceFile.
  // std::ofstream ascii (asciiTraceFile.c_str());
  // if (!ascii.is_open()) {
  //   NS_FATAL_ERROR("Could not open trace file.");
  // }
  // asHelper.EnableAsciiAll(ascii);

  Simulator::Run();
  asHelper.GetChannel()->PrintCounters();
  Simulator::Destroy();

  // save the trace files
  std::ofstream results ("efrlr_results.txt", std::ofstream::app);
  results << totalOrigPkts << "\t"
  << totalRecvDataPkts << "\t"
  << totalPhyTxPkts << "\t"
  << totalPhyRxPkts << "\t"
  << "\n";
  //

  std::cout << "fin.\n";
  return 0;
}
