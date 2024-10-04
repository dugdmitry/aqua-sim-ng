/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
*  
* Author: dmitrii <dugdmitry@gmail.com>
*/


#ifndef AQUA_SIM_ROUTING_EFRLR_H
#define AQUA_SIM_ROUTING_EFRLR_H

#include "aqua-sim-routing.h"

namespace ns3 {


/**
 * \ingroup aqua-sim-ng
 *
 * \brief EFRLR routing class.
 */
class AquaSimRoutingEfrlr : public AquaSimRouting {
 public:
  AquaSimRoutingEfrlr();
  static TypeId GetTypeId(void);
  int64_t AssignStreams (int64_t stream);

  virtual bool Recv(Ptr< Packet > packet, const Address &dest, uint16_t protocolNumber);

  void PrintGlobalTopology();

 protected:
  void DataForSink(Ptr<Packet> pkt);
  void MACsend(Ptr<Packet> pkt, Time delay=Seconds(0));
  virtual void DoDispose();

 private:
  Ptr<UniformRandomVariable> m_rand;

}; // class AquaSimRoutingEfrlr

} // namespace ns3

#endif /* AQUA_SIM_ROUTING_EFRLR_H */
