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

#ifndef AQUA_SIM_MAC_JAMMING_H
#define AQUA_SIM_MAC_JAMMING_H

#include "aqua-sim-mac.h"

namespace ns3 {

/**
 * \ingroup aqua-sim-ng
 *
 * \brief Jamming MAC using basic backoff mechanism
 */
enum JammingMacScheduleType{
    TRAIN_SCHEDULE,    // schedule packet transmissions "side-by-side" so they will arrive to a sink as a "train" of packets
    RANDOM_SCHEDULE,   // randomly allocate packet transmissions
    AUCTION_SCHEDULE   // use Auction algorithm from the paper
};

class AquaSimJammingMac : public AquaSimMac
{
public:
  AquaSimJammingMac();
  int64_t AssignStreams (int64_t stream);

  static TypeId GetTypeId(void);

  // Handle a packet coming from channel
  virtual bool RecvProcess (Ptr<Packet>);
  // Handle a packet coming from upper layers
  virtual bool TxProcess (Ptr<Packet>);

  // Start Channel-Competition request
  void StartCC();
  // Send a packet using pure ALOHA channel access
  void SendPacketAloha(Ptr<Packet> p);
  // Return a backoff value for ALOHA
  double GetBackoff();
  // Assemble all received cc-requests and send a cs-reply
  void TriggerCsReply();
  // Process incoming CC-request - obtain node info and store it
  void ProcessCcRequest(uint32_t node_id, Vector coords);
  // Create a schedule based on the received requests
  // returns: <node_id>:<tx_delay_ms>
  std::map<uint32_t, uint16_t> CreateSchedule(JammingMacScheduleType schedule_type);
  // Return a cs-reply packet based on a given schedule
  Ptr<Packet> GenerateCsReply(std::map<uint32_t, uint16_t> schedule);

protected:
  virtual void DoDispose();

private:
  Ptr<UniformRandomVariable> m_rand;
  uint16_t m_packetSize;
  std::list<Ptr<Packet>> m_sendQueue;
  // Define timer to accumulate all the incoming CC-requests when it runs
  Timer m_sink_timer;
  // Accumulation time of the incoming cc-requests
  Time m_cc_accumulation_time;
  // Interval between cc-requests at the sender side (nodes)
  Time m_epoch_time;
  // Store the incoming cc-requests in a map: <node_id>:<coords>
  std::map<uint32_t, Vector> m_request_list;
  // Guard interval in-between packets in a train
  Time m_guard_time;
  // Track the very first cc-procedure
  bool m_firstCcInit = false;
  // Keep track of a number of cc-->cs-->data phases
  uint32_t m_epoch_no = 0;

};  // class AquaSimJammingMac

} // namespace ns3

#endif /* AQUA_SIM_MAC_JAMMING_H */
