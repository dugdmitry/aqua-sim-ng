/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Dmitrii Dugaev <ddugaev@gradcenter.cuny.edu>
 * based on aqua-sim-mac-broadcast model
 */

#ifndef AQUA_SIM_MAC_ROUTING_H
#define AQUA_SIM_MAC_ROUTING_H

#include "aqua-sim-mac.h"

namespace ns3 {

#define BC_BACKOFF  0.1//0.5 //default is 0.1 the maximum time period for backoff
#define BC_MAC_ROUTING_MAXIMUMCOUNTER 4//15 //default is 4 the maximum number of backoff
#define BC_CALLBACK_DELAY 0.0001 // the interval between two consecutive sendings

/**
 * \ingroup aqua-sim-ng
 *
 * \brief Routing MAC, based on broadcast-mac with backoff mechanism
 */
class AquaSimRoutingMac : public AquaSimMac
{
public:
  AquaSimRoutingMac();
  int64_t AssignStreams (int64_t stream);

  static TypeId GetTypeId(void);
  int m_packetHeaderSize; //# of bytes in the header
  int m_packetSize;  //to test the optimized length of packet

  // to process the incoming packet
  virtual bool RecvProcess (Ptr<Packet>);
  void CallbackProcess ();
  void DropPacket (Ptr<Packet>);

  // Forward packet / frame from upper layers / network
  bool ForwardPacket (Ptr<Packet>);

  // Send down frame using broadcast-mac backoff logic
  bool SendDownFrame (Ptr<Packet>);

  // Select next_hop_node for given destination, return its address
  AquaSimAddress SelectNextHop (AquaSimAddress dst_addr);

  // Filter duplicate broadcast messages (RREQ, RREP)
  bool FilterDuplicateRreq (AquaSimAddress src_addr);
  bool FilterDuplicateRrep (AquaSimAddress src_addr);
  bool FilterDuplicateInit (AquaSimAddress src_addr);

  // Generate and send reward packet
  double GenerateReward (AquaSimAddress dst_addr);
  void SendReward (AquaSimAddress dst_addr, double reward, Ptr<Packet> p);
  // Schedule reward send events
  void SendRewardHandler (std::map<AquaSimAddress, AquaSimAddress> dst_sender_map, double reward, Ptr<Packet> p);

  // Update path weight in forwarding table by corresponding reward value
  bool UpdateWeight(AquaSimAddress address, AquaSimAddress next_hop_addr, double reward);

  // Calculate initial weight based on received RREQ/RREP frames
  double CalculateWeight (Ptr<Packet> frame);

  // to process the outgoing packet
  virtual bool TxProcess (Ptr<Packet>);
protected:
  void BackoffHandler(Ptr<Packet>);
  virtual void DoDispose();
  // Expiration handlers
  void RewardExpirationHandler(std::map<AquaSimAddress, AquaSimAddress> dst_to_next_hop_map);
  void RrepExpirationHandler(AquaSimAddress dst_address);

private:
  int m_backoffCounter;
  Ptr<UniformRandomVariable> m_rand;

  // Forwarding table to select next-hop nodes, in a format: {dst_addr : [next_hop_addr: weight]}
  std::map<AquaSimAddress, std::map<AquaSimAddress, double>> m_forwarding_table;

  // Store the packets in send_queue until path discovery procedure is successful or failed, in a format:
  // {dst_addr: packet_queue}
  std::map<AquaSimAddress, std::queue<Ptr<Packet>>> m_send_queue;

  // Store reward expiration timestamps, needed when the expire event is triggered by the scheduler,
  // or when the reward message is received,
  // in a format: {{dst_addr : next_hop_addr} : last_recevied_reward_timestamp}
  std::map<std::map<AquaSimAddress, AquaSimAddress>, Time> m_reward_expirations;

  // Store RREP expiration timestamps, needed when the expire event is triggered by the scheduler,
  // or when the RREP message is received,
  // in a format: {dst_addr: last_recevied_rrep_timestamp}
  std::map<AquaSimAddress, Time> m_rrep_expirations;

  // Store INIT expiration timestamps, needed when the expire event is triggered by the scheduler,
  // or when the INIT message is received,
  // in a format: {dst_addr: last_recevied_init_timestamp}
  std::map<AquaSimAddress, Time> m_init_expirations;

  // Store list of already received RREQs, with the last received timestamp to handle duplicate frames
  std::map<AquaSimAddress, Time> m_rreq_list;

  // Store list of already received RREPs, with the last received timestamp to handle duplicate frames
  std::map<AquaSimAddress, Time> m_rrep_list;

  // Store list of already received INITs, with the last received timestamp to handle duplicate frames
  std::map<AquaSimAddress, Time> m_init_list;

  // Store list of {dst_addr, sender_addr} pairs to periodically generate the reward packets back to sender
  std::map<std::map<AquaSimAddress, AquaSimAddress>, Time> m_reward_delays;

  // ACK / Reward delay on receiver side
  Time m_reward_delay = Seconds(1);
  // Reward wait timeout, in seconds
  Time m_reward_timeout = Seconds(30) + m_reward_delay;

  // RREP wait timeout, in seconds
  Time m_rrep_timeout = Seconds(100);

  // INIT wait timeout, in seconds
  Time m_init_timeout = Seconds(10);

  // Default negative reward value
  double m_negative_reward = -1;

  // Maximum hop-count for packets (to avoid loops)
  int m_max_hop_count = 10;

};  // class AquaSimRoutingMac

} // namespace ns3

#endif /* AQUA_SIM_MAC_ROUTING_H */
