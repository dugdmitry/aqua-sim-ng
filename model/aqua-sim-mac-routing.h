/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Author: Dmitrii Dugaev <ddugaev@gradcenter.cuny.edu>
 * based on aqua-sim-mac-broadcast model
 */

#ifndef AQUA_SIM_MAC_ROUTING_H
#define AQUA_SIM_MAC_ROUTING_H

#include "aqua-sim-mac.h"

namespace ns3 {

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

  // Forward packet / frame from upper layers / network, keep track of the sender node
  bool ForwardPacket (Ptr<Packet>, AquaSimAddress sender_addr, int hop_count, double reward);

  // Send down frame using broadcast-mac backoff logic
  bool SendDownFrame (Ptr<Packet>);

  // Select next_hop_node for given destination, return its address
  AquaSimAddress SelectNextHop (AquaSimAddress dst_addr);

  // Filter duplicate broadcast messages (RREQ, RREP)
  bool FilterDuplicateInit (AquaSimAddress src_addr);
  bool FilterDuplicateRts (AquaSimAddress src_addr);
  bool FilterDuplicateCts (AquaSimAddress src_addr);

  // Generate reward packet
  double GenerateReward (AquaSimAddress dst_addr);

  // Update path weight in forwarding table by corresponding reward value
  bool UpdateWeight(AquaSimAddress address, AquaSimAddress next_hop_addr, double reward);

  // Calculate weights based on given distances
  double CalculateWeight (double distance);

  // Calculate and update distance list
  void UpdateDistance (double tx_power, double rx, AquaSimAddress dst_addr);

  // Calculate the distance between the src and dst nodes, based on Tx and Rx powers
  // For that, a very rough approximation model of Rayleigh is used, if frequency = 25 kHz!
  double CalculateDistance (double tx_power, double rx_power);

  // Calculate Tx power given the distance and expected Rx_threshold
  // The calculation is based on Rayleight model, used in the aqua-sim-propagation module:
  // Rx = Tx / (d^k * alpha^(d/1000)), k = 2, alpha = 4, (f = 25kHz)
  double CalculateTxPower (double distance);

  // Calculate optimal metric based on the given distance between src and dst
  double CalculateOptimalMetric (double distance);

  // to process the outgoing packet
  virtual bool TxProcess (Ptr<Packet>);

  // RTS / CTS states logic
  enum MacRoutingStatus {
      CTS_WAIT,
	  RTS_TO,
	  CTS_TO,
	  DATA_TX,
	  DATA_RX,
	  IDLE
    };

protected:
  virtual void DoDispose();
  // Expiration handlers
  void RewardExpirationHandler(std::map<AquaSimAddress, AquaSimAddress> dst_to_next_hop_map);
  void InitExpirationHandler(AquaSimAddress dst_address);
  void RtsExpirationHandler(AquaSimAddress dst_address);
  void CtsExpirationHandler(AquaSimAddress dst_address);

  // RTS / CTS state timeout handler
  void StateTimeoutHandler (Time timeout);
  // Set current state to IDLE
  void SetToIdle ();

private:
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

  // Store INIT expiration timestamps, needed when the expire event is triggered by the scheduler,
  // or when the INIT message is received,
  // in a format: {dst_addr: last_recevied_init_timestamp}
  std::map<AquaSimAddress, Time> m_init_expirations;

  // Store RTS expiration timestamps, needed when the expire event is triggered by the scheduler,
  // or when the RTS message is received,
  // in a format: {dst_addr: last_recevied_rts_timestamp}
  std::map<AquaSimAddress, Time> m_rts_expirations;

  // Store CTS expiration timestamps, needed when the expire event is triggered by the scheduler,
  // or when the CTS message is received,
  // in a format: {dst_addr: last_recevied_cts_timestamp}
  std::map<AquaSimAddress, Time> m_cts_expirations;

  // Store list of already received INITs, with the last received timestamp to handle duplicate frames
  std::map<AquaSimAddress, Time> m_init_list;

  // Store list of already received RTSs, with the last received timestamp to handle duplicate frames
  std::map<AquaSimAddress, Time> m_rts_list;

  // Store list of already received CTSs, with the last received timestamp to handle duplicate frames
  std::map<AquaSimAddress, Time> m_cts_list;

  // Store list of {dst_addr, sender_addr} pairs to periodically generate the reward packets back to sender
  std::map<std::map<AquaSimAddress, AquaSimAddress>, Time> m_reward_delays;

  // ACK / Reward delay on receiver side
  Time m_reward_delay = Seconds(1);
  // Reward wait timeout, in seconds
  Time m_reward_timeout = Seconds(30) + m_reward_delay;

  // INIT wait timeout, in seconds
  Time m_init_timeout = Seconds(10);

  // RTS wait timeout, in seconds
  Time m_rts_timeout = Seconds(2);

  // CTS wait timeout, in seconds
  Time m_cts_timeout = Seconds(2);

  // Store the distances to each destination
  std::map<AquaSimAddress, double> m_distances;

  // Default negative reward value
  double m_negative_reward = -1;

  // Maximum hop-count for packets (to avoid loops)
  int m_max_hop_count = 10;

  // Optimal 1-hop distance (optimal metric)
  double m_optimal_metric = 0;

  // Max transmission range
  double m_max_range = 150; // meters

  // Expected Rx_threshold, in Watts
  double m_rx_threshold = 0.0005;

  // Max tx_power
  double m_max_tx_power = 20; // Watts

  // Store packets to be sent
  std::queue<Ptr<Packet>> m_send_buffer;

  // Current RTS/CTS state
  MacRoutingStatus m_status;

  // Current state timeout
  Time m_state_timeout = Seconds(0);

  // Hop count threshold - send the DATA packet directly to the destination, if the hop count value exceeds the threshold
  int m_hop_count_threshold = 3;

  // Minimum possible reward
  double m_min_reward = 0.001;

  // Data transmission time interval
  // Time duration of the DATA_TX / RX states before transition to IDLE
  // I.e., how much data should be transmitted wihtin a single RTS/CTS handshake
  Time m_data_timeout = Seconds(10);

  // Store a list of destination address, which are wihtin maximum transmission range
  std::vector<AquaSimAddress> m_inrange_addresses;

};  // class AquaSimRoutingMac

} // namespace ns3

#endif /* AQUA_SIM_MAC_ROUTING_H */
