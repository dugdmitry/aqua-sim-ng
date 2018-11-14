/*
 * aqua-sim-mac-multichannel.h
 *
 *  Created on: Jun 21, 2018
 *      Author: dmitry
 */

#ifndef SRC_AQUA_SIM_NG_MODEL_AQUA_SIM_MAC_MULTICHANNEL_H_
#define SRC_AQUA_SIM_NG_MODEL_AQUA_SIM_MAC_MULTICHANNEL_H_

#include "aqua-sim-mac.h"

#include <random>


namespace ns3 {


#define BC_BACKOFF  0.1//0.5 //default is 0.1 the maximum time period for backoff
#define BC_MAXIMUMCOUNTER 4//15 //default is 4 the maximum number of backoff
#define BC_CALLBACK_DELAY 0.0001 // the interval between two consecutive sendings


//class MultichannelMacCalendar: public Object
//{
//public:
////	MultichannelMacCalendar();
////    virtual ~MultichannelMacCalendar();
//   // ~MultichannelMacCalendar();
//
//    static TypeId GetTypeId(void);
//
//	double m_current_ts = 0;
//	int m_current_slot = 0;
//	int m_last_slot = 0;
//
//	double m_slot_duration = 0;
//
//	enum CalendarState {TX, RX};
//	// Calendar map: <slot_number : state (Rx/Tx)>
//	std::map<int, CalendarState> m_calendar_map;
//
//	// Get current slot number by timestamp
//	int GetSlotByTimestamp (Time ts);
//
//	// Get a timeslot, corresponding to a beginning of the given slot number
//	double GetTsBySlot (int slot_number);
//
//	// Add event to calendar
//	void AddEvent (int slot_number, CalendarState state);
//
//	int node_address = 0;
//
//};


class AquaSimMultichannelMac: public AquaSimMac {


public:
	AquaSimMultichannelMac();
	virtual ~AquaSimMultichannelMac();

	static TypeId GetTypeId(void);
	int64_t AssignStreams (int64_t stream);

	// to process the incoming packet
	virtual bool RecvProcess (Ptr<Packet>);

	// to process the outgoing packet
	virtual bool TxProcess (Ptr<Packet>);

	bool SendDown(Ptr<Packet> p, int channelId, TransStatus afterTrans = NIDLE);

	int m_packetSize;
	int m_packetHeaderSize;

	Ptr<Packet> GenerateRts (AquaSimAddress dst_address);
	Ptr<Packet> GenerateCts (int slot_number, AquaSimAddress dst_address);
	// Add MMAC header to the data packet coming from the app
	Ptr<Packet> CreateDataFrame (Ptr<Packet>, AquaSimAddress dst_address);

	void AllocateCalendars ();

	double m_current_ts = 0;
	double m_current_slot = 0;
	double m_last_slot = -1;

	enum CalendarState {RTS_TX, CTS_TX, DATA_TX, RTS_RX, CTS_RX, DATA_RX, IDLE};
	// Calendar map: <slot_number : state (Rx/Tx)>
	std::map<int, CalendarState> m_calendar_map;

	// Set current slot number by timestamp
	void SetCurrentTimeslot ();

	// Get a timeslot, corresponding to a beginning of the given slot number
	double GetTsBySlot (int slot_number);

//	// Add Tx event to calendar
//	void AddTxEvent ();
//
//	// Add Rx event to calendar
//	void AddRxEvent ();

	void AddEvent (CalendarState);
	// Add event by given slot number
	void AddEventBySlot (CalendarState, int slot_number);

	int node_address = 0;

	// Get current channel ID, based on the current time slot number and node_id
	int GetChannelId (int slot_number, int node_addr);

	// Get channel ID by central frequency
	int GetChannelIdByFreq (double freq);

	// Check if a current time slot is idle
	bool SlotIsIdle (int slot_number);

	// Filter received frame based on current timeslot and corresponding channel_id / centrar_frequency
	bool FilterFrame (double central_freq);

	// Separate method for sending down RTS. This is needed to schedule the RTS sending event, so that the actual RTS frame
	// will contain an up-to-date information about the IDLE time slots in the calendar, right before being sent down.
	bool SendDownRts (int channel_id, AquaSimAddress dst_address);

	// Flag for rescheduling RTS transmission in certain cases (e.g., when a CTS comes at the same time slot)
	bool m_reschedule_rts = false;

	// Get event type from the calendar
	CalendarState GetEventType (int slot_number);

	// Execute CTS wait expiration event
	void TriggerCtsExpiration (double cts_number);

	// Get packet from queue
	Ptr<Packet> GetPacketFromQueue ();

protected:
	void BackoffHandler(Ptr<Packet>);
	virtual void DoDispose();

private:
	int m_backoffCounter;
	Ptr<UniformRandomVariable> m_rand;
	// Store all available channel numbers, in kHz
	std::vector<int> m_subchannels;
	double m_nsubchannels;
	int m_current_channel = 0;

	// Map channel ID with the corresponding frequency, in kHz
	std::map<int, double> m_channel_freq_map;

	// Map central freq with channel id (to get the channel_id by given frequency)
	std::map<double, int> m_freq_channel_map;

	// Frequency step between the subchannels
	double m_freq_step;
	// Lower frequency bound
	double m_lower_freq_bound;
	// Higher frequency bound
	double m_higher_freq_bound;
	// Total channel bandwidth
	double m_total_bandwidth;

	// Flag for enabling per-packet channel hopping
	bool m_channel_hopping = false;
	// Flag for enabling random channel selection per incoming packet
	bool m_random_channel_hopping = false;
	// Count incoming packets in order to distribute them on different channels, according to packet hopping scheme
	double m_incoming_packet_counter = 0;

	int m_counter = 0;
	int m_remainder = 0;

	enum CtsStatus {WAIT, FREE};
	CtsStatus m_cts_status;

	// Store MAC status within current time slot
	CalendarState m_mac_state;

	// Start of MAC operation
	double m_start_time = 1;

	// Duration of time slot: transmission_delay + propagation_delay
	double m_slot_duration = 2;

	// Current scheduled time for the scheduler
	Time m_scheduled_time;

	// Store value of the scheduled slot number
	int m_scheduled_slot = 0;

	// Maximum number of available slots to transmit in RTS
	int m_max_rts_available_slots = 5;

	// Count number of received CTS messages. This is needed for correct execution of CTS expiration events.
	double m_cts_count = 0;

	// Store current maximum available slot number in the sent RTS
	int m_max_available_rts_slot = 0;

	// Queue to store incoming data packets
	std::queue<Ptr<Packet>> m_incoming_data_queue;

//	uint16_t x = 1;

};

} /* namespace ns3 */

#endif /* SRC_AQUA_SIM_NG_MODEL_AQUA_SIM_MAC_MULTICHANNEL_H_ */
