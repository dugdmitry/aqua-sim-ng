/*
 * aqua-sim-mac-multichannel.cc
 *
 *  Created on: Jun 21, 2018
 *      Author: dmitry
 */

#include "aqua-sim-mac-multichannel.h"
#include "ns3/log.h"
#include "aqua-sim-header.h"
#include "aqua-sim-header-mac.h"
#include "aqua-sim-phy-cmn.h"
#include "ns3/node-list.h"

#include <random>


using namespace ns3;

NS_LOG_COMPONENT_DEFINE("AquaSimMultichannelMac");
NS_OBJECT_ENSURE_REGISTERED(AquaSimMultichannelMac);


AquaSimMultichannelMac::AquaSimMultichannelMac() {
	m_rand = CreateObject<UniformRandomVariable> ();
	m_backoffCounter = 0;

	// Set default number of subchannels
	m_nsubchannels = 10;
//	m_nsubchannels = 2;

	// Set lower/higher frequency bounds, in kHz
	m_lower_freq_bound = 10;
	m_higher_freq_bound = 40;
	// Calculate total bandwidth
	m_total_bandwidth = m_higher_freq_bound - m_lower_freq_bound;
	// Calculate frequency step between subchannels
//	m_freq_step = m_total_bandwidth / m_nsubchannels;
	m_freq_step = 3;

//	// Set default subchannel/freq pairs
//	SetDefaultSubchannels();

//	// Allocate channel numbers in vector
//	AllocateChannels();

	// Set initial CTS status to FREE
	m_cts_status = CtsStatus::FREE;

	// Allocate channel-freq map
	double current_freq = m_lower_freq_bound;
	int i = 1;
	while (current_freq <= m_higher_freq_bound)
	{
		// Fill channel_id/frequency pair
		m_channel_freq_map.insert(std::make_pair(i, current_freq));

		// Fill the "inverted" frequency/channel_id as well
		m_freq_channel_map.insert(std::make_pair(current_freq, i));

		current_freq += m_freq_step;
		i += 1;
	}

}

AquaSimMultichannelMac::~AquaSimMultichannelMac() {
}

TypeId
AquaSimMultichannelMac::GetTypeId()
{
  static TypeId tid = TypeId("ns3::AquaSimMultichannelMac")
      .SetParent<AquaSimMac>()
      .AddConstructor<AquaSimMultichannelMac>()
      .AddAttribute("ChannelId", "Channel ID",
	IntegerValue(0),
	MakeIntegerAccessor (&AquaSimMultichannelMac::m_current_channel),
	MakeIntegerChecker<int> ())

    .AddAttribute("timeslot_duration", "Duration of subchannel timeslot",
	DoubleValue(2),
	MakeDoubleAccessor (&AquaSimMultichannelMac::m_slot_duration),
	MakeDoubleChecker<int> ())

	.AddAttribute("channel_hopping", "Channel hopping",
	BooleanValue(false),
	MakeBooleanAccessor (&AquaSimMultichannelMac::m_channel_hopping),
	MakeBooleanChecker ())

	.AddAttribute("random_channel_hopping", "Random channel hopping",
	BooleanValue(false),
	MakeBooleanAccessor (&AquaSimMultichannelMac::m_random_channel_hopping),
	MakeBooleanChecker ())

	;
  return tid;
}

void
AquaSimMultichannelMac::AllocateCalendars()
{
}

//// Fill m_channel_freq_map with the default channel/frequency pairs
//void
//AquaSimMultichannelMac::SetDefaultSubchannels()
//{
//	double current_freq = m_lower_freq_bound;
//	int i = 1;
//	while (current_freq <= m_higher_freq_bound)
//	{
//		// Fill channel_id/frequency pair
//		m_channel_freq_map.insert(std::make_pair(i, current_freq));
//
//		// Fill the "inverted" frequency/channel_id as well
//		m_freq_channel_map.insert(std::make_pair(current_freq, i));
//
//		current_freq += m_freq_step;
//		i += 1;
//	}
//
////	// Allocate channels 3 and 8 only
////	m_channel_freq_map.insert(std::make_pair(1, 13));
////	m_channel_freq_map.insert(std::make_pair(2, 34));
//
//}

// Handle received packets
bool
AquaSimMultichannelMac::RecvProcess (Ptr<Packet> pkt)
{

  NS_LOG_FUNCTION(this);

	AquaSimHeader ash;
	MacHeader mach;
	MultichannelHeader multichannel_mac;

    pkt->RemoveHeader(ash);
    pkt->RemoveHeader(mach);
    pkt->RemoveHeader(multichannel_mac);


//	std::cout << "MAC RECV CHANNEL ID: " << Phy()->GetSubchannelId() << "\n";


	AquaSimAddress dst = mach.GetDA();
//    AquaSimAddress dst = mach.GetRecvAddr();

	if (ash.GetErrorFlag())
	{
		NS_LOG_DEBUG("MultichannelMac:RecvProcess: received corrupt packet.");
		pkt=0;
		return false;
	}

	if (dst == AquaSimAddress::GetBroadcast() || dst == AquaSimAddress::ConvertFrom(m_device->GetAddress()))
	{

		// Filter incoming frame by central frequency (channel ID) mismatch
		SetCurrentTimeslot ();
		if (multichannel_mac.GetChannelId() != GetChannelId (m_current_slot, AquaSimAddress::ConvertFrom(m_device->GetAddress()).GetAsInt()))
		{
			std::cout << "WARNING! FRAME IS RECEIVED IN WRONG TIMESLOT!\n";
			std::cout << "RECEIVED CHANNEL_ID: " << multichannel_mac.GetChannelId() << "\n";
			std::cout << "CURRENT CHANNEL_ID: " << GetChannelId (m_current_slot, AquaSimAddress::ConvertFrom(m_device->GetAddress()).GetAsInt()) << "\n";
			pkt=0;
			return false;
		}

		SetCurrentTimeslot ();
		// Drop received frame if the MMAC is currently in TX mode
		if (GetEventType(m_current_slot) == RTS_TX)
		{
			pkt=0;
			return false;
		}
		if (GetEventType(m_current_slot) == CTS_TX)
		{
			pkt=0;
			return false;
		}
		if (GetEventType(m_current_slot) == DATA_TX)
		{
			pkt=0;
			return false;
		}


//		std::cout << "\nMAC type: " << mach;
//		std::cout << "MMAC type: " << multichannel_mac << "\n";

		//get a packet from modem, remove the sync hdr from txtime first
		//cmh->txtime() 	-= getSyncHdrLen();

//	    std::cout << "PhyFreqMac: " << Phy()->GetFrequency() << "\n";

//	    // Filter received frame by central frequency and current timeslot
//	    if (!FilterFrame (Phy()->GetFrequency()))
//	    {
//	    	std::cout << "ERROR!!! RECV FREQ DOES NOT MATCH!!!\n";
//	    	pkt = 0;
//	    	return false;
//	    }

		if (m_packetSize == 0)
		{
			ash.SetSize(ash.GetSize() - m_packetHeaderSize);
		}

		// Check if RTS has been received. Send back CTS.
		if (multichannel_mac.GetPType() == MultichannelHeader::MMAC_RTS)
		{

//			std::cout << "RTS has been received. Sending back CTS." << "\n";

			SetCurrentTimeslot();
			int i = 0;
			// Reserve first available slot for CTS transmission
			for (i = 0; i < m_max_rts_available_slots; i++)
			{
				// Check if the selected slot is not busy and is greater than the current time slot
				if ((multichannel_mac.GetSlotFromRts(i) > m_current_slot) && SlotIsIdle(multichannel_mac.GetSlotFromRts(i)))
				{
					break;
				}
			}

			// Check whether some slot was selected or not
			bool slot_is_selected = false;
			int j = 0;
			// Select some next available slot to reserve it for data receiving
			for (j = i + 1; j < m_max_rts_available_slots; j++)
			{
				// Check if the selected slot is not busy
				if (SlotIsIdle(multichannel_mac.GetSlotFromRts(j)))
				{
					slot_is_selected = true;
					break;
				}
			}

		  // If RX slot was selected, add the event to calendar
		  if (slot_is_selected)
		  {
				// Add CTS TX event to the calendar
				AddEventBySlot(CTS_TX, multichannel_mac.GetSlotFromRts(i));

				// Add DATA RX event to the calendar
				AddEventBySlot (DATA_RX, multichannel_mac.GetSlotFromRts(j));
//				std::cout << "Scheduled slot for DATA RX: " << multichannel_mac.GetSlotFromRts(j) << "\n";

			  // Schedule CTS transmission
//			  m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (multichannel_mac.GetSlotFromRts(i) + 1)), Time::S) - Simulator::Now();
			  m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (multichannel_mac.GetSlotFromRts(i))), Time::S) - Simulator::Now();

//			  std::cout << "Scheduled Slot: " << multichannel_mac.GetSlotFromRts(i) << "\n";
//			  std::cout << "Scheduled Time: " << m_scheduled_time << "\n";
//			  std::cout << "Current Sim Time: " << Simulator::Now() << "\n";

			  // Generating CTS
			  Ptr<Packet> frame = GenerateCts(multichannel_mac.GetSlotFromRts(j), mach.GetSA());

			  Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDown, this, frame,
					  GetChannelId(multichannel_mac.GetSlotFromRts(i), mach.GetSA().GetAsInt()), TransStatus::NIDLE);

			  return false;

		  }
		  else
		  {
//			  std::cout << "ERROR! The slot for DATA RX has not been selected! CTS is not sent!\n";
		  }

//		  // Schedule CTS transmission
//		  m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot + 1)), Time::S) - Simulator::Now();
//
//		  std::cout << "Scheduled Time: " << m_scheduled_time << "\n";
//		  std::cout << "Current Sim Time: " << Simulator::Now() << "\n";
//
//		  // Generating CTS
//		  Ptr<Packet> frame = GenerateCts();
//
//		  Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDown, this, frame, 1, TransStatus::NIDLE);
//
//
//			return false;
		}

		// Check if CTS has been received. Get app packet from the buffer and send it.
		if (multichannel_mac.GetPType() == MultichannelHeader::MMAC_CTS)
		{
//			std::cout << "CTS has been received. Sending DATA." << "\n";

			// Check if CTS slot for DATA RX is greater than current timeslot (this is needed in case if
			// the CTS has been received late)
			SetCurrentTimeslot();
			if (multichannel_mac.GetDataRxSlotFromCts() <= m_current_slot)
			{
//				std::cout << "WARNING! CTS IS TOO LATE!!! DROPPING CTS!\n";
				pkt=0;
				return false;
			}

			// Increment CTS counter
			m_cts_count++;

			if (!m_incoming_data_queue.empty())
			{
//				std::pair<Ptr<Packet>,TransStatus> sendPacket = SendQueuePop();
//
//				std::cout  << "Sending app packet from queue: " << sendPacket.first << sendPacket.second << "\n";
//				std::cout << "Remaining packets in queue: " << m_sendQueue.size() << "\n";
//
//				// Generate MMAC data frame
//				Ptr<Packet> data_frame = CreateDataFrame(sendPacket.first);
//				SendDown(data_frame, 1);


//					std::pair<Ptr<Packet>,TransStatus> sendPacket = SendQueuePop();
					Ptr<Packet> sendPacket = GetPacketFromQueue();

//					std::cout  << "Sending app packet from queue: " << sendPacket << "\n";
//					std::cout << "Remaining packets in queue: " << m_incoming_data_queue.size() << "\n";

//				  // Locate current timeslot, depending on given sim time
//				  if (Simulator::Now() < Time::FromDouble(m_start_time, Time::S))
//				  {
//					  m_current_slot = -1;
//				  }
//				  else
//				  {
//					  m_current_slot = (Simulator::Now()) / Time::FromDouble(m_slot_duration, Time::S) - 1;
//					  std::cout << "Current slot: " << m_current_slot << "\n";
//				  }

				  // Adding selected DATA RX slot from CTS to calendar
					if (SlotIsIdle (multichannel_mac.GetDataRxSlotFromCts()))
					{
					  // Add TX event to calendar
//					  std::cout << "Adding TX event to calendar...\n";
					  AddEventBySlot (DATA_TX, multichannel_mac.GetDataRxSlotFromCts());
					}
					else
					{
//						std::cout << "WARNING!!! CTS DATA RX SLOT IS NOT IDLE ON SENDER!\n";

						// If the scheduled event is RTS_TX, then replace it with DATA_RX and reschedule RTS_TX
						if (GetEventType (multichannel_mac.GetDataRxSlotFromCts() == RTS_TX))
						{
							// Reschedule the RTS_TX event
							m_reschedule_rts = true;
							// Add DATA_TX event
//							std::cout << "Replacing RTS_TX with DATA_TX in calendar...\n";
							AddEventBySlot (DATA_TX, multichannel_mac.GetDataRxSlotFromCts());
						}
						// If some other event type is scheduled, then return
						else
						{
//							std::cout << "The scheduled event cannot be replaced by DATA_TX!!! Dropping the CTS...\n";
							return false;
						}

					}

				  // Schedule DATA transmission
//				  m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (multichannel_mac.GetDataRxSlotFromCts() + 1)), Time::S) - Simulator::Now();
				  m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (multichannel_mac.GetDataRxSlotFromCts())), Time::S) - Simulator::Now();

//				  std::cout << "Scheduled Slot: " << multichannel_mac.GetDataRxSlotFromCts() << "\n";
//				  std::cout << "Scheduled Time: " << m_scheduled_time << "\n";
//				  std::cout << "Current Sim Time: " << Simulator::Now() << "\n";

				  // Generating DATA frame
				  Ptr<Packet> data_frame = CreateDataFrame(sendPacket, mach.GetSA());

				  Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDown, this, data_frame,
						  GetChannelId(multichannel_mac.GetDataRxSlotFromCts(), mach.GetSA().GetAsInt()), TransStatus::NIDLE);

			}

			else
			{
//				std::cout << "No packets in send queue!" << "\n";
			}

			// Change CTS state to FREE
			m_cts_status = CtsStatus::FREE;

			return false;
		}

		// Check if DATA frame has been received. Send to upper layers.
		if (multichannel_mac.GetPType() == MultichannelHeader::MMAC_DATA)
		{

//			std::cout << "DATA has been received. Sending up." << "\n";

//			  // Locate current timeslot, depending on given sim time
//			  if (Simulator::Now() < Time::FromDouble(m_start_time, Time::S))
//			  {
//				  m_current_slot = -1;
//			  }
//			  else
//			  {
//				  m_current_slot = (Simulator::Now()) / Time::FromDouble(m_slot_duration, Time::S) - 1;
//				  std::cout << "Current slot: " << m_current_slot << "\n";
//			  }


			pkt->AddHeader(ash);  //leave MacHeader off since sending to upper layers
			return SendUp(pkt);

		}


//		pkt->AddHeader(ash);  //leave MacHeader off since sending to upper layers
//		return SendUp(pkt);
	}

//	printf("underwaterAquaSimBroadcastMac: this is neither broadcast nor my packet, just drop it\n");
	pkt=0;
	return false;

}

// Handle transmitted packets
bool
AquaSimMultichannelMac::TxProcess(Ptr<Packet> pkt)
{

	NS_LOG_FUNCTION(this << pkt);
//  NS_LOG_FUNCTION("hello");


//  int n_subchannels = Phy()->GetNSubchannels();
//  std::cout << "\n N subchannels: " << n_subchannels << "\n";


//  AquaSimHeader ash;
//  MacHeader mach;
////  TMacHeader mach;
//  pkt->RemoveHeader(ash);
//
//  mach.SetDA(AquaSimAddress::GetBroadcast());
//  mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
//
////  mach.SetRecvAddr(AquaSimAddress::GetBroadcast());
////  mach.SetSenderAddr(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
////
////  mach.SetPtype(TMacHeader::PT_RTS);
//
//
//  if( m_packetSize != 0 )
//    ash.SetSize(m_packetSize);
//  else
//    ash.SetSize(m_packetHeaderSize + ash.GetSize());
//
//  ash.SetTxTime(GetTxTime(pkt));

//  // Current channel number
//  int channel_id = 0;

  AquaSimHeader asHeader;

//  // Current scheduled time
//  Time scheduled_time;


//  switch( m_device->GetTransmissionStatus() )
//  {
//  case SLEEP:
//      PowerOn();
//      break;
//  case NIDLE:
//
//      // Placing incoming app data packet to the sending queue
//      m_sendQueue.push(std::make_pair(pkt, NIDLE));
//
//      // Sending RTS if not in CTS WAIT state
//      if (m_cts_status == CtsStatus::FREE)
//      {
//          // Add TX event to calendar
//          AddEvent (RTS_TX);
//
//          // Schedule RTS transmission
////          m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot + 1)), Time::S) - Simulator::Now();
//          m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot)), Time::S) - Simulator::Now();
//
//          std::cout << "\nScheduled Time: " << m_scheduled_time << "\n";
//          std::cout << "Current Sim Time: " << Simulator::Now() << "\n";
//
//          // Get dst_address from the packet
//          pkt->PeekHeader(asHeader);
//
//    //      // Generating RTS
//    //      frame = GenerateRts(asHeader.GetDAddr());
//
//          std::cout << "Scheduling RTS transmission on timeslot: " << m_scheduled_slot << "\n";
//
//          // Changing state to CTS wait
//          m_cts_status = CtsStatus::WAIT;
//
//          Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDownRts, this,
//        		  GetChannelId(m_scheduled_slot, asHeader.GetDAddr().GetAsInt()), asHeader.GetDAddr());
//
//    //      Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDown, this, frame,
//    //    		  GetChannelId(m_scheduled_slot, asHeader.GetDAddr().GetAsInt()), TransStatus::NIDLE);
//
//      }
//
//      m_backoffCounter=0;
//
//      return true;
//
//  case RECV:
//    {
//      double backoff=m_rand->GetValue()*BC_BACKOFF;
//      NS_LOG_DEBUG("BACKOFF time:" << backoff << " on node:" << m_device->GetAddress() << "\n");
//      //pkt->AddHeader(mach);
//
////      pkt->AddHeader(ash);
//
//      Simulator::Schedule(Seconds(backoff),&AquaSimMultichannelMac::BackoffHandler,this,pkt);
//    }
//      return true;
//
//  case SEND:
//    {
//      double backoff=m_rand->GetValue()*BC_BACKOFF;
//      NS_LOG_DEBUG("BACKOFF time:" << backoff << " on node:" << m_device->GetAddress() << "\n");
//      //pkt->AddHeader(mach);
//
////      pkt->AddHeader(ash);
//
//      Simulator::Schedule(Seconds(backoff),&AquaSimMultichannelMac::BackoffHandler,this,pkt);
//    }
//    return true;
//      /*pkt=0;*/
//
//  default:
//      /*
//      * all cases have been processed above, so simply return
//      */
//    break;
//
//  }
//

  // Placing incoming app data packet to the sending queue
//  m_sendQueue.push(std::make_pair(pkt, NIDLE));
//  pkt->RemoveHeader(asHeader);
  m_incoming_data_queue.push(pkt);

  // Sending RTS if not in CTS WAIT state
  if (m_cts_status == CtsStatus::FREE)
  {
      // Add TX event to calendar
      AddEvent (RTS_TX);

      // Schedule RTS transmission
//          m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot + 1)), Time::S) - Simulator::Now();
      m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot)), Time::S) - Simulator::Now();

//      std::cout << "\nScheduled Time: " << m_scheduled_time << "\n";
//      std::cout << "Current Sim Time: " << Simulator::Now() << "\n";

      // Get dst_address from the packet
      pkt->PeekHeader(asHeader);

//      // Generating RTS
//      frame = GenerateRts(asHeader.GetDAddr());

//      std::cout << "Scheduling RTS transmission on timeslot: " << m_scheduled_slot << "\n";

      // Changing state to CTS wait
      m_cts_status = CtsStatus::WAIT;

      Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDownRts, this,
    		  GetChannelId(m_scheduled_slot, asHeader.GetDAddr().GetAsInt()), asHeader.GetDAddr());

//      Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDown, this, frame,
//    		  GetChannelId(m_scheduled_slot, asHeader.GetDAddr().GetAsInt()), TransStatus::NIDLE);

  }

  m_backoffCounter=0;

  return true;

//  return true;  //may be bug due to Sleep/default cases
}

// Separate method for sending down RTS. This is needed to schedule the RTS sending event, so that the actual RTS frame
// will contain an up-to-date information about the IDLE time slots in the calendar, right before being sent down.
bool
AquaSimMultichannelMac::SendDownRts(int channel_id, AquaSimAddress dst_address)
{
	// Check if the current RTS should be rescheduled
	if (m_reschedule_rts)
	{
		// Set the flag to false
		m_reschedule_rts = false;

		// Reschedule the RTS transmission:
		// Add TX event to calendar
		AddEvent (RTS_TX);

        // Schedule RTS transmission
//	    m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot + 1)), Time::S) - Simulator::Now();
	    m_scheduled_time = Time::FromDouble((m_start_time + m_slot_duration * (m_scheduled_slot)), Time::S) - Simulator::Now();

//	    std::cout << "Re-scheduling RTS transmission on timeslot: " << m_scheduled_slot << "\n";

	    Simulator::Schedule(m_scheduled_time, &AquaSimMultichannelMac::SendDownRts, this,
	    		GetChannelId(m_scheduled_slot, dst_address.GetAsInt()), dst_address);

	    return false;

	}
	else
	{
		// Current frame to send
		Ptr<Packet> frame = Create<Packet>();

		// Generate RTS frame with the most recent info about IDLE time slots
	    frame = GenerateRts(dst_address);

	  // Schedule CTS wait expiration event, in case if the incoming CTS is lost
//        std::cout << "Max RTS slot number: " << m_max_available_rts_slot << "\n";
//        std::cout << "Scheduled CTS EXPIRE: " << Time::FromDouble((m_start_time + m_slot_duration * (m_max_available_rts_slot + 1)), Time::S) - Simulator::Now() << "\n";
	  Simulator::Schedule(Time::FromDouble((m_start_time + m_slot_duration * (m_max_available_rts_slot + 1)), Time::S) - Simulator::Now(),
			  &AquaSimMultichannelMac::TriggerCtsExpiration, this, m_cts_count);

		return SendDown (frame, channel_id, TransStatus::NIDLE);
	}
}

bool
AquaSimMultichannelMac::SendDown(Ptr<Packet> p, int channelId, TransStatus afterTrans)
{
  NS_ASSERT(m_device);// && m_phy && m_rout);

////  // Setting MAC status to TX
////  m_mac_state = TX;
//
//  /*  For debugging:
//  std::cout << "\nMac @SendDown check:\n";
//  p->Print(std::cout);
//  std::cout << "\n";
//  */
//
//  if (m_device->GetTransmissionStatus() == SLEEP) {
//    NS_LOG_DEBUG("SendDown::Sleeping, drop pkt");
//      return false;
//   }
//
//  if (m_device->GetTransmissionStatus() == RECV) {
//      NS_LOG_DEBUG("SendDown::Recv, queuing pkt");
//      m_sendQueue.push(std::make_pair(p,afterTrans));
//      return true;
//  }
//  else {
//
//	  m_device->SetTransmissionStatus(SEND);
//
//      AquaSimHeader ash;
//
//      p->PeekHeader(ash);
//
//      if (ash.GetTxTime().IsNegative()) ash.SetTxTime(GetTxTime(p));
//
//      std::cout << "\nSending down a frame: " << p << " On channel: " << channelId << "\n";
//      std::cout << "TxTime: " << ash.GetTxTime() << "\n";
//      Simulator::Schedule(ash.GetTxTime(), &AquaSimNetDevice::SetTransmissionStatus,m_device,afterTrans);
//
//      //slightly awkard but for phy header Buffer
//      AquaSimPacketStamp pstamp;
//
//      p->AddHeader(pstamp);
//
//      // Send packet to Phy layer, with defined frequency
////      return Phy()->Recv(p, m_channel_freq_map.at(channelId));
////      return Phy()->Recv(p, m_channel_freq_map.at(channelId));
//      return Phy()->Recv(p);
//
//
//  }

  m_device->SetTransmissionStatus(SEND);

  AquaSimHeader ash;
  MacHeader mach;
  MultichannelHeader mmac_header;

  p->RemoveHeader(ash);
  p->RemoveHeader(mach);
  p->RemoveHeader(mmac_header);


  if (ash.GetTxTime().IsNegative()) ash.SetTxTime(GetTxTime(p));
//  std::cout << "TX_TIME: " << GetTxTime(p) << "\n";

//  std::cout << "\nSending down a frame: " << p << " On channel: " << channelId << "\n";
//  std::cout << "MMAC TxTime: " << ash.GetTxTime() << "\n";
  Simulator::Schedule(ash.GetTxTime(), &AquaSimNetDevice::SetTransmissionStatus,m_device,afterTrans);


  // Add channel ID to mmac_header
  mmac_header.AddChannelId(channelId);

  p->AddHeader(mmac_header);

  p->AddHeader(mach);

  p->AddHeader(ash);


  //slightly awkard but for phy header Buffer
  AquaSimPacketStamp pstamp;

  pstamp.SetFreq(m_channel_freq_map.find(channelId)->second);

  p->AddHeader(pstamp);

//  // Set channel ID
//  Phy()->SetSubchannelId(channelId);

  // Send packet down
  return Phy()->Recv(p);
}


Ptr<Packet>
AquaSimMultichannelMac::GenerateRts (AquaSimAddress dst_address)
{

	AquaSimHeader ash;
	MacHeader mach;
	MultichannelHeader multichannel_mach;

	Ptr<Packet> pkt = Create<Packet>();

	mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	mach.SetDA(dst_address);

	multichannel_mach.SetPType(MultichannelHeader::MMAC_RTS);

	// Set current timeslot
	SetCurrentTimeslot ();
	uint16_t slot = m_current_slot + 1;
	// Add available slot numbers into RTS
	for (int i = 1; i <= m_max_rts_available_slots; i++)
	{
		while (!SlotIsIdle(slot))
		{
			slot = slot + 1;
		}
		multichannel_mach.AddSlotToRts(slot);
		slot = slot + 1;
	}

	// Update current maximum available RTS slot number
	m_max_available_rts_slot = multichannel_mach.GetMaxRtsSlot();

	pkt->AddHeader(multichannel_mach);
	pkt->AddHeader(mach);
	ash.SetSize(multichannel_mach.GetSerializedSize());
	ash.SetTxTime(Phy()->CalcTxTime(multichannel_mach.GetSerializedSize()));

//	ash.SetUId(x);

//	ash.SetNumForwards(1);

	pkt->AddHeader(ash);

//	  std::cout << "\nMMAC: @SendDown check:\n";
//	  pkt->Print(std::cout);
//	  std::cout << "\n";

//	  x++;

	return pkt;
}


Ptr<Packet>
AquaSimMultichannelMac::GenerateCts (int slot_number, AquaSimAddress dst_address)
{

	AquaSimHeader ash;
	MacHeader mach;
	MultichannelHeader multichannel_mach;

	Ptr<Packet> pkt = Create<Packet>();

	mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	mach.SetDA(dst_address);

	multichannel_mach.SetPType(MultichannelHeader::MMAC_CTS);
	// Add DATA RX slot
	multichannel_mach.AddDataRxSlotForCts(slot_number);

	ash.SetSize(multichannel_mach.GetSerializedSize());
	ash.SetTxTime(Phy()->CalcTxTime(multichannel_mach.GetSerializedSize()));


	pkt->AddHeader(multichannel_mach);
	pkt->AddHeader(mach);
	pkt->AddHeader(ash);

	return pkt;
}

Ptr<Packet>
AquaSimMultichannelMac::CreateDataFrame (Ptr<Packet> app_packet, AquaSimAddress dst_address)
{
	  AquaSimHeader ash;
	  MacHeader mach;
	  MultichannelHeader multichannel_mach;
	  multichannel_mach.SetPType(MultichannelHeader::MMAC_DATA);

	  app_packet->RemoveHeader(ash);

	  mach.SetSA(AquaSimAddress::ConvertFrom(m_device->GetAddress()));
	  mach.SetDA(dst_address);

//	  if( m_packetSize != 0 )
//	    ash.SetSize(m_packetSize);
//	  else
//	    ash.SetSize(m_packetHeaderSize + ash.GetSize());

//	  std::cout << "DATA PACKET SIZE: " << app_packet->GetSize() << "\n";

	  ash.SetSize(multichannel_mach.GetSerializedSize() + app_packet->GetSize());
	  ash.SetTxTime(GetTxTime(app_packet));

	  ash.SetDirection(AquaSimHeader::DOWN);
	  //ash->addr_type()=NS_AF_ILINK;
	  //add the sync hdr

	  app_packet->AddHeader(multichannel_mach);

	  app_packet->AddHeader(mach);
	  app_packet->AddHeader(ash);

	return app_packet;
}

void
AquaSimMultichannelMac::BackoffHandler(Ptr<Packet> pkt)
{
  m_backoffCounter++;
  if (m_backoffCounter<BC_MAXIMUMCOUNTER)
    TxProcess(pkt);
  else
    {
      NS_LOG_INFO("BackoffHandler: too many backoffs");
//      std::cout << "BackoffHandler: too many backoffs" << "\n";
      m_backoffCounter=0;
//      DropPacket(pkt);
      pkt = 0;
    }
}

int64_t
AquaSimMultichannelMac::AssignStreams (int64_t stream)
{
  NS_LOG_FUNCTION (this << stream);
  m_rand->SetStream(stream);
  return 1;
}

void
AquaSimMultichannelMac::DoDispose()
{
  NS_LOG_FUNCTION(this);
  while(!m_incoming_data_queue.empty()) {
	  m_incoming_data_queue.front()=0;
	  m_incoming_data_queue.pop();
  }
  AquaSimMac::DoDispose();
}

void
AquaSimMultichannelMac::AddEvent (CalendarState state)
{
	// Set current timeslot
	SetCurrentTimeslot ();
	int slot = m_current_slot + 1;

	// Find idle slot in the calendar map
	while (!SlotIsIdle (slot))
	{
		slot = slot + 1;
	}

	m_scheduled_slot = slot;

//	std::cout << "Scheduled Slot: " << m_scheduled_slot << "\n";

	// Add Tx event for the slot
	m_calendar_map.insert(std::make_pair(slot, state));
}

void
AquaSimMultichannelMac::AddEventBySlot (CalendarState state, int slot_number)
{
	// Add event for the slot
	m_calendar_map.insert(std::make_pair(slot_number, state));
}

int
AquaSimMultichannelMac::GetChannelId (int slot_number, int node_addr)
{
//	// Get my own node address
////	int node_addr = AquaSimAddress::ConvertFrom(m_device->GetAddress());
//	int node_addr = m_device->GetNode()->GetId();

	// Return channel ID
	int channel_id = std::fmod(slot_number + node_addr, 10);

	if (channel_id == 0)
	{
		channel_id = 10;
	}

//	std::cout << "Node Address: " << node_addr << "\n";
//	std::cout << "Slot Number: " << slot_number << "\n";
//	std::cout << "Channel ID: " << channel_id << "\n";


	return channel_id;
//	return 1;
}

int
AquaSimMultichannelMac::GetChannelIdByFreq (double freq)
{
	// Return channel ID
	return m_freq_channel_map.find(freq)->second;
}

bool
AquaSimMultichannelMac::SlotIsIdle(int slot_number)
{
	// Check if the given slot_number exists in the map
	if (m_calendar_map.count(slot_number) != 0)
	{
		// If exists, then the given slot_number is reserved by Tx/Rx states
		return false;
	}
	else
	{
		return true;
	}
}

AquaSimMultichannelMac::CalendarState
AquaSimMultichannelMac::GetEventType(int slot_number)
{
	// Check if the given slot_number exists in the map
	if (m_calendar_map.count(slot_number) != 0)
	{
		// If exists, then return its state
		return m_calendar_map.at(slot_number);
	}
	else
	{
		return IDLE;
	}
}

void
AquaSimMultichannelMac::SetCurrentTimeslot ()
{
	  if (Simulator::Now() < Time::FromDouble(m_start_time, Time::S))
	  {
		  m_current_slot = -1;
	  }
	  else
	  {
//		  m_current_slot = (Simulator::Now()) / Time::FromDouble(m_slot_duration, Time::S) - 1;
		  m_current_slot = ((Simulator::Now()) - Time::FromDouble(m_start_time, Time::S)) / Time::FromDouble(m_slot_duration, Time::S);
	  }
}

bool
AquaSimMultichannelMac::FilterFrame (double central_freq)
{
	// Set current timeslot number
	SetCurrentTimeslot ();

	// If the received frequency matches with the current timeslot / node_addr, then return true, otherwise - false
//	if (m_freq_channel_map.at(central_freq) == GetChannelId (m_current_slot, m_device->GetNode()->GetId() + 1))
	if (m_freq_channel_map.at(central_freq) == GetChannelId (m_current_slot, AquaSimAddress::ConvertFrom(m_device->GetAddress()).GetAsInt()))

	{
		return true;
	}
	else
	{
		return false;
	}
}

// Put CTS expiration event to the scheduler, in case if the CTS has not arrived within the given RTS slot numbers
void
AquaSimMultichannelMac::TriggerCtsExpiration (double cts_number)
{
	// If current CTS count matches with the scheduled cts_number, change CTS status back to FREE
	if (m_cts_count == cts_number)
	{
//		std::cout << "CTS WAIT IS EXIPRED!!! On node number: " << AquaSimAddress::ConvertFrom(m_device->GetAddress()).GetAsInt() << "\n";
//		std::cout << "Sim Time: " << Simulator::Now() << "\n";
		m_cts_status = CtsStatus::FREE;
	}
}

// Return packet from data queue
Ptr<Packet>
AquaSimMultichannelMac::GetPacketFromQueue()
{
	  Ptr<Packet> element = m_incoming_data_queue.front();
	  m_incoming_data_queue.front()=0;
	  m_incoming_data_queue.pop();
	  return element;
}
