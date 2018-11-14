/*
 * aqua-sim-phy-multi.h
 *
 *  Created on: Oct 11, 2018
 *      Author: dmitry
 */
#ifndef AQUA_SIM_PHY_MULTI_H
#define AQUA_SIM_PHY_MULTI_H

#include "ns3/object.h"
#include "aqua-sim-phy-cmn.h"


/*
 * Baseclass for Aqua-Sim Multichannel Phy
 *
 */
namespace ns3 {

  class AquaSimPhyCmn;

  /**
   * \ingroup aqua-sim-ng
   *
   * \brief Base class for Multichannel Phy layer.
   *
   * Overall modem status can be found under AquaSimNetDevice
   */

  class AquaSimPhyMulti : public AquaSimPhyCmn
//  class AquaSimPhyMulti : public AquaSimPhy
  //  class AquaSimPhyMulti : public Object
  {
  public:
    AquaSimPhyMulti(void);
    virtual ~AquaSimPhyMulti(void);

    static TypeId GetTypeId();

//    inline Time CalcTxTime(uint32_t pktsize, std::string * modName = NULL);
//    inline double CalcPktSize(double txtime, std::string * modName = NULL);
//    virtual inline bool MatchFreq(double freq);

    // Allocate subchannels
    void AllocateSubchannels ();
    // Set sub-channel id to send down a packet
    void SetSubchannelId (int channel_id);

//    int GetChannelId();

    // Explicitly set number of subchannels, i.e. from the multichannel MAC module
    void SetSubchannelNumber (int n_subchannels);
    void SetSubchannelBandwidth (double bandwidth);
    int GetSubchannelId ();

    // Recv method for upper layers, i.e. MMAC
//    bool Recv (Ptr<Packet> p, Ptr<AquaSimPhyCmn> phy_cmn);
    bool Recv (Ptr<Packet> p);

    // Re-implement this function in order to correctly count all receisved packets from different PhyCmn,
    // which correspond to given sub-channels
    int PktRecvCount();

//    bool Decodable(double noise, double ps);

    void SetSinrChecker(Ptr<AquaSimSinrChecker> sinrChecker);
    void SetTransRange(double range);

  protected:

  private:
    // Number of subchannels. For each subchannel to allocate PhyCmn object
    int m_nsubchannels;
    // Bandwidth of each subchannel
    double m_subchannel_bandwidth;
    // Starting value of central frequency
    double m_central_freq_start;
//    // End value of central frequency
//    double m_central_freq_end;
    // Step between adjacent central frequencies
    double m_central_freq_step;
    // Map channel_id and corresponding PhyCmn object
    std::map<int, Ptr<AquaSimPhyCmn>> m_channel_phy_map;

    // Store frequency <---> channel_id map
    std::map<double, int> m_freq_channel_map;

    // Store number of received frames from all sub-channels
    int m_recvd_frames_counter = 0;

    // Store current sub-channel id to send down a packet
    int m_current_subchannel_id = 0;

  }; //AquaSimPhyMulti class

} //ns3 namespace

#endif /* AQUA_SIM_PHY_MULTI_H */
