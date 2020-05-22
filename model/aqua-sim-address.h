/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Copyright (c) 2016 University of Connecticut
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
 * Author: Robert Martin <robert.martin@engr.uconn.edu>
 */

/************

*************/

#ifndef AQUA_SIM_ADDRESS_H
#define AQUA_SIM_ADDRESS_H

#include <stdint.h>
#include "ns3/address.h"
#include "ns3/mac48-address.h"
#include <iostream>
#include <cstring>
#include <iomanip>


namespace ns3 {

class Address;
/**
 * \ingroup aqua-sim-ng
 *
 * \brief Specialized address for distinguishing nodes and protocol support. 48 bit addresses supported.
 *
 * Inherited from Mac48Address and added some methods for correct serialization/deserialization from to/from aqua-sim-header.
 */

class AquaSimAddress : public Mac48Address
{
public:
  AquaSimAddress ();
  AquaSimAddress (const char *str);
  AquaSimAddress (uint16_t addr);
  virtual ~AquaSimAddress ();

  void CopyFrom (const uint8_t buffer[6]);
  void CopyTo (uint8_t buffer[6]) const;
  operator Address () const;
  static AquaSimAddress ConvertFrom (const Address &address);
  static bool IsMatchingType (const Address &address);
  static AquaSimAddress Allocate (void);
  bool IsBroadcast (void) const;
  bool IsGroup (void) const;
  static AquaSimAddress GetBroadcast (void);
  static AquaSimAddress GetMulticast (Ipv4Address address);
  static AquaSimAddress GetMulticast (Ipv6Address address);
  static AquaSimAddress GetMulticastPrefix (void);
  static AquaSimAddress GetMulticast6Prefix (void);
  typedef void (* TracedCallback)(AquaSimAddress value);
  // Aqua-sim-address methods
  uint16_t GetAsInt (void) const;

  
private:
  Address ConvertTo (void) const;
  static uint8_t GetType (void);
  friend bool operator == (const AquaSimAddress &a, const AquaSimAddress &b);
  friend bool operator != (const AquaSimAddress &a, const AquaSimAddress &b);
  friend bool operator < (const AquaSimAddress &a, const AquaSimAddress &b);
  friend std::ostream& operator<< (std::ostream& os, const AquaSimAddress & address);
  friend std::istream& operator>> (std::istream& is, AquaSimAddress & address);

  uint8_t m_address[6]; //!< address value
};

inline bool operator < (const AquaSimAddress &a, const AquaSimAddress &b)
{
  return memcmp (a.m_address, b.m_address, 6) < 0;
}

inline bool operator == (const AquaSimAddress &a, const AquaSimAddress &b)
{
  return memcmp (a.m_address, b.m_address, 6) == 0;
}

inline bool operator != (const AquaSimAddress &a, const AquaSimAddress &b)
{
  return memcmp (a.m_address, b.m_address, 6) != 0;
}

std::ostream& operator<< (std::ostream& os, const AquaSimAddress & address);

std::istream& operator>> (std::istream& is, AquaSimAddress & address);

} // namespace ns3

#endif /* AQUA_SIM_ADDRESS_H */
