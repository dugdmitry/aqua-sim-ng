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

#include "aqua-sim-address.h"
#include "ns3/address.h"


namespace ns3 {

// Taken from Mac48Address implementation
#define ASCII_a (0x41)
#define ASCII_z (0x5a)
#define ASCII_A (0x61)
#define ASCII_Z (0x7a)
#define ASCII_COLON (0x3a)
#define ASCII_ZERO (0x30)

/**
 * Converts a char to lower case.
 * \param c the char
 * \returns the lower case
 */
static char
AsciiToLowCase (char c)
{
  if (c >= ASCII_a && c <= ASCII_z) {
      return c;
    } else if (c >= ASCII_A && c <= ASCII_Z) {
      return c + (ASCII_a - ASCII_A);
    } else {
      return c;
    }
}

AquaSimAddress::AquaSimAddress ()
{
  std::memset (m_address, 0, 6);
}

AquaSimAddress::AquaSimAddress (const char *str)
{
  int i = 0;
  while (*str != 0 && i < 6) 
    {
      uint8_t byte = 0;
      while (*str != ASCII_COLON && *str != 0) 
        {
          byte <<= 4;
          char low = AsciiToLowCase (*str);
          if (low >= ASCII_a)
            {
              byte |= low - ASCII_a + 10;
            }
          else
            {
              byte |= low - ASCII_ZERO;
            }
          str++;
        }
      m_address[i] = byte;
      i++;
      if (*str == 0) 
        {
          break;
        }
      str++;
    }
  NS_ASSERT (i == 6);
}

void 
AquaSimAddress::CopyTo (uint8_t buffer[6]) const
{
  std::memcpy (buffer, m_address, 6);
}

AquaSimAddress::operator Address () const
{
  return ConvertTo ();
}

Address 
AquaSimAddress::ConvertTo (void) const
{
  return Address (GetType (), m_address, 6);
}

AquaSimAddress 
AquaSimAddress::ConvertFrom (const Address &address)
{
  // NS_ASSERT (address.CheckCompatible (GetType (), 6));
  AquaSimAddress retval;
  address.CopyTo (retval.m_address);
  return retval;
}

AquaSimAddress 
AquaSimAddress::Allocate (void)
{
  static uint64_t id = 0;
  id++;
  AquaSimAddress address;
  address.m_address[0] = (id >> 40) & 0xff;
  address.m_address[1] = (id >> 32) & 0xff;
  address.m_address[2] = (id >> 24) & 0xff;
  address.m_address[3] = (id >> 16) & 0xff;
  address.m_address[4] = (id >> 8) & 0xff;
  address.m_address[5] = (id >> 0) & 0xff;
  return address;
}
uint8_t 
AquaSimAddress::GetType (void)
{
  // static uint8_t type = Address::Register ();
  // return type;
  // Set type to 0 to pass matching check with Mac48Address
  return 0;
}

bool
AquaSimAddress::IsBroadcast (void) const
{
  return *this == GetBroadcast ();
}

AquaSimAddress
AquaSimAddress::GetBroadcast (void)
{
  static AquaSimAddress broadcast = AquaSimAddress ("ff:ff:ff:ff:ff:ff");
  return broadcast;
}

std::ostream& operator<< (std::ostream& os, const AquaSimAddress & address)
{
  uint8_t ad[6];
  address.CopyTo (ad);

  os.setf (std::ios::hex, std::ios::basefield);
  os.fill ('0');
  for (uint8_t i=0; i < 5; i++) 
    {
      os << std::setw (2) << (uint32_t)ad[i] << ":";
    }
  // Final byte not suffixed by ":"
  os << std::setw (2) << (uint32_t)ad[5];
  os.setf (std::ios::dec, std::ios::basefield);
  os.fill (' ');
  return os;
}

std::istream& operator>> (std::istream& is, AquaSimAddress & address)
{
  std::string v;
  is >> v;

  std::string::size_type col = 0;
  for (uint8_t i = 0; i < 6; ++i)
    {
      std::string tmp;
      std::string::size_type next;
      next = v.find (":", col);
      if (next == std::string::npos)
        {
          tmp = v.substr (col, v.size ()-col);
          address.m_address[i] = strtoul (tmp.c_str(), 0, 16);
          break;
        }
      else
        {
          tmp = v.substr (col, next-col);
          address.m_address[i] = strtoul (tmp.c_str(), 0, 16);
          col = next + 1;
        }
    }
  return is;
}

AquaSimAddress::AquaSimAddress (uint16_t addr)
{
  m_address[0] = (addr >> 8) & 0xff;
  m_address[1] = (addr >> 8) & 0xff;
  m_address[2] = (addr >> 8) & 0xff;
  m_address[3] = (addr >> 8) & 0xff;
  m_address[4] = (addr >> 8) & 0xff;
  m_address[5] = (addr >> 0) & 0xff;
}

AquaSimAddress::~AquaSimAddress ()
{
}

uint16_t
AquaSimAddress::GetAsInt (void) const
{
  // return ((m_address[0] << 8) | (m_address[1] & 0xff) | (m_address[2] & 0xff) | (m_address[3] & 0xff) | (m_address[4] & 0xff) | (m_address[5] & 0xff));
  return ((m_address[0] << 8) | (m_address[1] << 8) | (m_address[2] << 8) | (m_address[3] << 8) | (m_address[4] << 8) | (m_address[5] & 0xff));
}

} // namespace ns3
