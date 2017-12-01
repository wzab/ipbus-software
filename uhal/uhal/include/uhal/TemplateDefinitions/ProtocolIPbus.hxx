/*
---------------------------------------------------------------------------

    This file is part of uHAL.

    uHAL is a hardware access library and programming framework
    originally developed for upgrades of the Level-1 trigger of the CMS
    experiment at CERN.

    uHAL is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    uHAL is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with uHAL.  If not, see <http://www.gnu.org/licenses/>.


      Andrew Rose, Imperial College, London
      email: awr01 <AT> imperial.ac.uk

      Marc Magrans de Abril, CERN
      email: marc.magrans.de.abril <AT> cern.ch

---------------------------------------------------------------------------
*/

#include "boost/date_time/gregorian/gregorian.hpp"

#include <arpa/inet.h>

#include "uhal/Buffers.hpp"


namespace uhal
{

  // --------------------------------------------------------------------------------------------------------------------------------------------------------------
  template< uint8_t IPbus_minor >
  IPbus< 1 , IPbus_minor >::IPbus ( const std::string& aId, const URI& aUri ) :
    IPbusCore ( aId , aUri , boost::posix_time::seconds ( 1 ) )
    // , mSendPadding ( 8 , implementCalculateHeader ( B_O_T , 0 , 0 ) ),
    // mReplyPadding ( 8 , 0x00000000 )
  {
  }


  template< uint8_t IPbus_minor >
  IPbus< 1 , IPbus_minor >::~IPbus()
  {
  }

  template< uint8_t IPbus_minor >
  void IPbus< 1 , IPbus_minor >::preamble ( boost::shared_ptr< Buffers > aBuffers )
  {
    implementBOT();   //this is really just initializing the payload, rather than a true preamble
  }

  template< uint8_t IPbus_minor >
  uint32_t IPbus< 1 , IPbus_minor >::getPreambleSize()
  {
    return 1;
  }

  template< uint8_t IPbus_minor >
  void IPbus< 1 , IPbus_minor >::predispatch ( boost::shared_ptr< Buffers > aBuffers )
  {
    uint32_t lWords ( aBuffers->sendCounter()  >> 2 );
    //IPbus 1.3 requires that there are 8 words of IPbus payload, excluding any non-payload preamble. In this version of the protocol, the preamble is really just initializing the payload, rather than a true preamble, so, if nothing else was sent, then we need 7 more words of padding.
    int32_t lPaddingWords ( ( 7 + this->getPreambleSize() ) - lWords );

    if ( lPaddingWords >  0 )
    {
      // aBuffers->send ( ( uint8_t* ) ( & ( mSendPadding[0] ) ) , lPaddingWords<<2 );
      // aBuffers->receive ( ( uint8_t* ) ( & ( mReplyPadding[0] ) ) , lPaddingWords<<2 );
      for ( int32_t lWords = 0 ; lWords != lPaddingWords ; ++lWords )
      {
        log ( Debug() , "Adding padding word." );
        // We do not need to check for space here as this condition is only met when the current filling buffer is severely underfull
        aBuffers->send ( CalculateHeader ( B_O_T , 0 , 0 ) );
        std::pair < ValHeader , _ValHeader_* > lReply ( CreateValHeader() );
        lReply.second->IPbusHeaders.push_back ( 0 );
        aBuffers->add ( lReply.first );
        aBuffers->receive ( lReply.second->IPbusHeaders.back() );
      }
    }
  }

  template< uint8_t IPbus_minor >
  uint32_t IPbus< 1 , IPbus_minor >::CalculateHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount , const uint32_t& aTransactionId , const uint8_t& aInfoCode )
  {
    uint8_t lType ( 0x00 );

    switch ( aType )
    {
      case B_O_T :
        lType = 0xF8;
        break;
      case READ :
        lType = 0x18;
        break;
      case WRITE :
        lType = 0x20;
        break;
      case RMW_BITS :
        lType = 0x28;
        break;
      case RMW_SUM :
        lType = 0x30;
        break;
      case R_A_I :
        lType = 0xF0;
        break;
      case NI_READ :
        lType = 0x40;
        break;
      case NI_WRITE :
        lType = 0x48;
        break;
      case CONFIG_SPACE_READ :
      {
        exception::ValidationError lExc;
        log ( lExc , "Configuration space read undefined in IPbus version 1" );
        throw lExc;
      }
    }

    return ( 0x10000000 | ( ( aTransactionId&0x7ff ) <<17 ) | ( ( aWordCount&0x1ff ) <<8 ) | lType | ( aInfoCode&0x7 ) );
  }


  template< uint8_t IPbus_minor >
  uint32_t IPbus< 1 , IPbus_minor >::ExpectedHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount , const uint32_t& aTransactionId, const uint8_t& aInfoCode )
  {
    return ( IPbus< 1 , IPbus_minor >::CalculateHeader ( aType , aWordCount , aTransactionId , aInfoCode | 0x4 ) );
  }

  template< uint8_t IPbus_minor >
  bool IPbus< 1 , IPbus_minor >::ExtractHeader ( const uint32_t& aHeader , eIPbusTransactionType& aType , uint32_t& aWordCount , uint32_t& aTransactionId , uint8_t& aInfoCode )
  {
    uint32_t lProtocolVersion ( ( aHeader >> 28 ) & 0xF );

    if ( lProtocolVersion != 1 )
    {
      log ( Error() , "Wrong Protocol Version! " , Integer ( lProtocolVersion ) , " != 1" );
      return false;
    }

    switch ( aHeader & 0xF8 )
    {
      case 0xF8 :
        aType = B_O_T;
        break;
      case 0x18 :
        aType = READ;
        break;
      case 0x20 :
        aType = WRITE;
        break;
      case 0x28 :
        aType = RMW_BITS;
        break;
      case 0x30 :
        aType = RMW_SUM;
        break;
      case 0xF0 :
        aType = R_A_I;
        break;
      case 0x40 :
        aType = NI_READ;
        break;
      case 0x48 :
        aType = NI_WRITE;
        break;
      default:
        log ( Error() , "Unknown IPbus-header " , Integer ( uint8_t ( ( aHeader & 0xF8 ) ) , IntFmt<hex,fixed>() ) );
        return false;
    }

    aWordCount = ( aHeader >> 8 ) & 0x1ff;
    aTransactionId = ( aHeader >> 17 ) & 0x7ff;
    aInfoCode = aHeader & 0x3;
    return true;
  }

  template< uint8_t IPbus_minor >
  uint32_t IPbus< 1 , IPbus_minor >::implementCalculateHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount , const uint32_t& aTransactionId , const uint8_t& aInfoCode )
  {
    return IPbus< 1 , IPbus_minor >::CalculateHeader ( aType , aWordCount , aTransactionId , aInfoCode );
  }

  template< uint8_t IPbus_minor >
  bool IPbus< 1 , IPbus_minor >::implementExtractHeader ( const uint32_t& aHeader , eIPbusTransactionType& aType , uint32_t& aWordCount , uint32_t& aTransactionId , uint8_t& aInfoCode )
  {
    return IPbus< 1 , IPbus_minor >::ExtractHeader ( aHeader , aType , aWordCount , aTransactionId , aInfoCode );
  }

  template< uint8_t IPbus_minor >
  void IPbus< 1 , IPbus_minor >::dispatchExceptionHandler()
  {
    log ( Info() , ThisLocation() );
    IPbusCore::dispatchExceptionHandler();
  }

  template< uint8_t IPbus_minor >
  void IPbus< 1 , IPbus_minor >::translateInfoCode(std::ostream& aStream, const uint8_t& aInfoCode) {
    switch (aInfoCode) {
      case 0:
        aStream << "success";
        break;
      case 1:
        aStream << "partial";
        break;
      case 2:
        aStream << "failure";
        break;
      default:
        aStream << "UNKNOWN";
    }
  }
  // --------------------------------------------------------------------------------------------------------------------------------------------------------------



  // --------------------------------------------------------------------------------------------------------------------------------------------------------------
  template< uint8_t IPbus_minor >
  IPbus< 2 , IPbus_minor >::IPbus ( const std::string& aId, const URI& aUri ) :
    IPbusCore ( aId , aUri , boost::posix_time::seconds ( 1 ) ),
    mPacketCounter (
#ifndef DISABLE_PACKET_COUNTER_HACK
      1
#else
      0
#endif
    )
  {
  }


  template< uint8_t IPbus_minor >
  IPbus< 2 , IPbus_minor >::~IPbus()
  {
  }

  template< uint8_t IPbus_minor >
  void IPbus< 2 , IPbus_minor >:: preamble ( boost::shared_ptr< Buffers > aBuffers )
  {
    aBuffers->send ( 0x200000F0 | ( ( mPacketCounter&0xffff ) <<8 ) );
#ifndef DISABLE_PACKET_COUNTER_HACK
    mPacketCounter++;
#endif
    {
      boost::lock_guard<boost::mutex> lLock ( mReceivePacketMutex );
      mReceivePacketHeader.push_back ( 0x00000000 );
      aBuffers->receive ( mReceivePacketHeader.back() );
    }
  }


  template< uint8_t IPbus_minor >
  uint32_t IPbus< 2 , IPbus_minor >::getPreambleSize()
  {
    return 1;
  }


  template< uint8_t IPbus_minor >
  void IPbus< 2 , IPbus_minor >::predispatch ( boost::shared_ptr< Buffers > aBuffers )
  {
  }



  template< uint8_t IPbus_minor >
  exception::exception* IPbus< 2 , IPbus_minor >::validate ( uint8_t* aSendBufferStart ,
      uint8_t* aSendBufferEnd ,
      std::deque< std::pair< uint8_t* , uint32_t > >::iterator aReplyStartIt ,
      std::deque< std::pair< uint8_t* , uint32_t > >::iterator aReplyEndIt )
  {
    //log ( Debug() , ThisLocation() );
    //log ( Notice() , "Memory location = " , Integer ( ( std::size_t ) ( aReplyStartIt->first ) , IntFmt<hex,fixed>() ), " Memory value = " , Integer ( * ( std::size_t* ) ( aReplyStartIt->first ) , IntFmt<hex,fixed>() ), " & size = " , Integer ( aReplyStartIt->second ) );
    if ( * ( uint32_t* ) ( aSendBufferStart ) != * ( uint32_t* ) ( aReplyStartIt ->first ) )
    {
      uhal::exception::IPbus2PacketHeaderMismatch* lExc = new uhal::exception::IPbus2PacketHeaderMismatch();
      log ( *lExc , "Returned Packet Header from URI " , Quote ( this->uri() ) , ", " , Integer ( * ( uint32_t* ) ( aReplyStartIt ->first ) , IntFmt<hex,fixed>() ) ,
            " does not match that sent " , Integer ( * ( uint32_t* ) ( aSendBufferStart ) , IntFmt<hex,fixed>() ) );
      return lExc;
    }

    {
      boost::lock_guard<boost::mutex> lLock ( mReceivePacketMutex );
      mReceivePacketHeader.pop_front();
    }

    // log ( Info() , "IPbus 2.0 has validated the packet header" );
    return IPbusCore::validate ( ( aSendBufferStart+=4 ) , aSendBufferEnd , ( ++aReplyStartIt ) , aReplyEndIt );
  }




  template< uint8_t IPbus_minor >
  uint32_t IPbus< 2 , IPbus_minor >::CalculateHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount , const uint32_t& aTransactionId, const uint8_t& aInfoCode )
  {
    uint8_t lType ( 0x00 );

    switch ( aType )
    {
      case B_O_T :
      {
        exception::ValidationError lExc;
        log ( lExc , "Byte-Order-Transaction undefined in IPbus version 2" );
        throw lExc;
      }
      case READ :
        lType = 0x00;
        break;
      case WRITE :
        lType = 0x10;
        break;
      case NI_READ :
        lType = 0x20;
        break;
      case NI_WRITE :
        lType = 0x30;
        break;
      case RMW_BITS :
        lType = 0x40;
        break;
      case RMW_SUM :
        lType = 0x50;
        break;
      case CONFIG_SPACE_READ :
        lType = 0x60;
        break;
      case R_A_I :
      {
        exception::ValidationError lExc;
        log ( lExc , "Reserved address information transaction is undefined in IPbus version 2" );
        throw lExc;
      }
    }

    return ( 0x20000000 | ( ( aTransactionId&0xfff ) <<16 ) | ( ( aWordCount&0xff ) <<8 ) | lType | ( aInfoCode&0xF ) );
  }


  template< uint8_t IPbus_minor >
  uint32_t IPbus< 2 , IPbus_minor >::ExpectedHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount , const uint32_t& aTransactionId, const uint8_t& aInfoCode )
  {
    return ( IPbus< 2 , IPbus_minor >::CalculateHeader ( aType , aWordCount , aTransactionId , aInfoCode ) );
  }


  template< uint8_t IPbus_minor >
  bool IPbus< 2 , IPbus_minor >::ExtractHeader ( const uint32_t& aHeader , eIPbusTransactionType& aType , uint32_t& aWordCount , uint32_t& aTransactionId , uint8_t& aInfoCode )
  {
    uint32_t lProtocolVersion ( ( aHeader >> 28 ) & 0xF );

    if ( lProtocolVersion != 2 )
    {
      log ( Error() , "Wrong Protocol Version! " , Integer ( lProtocolVersion ) , " != 2" );
      return false;
    }

    switch ( aHeader & 0xF0 )
    {
      case 0x00 :
        aType = READ;
        break;
      case 0x10 :
        aType = WRITE;
        break;
      case 0x20 :
        aType = NI_READ;
        break;
      case 0x30 :
        aType = NI_WRITE;
        break;
      case 0x40 :
        aType = RMW_BITS;
        break;
      case 0x50 :
        aType = RMW_SUM;
        break;
      case 0x60 :
        aType = CONFIG_SPACE_READ;
        break;
      default:
        log ( Error() , "Unknown IPbus-header " , Integer ( uint8_t ( ( aHeader & 0xF0 ) >>4 ) , IntFmt<hex,fixed>() ) );
        return false;
    }

    aWordCount = ( aHeader >> 8 ) & 0xff;
    aTransactionId = ( aHeader >> 16 ) & 0xfff;
    aInfoCode = aHeader & 0xf;
    return true;
  }

  template< uint8_t IPbus_minor >
  uint32_t IPbus< 2 , IPbus_minor >::implementCalculateHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount , const uint32_t& aTransactionId , const uint8_t& aInfoCode )
  {
    return IPbus< 2 , IPbus_minor >::CalculateHeader ( aType , aWordCount , aTransactionId , aInfoCode );
  }

  template< uint8_t IPbus_minor >
  bool IPbus< 2 , IPbus_minor >::implementExtractHeader ( const uint32_t& aHeader , eIPbusTransactionType& aType , uint32_t& aWordCount , uint32_t& aTransactionId , uint8_t& aInfoCode )
  {
    return IPbus< 2 , IPbus_minor >::ExtractHeader ( aHeader , aType , aWordCount , aTransactionId , aInfoCode );
  }

  template< uint8_t IPbus_minor >
  void IPbus< 2 , IPbus_minor >::dispatchExceptionHandler()
  {
    log ( Info() , ThisLocation() );
#ifndef DISABLE_PACKET_COUNTER_HACK
    mPacketCounter = 1;
#else
    mPacketCounter = 0;
#endif
    mReceivePacketHeader.clear();
    IPbusCore::dispatchExceptionHandler();
  }

  template< uint8_t IPbus_minor >
  void IPbus< 2 , IPbus_minor >::translateInfoCode(std::ostream& aStream, const uint8_t& aInfoCode)
  {
    switch (aInfoCode) {
      case 0:
        aStream << "success";
        break;
      case 1:
        aStream << "bad header";
        break;
      case 4:
        aStream << "bus error on read";
        break;
      case 5:
        aStream << "bus error on write";
        break;
      case 6:
        aStream << "bus timeout on read";
        break;
      case 7:
        aStream << "bus timeout on wite";
        break;
      case 0xf:
        aStream << "outbound request";
        break;
      default:
        aStream << "UNKNOWN";
    }
  }

  // --------------------------------------------------------------------------------------------------------------------------------------------------------------

}


