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

/**
	@file
	@author Andrew W. Rose
	@date 2012
*/

#ifndef _uhal_PackingProtocol_ControlHubHost_hpp_
#define _uhal_PackingProtocol_ControlHubHost_hpp_

#include "uhal/ProtocolInterfaces.hpp"

//#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>

namespace uhal
{

  //! A concrete class implementing the packing necessary for connection to IPbus hardware via a Control Hub PC
  template< uint8_t IPbus_major , uint8_t IPbus_minor >
  class ControlHubHostPackingProtocol : public PackingProtocol
  {
    public:
      /**
      	Constructor
      	@param aDeviceIPaddr The IP address of the target device that is connected to the Control Hub
      	@param aDevicePort The port number of the target device that is connected to the Control Hub
      	@param aMaxSendSize The size of the buffer in the target device for receiving IPbus data packets from uhal via the Control Hub
      	@param aMaxReplySize The size of the buffer in the target device for sending IPbus data packets to uhal via the Control Hub
      */
      ControlHubHostPackingProtocol ( const uint32_t& aDeviceIPaddr , const uint16_t& aDevicePort , const uint32_t& aMaxSendSize , const uint32_t& aMaxReplySize );

      /**
      	Destructor
      */
      virtual ~ControlHubHostPackingProtocol();

      /**
      	Concrete implementaion of function to calculate the IPbus header for a particular protocol version
      	@param aType the type of the IPbus transaction
      	@param aWordCount the word count field of the IPbus header
      	@return an IPbus header
      */
      virtual uint32_t calculateIPbusHeader ( const eIPbusTransactionType& aType , const uint32_t& aWordCount );

      /**
      	Concrete implementaion of function to parse an IPbus header for a particular protocol version
      	@param aHeader an IPbus header to be parsed
      	@param aType return the type of the IPbus transaction
      	@param aWordCount return the word count field of the IPbus header
      	@param aTransactionId return the TransactionId of the IPbus header
      	@param aResponseGood return the response status of the IPbus header
      	@return whether extraction succeeded
      */
      virtual bool extractIPbusHeader ( const uint32_t& aHeader , eIPbusTransactionType& aType , uint32_t& aWordCount , uint32_t& aTransactionId , uint8_t& aResponseGood );

      /**
      	Add a preamble to an IPbus buffer
      */
      virtual void Preamble( );

      /**
      	Finalize an IPbus buffer before it is transmitted
      */
      virtual void Predispatch( );

      /**
      	Function which the transport protocol calls when the IPbus reply is received to check that the headers are as expected
      	@param aBuffers the buffer object wrapping the send and recieve buffers that have been transported and are to be validated
      	@return whether the returned IPbus packet is valid
      	@warning ----------------------------------------------------------------------------------------------------------------------------------------------------------------
      	@warning NOTE! THIS FUNCTION MUST BE THREAD SAFE: THAT IS:
      	@warning IT MUST ONLY USE LOCAL VARIABLES
      	@warning 		   --- OR ---
      	@warning IT MUST MUTEX PROTECT ACCESS TO MEMBER VARIABLES!
      	@warning ----------------------------------------------------------------------------------------------------------------------------------------------------------------
      */
      virtual bool Validate ( Buffers* aBuffers );

    private:
      //! The IP address of the target device that is connected to the Control Hub
      uint32_t mDeviceIPaddress;

      //! The port number of the target device that is connected to the Control Hub
      uint16_t mDevicePort;

      //! The transaction counter which will be incremented in the sent IPbus headers
      uint32_t mTransactionCounter;

      //! A struct representing the preamble which will be prepended to an IPbus buffer for the benefit of the Control Hub
      struct tPreamble
      {
        //! The total number of bytes that follow (outgoing) forming a logical packet (IDs + IPbus packet)
        uint32_t* mSendByteCountPtr;
        //! The number of 32-bit words in the IPbus packet (legacy and could be removed)
        uint16_t* mSendWordCountPtr;

        //! The total number of bytes that follow (incoming) forming a logical packet (IDs + IPbus packet)
        uint32_t mReplyTotalByteCounter;
        //! A legacy counter
        uint32_t mReplyChunkByteCounter;
        //! The returned target device ID (IP address)
        uint32_t mReplyDeviceIPaddress;
        //! The returned target device ID (port number)
        uint16_t mReplyDevicePort;
        //! An error code returned describing the status of the control hub
        uint16_t mReplyErrorCode;
      };

      //! A MutEx lock used to make the Validate function thread safe
      boost::mutex mMutex;

      //! A queue of preample structs making the memory used by the preambles persistent during the dispatch
      std::deque< tPreamble > mPreambles;

  };

}

#include "uhal/TemplateDefinitions/PackingProtocol_ControlHubHost.hxx"


#endif
