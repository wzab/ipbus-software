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

#ifndef _uhal_ProtocolUDP_hpp_
#define _uhal_ProtocolUDP_hpp_

#include "uhal/ClientInterface.hpp"
#include "uhal/log/exception.hpp"
#include "uhal/log/log.hpp"

#include <iostream>
#include <iomanip>

#include <boost/shared_ptr.hpp>
#include <boost/asio/io_service.hpp>
#include <boost/asio/ip/udp.hpp>
#include <boost/asio/deadline_timer.hpp>

#ifdef RUN_ASIO_MULTITHREADED
#include <boost/thread/thread.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/condition_variable.hpp>
#endif

#include <string>

namespace uhal
{

  namespace exception
  {
    //! Exception class to handle the case where the UDP connection timed out.
    UHAL_DEFINE_EXCEPTION_CLASS ( UdpTimeout , "Exception class to handle the case where the UDP connection timed out." )
    //! Exception class to handle a failure to create a UDP socket.
    UHAL_DEFINE_EXCEPTION_CLASS ( ErrorAtUdpSocketCreation , "Exception class to handle a failure to create a UDP socket." )
    //! Exception class to handle the case where ASIO returned an error.
    UHAL_DEFINE_EXCEPTION_CLASS ( ASIOUdpError , "Exception class to handle the case where ASIO returned an error." )
  }

  //! Transport protocol to transfer an IPbus buffer via UDP
  template < typename InnerProtocol >
  class UDP : public InnerProtocol
  {

    public:
      //! Functor class to perform the actual transport, Like this to allow multithreading if desirable.

      /**
      	Constructor
      	@param aId the uinique identifier that the client will be given.
      	@param aUri a struct containing the full URI of the target.
      */
      UDP ( const std::string& aId, const URI& aUri );

      /**
        Copy Constructor
        This creates a new socket, dispatch queue, dispatch thread, etc. which connects to the same target ip/port
        @param aUDP a UDP-protocol object to copy
      */
      UDP ( const UDP& aUDP );

      /**
       Assignment operator
       This reassigns the endpoint, closes the existing socket and cleans up the buffers, etc. On the next call which requires the socket, it will be reopened with the new endpoint.
       @param aUDP a UDP-protocol object to copy
       @return reference to the current object to allow chaining of assignments
            */
      UDP& operator= ( const UDP& aUDP );


      /**
      	Destructor
      */
      virtual ~UDP();

      /**
      	Send the IPbus buffer to the target, read back the response and call the packing-protocol's validate function
      	@param aBuffers the buffer object wrapping the send and recieve buffers that are to be transported
      	If multithreaded, adds buffer to the dispatch queue and returns. If single-threaded, calls the dispatch-worker dispatch function directly and blocks until the response is validated.
      */
      void implementDispatch ( boost::shared_ptr< Buffers > aBuffers );

      /**
      Concrete implementation of the synchronization function to block until all buffers have been sent, all replies received and all data validated
       */
      virtual void Flush( );


    protected:
      /**
              Function which tidies up this protocol layer in the event of an exception
             */
      virtual void dispatchExceptionHandler();


    private:
      /**
        Set up the UDP socket
      */
      void connect();
      /**
        Initialize performing the next UDP write operation
        In multi-threaded mode, this runs the ASIO async send and exits
        In single-threaded mode, this runs the ASIO async send and blocks
      */
      void write ( );
      /**
        Callback function which is called upon completion of the ASIO async send
        This, then, makes a call to read to read back the reply to what has just been sent
        @param aErrorCode the error code with which the ASIO operation completed
      */
      void write_callback ( const boost::system::error_code& aErrorCode , std::size_t aBytesTransferred );

      /**
        Initialize performing the next UDP read operation
        In multi-threaded mode, this runs the ASIO async receive and exits
        In single-threaded mode, this runs the ASIO async receive and blocks
      */
      void read ( );

      /**
        Callback function which is called upon completion of the ASIO async receive
        This, then, checks the queue to see if there are more packets to be sent and if so, calls write
        @param aErrorCode the error code with which the ASIO operation completed
      */
      void read_callback ( const boost::system::error_code& aErrorCode , std::size_t aBytesTransferred );

      /**
        Function called by the ASIO deadline timer
      */
      void CheckDeadline();

      /**
       Function to set the value of a variable associated with a BOOST conditional-variable and then notify that conditional variable
       @param aValue a value to which to update the variable associated with a BOOST conditional-variable
            */
      void NotifyConditionalVariable ( const bool& aValue );

      /**
        Function to block a thread pending a BOOST conditional-variable and its associated regular variable
      */
      void WaitOnConditionalVariable();


    private:
      //! The boost::asio::io_service used to create the connections
      boost::asio::io_service mIOservice;

      //! A shared pointer to a boost::asio udp socket through which the operation will be performed
      boost::asio::ip::udp::socket mSocket;

      //! A shared pointer to a boost::asio udp endpoint - used in the ASIO send and receive functions (UDP has no concept of a connection)
      boost::asio::ip::udp::endpoint mEndpoint;

      //! The mechanism for providing the time-out
      boost::asio::deadline_timer mDeadlineTimer;

      /**
        A block of memory into which we write replies, before copying them to their final destination
        @note This should not be necessary and was, for a while, removed, with the buffer sequence created, instead, pointing to the final destinations
        @note Tom Williams, however believes that there is a problem with scatter-gather operations of size>64 with the UDP and so has reverted it -- see https://svnweb.cern.ch/trac/cactus/ticket/259#comment:17
        @note Reverting the code, however, may have caused a seg-fault!
        @todo THIS NEEDS TO BE UNDERSTOOD
      */
      std::vector<uint8_t> mReplyMemory;

#ifdef RUN_ASIO_MULTITHREADED
      /// Needed when multi-threading to stop the boost::asio::io_service thinking it has nothing to do and so close the socket
      boost::asio::io_service::work mIOserviceWork;

      //! The Worker thread in Multi-threaded mode
      boost::thread mDispatchThread;

      //! A MutEx lock used to make sure the access functions are thread safe
      boost::mutex mTransportLayerMutex;

      //! The list of buffers still waiting to be sent
      std::deque < boost::shared_ptr< Buffers > > mDispatchQueue;
      //! The list of buffers still awaiting a reply
      std::deque < boost::shared_ptr< Buffers > > mReplyQueue;

      //! Counter of how many writes have been sent, for which no reply has yet been received
      uint32_t mPacketsInFlight;

      //! A mutex for use by the conditional variable
      boost::mutex mConditionalVariableMutex;
      //! A conditional variable for blocking the main thread until the variable with which it is associated is set correctly
      boost::condition_variable mConditionalVariable;
      //! A variable associated with the conditional variable which specifies whether all packets have been sent and all replies have been received
      bool mFlushDone;
#endif

      //! The send operation currently in progress
      boost::shared_ptr< Buffers > mDispatchBuffers;
      //! The receive operation currently in progress or the next to be done
      boost::shared_ptr< Buffers > mReplyBuffers;

      /**
        A pointer to an exception object for passing exceptions from the worker thread to the main thread.
        Exceptions must always be created on the heap (i.e. using `new`) and deletion will be handled in the main thread
      */
      uhal::exception::exception* mAsynchronousException;

  };


}

#include "uhal/TemplateDefinitions/ProtocolUDP.hxx"

#endif
