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

#include "uhal/ProtocolUDP.hpp"


#include <sys/time.h>

#include <boost/bind/bind.hpp>
#include <boost/lambda/lambda.hpp>
#include <boost/asio/connect.hpp>
#include <boost/asio/write.hpp>
#include <boost/asio/read.hpp>
#include <boost/asio/placeholders.hpp>
#include "boost/date_time/posix_time/posix_time.hpp"

#include "uhal/Buffers.hpp"
#include "uhal/IPbusInspector.hpp"
#include "uhal/ProtocolIPbus.hpp"


namespace uhal
{

  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  template < typename InnerProtocol >
  UDP< InnerProtocol >::UDP ( const std::string& aId, const URI& aUri ) :
    InnerProtocol ( aId , aUri ),
    mIOservice ( ),
    mSocket ( mIOservice , boost::asio::ip::udp::endpoint ( boost::asio::ip::udp::v4(), 0 ) ),
    mEndpoint ( *boost::asio::ip::udp::resolver ( mIOservice ).resolve ( boost::asio::ip::udp::resolver::query ( boost::asio::ip::udp::v4() , aUri.mHostname , aUri.mPort ) ) ),
    mDeadlineTimer ( mIOservice ),
    mReplyMemory ( 1500 , 0x00000000 ),
    mIOserviceWork ( mIOservice ),
    mDispatchThread ( boost::bind ( &boost::asio::io_service::run , & ( mIOservice ) ) ),
    mDispatchQueue(),
    mReplyQueue(),
    mPacketsInFlight ( 0 ),
    mFlushDone ( true ),
    //    mDispatchBuffers ( NULL ),
    //    mReplyBuffers ( NULL ),
    mAsynchronousException ( NULL )
  {
    mDeadlineTimer.async_wait ( boost::bind ( &UDP::CheckDeadline, this ) );
  }


  template < typename InnerProtocol >
  UDP< InnerProtocol >::~UDP()
  {
    try
    {
      mSocket.close();

      while ( mSocket.is_open() )
        {}

      mIOservice.stop();
      mDispatchThread.join();
      boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );
      ClientInterface::returnBufferToPool ( mDispatchQueue );
      ClientInterface::returnBufferToPool ( mReplyQueue );
    }
    catch ( const std::exception& aExc )
    {
      log ( Error() , "Exception " , Quote ( aExc.what() ) , " caught at " , ThisLocation() );
    }
  }



  template < typename InnerProtocol >
  void UDP< InnerProtocol >::implementDispatch ( boost::shared_ptr< Buffers > aBuffers )
  {
    boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );

    if ( mAsynchronousException )
    {
      log ( *mAsynchronousException , "Rethrowing Asynchronous Exception from " , ThisLocation() );
      mAsynchronousException->ThrowAsDerivedType();
    }

    if ( ! mSocket.is_open() )
    {
      connect();
    }


    if ( mDispatchBuffers || mPacketsInFlight == this->getMaxNumberOfBuffers() )
    {
      mDispatchQueue.push_back ( aBuffers );
      //   std::cout << "extended mDispatchQueue" << std::endl;
    }
    else
    {
      mDispatchBuffers = aBuffers;
      write ( );
    }
  }


  template < typename InnerProtocol >
  uint32_t UDP< InnerProtocol >::getMaxSendSize()
  {
    return (350 * 4);
  }


  template < typename InnerProtocol >
  uint32_t UDP< InnerProtocol >::getMaxReplySize()
  {
    return (350 * 4);
  }


  template < typename InnerProtocol >
  void UDP< InnerProtocol >::connect()
  {
    log ( Info() , "Creating new UDP socket for device " , Quote ( this->uri() ) , ", as it appears to have been closed..." );
    //mSocket = boost::asio::ip::udp::socket ( mIOservice , boost::asio::ip::udp::endpoint ( boost::asio::ip::udp::v4(), 0 ) );
    mSocket.open ( boost::asio::ip::udp::v4() );
    //    boost::asio::socket_base::non_blocking_io lNonBlocking ( true );
    //    mSocket.io_control ( lNonBlocking );
    log ( Info() , "UDP socket created successfully." );
  }


  template < typename InnerProtocol >
  void UDP< InnerProtocol >::write ( )
  {
    NotifyConditionalVariable ( false );

    if ( !mDispatchBuffers )
    {
      log ( Error() , __PRETTY_FUNCTION__ , " called when 'mDispatchBuffers' was NULL" );
      return;
    }

    /*    std::vector<uint32_t>::const_iterator lBegin ( reinterpret_cast<uint32_t*> ( aBuffers->getSendBuffer() ) );
        std::vector<uint32_t>::const_iterator lEnd = lBegin + ( aBuffers->sendCounter() >>2 );
        std::vector<uint32_t> lData;

        for ( ; lBegin!=lEnd ; ++lBegin )
        {
          std::cout << std::setfill ( '0' ) << std::hex << std::setw ( 8 ) <<  *lBegin << std::endl;
        }*/
    //     mAsioSendBuffer.clear();
    std::vector< boost::asio::const_buffer > lAsioSendBuffer;
    lAsioSendBuffer.push_back ( boost::asio::const_buffer ( mDispatchBuffers->getSendBuffer() , mDispatchBuffers->sendCounter() ) );
    log ( Debug() , "Sending " , Integer ( mDispatchBuffers->sendCounter() ) , " bytes" );
    mDeadlineTimer.expires_from_now ( this->getBoostTimeoutPeriod() );

    // Patch for suspected bug in using boost asio with boost python; see https://svnweb.cern.ch/trac/cactus/ticket/323#comment:7
    while ( mDeadlineTimer.expires_from_now() < boost::posix_time::microseconds ( 600 ) )
    {
      log ( Debug() , "Resetting deadline timer since it just got set to strange value, likely due to a bug within boost (expires_from_now was: ", mDeadlineTimer.expires_from_now() , ")." );
      mDeadlineTimer.expires_from_now ( this->getBoostTimeoutPeriod() );
    }

    mSocket.async_send_to ( lAsioSendBuffer , mEndpoint , boost::bind ( &UDP< InnerProtocol >::write_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
    mPacketsInFlight++;
  }


  template < typename InnerProtocol >
  void UDP< InnerProtocol >::write_callback ( const boost::system::error_code& aErrorCode , std::size_t aBytesTransferred )
  {
    //     if( !mDispatchBuffers)
    //     {
    //       log( Error() , __PRETTY_FUNCTION__ , " called when 'mDispatchBuffers' was NULL" );
    //       return;
    //     }
    boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );

    if ( mAsynchronousException )
    {
      NotifyConditionalVariable ( true );
      return;
    }

    if ( mDeadlineTimer.expires_at () == boost::posix_time::pos_infin )
    {
      exception::UdpTimeout* lExc = new exception::UdpTimeout();
      log ( *lExc , "Timeout (" , Integer ( this->getBoostTimeoutPeriod().total_milliseconds() ) , " milliseconds) occurred for UDP send to target with URI: ", this->uri() );

      if ( aErrorCode && aErrorCode != boost::asio::error::operation_aborted )
      {
        log ( *lExc , "ASIO reported an error: " , Quote ( aErrorCode.message() ) );
      }

      mAsynchronousException = lExc;
      NotifyConditionalVariable ( true );
      return;
    }

    if ( ( aErrorCode && ( aErrorCode != boost::asio::error::eof ) ) || ( aBytesTransferred != mDispatchBuffers->sendCounter() ) )
    {
      mSocket.close();
      exception::ASIOUdpError* lExc = new exception::ASIOUdpError();
      if ( aErrorCode )
      {
        log ( *lExc , "Error ", Quote ( aErrorCode.message() ) , " encountered during send to UDP target with URI: " , this->uri() );
      }
      if ( aBytesTransferred != mDispatchBuffers->sendCounter() )
      {
        log ( *lExc , "Only ", Integer ( aBytesTransferred ) , " of " , Integer ( mDispatchBuffers->sendCounter() ) , " bytes transferred in UDP send to URI: " , this->uri() );
      }
      mAsynchronousException = lExc;
      NotifyConditionalVariable ( true );
      return;
    }

    if ( mReplyBuffers )
    {
      mReplyQueue.push_back ( mDispatchBuffers );
      //   std::cout << "extended mReplyQueue" << std::endl;
    }
    else
    {
      mReplyBuffers = mDispatchBuffers;
      read ( );
    }

    if ( mDispatchQueue.size() && mPacketsInFlight != this->getMaxNumberOfBuffers() )
    {
      mDispatchBuffers = mDispatchQueue.front();
      mDispatchQueue.pop_front();
      //std::cout << "reduced mDispatchQueue" << std::endl;
      write();
    }
    else
    {
      mDispatchBuffers.reset();
    }
  }


  template < typename InnerProtocol >
  void UDP< InnerProtocol >::read ( )
  {
    if ( !mReplyBuffers )
    {
      log ( Error() , __PRETTY_FUNCTION__ , " called when 'mReplyBuffers' was NULL" );
      NotifyConditionalVariable ( true );
      return;
    }

    std::vector<boost::asio::mutable_buffer> lAsioReplyBuffer ( 1 , boost::asio::mutable_buffer ( & ( mReplyMemory.at ( 0 ) ) , mReplyBuffers->replyCounter() ) );
    log ( Debug() , "Expecting " , Integer ( mReplyBuffers->replyCounter() ) , " bytes in reply." );
    mDeadlineTimer.expires_from_now ( this->getBoostTimeoutPeriod() );

    // Patch for suspected bug in using boost asio with boost python; see https://svnweb.cern.ch/trac/cactus/ticket/323#comment:7
    while ( mDeadlineTimer.expires_from_now() < boost::posix_time::microseconds ( 600 ) )
    {
      log ( Debug() , "Resetting deadline timer since it just got set to strange value, likely due to a bug within boost (expires_from_now was: ", mDeadlineTimer.expires_from_now() , ")." );
      mDeadlineTimer.expires_from_now ( this->getBoostTimeoutPeriod() );
    }

    mSocket.async_receive ( lAsioReplyBuffer , 0 , boost::bind ( &UDP<InnerProtocol>::read_callback, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred ) );
  }


  template < typename InnerProtocol >
  void UDP< InnerProtocol >::read_callback ( const boost::system::error_code& aErrorCode , std::size_t aBytesTransferred )
  {
    {
      boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );
      if ( mAsynchronousException )
      {
        NotifyConditionalVariable ( true );
        return;
      }

      if ( mDeadlineTimer.expires_at () == boost::posix_time::pos_infin )
      {
        mAsynchronousException = new exception::UdpTimeout();
        log ( *mAsynchronousException , "Timeout (" , Integer ( this->getBoostTimeoutPeriod().total_milliseconds() ) , " milliseconds) occurred for UDP receive from target with URI: ", this->uri() );

        if ( aErrorCode && aErrorCode != boost::asio::error::operation_aborted )
        {
          log ( *mAsynchronousException , "ASIO reported an error: " , Quote ( aErrorCode.message() ) );
        }

        NotifyConditionalVariable ( true );
        return;
      }
    }

    if ( !mReplyBuffers )
    {
      log ( Error() , __PRETTY_FUNCTION__ , " called when 'mReplyBuffers' was NULL" );
      return;
    }

    if ( aBytesTransferred != mReplyBuffers->replyCounter() )
    {
      log ( Error() , "Expected " , Integer ( mReplyBuffers->replyCounter() ) , "-byte UDP payload from target " , Quote ( this->uri() ) , ", but only received " , Integer ( aBytesTransferred ) , " bytes. Validating returned data to work out where error occurred." );
    }

    if ( aErrorCode && ( aErrorCode != boost::asio::error::eof ) )
    {
      mSocket.close();

      boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );
      mAsynchronousException = new exception::ASIOUdpError();
      log ( *mAsynchronousException , "Error ", Quote ( aErrorCode.message() ) , " encountered during receive from UDP target with URI: " , this->uri() );

      NotifyConditionalVariable ( true );
      return;
    }

    //     uint32_t lCounter(0);
    //     for ( std::vector< boost::asio::mutable_buffer >::iterator lIt = lAsioReplyBuffer.begin() ; lIt != lAsioReplyBuffer.end() ; ++lIt )
    //     {
    //     uint32_t s1 = boost::asio::buffer_size(*lIt)>>2;
    //     uint32_t* p1 = boost::asio::buffer_cast<uint32_t*>(*lIt);
    //
    //     for( uint32_t i(0) ; i!= s1 ; ++i , ++p1 )
    //     {
    //     log ( Debug() , Integer ( lCounter++ ) , " : " , Integer ( *p1 , IntFmt<hex,fixed>() ) );
    //     }
    //     }
    //         TargetToHostInspector< 2 , 0 > lT2HInspector;
    //         std::vector<uint32_t>::const_iterator lBegin2 ( ( uint32_t* ) ( & mReplyMemory[0] ) );
    //         std::vector<uint32_t>::const_iterator lEnd2 ( ( uint32_t* ) ( & mReplyMemory[aBuffers->replyCounter() ] ) );
    //         lT2HInspector.analyze ( lBegin2 , lEnd2 );
    //  std::cout << "Filling reply buffer : " << mReplyBuffers << std::endl;
    std::deque< std::pair< uint8_t* , uint32_t > >& lReplyBuffers ( mReplyBuffers->getReplyBuffer() );
    uint8_t* lReplyBuf ( & ( mReplyMemory.at ( 0 ) ) );

    for ( std::deque< std::pair< uint8_t* , uint32_t > >::iterator lIt = lReplyBuffers.begin() ; lIt != lReplyBuffers.end() ; ++lIt )
    {
      // Don't copy more of mReplyMemory than was written to, for cases when less data received than expected
      if ( static_cast<uint32_t> ( lReplyBuf - ( & mReplyMemory.at ( 0 ) ) ) >= aBytesTransferred )
      {
        break;
      }

      uint32_t lNrBytesToCopy = std::min ( lIt->second , static_cast<uint32_t> ( aBytesTransferred - ( lReplyBuf - ( & mReplyMemory.at ( 0 ) ) ) ) );
      memcpy ( lIt->first, lReplyBuf, lNrBytesToCopy );
      lReplyBuf += lNrBytesToCopy;
    }

    try
    {
      if ( uhal::exception::exception* lExc = ClientInterface::validate ( mReplyBuffers ) ) //Control of the pointer has been passed back to the client interface
      {
        boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );
        mAsynchronousException = lExc;
      }
    }
    catch ( exception::exception& aExc )
    {
      boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );
      mAsynchronousException = new exception::ValidationError ();
      log ( *mAsynchronousException , "Exception caught during reply validation for UDP device with URI " , Quote ( this->uri() ) , "; what returned: " , Quote ( aExc.what() ) );
    }

    boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );

    if ( mAsynchronousException )
    {
      NotifyConditionalVariable ( true );
      return;
    }

    if ( mReplyQueue.size() )
    {
      mReplyBuffers = mReplyQueue.front();
      mReplyQueue.pop_front();
      // std::cout << "reduced mReplyQueue" << std::endl;
      read();
    }
    else
    {
      mReplyBuffers.reset();
    }

    mPacketsInFlight--;

    if ( !mDispatchBuffers && mDispatchQueue.size() && mPacketsInFlight != this->getMaxNumberOfBuffers() )
    {
      mDispatchBuffers = mDispatchQueue.front();
      mDispatchQueue.pop_front();
      //std::cout << "reduced mDispatchQueue" << std::endl;
      write();
    }

    if ( !mDispatchBuffers && !mReplyBuffers )
    {
      mDeadlineTimer.expires_from_now( boost::posix_time::seconds(60) );
      NotifyConditionalVariable ( true );
    }
  }



  template < typename InnerProtocol >
  void UDP< InnerProtocol >::CheckDeadline()
  {
    // Check whether the deadline has passed. We compare the deadline against
    // the current time since a new asynchronous operation may have moved the
    // deadline before this actor had a chance to run.
    boost::lock_guard<boost::mutex> lLock ( this->mTransportLayerMutex );

    if ( mDeadlineTimer.expires_at() <= boost::asio::deadline_timer::traits_type::now() )
    {
      // SETTING THE EXCEPTION HERE CAN APPEAR AS A TIMEOUT WHEN NONE ACTUALLY EXISTS
      if (  mDispatchBuffers || mReplyBuffers )
      {
        log ( Warning() , "Closing UDP socket for URI " , Quote ( this->uri() ) , " since deadline has passed" );
      }
      else
      {
        log ( Debug() , "Closing UDP socket for URI " , Quote ( this->uri() ) , " since no communication in last 60 seconds" );
      }

      // The deadline has passed. The socket is closed so that any outstanding
      // asynchronous operations are cancelled.
      mSocket.close();
      // There is no longer an active deadline. The expiry is set to positive
      // infinity so that the actor takes no action until a new deadline is set.
      mDeadlineTimer.expires_at ( boost::posix_time::pos_infin );
      // log ( Error() , "ASIO deadline timer timed out" );
    }

    // Put the actor back to sleep.
    mDeadlineTimer.async_wait ( boost::bind ( &UDP::CheckDeadline, this ) );
  }


  template < typename InnerProtocol >
  void UDP< InnerProtocol >::Flush( )
  {
    WaitOnConditionalVariable();

    boost::lock_guard<boost::mutex> lLock ( mTransportLayerMutex );
    if ( mAsynchronousException )
    {
      mAsynchronousException->ThrowAsDerivedType();
    }
  }



  template < typename InnerProtocol >
  void UDP< InnerProtocol >::dispatchExceptionHandler()
  {
    log ( Warning() , "Closing Socket since exception detected." );

    if ( mSocket.is_open() )
    {
      try
      {
        mSocket.close();

        while ( mSocket.is_open() )
          {}
      }
      catch ( ... )
        {}
    }

    NotifyConditionalVariable ( true );

    if ( mAsynchronousException )
    {
      delete mAsynchronousException;
      mAsynchronousException = NULL;
    }

    ClientInterface::returnBufferToPool ( mDispatchQueue );
    ClientInterface::returnBufferToPool ( mReplyQueue );
    mPacketsInFlight = 0;

    ClientInterface::returnBufferToPool ( mDispatchBuffers );
    mDispatchBuffers.reset();
    ClientInterface::returnBufferToPool ( mReplyBuffers );
    mReplyBuffers.reset();

    InnerProtocol::dispatchExceptionHandler();
  }

  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  //--------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


  template < typename InnerProtocol  >
  void UDP< InnerProtocol >::NotifyConditionalVariable ( const bool& aValue )
  {
    {
      boost::lock_guard<boost::mutex> lLock ( mConditionalVariableMutex );
      mFlushDone = aValue;
    }
    mConditionalVariable.notify_one();
  }


  template < typename InnerProtocol  >
  void UDP< InnerProtocol >::WaitOnConditionalVariable()
  {
    boost::unique_lock<boost::mutex> lLock ( mConditionalVariableMutex );

    while ( !mFlushDone )
    {
      mConditionalVariable.wait ( lLock );
    }
  }


  template class UDP< IPbus< 1 , 3 > >;
  template class UDP< IPbus< 2 , 0 > >;
}

