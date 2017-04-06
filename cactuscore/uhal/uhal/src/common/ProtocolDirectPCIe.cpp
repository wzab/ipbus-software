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

#include "uhal/ProtocolDirectPCIe.hpp"

#include "boost/date_time/posix_time/posix_time.hpp"

#include "uhal/Buffers.hpp"
#include "uhal/log/log.hpp"



namespace uhal
{


  DirectPCIe::DirectPCIe ( const std::string& aId, const URI& aUri) :
    ClientInterface ( aId , aUri , boost::posix_time::milliseconds(1000) )
    // mTransactionCounter ( 0x00000000 ),
    // mMaxSendSize ( aMaxSendSize<<2 ),
    // mMaxReplySize ( aMaxReplySize<<2 )
  {

  }


  DirectPCIe::~DirectPCIe()
  {}





  // // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // // NOTE! THIS FUNCTION MUST BE THREAD SAFE: THAT IS:
  // // IT MUST ONLY USE LOCAL VARIABLES
  // //            --- OR ---
  // // IT MUST MUTEX PROTECT ACCESS TO MEMBER VARIABLES!
  // // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  // bool IPbusCore::Validate ( boost::shared_ptr< Buffers > lBuffers )
  // {
  //
  // bool lRet = Validate ( lBuffers->getSendBuffer() ,
  // lBuffers->getSendBuffer() +lBuffers->sendCounter() ,
  // lBuffers->getReplyBuffer().begin() ,
  // lBuffers->getReplyBuffer().end() );

  // if ( lRet )
  // {
  // lBuffers->validate ( boost::shared_ptr< Buffers > lBuffers );
  // delete lBuffers; //We have now checked the returned data and marked as valid the underlying memory. We can, therefore, delete the local storage and from this point onward, the validated memory will only exist if the user kept their own copy
  // }

  // return lRet;
  // }

  // ----------------------------------------------------------------------------------------------------------------------------------------------------------------
  ValHeader DirectPCIe::implementBOT()
  {
    /* Add byte-order transaction implementation if applicable */
    return ValHeader();
  }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  ValHeader DirectPCIe::implementWrite ( const uint32_t& aAddr, const uint32_t& aSource )
  {
    /* add single-word write implementation here */
    return ValHeader();
  }

  ValHeader DirectPCIe::implementWriteBlock ( const uint32_t& aAddr, const std::vector< uint32_t >& aSource, const defs::BlockReadWriteMode& aMode )
  {
    /* add block write implementation here */
    return ValHeader();
  }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------

  ValWord< uint32_t > DirectPCIe::implementRead ( const uint32_t& aAddr, const uint32_t& aMask )
  {
    /* add single-word read implementation here */
    return ValWord<uint32_t>(0x0, true);
  }

  ValVector< uint32_t > DirectPCIe::implementReadBlock ( const uint32_t& aAddr, const uint32_t& aSize, const defs::BlockReadWriteMode& aMode )
  {
    /* add block read implementation here */
    return ValVector<uint32_t>();
  }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  ValWord< uint32_t > DirectPCIe::implementRMWbits ( const uint32_t& aAddr , const uint32_t& aANDterm , const uint32_t& aORterm )
  {
    /* add RMW sum implementation here */
    return ValWord<uint32_t>(0x0, true);
  }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------


  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------
  ValWord< uint32_t > DirectPCIe::implementRMWsum ( const uint32_t& aAddr , const int32_t& aAddend )
  {
     /* add RMW sum implementation here */
    return ValWord<uint32_t>(0x0, true);
  }
  //-------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------



}


