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

      Marc Magrans de Abril, CERN
      email: marc.magrans.de.abril <AT> cern.ch

      Andrew Rose, Imperial College, London
      email: awr01 <AT> imperial.ac.uk

      Tom Williams, Rutherford Appleton Laboratory, Oxfordshire
      email: tom.williams <AT> cern.ch

---------------------------------------------------------------------------
*/

#include "uhal/uhal.hpp"

#include "uhal/tests/tools.hpp"

#include <boost/test/unit_test.hpp>

#include <iostream>
#include <cstdlib>
#include <typeinfo>


namespace uhal {
namespace tests {

BOOST_AUTO_TEST_SUITE(NodeMetainfoTestSuite)


BOOST_AUTO_TEST_CASE(check_meta_info)
{
  ConnectionManager manager ( TestFixture::sConnectionFile );
  HwInterface hw=manager.getDevice ( TestFixture::sDeviceId );

  boost::unordered_map<std::string,std::string> lPars;
  boost::unordered_map<std::string,std::string>::iterator iPar;
  boost::unordered_map<std::string,std::string> lFwInfo;
  boost::unordered_map<std::string,std::string>::iterator iFwInfo;

  //REG
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getAddress(), 0x000001 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getId(), "REG" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG" ).getTags(), "test" );
  BOOST_CHECK ( hw.getNode ( "REG" ).getParameters().empty() );
  //REG_READ_ONLY
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getAddress(), 0x0002 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getId(), "REG_READ_ONLY" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getPermission(), uhal::defs::READ );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_READ_ONLY" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "REG_READ_ONLY" ).getParameters().empty() );
  //REG_WRITE_ONLY
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getAddress(), 0x0003 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getId(), "REG_WRITE_ONLY" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getPermission(), uhal::defs::WRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_WRITE_ONLY" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "REG_WRITE_ONLY" ).getParameters().empty() );
  //REG_UPPER_MASK
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getAddress(), 0x0004 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getId(), "REG_UPPER_MASK" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getMask(), 0xFFFF0000 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_UPPER_MASK" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "REG_UPPER_MASK" ).getParameters().empty() );
  //REG_LOWER_MASK 
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getAddress(), 0x0004 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getId(), "REG_LOWER_MASK" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getMask(), 0x0000FFFF );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_LOWER_MASK" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "REG_LOWER_MASK" ).getParameters().empty() );
  //REG_MASKED_READ_ONLY 
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getAddress(), 0x0005 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getId(), "REG_MASKED_READ_ONLY" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getPermission(), uhal::defs::READ );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getMask(), 0xFFFF0000 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "REG_MASKED_READ_ONLY" ).getParameters().empty() );
  //REG_MASKED_WRITE_ONLY 
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getAddress(), 0x0005 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getId(), "REG_MASKED_WRITE_ONLY" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getPermission(), uhal::defs::WRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getMask(), 0x0000FFFF );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "REG_MASKED_WRITE_ONLY" ).getParameters().empty() );
  //REG_WITH_PARS
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getAddress(), 0x0006 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getId(), "REG_PARS" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getTags(), "" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "REG_PARS" ).getParameters().size(), 2 );
  lPars = hw.getNode("REG_PARS").getParameters();
  BOOST_CHECK ( ( iPar = lPars.find("arg0") ) != lPars.end() && iPar->second == "val100" );
  BOOST_CHECK ( ( iPar = lPars.find("arg1") ) != lPars.end() && iPar->second == "val101" );
  //SUBSYSTEM1.REG
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getAddress(), 0x210002 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getId(), "REG" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.REG" ).getTags(), "test" );
  BOOST_CHECK ( hw.getNode ( "SUBSYSTEM1.REG" ).getParameters().empty() );
  //SUBSYSTEM2.REG
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getAddress(), 0x310002 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getId(), "REG" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getSize(), 1 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getMode(), uhal::defs::SINGLE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.REG" ).getTags(), "test" );
  BOOST_CHECK ( hw.getNode ( "SUBSYSTEM2.REG" ).getParameters().empty() );
  //FIFO
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getAddress(), 0x0100 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getId(), "FIFO" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getSize(), 1024*1024*1024/4 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getMode(), uhal::defs::NON_INCREMENTAL );
  BOOST_CHECK_EQUAL ( hw.getNode ( "FIFO" ).getTags(), "test" );
  BOOST_CHECK ( hw.getNode ( "FIFO" ).getParameters().empty() );
  //MEM
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getAddress(), 0x100000 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getId(), "MEM" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getSize(), 1024*1024/4 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getMode(), uhal::defs::INCREMENTAL );
  BOOST_CHECK_EQUAL ( hw.getNode ( "MEM" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "MEM" ).getParameters().empty() );
  //SMALL_MEM
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getAddress(), 0x400000 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getId(), "SMALL_MEM" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getSize(), 256 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getMode(), uhal::defs::INCREMENTAL );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SMALL_MEM" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "SMALL_MEM" ).getParameters().empty() );
  //LARGE_MEM
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getAddress(), 0x500000 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getId(), "LARGE_MEM" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getSize(), 100*1024*1024/4 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getMode(), uhal::defs::INCREMENTAL );
  BOOST_CHECK_EQUAL ( hw.getNode ( "LARGE_MEM" ).getTags(), "" );
  BOOST_CHECK ( hw.getNode ( "LARGE_MEM" ).getParameters().empty() );
  //SUBSYSTEM1.MEM
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getAddress(), 0x210003 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getId(), "MEM" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getSize(), 1024*1024/4 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getMode(), uhal::defs::INCREMENTAL );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM1.MEM" ).getTags(), "test" );
  BOOST_CHECK ( hw.getNode ( "SUBSYSTEM1.MEM" ).getParameters().empty() );
  //SUBSYSTEM2.MEM
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getAddress(), 0x310003 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getId(), "MEM" );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getPermission(), uhal::defs::READWRITE );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getSize(), 1024*1024/4 );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getMask(), uhal::defs::NOMASK );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getMode(), uhal::defs::INCREMENTAL );
  BOOST_CHECK_EQUAL ( hw.getNode ( "SUBSYSTEM2.MEM" ).getTags(), "test" );
  BOOST_CHECK ( hw.getNode ( "SUBSYSTEM2.MEM" ).getParameters().empty() );
  //SUBSYSTEM1
  lPars = hw.getNode("SUBSYSTEM1").getParameters();
  BOOST_CHECK( ( iPar = lPars.find( "arg0" ) ) != lPars.end() && iPar->second == "val200" );
  BOOST_CHECK( ( iPar = lPars.find( "arg1" ) ) != lPars.end() && iPar->second == "val201" );
  BOOST_CHECK( ( iPar = lPars.find( "arg2" ) ) != lPars.end() && iPar->second == "val202" );
  BOOST_CHECK_EQUAL( lPars.size(), 3 );
  //SUBSYSTEM2
  lPars = hw.getNode("SUBSYSTEM2").getParameters();
  BOOST_CHECK( ( iPar = lPars.find( "arg0" ) ) != lPars.end() && iPar->second == "val10000" );
  BOOST_CHECK( ( iPar = lPars.find( "arg1" ) ) != lPars.end() && iPar->second == "val201" );
  BOOST_CHECK( ( iPar = lPars.find( "arg2" ) ) != lPars.end() && iPar->second == "val202" );
  BOOST_CHECK( ( iPar = lPars.find( "arg5" ) ) != lPars.end() && iPar->second == "val10005" );
  BOOST_CHECK_EQUAL( lPars.size(), 4 );
  //SUBSYSTEM2.SUBMODULE
  lPars = hw.getNode("SUBSYSTEM1.SUBMODULE").getParameters();
  BOOST_CHECK( ( iPar = lPars.find( "arg0" ) ) != lPars.end() && iPar->second == "val300" );
  BOOST_CHECK( ( iPar = lPars.find( "arg1" ) ) != lPars.end() && iPar->second == "val301" );
  BOOST_CHECK( ( iPar = lPars.find( "arg2" ) ) != lPars.end() && iPar->second == "val10302" );
  BOOST_CHECK( ( iPar = lPars.find( "arg3" ) ) != lPars.end() && iPar->second == "val10303" );
  BOOST_CHECK_EQUAL( lPars.size(), 4 );
  
  lFwInfo = hw.getNode("IPBUS_ENDPOINT").getFirmwareInfo();
  
  BOOST_CHECK( ( iFwInfo = lFwInfo.find( "type" ) ) != lFwInfo.end() && iFwInfo->second == "endpoint" );  
  BOOST_CHECK( ( iFwInfo = lFwInfo.find( "width" ) ) != lFwInfo.end() && iFwInfo->second == "0x10" );
  
}


BOOST_AUTO_TEST_SUITE_END()

} // end ns tests
} // end ns uhal
