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
#include <boost/filesystem.hpp>

#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <cstdlib>
#include <typeinfo>


namespace uhal {
namespace tests {

BOOST_AUTO_TEST_SUITE( SingleReadWriteTestSuite )


BOOST_FIXTURE_TEST_CASE(connect_write_read, TestFixture)
{
  ConnectionManager manager ( sConnectionFile );
  HwInterface hw=manager.getDevice ( sDeviceId );
  // hw.ping();
  uint32_t x1 = static_cast<uint32_t> ( rand() );
  uint32_t x2 = static_cast<uint32_t> ( rand() );
  hw.getNode ( "SUBSYSTEM1.REG" ).write ( x1 );
  hw.getNode ( "SUBSYSTEM2.REG" ).write ( x2 );
  ValWord< uint32_t > mem1 = hw.getNode ( "SUBSYSTEM1.REG" ).read();
  ValWord< uint32_t > mem2 = hw.getNode ( "SUBSYSTEM2.REG" ).read();
  BOOST_CHECK ( !mem1.valid() );
  BOOST_CHECK ( !mem2.valid() );
  BOOST_CHECK_THROW ( mem1.value(), uhal::exception::NonValidatedMemory );
  BOOST_CHECK_THROW ( mem2.value(), uhal::exception::NonValidatedMemory );
  BOOST_CHECK_NO_THROW ( hw.dispatch() );
  BOOST_CHECK ( mem1.valid() );
  BOOST_CHECK_EQUAL ( mem1.value(), x1 );
  BOOST_CHECK_EQUAL ( mem2.value(), x2 );
}


BOOST_FIXTURE_TEST_CASE(on_the_fly_connect_write_read, TestFixture)
{
  //get location of address file. Assumption: it is located with the connection file
  std::string address_file;
  {
    boost::filesystem::path conn_fn ( sConnectionFile );
    boost::filesystem::path fn ( "dummy_address.xml" );
    address_file = ( conn_fn.parent_path() /fn ).string();
  }
  //get the parameters from the file
  std::string uri;
  {
    ConnectionManager manager ( sConnectionFile );
    uri = manager.getDevice ( sDeviceId ).uri();
  }
  HwInterface hw=ConnectionManager::getDevice ( "test_device_id", uri, address_file );
  uint32_t x = static_cast<uint32_t> ( rand() );
  hw.getNode ( "REG" ).write ( x );
  ValWord< uint32_t > mem = hw.getNode ( "REG" ).read();
  BOOST_CHECK ( !mem.valid() );
  BOOST_CHECK_THROW ( mem.value(),uhal::exception::NonValidatedMemory );
  BOOST_CHECK_NO_THROW ( hw.dispatch() );
  BOOST_CHECK ( mem.valid() );
  BOOST_CHECK_EQUAL ( mem.value(), x );
}


BOOST_AUTO_TEST_CASE(search_device_id)
{
  ConnectionManager manager (TestFixture::sConnectionFile);
  std::vector<std::string> ids = manager.getDevices ( "^" + TestFixture::sDeviceId + "$" );
  BOOST_CHECK ( std::find ( ids.begin(),ids.end(),TestFixture::sDeviceId ) != ids.end() );
}


BOOST_AUTO_TEST_SUITE_END()

} // end ns tests
} // end ns uhal

