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

#include "uhal/tests/PerfTester.hxx"
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


void report_rx_performance(ClientInterface& aClient, const uint32_t aBaseAddr, const uint32_t aDepth, const size_t aNrIterations, const bool aDispatchEachIteration)
{
  std::vector<ClientInterface*> lClients;
  lClients.push_back(&aClient);

  double totalSeconds = measureRxPerformance(lClients, aBaseAddr, aDepth, aNrIterations, aDispatchEachIteration, NULL);
  double totalPayloadKB = aNrIterations * aDepth * 4. / 1024.;
  double dataRateKB_s = totalPayloadKB/totalSeconds;
  std::cout << " --> " << aNrIterations << " reads, each " << aDepth << " x 32-bit words, took " << totalSeconds << " seconds" << std::endl
            << "Total IPbus payload received    = " << totalPayloadKB << " KB\n"
            << "Average read bandwidth          = " << dataRateKB_s << " KB/s" << std::endl;
}


void report_tx_performance(ClientInterface& aClient, const uint32_t aBaseAddr, const uint32_t aDepth, const size_t aNrIterations, const bool aDispatchEachIteration)
{
  std::vector<ClientInterface*> lClients;
  lClients.push_back(&aClient);

  double totalSeconds = measureTxPerformance(lClients, aBaseAddr, aDepth, aNrIterations, aDispatchEachIteration, NULL);
  double totalPayloadKB = aNrIterations * aDepth * 4. / 1024.;
  double dataRateKB_s = totalPayloadKB/totalSeconds;
  std::cout << " --> " << aNrIterations << " writes, each " << aDepth << " x 32-bit words, took " << totalSeconds << " seconds" << std::endl
            << "Total IPbus payload received    = " << totalPayloadKB << " KB\n"
            << "Average read bandwidth          = " << dataRateKB_s << " KB/s" << std::endl;

}


BOOST_AUTO_TEST_SUITE( SoakTestSuite )


BOOST_FIXTURE_TEST_CASE(bandwidth_rx, TestFixture)
{
  ConnectionManager manager ( sConnectionFile );
  HwInterface hw=manager.getDevice ( sDeviceId );

  BOOST_CHECK_NO_THROW( report_rx_performance(hw.getClient(), 0x01, 1, 100, true) );
  BOOST_CHECK_NO_THROW( report_rx_performance(hw.getClient(), 0x01, 262144, 100, true) );
}


BOOST_FIXTURE_TEST_CASE(bandwidth_tx, TestFixture)
{
  ConnectionManager manager ( sConnectionFile );
  HwInterface hw=manager.getDevice ( sDeviceId );

  BOOST_CHECK_NO_THROW( report_tx_performance(hw.getClient(), 0x01, 1, 100, true) );
  BOOST_CHECK_NO_THROW( report_tx_performance(hw.getClient(), 0x01, 262144, 100, true) );
}


BOOST_FIXTURE_TEST_CASE(quick_soak, TestFixture)
{
  ConnectionManager manager ( sConnectionFile );
  HwInterface hw=manager.getDevice ( sDeviceId );

  std::vector<ClientInterface*> lClients;
  lClients.push_back(&hw.getClient());

  BOOST_CHECK( PerfTester::runValidationTest(lClients, 0x1000, 1024, 2000, false, false) );
}


BOOST_AUTO_TEST_SUITE_END()

} // end ns tests
} // end ns uhal

