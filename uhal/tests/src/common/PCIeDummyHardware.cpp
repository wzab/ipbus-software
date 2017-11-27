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

#include "uhal/tests/PCIeDummyHardware.hpp"


#include <errno.h>
#include <fcntl.h>
#include <pty.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>


namespace uhal {
namespace tests {

PCIeDummyHardware::PCIeDummyHardware(const std::string& aDevicePathHostToFPGA, const std::string& aDevicePathFPGAToHost, const uint32_t& aReplyDelay, const bool& aBigEndianHack) :
  DummyHardware<2, 0>(aReplyDelay, aBigEndianHack),
  mDevicePathHostToFPGA(aDevicePathHostToFPGA),
  mDevicePathFPGAToHost(aDevicePathFPGAToHost),
  mNumberOfPages(1),
  mWordsPerPage(360),
  mNextPageIndex(0),
  mPublishedPageCount(0),
  mStop(false),
  mDeviceFileHostToFPGA(-1),
  mDeviceFileFPGAToHost(-1)
{
  log(Debug(), "PCIe dummy hardware is creating client-to-device named PIPE ", Quote (mDevicePathHostToFPGA));
  int rc = mkfifo(mDevicePathHostToFPGA.c_str(), 0666);
  if ( rc != 0 ) {
    std::runtime_error lExc("Cannot create FIFO; mkfifo returned " + boost::lexical_cast<std::string>(rc) + ", errno=" + boost::lexical_cast<std::string>(errno));
    throw lExc;
  }

  mDeviceFileHostToFPGA = open(mDevicePathHostToFPGA.c_str(), O_RDONLY | O_NONBLOCK); /* O_NONBLOCK so that open does not hang */
  if ( mDeviceFileHostToFPGA < 0 ) {
    std::runtime_error lExc("Problem opening host-to-FPGA device file '" + mDevicePathHostToFPGA + "', errno=" + boost::lexical_cast<std::string>(errno) + " (dummy hw)");
    throw lExc;
  }

  // Set the new flags with O_NONBLOCK masked out
  int lFileFlags = fcntl(mDeviceFileHostToFPGA, F_GETFL);
  fcntl(mDeviceFileHostToFPGA, F_SETFL, lFileFlags & ~O_NONBLOCK);

  log(Debug(), "PCIe dummy hardware is creating device-to-client file ", Quote (mDevicePathFPGAToHost));
  mDeviceFileFPGAToHost = open(mDevicePathFPGAToHost.c_str(), O_RDWR | O_CREAT | O_DIRECTORY, 0666 /* permission */);
  if ( mDeviceFileFPGAToHost < 0 ) {
    std::runtime_error lExc("Cannot open FPGA-to-host device file '" + mDevicePathFPGAToHost + "' (dummy hw)");
    throw lExc;
  }

  std::vector<uint32_t> lFPGAToHostData(4 + mNumberOfPages * mWordsPerPage, 0);
  lFPGAToHostData.at(0) = mNumberOfPages;
  lFPGAToHostData.at(1) = mWordsPerPage;
  lFPGAToHostData.at(2) = mNumberOfPages;
  lFPGAToHostData.at(2) = mNumberOfPages;

  dmaWrite(mDeviceFileFPGAToHost, 0, lFPGAToHostData);

  log(Notice(), "Starting IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost), "; ", Integer(mNumberOfPages), " pages, ", Integer(mWordsPerPage), " words per page");
}


PCIeDummyHardware::~PCIeDummyHardware()
{
  log(Notice(), "Destroying IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost));

  if (close(mDeviceFileHostToFPGA))
    log(Fatal(), "Problem occurred when closing ", Quote(mDevicePathHostToFPGA), " during dummy hardware destruction");
  if (remove(mDevicePathHostToFPGA.c_str()))
    log(Fatal(), "Problem occurred when removing ", Quote(mDevicePathHostToFPGA), " during dummy hardware destruction");

  if (close(mDeviceFileFPGAToHost))
    log(Fatal(), "Problem occurred when closing ", Quote(mDevicePathFPGAToHost), " during dummy hardware destruction");
  if (remove(mDevicePathFPGAToHost.c_str()))
    log(Fatal(), "Problem occurred when removing ", Quote(mDevicePathFPGAToHost), " during dummy hardware destruction");
}


void PCIeDummyHardware::run()
{
  log(Info(), "Entering run method for IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost));

  while ( !mStop ) {
    fd_set lInputFileSet;
    FD_ZERO(&lInputFileSet);
    FD_SET(mDeviceFileHostToFPGA, &lInputFileSet);

    timeval lTimeout;
    lTimeout.tv_sec = 0;
    lTimeout.tv_usec = 50000;
    int rc = select(FD_SETSIZE, &lInputFileSet, NULL, NULL, &lTimeout);
    log (Debug(), "PCIeDummyHardware::run  -  select returns ", rc, " Input file set ", (FD_ISSET(mDeviceFileHostToFPGA, &lInputFileSet) ? "contains" : "does not contain"),  " pty");

    if (rc == 0)
      continue;

    log(Debug(), "IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost), " : data ready; reading 'packet length' header");
    // FIXME: Define template method that reads sizeof(T) bytes and returns T
    std::vector<uint32_t> lPageHeader;
    dmaBlockingRead(mDeviceFileHostToFPGA, 0x0, 1, lPageHeader);

    if (lPageHeader.at(0) >= mWordsPerPage) {
      log(Fatal(), "PCIeDummyHardware::run  -  returning early since receiving ", lPageHeader.at(0), "-word packet, but there's only ", mWordsPerPage ," words per page"); 
      return;
    }

    log(Info(), "IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost), " : reading ", lPageHeader.at(0), "-word packet");
    mReceive.clear();
    dmaBlockingRead(mDeviceFileHostToFPGA, 0x0, lPageHeader.at(0), mReceive);

    // std::cout << "Received:" << std::endl;
    // for(size_t i=0; i<mReceive.size(); i++)
    //   std::cout  << " @" << i << "   0x" << std::hex << mReceive.at(i) << std::dec << std::endl;

    mReply.clear();
    AnalyzeReceivedAndCreateReply(4 * lPageHeader.at(0));

    log(Info(), "IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost), " : writing ", mReply.size(), "-word reply to page ", mNextPageIndex);
    lPageHeader.clear();
    lPageHeader.push_back(mReply.size());
    dmaWrite(mDeviceFileFPGAToHost, 4 + mNextPageIndex * mWordsPerPage, lPageHeader); // FIXME: Define template dmaWrite method that takes const T& as final argument, so that don't have to define vector
    dmaWrite(mDeviceFileFPGAToHost, 4 + mNextPageIndex * mWordsPerPage + 1, mReply);

    std::vector<uint32_t> lData;
    lData.push_back(mPublishedPageCount+1); // FIXME: Define template dmaWrite method that takes const T& as final argument, so that don't have to define vector
    dmaWrite(mDeviceFileFPGAToHost, 3, lData);

    mNextPageIndex = (mNextPageIndex + 1) % mNumberOfPages;
    mPublishedPageCount++;
  }

  log(Info(), "Exiting run method for IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost));
}


void PCIeDummyHardware::stop()
{
  log(Info(), "Stopping IPbus 2.0 PCIe dummy hardware ", Quote(mDevicePathHostToFPGA), ", ", Quote(mDevicePathFPGAToHost));
  mStop = true;
}


void PCIeDummyHardware::dmaRead(int aFileDescriptor, const uint32_t aAddr, const uint32_t aNrWords, std::vector<uint32_t>& aValues)
{
  char *allocated = NULL;
  posix_memalign((void **)&allocated, 4096/*alignment*/, 4*aNrWords + 4096);
  assert(allocated);

  /* select AXI MM address */
  char* buffer = allocated;
  off_t off = lseek(aFileDescriptor, 4*aAddr, SEEK_SET);

  /* read data from AXI MM into buffer using SGDMA */
  int rc = ::read(aFileDescriptor, buffer, 4*aNrWords);
  assert(rc >= 0);
  if ((rc % 4) != 0)
    log(Fatal(), "PCIeDummyHardware::dmaRead  -  rc = ", rc); 
  assert((rc % 4) == 0);
  if ((rc > 0) && (rc < 4*aNrWords)) {
    std::cout << "Short read of " << rc << " bytes into a " << 4*aNrWords << " bytes buffer, could be a packet read?\n";
  }

  aValues.insert(aValues.end(), reinterpret_cast<uint32_t*>(buffer), reinterpret_cast<uint32_t*>(buffer)+ aNrWords);
//    for (unsigned i=0; i<aNrWords; i++) {
//      //uint32_t val = (buffer[4*i]&0xff)+((buffer[4*i+1]&0xff)<<8)+((buffer[4*i+2]&0xff)<<16)
//      //  +((buffer[4*i+3]&0xff)<<24);
//      uint32_t val = *reinterpret_cast<uint32_t*>(buffer);
//      printf("ipbus address : 0x%08x\n", aAddr);
//      printf("readback value: 0x%08x\n\n",val);
//    }

  free(allocated);
}


void PCIeDummyHardware::dmaBlockingRead(int aFileDescriptor, const uint32_t aAddr, const uint32_t aNrWords, std::vector<uint32_t>& aValues)
{
  char *allocated = NULL;
  posix_memalign((void **)&allocated, 4096/*alignment*/, 4*aNrWords + 4096);
  assert(allocated);

  /* select AXI MM address */
  char* buffer = allocated;
  off_t off = lseek(aFileDescriptor, 4*aAddr, SEEK_SET);

  memset(buffer, 0, 4 * aNrWords);

  /* read data from AXI MM into buffer using SGDMA */
  int lNrBytesRead = 0;
  do {
    if (lNrBytesRead != 0)
      log (Fatal(), "dmaBlockingRead calling ::read multiple times to get all expected ", aNrWords * 4, " bytes (only ", lNrBytesRead, " so far)");
    int rc = ::read(aFileDescriptor, buffer + lNrBytesRead, 4*aNrWords - lNrBytesRead);
    if (rc <= 0)
      log (Fatal(), "dmaBlockingRead   -   read returned ", rc, ", bugger! errno = ", errno);
    assert (rc > 0);
    lNrBytesRead += rc;
  } while (lNrBytesRead < (4 * aNrWords) );
  if (lNrBytesRead > (4 * aNrWords))
    log (Fatal(), "dmaBlockingRead   -   read ", lNrBytesRead, " but only wanted to read  ", aNrWords * 4, " bytes");
  // int rc = ::read(aFileDescriptor, buffer, 4*aNrWords);
  // assert(rc >= 0);
  // if ((rc % 4) != 0)
  //   log(Fatal(), "PCIeDummyHardware::dmaRead  -  rc = ", rc); 
  // assert((rc % 4) == 0);
  // if ((rc > 0) && (rc < 4*aNrWords)) {
  //   std::cout << "Short read of " << rc << " bytes into a " << 4*aNrWords << " bytes buffer, could be a packet read?\n";
  // }

  aValues.insert(aValues.end(), reinterpret_cast<uint32_t*>(buffer), reinterpret_cast<uint32_t*>(buffer)+ aNrWords);
//    for (unsigned i=0; i<aNrWords; i++) {
//      //uint32_t val = (buffer[4*i]&0xff)+((buffer[4*i+1]&0xff)<<8)+((buffer[4*i+2]&0xff)<<16)
//      //  +((buffer[4*i+3]&0xff)<<24);
//      uint32_t val = *reinterpret_cast<uint32_t*>(buffer);
//      printf("ipbus address : 0x%08x\n", aAddr);
//      printf("readback value: 0x%08x\n\n",val);
//    }

  free(allocated);
}



bool PCIeDummyHardware::dmaWrite(int aFileDescriptor, const uint32_t aAddr, const std::vector<uint32_t>& aValues) 
{
  char *allocated = NULL;
  posix_memalign((void **)&allocated, 4096/*alignment*/, 4*aValues.size() + 4096);
  assert(allocated);

  // data to write to register address
  char* buffer = allocated;
  for (unsigned i=0; i < aValues.size(); i++) {
    uint32_t* ptr = reinterpret_cast<uint32_t*>(&(buffer[i*4]));
    *ptr=aValues.at(i);
  }

  /* select AXI MM address */
  off_t off = lseek(aFileDescriptor, 4*aAddr, SEEK_SET);

  /* write buffer to AXI MM address using SGDMA */
  int rc = ::write(aFileDescriptor, buffer, 4*aValues.size());
  assert(rc == 4*aValues.size());

  free(allocated);

  return true;
}


bool PCIeDummyHardware::dmaWrite(int aFileDescriptor, const uint32_t aAddr, const uint8_t* const aPtr, const size_t aNrBytes) 
{
  assert((aNrBytes % 4) == 0);

  char *allocated = NULL;
  posix_memalign((void **)&allocated, 4096/*alignment*/, aNrBytes + 4096);
  assert(allocated);

  // data to write to register address
  char* buffer = allocated;
  memcpy(buffer, aPtr, aNrBytes);

  /* select AXI MM address */
  off_t off = lseek(aFileDescriptor, 4*aAddr, SEEK_SET);

  /* write buffer to AXI MM address using SGDMA */
  int rc = ::write(aFileDescriptor, buffer, aNrBytes);
  assert(rc == aNrBytes);

  free(allocated);

  return true;
}


} // end ns tests
} // end ns uhal