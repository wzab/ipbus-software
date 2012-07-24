#include "uhal/uhal.hpp"

#include "uhal/tests/tools.hpp"

#include <vector>
#include <iostream>
#include <cstdlib>
#include <typeinfo>

using namespace uhal;

#define N_4B    1
#define N_1kB   1024/4
#define N_1MB   1024*1024/4

void block_write_read ( size_t N,const std::string& connection, const std::string& id )
{
	ConnectionManager manager ( connection );
	HwInterface hw=manager.getDevice ( id );
	std::vector<uint32_t> xx;

	for ( size_t i=0; i!= N; ++i )
	{
		xx.push_back ( static_cast<uint32_t> ( rand() ) );
	}

	hw.getNode ( "MEM" ).writeBlock ( xx );
	ValVector< uint32_t > mem = hw.getNode ( "MEM" ).readBlock ( N );
	CACTUS_CHECK ( !mem.valid() );
	CACTUS_CHECK ( mem.size() == N );
	CACTUS_TEST_THROW ( mem.at ( 0 ),uhal::NonValidatedMemory );
	CACTUS_TEST ( hw.dispatch() );
	CACTUS_CHECK ( mem.valid() );
	CACTUS_CHECK ( mem.size() == N );
	bool correct_block_write_read = true;
	ValVector< uint32_t >::const_iterator i=mem.begin();
	std::vector< uint32_t >::const_iterator j=xx.begin();

	for ( ValVector< uint32_t >::const_iterator i ( mem.begin() ); i!=mem.end(); ++i , ++j )
	{
		correct_block_write_read = correct_block_write_read && ( *i == *j );
	}

	CACTUS_CHECK ( correct_block_write_read );


}

void fifo_write_read ( size_t N,const std::string& connection, const std::string& id )
{
	ConnectionManager manager ( connection );
	HwInterface hw=manager.getDevice ( id );
	std::vector<uint32_t> xx;

	for ( size_t i=0; i!= N; ++i )
	{
		xx.push_back ( static_cast<uint32_t> ( rand() ) );
	}

	hw.getNode ( "FIFO" ).writeBlock ( xx );
	ValVector< uint32_t > mem = hw.getNode ( "FIFO" ).readBlock ( N );
	CACTUS_CHECK ( !mem.valid() );
	CACTUS_CHECK ( mem.size() == N );
	CACTUS_TEST_THROW ( mem.at ( 0 ),uhal::NonValidatedMemory );

	CACTUS_TEST ( hw.dispatch() );

	CACTUS_CHECK ( mem.valid() );
	CACTUS_CHECK ( mem.size() == N );
	//The FIFO implementation on the dummy HW is a single memory location so there is not much to check
}

void block_transfer_too_big(const std::string& connection, const std::string& id ) {
	ConnectionManager manager ( connection );
	HwInterface hw=manager.getDevice ( id );

  	std::vector<uint32_t> xx;
	for ( size_t i=0; i!= 100*1024*1024; ++i )
	{
		xx.push_back ( 0x0 );
	}

	CACTUS_TEST_THROW ( hw.getNode ( "LARGE_MEM" ).writeBlock ( xx ) , uhal::BulkTransferRequestedTooLarge );
	CACTUS_TEST_THROW ( ValVector< uint32_t > mem = hw.getNode ( "LARGE_MEM" ).readBlock ( 100*1024*1024 ) , uhal::BulkTransferRequestedTooLarge);
}

void block_bigger_than_size_attribute(const std::string& connection, const std::string& id ) {
	ConnectionManager manager ( connection );
	HwInterface hw=manager.getDevice ( id );

  	std::vector<uint32_t> xx;
	for ( size_t i=0; i!= 1024*1024; ++i )
	{
		xx.push_back ( 0x0 );
	}

	CACTUS_TEST_THROW ( hw.getNode ( "SMALL_MEM" ).writeBlock ( xx ) , uhal::BulkTransferRequestedTooLarge );
	CACTUS_TEST_THROW ( ValVector< uint32_t > mem = hw.getNode ( "SMALL_MEM" ).readBlock ( 100*1024*1024 ) , uhal::BulkTransferRequestedTooLarge);
}

int main ( int argc,char* argv[] )
{
	std::map<std::string,std::string> params = tests::default_arg_parsing ( argc,argv );
	std::string connection_file = params["connection_file"];
	std::string device_id = params["device_id"];
	std::cout << "STARTING TEST " << argv[0] << " (connection_file='" << connection_file<<"', device_id='" << device_id << "')..." << std::endl;
	
	//Memory
	CACTUS_TEST ( block_write_read ( N_4B,connection_file,device_id ) );
	CACTUS_TEST ( block_write_read ( N_1kB,connection_file,device_id ) );
	CACTUS_TEST ( block_write_read ( N_1MB,connection_file,device_id ) );
	//FIFO
	CACTUS_TEST ( fifo_write_read ( N_4B,connection_file,device_id ) );
	CACTUS_TEST ( fifo_write_read ( N_1kB,connection_file,device_id ) );
	CACTUS_TEST ( fifo_write_read ( N_1MB,connection_file,device_id ) );
	//Block to big
	//CACTUS_TEST_THROW(block_bigger_than_size_attribute(connection_file,device_id ),uhal::BulkTransferRequestedTooLarge);
	block_transfer_too_big(connection_file,device_id );
	block_bigger_than_size_attribute(connection_file,device_id );

	return 0;
}
