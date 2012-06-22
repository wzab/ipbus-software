/**
	@file
	@author Andrew W. Rose
	@date 2012
*/

#ifndef _uhal_HwInterface_hpp_
#define _uhal_HwInterface_hpp_

#include "uhal/Node.hpp"
#include "uhal/exception.hpp"
#include "uhal/ClientInterface.hpp"

#include <boost/regex.hpp>


namespace uhal
{
	//! A class which bundles a node tree and an IPbus client interface together providing everything you need to navigate and perform hardware access
	class HwInterface
	{
		public:
			/**
				Constructor
			*/
			HwInterface ( const boost::shared_ptr<ClientInterface>& aClientInterface , const boost::shared_ptr< const Node >& aNode );

			/**
				Destructor
			*/
			virtual ~HwInterface();

			/**
				Get the underlying IPbus client
				@return the underlying IPbus client
			*/
			boost::shared_ptr<ClientInterface> getClient();

			/**
				Make the IPbus client issue a dispatch
			*/
			void dispatch ();

			/**
				A method to modify the timeout period for any pending or future transactions
				@param aTimeoutPeriod the desired timeout period in seconds
			*/
			void setTimeoutPeriod ( const uint32_t& aTimeoutPeriod );

			/**
				A method to retrieve the timeout period currently being used
				@return the timeout period currently being used
			*/
			const uint32_t& getTimeoutPeriod();

			/**
				Ping the target for this client
			*/
			void ping();

			/**
				Retrieve the Node given by a full-stop delimeted name path relative, to the top-level node
				@param aId a full-stop delimeted name path to a node, relative to the top-level node
				@return the Node given by the identifier
			*/
			Node& getNode ( const std::string& aId );

			/**
				Retrieve the Node given by a full-stop delimeted name path relative, to the current node and cast it to a particular node type
				@param aId a full-stop delimeted name path to a node, relative to the current node
				@return the Node given by the identifier
			*/
			template< typename T>
			T& getNode ( const std::string& aId );


			/**
				Return all node IDs known to this HwInterface
				@return all node IDs known to this HwInterface
			*/
			std::vector<std::string> getNodes();

			/**
				Return all node IDs known to this connection manager which match a (boost) regular expression
				@param aRegex a (boost) regular expression against which the node IDs are tested
				@return all node IDs known to this connection manager
			*/
			std::vector<std::string> getNodes ( const boost::regex& aRegex );
			/**
				Return all node IDs known to this connection manager which match a (boost) regular expression
				@param aRegex a const char* expression which is converted to a (boost) regular expression against which the node IDs are tested
				@return all node IDs known to this connection manager
			*/
			std::vector<std::string> getNodes ( const char* aRegex );
			/**
				Return all node IDs known to this connection manager which match a (boost) regular expression
				@param aRegex a string expression which is converted to a (boost) regular expression against which the node IDs are tested
				@return all node IDs known to this connection manager
			*/
			std::vector<std::string> getNodes ( const std::string& aRegex );

			/**
				Get the target device's reserved address information
				@return a Validated Memory which wraps the location to which the reserved address information will be written
			*/
			ValVector< uint32_t > readReservedAddressInfo ();

		private:
			/**
				A function which sets the HwInterface pointer in the Node to point to this HwInterface
				@param aNode a Node that is to be claimed
			*/
			void claimNode ( Node& aNode );

			//! A shared pointer to the IPbus client through which the transactions will be sent
			boost::shared_ptr<ClientInterface> mClientInterface;

			//! A node tree
			boost::shared_ptr<Node> mNode;

	};

	template< typename T>
	T& HwInterface::getNode ( const std::string& aId )
	{
		try
		{
			return dynamic_cast< T > ( getNode ( aId ) );
		}
		catch ( const std::exception& aExc )
		{
			log ( Error() , "Exception " , Quote( aExc.what() ) , " caught at " , ThisLocation() );
			throw uhal::exception ( aExc );
		}
	}

}

#endif
