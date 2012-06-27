#include "uhal/ClientImplementation.hpp"



namespace uhal
{

	std::pair< uint32_t , uint16_t > ExtractTargetID ( const URI& aUri )
	{
		NameValuePairVectorType::const_iterator lIt = aUri.mArguments.begin();

		for ( ; lIt != aUri.mArguments.end() ; ++lIt )
		{
			if ( lIt->first == "target" )
			{
				break;
			}
		}

		if ( lIt == aUri.mArguments.end() )
		{
			log ( Error() , "This function expects arguments of the form " , Quote ( "target=192.168.200.200:50001" ) ,". It appears that this is missing." );
			log ( Error() , "Throwing at " , ThisLocation() );
			throw XMLfileMissingRequiredParameters();
		}

		IPaddr lIP;
		boost::spirit::qi::phrase_parse (	lIt->second.begin() ,
											lIt->second.end() ,
											( boost::spirit::qi::uint_ > boost::spirit::qi::lit ( "." ) > boost::spirit::qi::uint_ > boost::spirit::qi::lit ( "." ) > boost::spirit::qi::uint_ > boost::spirit::qi::lit ( "." ) > boost::spirit::qi::uint_ > boost::spirit::qi::lit ( ":" ) > boost::spirit::qi::uint_ ) ,
											boost::spirit::ascii::space ,
											lIP
										);
		uint32_t lIPaddress = ( lIP.mIP1 <<24 ) | ( lIP.mIP2 <<16 ) | ( lIP.mIP3 <<8 ) | ( lIP.mIP4 );
		log ( Info() , "Converted IP address string " ,  Quote ( lIt->second ) ,
			  " to " , Integer ( lIP.mIP1 ) , "." , Integer ( lIP.mIP2 ) , "." , Integer ( lIP.mIP3 ) , "." , Integer ( lIP.mIP4 ) , ":" , Integer ( lIP.mPort ) ,
			  " and converted this to IP " , Integer ( lIPaddress, IntFmt< hex , fixed >() ) , ", port " , Integer ( lIP.mPort, IntFmt< hex , fixed >() )	);
		return std::make_pair ( lIPaddress , lIP.mPort );
	}

}


