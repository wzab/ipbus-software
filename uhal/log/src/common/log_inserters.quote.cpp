
#include <uhal/log/log_inserters.quote.hpp>

#include <uhal/log/log_configuration.hpp>

namespace uhal
{

	_Quote< const char* > Quote ( const char* aStr )
	{
		return _Quote< const char* > ( aStr  );
	}

}
