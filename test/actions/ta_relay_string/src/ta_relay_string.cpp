
#include "ta_relay_string/temoto_action.hpp"

class TaRelayString : public TemotoAction
{
public:

TaRelayString() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  params_out.string_out = params_in.string_in;
  TEMOTO_PRINT_OF("relaying: " + params_out.string_out, getName());

  return true;
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
}

void onContinue()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
}

~TaRelayString()
{
}

}; // TaRelayString class

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TaRelayString>(new TaRelayString());
}

BOOST_DLL_ALIAS(factory, TaRelayString)
