#include <class_loader/class_loader.hpp>
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

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaRelayString, ActionBase);
