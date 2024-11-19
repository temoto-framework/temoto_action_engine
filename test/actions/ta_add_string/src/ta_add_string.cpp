#include <class_loader/class_loader.hpp>
#include "ta_add_string/temoto_action.hpp"

class TaAddString : public TemotoAction
{
public:

TaAddString() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  TEMOTO_PRINT_OF("Running", getName());

  params_out.result = params_in.str_a + " and " +  params_in.str_b;
  TEMOTO_PRINT_OF("got: " + params_out.result, getName());

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

~TaAddString()
{
}

}; // TaAddString class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaAddString, ActionBase);
