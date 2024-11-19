#include <class_loader/class_loader.hpp>
#include "ta_compare_string/temoto_action.hpp"

class TaCompareString : public TemotoAction
{
public:

TaCompareString() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  if (params_in.str_a == params_in.str_b)
  {
    TEMOTO_PRINT_OF("strings '" + params_in.str_a + "' and '" + params_in.str_b + "' are EQUAL", getName());
    return true;
  }
  else
  {
    TEMOTO_PRINT_OF("strings '" + params_in.str_a + "' and '" + params_in.str_b + "' NOT EQUAL", getName());
    return false;
  }
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

~TaCompareString()
{
}

}; // TaCompareString class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaCompareString, ActionBase);
