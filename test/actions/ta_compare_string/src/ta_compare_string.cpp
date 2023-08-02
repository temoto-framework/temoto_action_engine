#include <chrono>
#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_example_1/temoto_action.h"

class TaCompareString : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaCompareString()
{}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

// REQUIRED BY TEMOTO
bool onRun()
{
  // Input parameters
  std::string in_str_a = GET_PARAMETER("str_a", std::string);
  std::string in_str_b = GET_PARAMETER("str_b", std::string);

  if (in_str_a == in_str_b)
  {
    TEMOTO_PRINT_OF("strings '" + in_str_a + "' and '" + in_str_b + "' are EQUAL", getName());
    return true;
  }
  else
  {
    TEMOTO_PRINT_OF("strings '" + in_str_a + "' and '" + in_str_b + "' NOT EQUAL", getName());
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

// Destructor
~TaCompareString()
{}

}; // TaCompareString class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaCompareString, ActionBase);
