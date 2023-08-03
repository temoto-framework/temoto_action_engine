#include <chrono>
#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_example_1/temoto_action.h"

class TaRelayString : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaRelayString()
{}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

// REQUIRED BY TEMOTO
bool onRun()
{
  // Input parameters
  std::string string_in = GET_PARAMETER("string_in", std::string);
  std::string string_out = string_in;

  TEMOTO_PRINT_OF("relaying: " + string_out, getName());

  // Pass the output parameters to the action engine
  SET_PARAMETER("string_out", "string", string_out);
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

// Destructor
~TaRelayString()
{}

}; // TaRelayString class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaRelayString, ActionBase);
