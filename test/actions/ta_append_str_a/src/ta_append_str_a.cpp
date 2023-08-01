#include <chrono>
#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_example_1/temoto_action.h"

class TaAppendStrA : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaAppendStrA()
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

  // Declaration of output parameters
  std::string out_str_a = in_str_a + " and " + in_str_b;

  TEMOTO_PRINT_OF("got: " + in_str_a + " and " + in_str_b, getName());
  std::this_thread::sleep_for(std::chrono::milliseconds(100));

  // Pass the output parameters to the action engine
  SET_PARAMETER("str_a", "string", out_str_a);
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
~TaAppendStrA()
{}

}; // TaAppendStrA class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaAppendStrA, ActionBase);
