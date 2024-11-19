#include <class_loader/class_loader.hpp>
#include "ta_trigger_result/temoto_action.hpp"

class TaTriggerResult : public TemotoAction
{
public:

TaTriggerResult() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  // Input parameters
  const auto& in_result = params_in.result;

  TEMOTO_PRINT_OF("triggering a result: " + in_result, getName());

  if (in_result == "on_true")
  {
    return true;
  }
  else if (in_result == "on_false")
  {
    return false;
  }
  else if (in_result == "on_error")
  {
    throw std::runtime_error("Deliberate error generated by '" + getName() + "'");
  }
  else
  {
    throw std::runtime_error("Unrecognized input '" + in_result + "'");
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

~TaTriggerResult()
{
}

}; // TaTriggerResult class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTriggerResult, ActionBase);
