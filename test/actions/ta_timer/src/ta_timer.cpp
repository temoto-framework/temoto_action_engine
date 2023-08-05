#include <chrono>
#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_example_1/temoto_action.h"

class TaTimer : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaTimer()
{}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

// REQUIRED BY TEMOTO
bool onRun()
{
  // Input parameters
  count_until = GET_PARAMETER("count_until", double);

  double starting_count = current_count;
  auto start = std::chrono::high_resolution_clock::now();
  action_stop = false;
  action_pause = false;

  if (starting_count < 0)
  {
    TEMOTO_PRINT_OF("continuing the timer from: " + std::to_string(current_count) + " s", getName());
  }

  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    current_count = starting_count + std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start).count() / 1000.0;

    if (action_stop || action_pause)
      break;

    if (count_until > 0 && current_count >= count_until)
      break;
  }

  if (action_stop)
  {
    TEMOTO_PRINT_OF("stopping the timer on: " + std::to_string(current_count) + " s", getName());
  }
  else if (action_pause)
  {
    TEMOTO_PRINT_OF("pausing the timer on: " + std::to_string(current_count) + " s", getName());
  }
  else
  {
    TEMOTO_PRINT_OF("counter reached: " + std::to_string(current_count) + " s", getName());
  }

  // Pass the output parameters to the action engine
  SET_PARAMETER("final_count", "number", current_count);
  return true;
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
  action_pause = true;
}

void onContinue()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
  action_stop = true;
}

// Destructor
~TaTimer()
{}

private:

double current_count = 0;
double count_until;
bool action_stop;
bool action_pause;

}; // TaTimer class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaTimer, ActionBase);
