
#include "ta_timer/temoto_action.hpp"

#include <chrono>
#include <thread>

using namespace std::chrono;

class TaTimer : public TemotoAction
{
public:

TaTimer() // REQUIRED
: action_stop{false}
, action_pause{false}
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  auto start = std::chrono::high_resolution_clock::now();

  while (true)
  {
    std::this_thread::sleep_for(milliseconds(100));
    current_count = duration_cast<milliseconds>(high_resolution_clock::now() - start).count() / 1000.0;

    if (action_stop)
    {
      break;
    }

    if (params_in.count_until > 0 && current_count >= params_in.count_until)
    {
      break;
    }
  }

  if (action_stop)
  {
    TEMOTO_PRINT_OF("stopping the timer on: " + std::to_string(current_count) + " s", getName());
  }
  else
  {
    TEMOTO_PRINT_OF("counter reached: " + std::to_string(current_count) + " s", getName());
  }

  // Pass the output parameters to the action engine
  params_out.final_count = current_count;
  return true;
}

void onPause()
{
  TEMOTO_PRINT_OF("Pausing", getName());
  action_pause = true;
}

void onResume()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
  action_stop = true;
}

~TaTimer()
{
}

private:

double current_count = 0;
double count_until;
bool action_stop;
bool action_pause;

}; // TaTimer class

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TaTimer>(new TaTimer());
}

BOOST_DLL_ALIAS(factory, TaTimer)
