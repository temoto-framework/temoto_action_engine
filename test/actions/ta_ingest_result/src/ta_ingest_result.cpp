#include <chrono>
#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_example_1/temoto_action.h"

class TaIngestResult : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaIngestResult()
{}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

// REQUIRED BY TEMOTO
bool onRun()
{
  // Input parameters
  if (current_ingest_count == 0)
  {
    final_ingest_count = int(GET_PARAMETER("ingest_count", double));
    timeout = GET_PARAMETER("timeout", double);
  }

  TEMOTO_PRINT_OF("ingested a result " + std::to_string(++current_ingest_count) +
    " out of " + std::to_string(final_ingest_count), getName());

  if (current_ingest_count >= final_ingest_count)
  {
    return true;
  }

  auto start = std::chrono::high_resolution_clock::now();
  bool timeout_reached = false;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::high_resolution_clock::now() - start).count() / 1000.0;

    if (duration >= timeout)
    {
      timeout_reached = true;
      break;
    }
    else if (action_stop)
    {
      break;
    }
  }

  if (timeout_reached)
  {
    TEMOTO_PRINT_OF("reached the timeout of: " + std::to_string(timeout) + " s", getName());
    return false;
  }
  else if (action_stop)
  {
    TEMOTO_PRINT_OF("externally stopped", getName());
    return true;
  }
  else
  {
    throw CREATE_TEMOTO_ERROR_STACK("The result ingestor is broken");
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
  action_stop = true;
}

// Destructor
~TaIngestResult()
{}

private:

bool action_stop = false;
int current_ingest_count;
int final_ingest_count;
double timeout;

}; // TaIngestResult class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaIngestResult, ActionBase);
