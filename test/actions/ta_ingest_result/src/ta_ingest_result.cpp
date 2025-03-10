
#include "ta_ingest_result/temoto_action.hpp"

#include <chrono>
#include <thread>

class TaIngestResult : public TemotoAction
{
public:

TaIngestResult() // REQUIRED
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  // Input parameters
  if (current_ingest_count == 0)
  {
    final_ingest_count = params_in.ingest_count;
    timeout = params_in.timeout;
  }

  TEMOTO_PRINT_OF("ingested a result " + std::to_string(++current_ingest_count) +
    " out of " + std::to_string(final_ingest_count), getName());

  if (current_ingest_count >= final_ingest_count)
  {
    TEMOTO_PRINT_OF("Final ingestion count reached", getName());
    return true;
  }

  auto start = std::chrono::high_resolution_clock::now();
  bool timeout_reached = false;
  while (true)
  {
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
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
    action_stop = false;
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

void onResume()
{
  TEMOTO_PRINT_OF("Continuing", getName());
}

void onStop()
{
  TEMOTO_PRINT_OF("Stopping", getName());
  action_stop = true;
}

~TaIngestResult()
{
}

private:

bool action_stop = false;
int current_ingest_count = 0;
int final_ingest_count;
double timeout;

}; // TaIngestResult class

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TaIngestResult>(new TaIngestResult());
}

BOOST_DLL_ALIAS(factory, TaIngestResult)
