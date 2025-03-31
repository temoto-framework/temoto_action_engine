#include "ta_logger/temoto_action.hpp"

class TaLogger : public TemotoAction
{
public:

/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 *
 * REQUIRED class methods, do not remove them
 *
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

TaLogger()
{
}

bool onRun()
{
  TEMOTO_PRINT_OF("Running", getName());

  for (const auto& message : params_in.messages)
  {
    TEMOTO_PRINT_OF("Writing message: '" + message + "'", getName());
    writeLog(message);
  }

  return true;
}

~TaLogger()
{
}

}; // TaLogger class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TaLogger>(new TaLogger());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, TaLogger)
