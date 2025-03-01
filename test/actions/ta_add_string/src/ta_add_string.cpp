
#include "ta_add_string/temoto_action.hpp"

class TaAddString : public TemotoAction
{
public:

TaAddString() // REQUIRED
{
  TEMOTO_PRINT_OF("Constructed", getName());
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun() // REQUIRED
{
  TEMOTO_PRINT_OF("Running", getName());

  params_out.result = params_in.str_a + " and " +  params_in.str_b;
  TEMOTO_PRINT_OF("got(" + std::to_string(++count) + "): " + params_out.result, getName());

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

~TaAddString()
{
}

int count{0};

}; // TaAddString class

boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TaAddString>(new TaAddString());
}

BOOST_DLL_ALIAS(factory, TaAddString)
