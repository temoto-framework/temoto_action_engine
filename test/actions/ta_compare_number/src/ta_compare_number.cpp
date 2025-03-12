#include "ta_compare_number/temoto_action.hpp"

class TaCompareNumber : public TemotoAction
{
public:

TaCompareNumber()
{
}

void onInit()
{
  TEMOTO_PRINT_OF("Initializing", getName());
}

bool onRun()
{
  const auto& operation = params_in.operation;
  const auto& a = params_in.num_a;
  const auto& b = params_in.num_b;
  bool res;

  if      (params_in.operation == "==") {res = a == b;}
  else if (params_in.operation == "!=") {res = a != b;}
  else if (params_in.operation == "<")  {res = a < b;}
  else if (params_in.operation == "<=") {res = a <= b;}
  else if (params_in.operation == ">")  {res = a > b;}
  else if (params_in.operation == ">=") {res = a >= b;}
  else
  {
    throw CREATE_TEMOTO_ERROR_STACK("Unknown operator '" + operation + "'");
  }

  TEMOTO_PRINT_OF(std::to_string(a) + " " + operation + " " + std::to_string(b) + " = " + (res ? "true" : "false"), getName());
  return res;
}

}; // TaCompareNumber class

// REQUIRED, do not remove
boost::shared_ptr<ActionBase> factory()
{
    return boost::shared_ptr<TaCompareNumber>(new TaCompareNumber());
}

// REQUIRED, do not remove
BOOST_DLL_ALIAS(factory, TaCompareNumber)
