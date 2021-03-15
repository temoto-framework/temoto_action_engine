#include <chrono>
#include <thread>
#include <class_loader/class_loader.hpp>
#include "ta_example_1/temoto_action.h"

class TaExample1 : public TemotoAction
{
public:

// Constructor. REQUIRED BY TEMOTO
TaExample1()
{}

// REQUIRED BY TEMOTO
void executeTemotoAction()
{
  // Input parameters
  double in_param_distance_amount = GET_PARAMETER("distance::amount", double);
  std::string in_param_distance_unit = GET_PARAMETER("distance::unit", std::string);

  // Declaration of output parameters
  double out_param_distance_amount;
  std::string out_param_distance_unit;

  TEMOTO_PRINT_OF("got: " + std::to_string(in_param_distance_amount) + " " + in_param_distance_unit, getUmrfNode().getFullName());
  out_param_distance_amount = in_param_distance_amount + 1;
  out_param_distance_unit = in_param_distance_unit;
  std::this_thread::sleep_for(std::chrono::milliseconds(1000));

  // Pass the output parameters to the action engine
  SET_PARAMETER("distance::amount", "number", out_param_distance_amount);
  SET_PARAMETER("distance::unit", "string", out_param_distance_unit);
}

// Destructor
~TaExample1()
{}

}; // TaExample1 class

/* REQUIRED BY CLASS LOADER */
CLASS_LOADER_REGISTER_CLASS(TaExample1, ActionBase);
