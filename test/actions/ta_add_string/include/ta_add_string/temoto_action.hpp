#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#include "ta_add_string/input_parameters.hpp"
#include "ta_add_string/output_parameters.hpp"

/**
 * @brief Class that integrates TeMoto Base Subsystem specific and Action Engine specific codebases.
 *
 */
class TemotoAction : public ActionBase
{
public:

  TemotoAction()
  {}

  /**
   * @brief Get the Name of the action
   *
   * @return const std::string&
   */
  const std::string& getName()
  {
    return getUmrfNodeConst().getFullName();
  }

  virtual void updateParameters(const ActionParameters& parameters_in)
  {
  }

  input_parameters_t params_in;
  output_parameters_t params_out;

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.str_a = params.getParameterData<std::string>("str_a");
    params_in.str_b = params.getParameterData<std::string>("str_b");
  }

  void setOutputParameters()
  {
    auto& params{getUmrfNode().getOutputParametersNc()};

    params.setParameter("result", "string", boost::any(params_out.result));
  }
};
