#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/util/error.hpp"
#include "temoto_action_engine/util/logging.hpp"

#include "ta_compare_number/input_parameters.hpp"

#include <boost/config.hpp>
#include <boost/shared_ptr.hpp>
#include <boost/dll/alias.hpp>

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

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.num_a = params.getParameterData<double>("num_a");
    params_in.num_b = params.getParameterData<double>("num_b");
    params_in.operation = params.getParameterData<std::string>("operation");
  }

  void setOutputParameters()
  {
  }
};
