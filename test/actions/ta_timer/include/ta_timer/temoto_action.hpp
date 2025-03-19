#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/util/error.hpp"
#include "temoto_action_engine/util/logging.hpp"

#include "ta_timer/input_parameters.hpp"
#include "ta_timer/output_parameters.hpp"

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
  output_parameters_t params_out;

private:

  void getInputParameters()
  {
    const auto& params{getUmrfNodeConst().getInputParameters()};

    params_in.count_until = params.getParameterData<double>("count_until");
  }

  void setOutputParameters()
  {
    auto& params{getUmrfNode().getOutputParametersNc()};

    params.setParameter("final_count", "number", boost::any(params_out.final_count));
  }
};
