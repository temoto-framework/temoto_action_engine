#pragma once

#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/util/error.hpp"
#include "temoto_action_engine/util/logging.hpp"

#include "ta_ingest_result/input_parameters.hpp"

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

    params_in.ingest_count = params.getParameterData<double>("ingest_count");
    params_in.timeout = params.getParameterData<double>("timeout");
  }

  void setOutputParameters()
  {
  }
};
