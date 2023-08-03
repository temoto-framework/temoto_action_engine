
#ifndef ta_example_1__TEMOTO_ACTION_H
#define ta_example_1__TEMOTO_ACTION_H

/* REQUIRED BY TEMOTO */
#include "temoto_action_engine/action_base.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

#define GET_PARAMETER(name, type) getUmrfNodeConst().getInputParameters().getParameterData<type>(name)
#define SET_PARAMETER(name, type, value) getUmrfNode().getOutputParametersNc().setParameter(name, type, boost::any(value))

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
};

#endif
