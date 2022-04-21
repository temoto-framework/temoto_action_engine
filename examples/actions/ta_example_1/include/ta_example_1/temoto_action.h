
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

  /**
   * @brief Wraps the executeTemotoAction and converts TeMoto specific errors to action engine errors.
   * 
   */
  void executeAction()
  {
    try
    {
      executeTemotoAction();
    }
    catch(TemotoErrorStack e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK(e.what());
    }
    catch(...)
    {
      throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled exception");
    }
  }

  virtual void updateParameters(const ActionParameters& parameters_in)
  {
    for (const auto& p_in : parameters_in)
    {
      boost::any param_data;

      if (!getUmrfNodeConst().getInputParameters().hasParameter(p_in))
      {
        throw CREATE_TEMOTO_ERROR_STACK("This action has no parameter '" + p_in.getName() + "'");
      }
      else if (p_in.getType() == "number")
      {
        param_data = boost::any_cast<double>(p_in.getData());
      }
      else if (p_in.getType() == "string")
      {
        param_data = boost::any_cast<std::string>(p_in.getData());
      }

      else
      {
        throw CREATE_TEMOTO_ERROR_STACK("No matching data type");
      }

      getUmrfNode().getInputParametersNc().setParameterData(p_in.getName(), param_data);
    }
  }

  /**
   * @brief Has to be implemented by an action.
   * 
   */
  virtual void executeTemotoAction() = 0;
};

#endif
