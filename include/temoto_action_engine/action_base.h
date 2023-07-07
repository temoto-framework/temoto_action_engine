/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 * 
 *     http://www.apache.org/licenses/LICENSE-2.0
 * 
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * */

#ifndef TEMOTO_ACTION_ENGINE__ACTION_BASE_H
#define TEMOTO_ACTION_ENGINE__ACTION_BASE_H

#include <memory>
#include <iostream>
#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

/**
 * @brief This is the abstract base action that every action has to inherit and implement
 * 
 */
class ActionBase
{
friend class UmrfNodeExec;

public:

  /**
   * @brief Parameter updating routine
   * 
   */
  virtual void updateParameters(const ActionParameters& parameters_in) = 0;

  /**
   * @brief Sets the STOP_REQUESTED_ member variable to "true" which is used by actionOk()
   * 
   */
  bool stopAction()
  {
    STOP_REQUESTED_ = true;
    onStop();
    return true;
  }

  /**
   * @brief Set the UmrfNode object
   */
  void setUmrf(UmrfNode umrf)
  {
    umrf_node_ = umrf;
    umrf_set_ = true;
  }

  const UmrfNode& getUmrfNodeConst() const
  {
    return umrf_node_;
  }

  virtual ~ActionBase(){};

protected:

  /**
   * @brief Method that is invoked when action is initilized. Has to be implemented by an action that
   * inherits this class.
   * 
   */
  virtual void onInit(){}

  /**
   * @brief Method that is invoked when action is executed. Has to be implemented by an action that
   * inherits this class.
   * 
   */
  virtual bool onRun() = 0;

  /**
   * @brief Method that is invoked when action is paused. Has to be implemented by an action that
   * inherits this class.
   * 
   */
  virtual void onPause() = 0;

  /**
   * @brief Method that is invoked when action is continued. Has to be implemented by an action that
   * inherits this class.
   * 
   */
  virtual void onContinue() = 0;

  /**
   * @brief Method that is invoked when action is stopped. Has to be implemented by an action that
   * inherits this class.
   * 
   */
  virtual void onStop() = 0;

  /**
   * @brief A method that inheriting actions must use to determine if the action is required to stop or not.
   * Typical use case would be evaluation of a loop condition.
   * 
   * @return true if the action is not required to stop
   * @return false if the action is required to stop
   */
  bool actionOk()
  {
    return !STOP_REQUESTED_;
  }

  UmrfNode& getUmrfNode()
  {
    return umrf_node_;
  }

private:

  /**
   * @brief Wraps the virtual initializeAction() method with common catch blocks.
   * 
   */
  void initializeActionWrapped()
  try
  {
    onInit();
  }
  catch(TemotoErrorStack& e)
  {
    throw FORWARD_TEMOTO_ERROR_STACK(e);
  }
  catch(const std::exception& e)
  {
    throw CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
  }
  catch(...)
  {  
    throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
  }

  /**
   * @brief Wraps the virtual executeAction() method with common catch blocks and checks if the invoked action is initialized.
   * 
   */
  bool executeActionWrapped()
  {
    if (!isInitialized())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Failed to execute the action because the UMRF is uninitialised");
    }
    
    try
    {
      return onRun();
    }
    catch(TemotoErrorStack& e)
    {
      throw FORWARD_TEMOTO_ERROR_STACK(e);
    }
    catch(const std::exception& e)
    {
      throw CREATE_TEMOTO_ERROR_STACK(std::string(e.what()));
    }
    catch(...)
    {  
      throw CREATE_TEMOTO_ERROR_STACK("Caught an unhandled error.");
    }
  }

  /// Checks if the pointer to a UMRF has been set or not
  bool isInitialized()
  {
    return umrf_set_;
  }

  bool STOP_REQUESTED_ = false;
  UmrfNode umrf_node_;
  bool umrf_set_ = false;
};
#endif