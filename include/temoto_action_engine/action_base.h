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

/* Author: Robert Valner */

#ifndef TEMOTO_ACTION_ENGINE__ACTION_BASE_H
#define TEMOTO_ACTION_ENGINE__ACTION_BASE_H

#include <memory>
#include <iostream>
#include "temoto_action_engine/umrf.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/messaging.h"

/**
 * @brief This is the base action that every action has to inherit and implement
 * 
 */
class ActionBase
{
public:
  void executeActionWrapped()
  {
    if (!isInitialized())
    {
      throw CREATE_TEMOTO_ERROR_STACK("Failed to execute the action because UMRF pointer is uninitialised");
    }
    
    try
    {
      executeAction();
    }
    catch(TemotoErrorStack e)
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

  bool stopAction()
  {
    STOP_REQUESTED_ = true;
    return true;
  }

  void setUmrf(const std::shared_ptr<Umrf>& umrf)
  {
    umrf_ = umrf;
  }

  virtual ~ActionBase(){};

protected:
  virtual void executeAction() = 0;

  bool actionOk()
  {
    return !STOP_REQUESTED_;
  }

  std::shared_ptr<Umrf> getUmrfPtr()
  {
    return umrf_;
  }

private:
  bool isInitialized()
  {
    if(umrf_)
    {
      return true;
    }
    else
    {
      return false;
    }
  }

  bool STOP_REQUESTED_ = false;
  std::shared_ptr<Umrf> umrf_;
};
#endif