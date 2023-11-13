/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2023 TeMoto Framework
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

#ifndef TEMOTO_ACTION_ENGINE__MESSAGING_H
#define TEMOTO_ACTION_ENGINE__MESSAGING_H

#include <string>
#include <iostream>
#include "temoto_action_engine/threadsafe_print.h"

#define TEMOTO_PRINT(message) temoto_messaging::print(message, __func__)
#define TEMOTO_PRINT_OF(message, of) temoto_messaging::printOf(message, __func__, of)

namespace temoto_messaging
{
  inline void print(const std::string& message, const std::string& prefix)
  {
    PrintThread{} << "[" << prefix << "] " << message << std::endl;
  }

  inline void printOf(const std::string& message, const std::string& prefix, const std::string& of)
  {
    PrintThread{} << "[" << of << "::" << prefix << "] " << message << std::endl;
  }
}

#endif