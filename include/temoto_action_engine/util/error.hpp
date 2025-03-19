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

#ifndef TEMOTO_ACTION_ENGINE__TEMOTO_ERROR_H
#define TEMOTO_ACTION_ENGINE__TEMOTO_ERROR_H

#pragma GCC diagnostic ignored "-Wcatch-value" // TODO: catch TemotoErrors by const reference

#include <vector>
#include <string>
#include <exception>
#include <time.h>

#define CREATE_TEMOTO_ERROR(message) TemotoError(message, __func__)
#define CREATE_TEMOTO_ERROR_STACK(message) TemotoErrorStack(message, __func__)
#define FORWARD_TEMOTO_ERROR_STACK(error_stack) error_stack.appendError("forwarding", __func__)
#define FORWARD_TEMOTO_ERROR_STACK_WMSG(error_stack, msg) error_stack.appendError(msg, __func__)

class TemotoError : public std::exception
{
public:
  TemotoError(std::string message, std::string origin)
  : time_(time(NULL))
  , message_(message)
  , origin_(origin)
  {}

  TemotoError(unsigned int time, std::string message, std::string origin)
  : time_(time)
  , message_(message)
  , origin_(origin)
  {}

  TemotoError(const TemotoError& te)
  : time_(te.time_)
  , message_(te.message_)
  , origin_(te.origin_)
  {}

  const unsigned long& getTime() const
  {
    return time_;
  }

  const std::string& getMessage() const
  {
    return message_;
  }

  const std::string& getOrigin() const
  {
    return origin_;
  }

  virtual const char* what() const noexcept
  {
    return getMessage().c_str();
  }

private:
  unsigned long time_;
  std::string message_;
  std::string origin_;
};

class TemotoErrorStack : public std::exception
{
public:
  TemotoErrorStack()
  {}

  TemotoErrorStack(std::string error_message, std::string origin)
  : error_stack_({TemotoError(error_message, origin)})
  {}

  TemotoErrorStack(const TemotoErrorStack& tes)
  : error_stack_(tes.error_stack_)
  , messages_(tes.messages_)
  {}

  void operator=(const TemotoErrorStack& tes)
  {
    error_stack_ = tes.error_stack_;
    messages_ = tes.messages_;
  }

  TemotoErrorStack appendError(std::string error_message, std::string origin)
  {
    error_stack_.emplace_back(error_message, origin);
    return *this;
  }

  void appendError(const TemotoErrorStack& tes)
  {
    error_stack_.insert(error_stack_.end(), tes.getErrorStack().begin(), tes.getErrorStack().end());
  }

  const std::string& getMessage() const
  {
    messages_.clear();

    for (unsigned int i=0; i<error_stack_.size(); i++)
    {
      if (i==0)
      {
        messages_ += "origin [in " + error_stack_.at(i).getOrigin() + "]: ";
        messages_ += error_stack_.at(i).getMessage() + "\n";
      }
      else
      {
        messages_ += "   fwd [in " + error_stack_.at(i).getOrigin() + "]: ";
        messages_ += error_stack_.at(i).getMessage() + "\n";
      }
    }
    return messages_;
  }

  const std::vector<TemotoError>& getErrorStack() const
  {
    return error_stack_;
  }

  virtual const char* what() const noexcept
  {
    return getMessage().c_str();
  }
private:
  std::vector<TemotoError> error_stack_;
  mutable std::string messages_;
};

#endif
