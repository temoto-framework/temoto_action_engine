/* * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * * *
 * Copyright 2019 TeMoto Telerobotics
 * 
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License atUNDEFINED_SOURCE
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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_PARAMETER_H
#define TEMOTO_ACTION_ENGINE__ACTION_PARAMETER_H

#include <vector>
#include <string>
#include "temoto_action_engine/temoto_error.h"

/*
 * Action Parameter
 */ 
template <class T>
class ActionParameter
{
public:
  const static int64_t UNDEFINED_SOURCE = 999;
  ActionParameter( std::string name
                 , std::string type
                 , std::string specification = ""
                 , int64_t source_id = UNDEFINED_SOURCE
                 , bool required = true
                 , bool updatable = false)
  : name_(name)
  , type_(type)
  , specification_(specification)
  , source_id_(source_id)
  , required_(required)
  , updatable_(updatable)
  {}

  ActionParameter(const ActionParameter<T>& ap)
  : name_(ap.name_)
  , type_(ap.type_)
  , specification_(ap.specification_)
  , source_id_(ap.source_id_)
  , timestamp_(ap.timestamp_)
  , required_(ap.required_)
  , updatable_(ap.updatable_)
  , data_(ap.data_)
  {}

  ActionParameter(const std::string& name)
  : name_(name)
  , source_id_(UNDEFINED_SOURCE)
  {}

  ActionParameter(std::string&& name)
  : name_(std::move(name))
  , source_id_(UNDEFINED_SOURCE)
  {}

  void setData(const T& data)
  {
    data_.clear();
    data_.push_back(data);
  }

  const T& getData() const
  {
    if (data_.empty())
    {
      throw CREATE_TEMOTO_ERROR_STACK("No data to retrieve");
    }
    return data_.back();
  }

  unsigned int getDataSize() const
  {
    return data_.size();
  }

  const int64_t& getSourceId() const
  {
    return source_id_;
  }

  const std::string& getName() const
  {
    return name_;
  }

  const std::string& getType() const
  {
    return type_;
  }

  bool operator<(const ActionParameter<T>& rhs) const
  {
    return name_ < rhs.getName();
  }

  bool isRequired() const
  {
    return required_;
  }

  void setRequired(bool required)
  {
    required_ = required;
  }

  ~ActionParameter()
  {
    data_.clear();
  }

private:
  std::string name_;
  std::string type_;
  std::string specification_;
  int64_t source_id_;
  int64_t timestamp_;
  bool required_;
  bool updatable_;
  bool quaranteed_;
  mutable std::vector<T> data_;
};

// template <class T>
// bool operator<(const ActionParameter<T>& lhs, const ActionParameter<T>& rhs)
// {
//   return lhs.getName() < rhs.getName();
// }

#endif
