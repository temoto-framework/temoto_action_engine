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
#include <map>
#include "temoto_action_engine/temoto_error.h"
#include <boost/algorithm/string.hpp>

/*
 * Action Parameter
 */
namespace action_parameter
{
  const std::map<std::string, std::string> PARAMETER_MAP {
    {"number", "double"},
    {"string", "std::string"}
  };
}

template <class T>
class ActionParameter
{
public:
  const static int64_t UNDEFINED_SOURCE = 999;

  ActionParameter( std::string name
                 , std::string type
                 , std::string example = ""
                 , int64_t source_id = UNDEFINED_SOURCE
                 , bool required = true
                 , bool updatable = false
                 , bool quaranteed = false)
  : name_(name)
  , type_(type)
  , example_(example)
  , source_id_(source_id)
  , required_(required)
  , updatable_(updatable)
  , quaranteed_(quaranteed)
  {}

  ActionParameter(const ActionParameter<T>& ap)
  : name_(ap.name_)
  , type_(ap.type_)
  , example_(ap.example_)
  , source_id_(ap.source_id_)
  , timestamp_(ap.timestamp_)
  , required_(ap.required_)
  , quaranteed_(ap.quaranteed_)
  , updatable_(ap.updatable_)
  , data_(ap.data_)
  {}

  ActionParameter(const std::string& name)
  : ActionParameter(name, "undef")
  {}

  ActionParameter(std::string&& name)
  : ActionParameter(std::move(name), "undef")
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

  std::string getNameNoNamespace() const
  {
    std::vector<std::string> name_comp_vec;
    boost::split(name_comp_vec, name_, boost::is_any_of("::"));
    return name_comp_vec.back();
  }

   std::string getNamespace() const
  {
    std::vector<std::string> name_comp_vec;
    boost::split(name_comp_vec, name_, boost::is_any_of("::"));
    return name_comp_vec.back();
  }

  void setName(const std::string& name)
  {
    name_ = name;
  }

  void setNameKeepNamespace(const std::string& name)
  {
    std::vector<std::string> name_comp_vec;
    boost::split(name_comp_vec, name_, boost::is_any_of("::"));
    
    std::string final_name;
    name_comp_vec.pop_back(); // Remove the name

    for (const auto& name_comp : name_comp_vec)
    {
      if (name_comp.empty())
      {
        continue;
      }
      final_name += name_comp + "::";
    }
    final_name += name;
    name_ = final_name;
  }

  void removeNamespaceLevel()
  {
    std::vector<std::string> name_comp_vec;
    boost::split(name_comp_vec, name_, boost::is_any_of("::"));
    
    if (name_comp_vec.size() == 1)
    {
      return;
    }

    std::string name_no_ns = name_comp_vec.back();
    std::string final_name;
    name_comp_vec.pop_back(); // Remove the name
    name_comp_vec.pop_back(); // Remove the empty char
    name_comp_vec.pop_back(); // Remove the namespace

    for (const auto& name_comp : name_comp_vec)
    {
      if (name_comp.empty())
      {
        continue;
      }
      final_name += name_comp + "::";
    }
    final_name += name_no_ns;
    name_ = final_name;
  }

  const std::string& getType() const
  {
    return type_;
  }

  void setType(const std::string& type)
  {
    type_ = type;
  }

  const std::string& getExample() const
  {
    return example_;
  }

  void setExample(const std::string& example)
  {
    example_ = example;
  }

  /**
   * @brief Operator for maining parameters in std::set. The comparison operator must stay in this form, i.e.
   * only the names should be compared.
   * 
   * @param rhs 
   * @return true 
   * @return false 
   */
  bool operator<(const ActionParameter<T>& rhs) const
  {
    return name_ < rhs.getName();
  }

  bool isEqualNoDataNoUpdate(const ActionParameter<T>& ap) const
  {
    return (name_ == ap.name_ &&
      type_ == ap.type_ &&
      required_ == ap.required_ &&
      quaranteed_ == ap.quaranteed_);
  }

  /**
   * @brief Checks if the action parameter is equal. Data is not checked.
   * 
   * @param ap 
   * @return true 
   * @return false 
   */
  bool isEqualNoData(const ActionParameter<T>& ap) const
  {
    return (name_ == ap.name_ &&
      type_ == ap.type_ &&
      required_ == ap.required_ &&
      updatable_ == ap.updatable_ &&
      quaranteed_ == ap.quaranteed_);
  }

  /**
   * @brief Checks if parameters are equal. Exact contents of the data is not checked.
   * 
   * @param ap 
   * @return true 
   * @return false 
   */
  bool isEqual(const ActionParameter<T>& ap) const
  {
    return (isEqualNoData(ap) && (data_.size() == ap.data_.size()));
  }

  bool isRequired() const
  {
    return required_;
  }

  bool isUpdatable() const
  {
    return updatable_;
  }

  void setUpdatable(bool updatable)
  {
    updatable_ = updatable;
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
  std::string example_;
  int64_t source_id_;
  int64_t timestamp_;
  bool required_;
  bool updatable_;
  bool quaranteed_;
  mutable std::vector<T> data_;
};

#endif
