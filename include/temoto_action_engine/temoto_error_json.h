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

#ifndef TEMOTO_ACTION_ENGINE__TEMOTO_ERROR_JSON_H
#define TEMOTO_ACTION_ENGINE__TEMOTO_ERROR_JSON_H

#include "temoto_error.h"
#include "rapidjson/document.h"
#include "rapidjson/prettywriter.h"

namespace temoto_error_json
{
  std::string toJsonStr(const TemotoError& te)
  {
    rapidjson::Document json_doc;

    rapidjson::Value time(te.getTime());
    rapidjson::Value message(rapidjson::StringRef(te.getMessage().c_str()));
    rapidjson::Value origin(rapidjson::StringRef(te.getOrigin().c_str()));

    json_doc.AddMember("time", time, json_doc.GetAllocator());
    json_doc.AddMember("message", message, json_doc.GetAllocator());
    json_doc.AddMember("origin", origin, json_doc.GetAllocator());

    rapidjson::StringBuffer buffer;
    buffer.Clear();
    rapidjson::Writer<rapidjson::StringBuffer> writer(buffer);
    json_doc.Accept(writer);

    return std::string(buffer.GetString());
  }

  TemotoError fromJsonStr(const std::string& json_str)
  {
    rapidjson::Document json_doc;
    json_doc.Parse(json_str.c_str());
    if (json_doc.HasParseError())
    {
      throw TemotoError("Invalid JSON string", "testing");
    }

    if (!json_doc.HasMember("time") || !json_doc.HasMember("message") || !json_doc.HasMember("origin"))
    {
      throw TemotoError("The Error JSON is not fully defined (missing either time, message or origin)", "testing");
    }

    if (!json_doc["time"].IsNumber())
    {
      throw TemotoError("The time must be expressed in a numerical format", "testing");
    }
    if (!json_doc["message"].IsString())
    {
      throw TemotoError("The message must be expressed in a string format", "testing");
    }
    if (!json_doc["origin"].IsString())
    {
      throw TemotoError("The origin must be expressed in a string format", "testing");
    }

    unsigned long time = json_doc["time"].GetUint64();
    std::string message = json_doc["message"].GetString();
    std::string origin = json_doc["origin"].GetString();

    return TemotoError(time, message, origin);
  }
}



#endif