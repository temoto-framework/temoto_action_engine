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

#ifndef TEMOTO_ACTION_ENGINE__ACTION_CLASS_LOADER_H
#define TEMOTO_ACTION_ENGINE__ACTION_CLASS_LOADER_H

#include <map>
#include <memory>
#include <string>
#include <class_loader/multi_library_class_loader.h>

class ActionClassLoader
{
public:
  bool loadLibrary(const std::string& library_name);
  bool loadClass(const std::string& library_name, const std::string& class_name = "");

private:
  std::map<std::string, std::shared_ptr<class_loader::ClassLoader>> class_loaders_;
};
#endif