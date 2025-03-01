#pragma once

#include <class_loader/class_loader.hpp>
#include <string>
#include <memory>

#include "temoto_action_engine/temoto_error.h"

template <typename T>
class ActionPlugin
{
public:

	ActionPlugin(const std::string& action_name)
	: action_name_{action_name}
	{
		std::string library_name = "lib" + camelToSnake(action_name) + ".so";
    	class_loader_ = std::make_shared<class_loader::ClassLoader>(library_name, false);

		// Check if the classloader actually contains the required action
		std::vector<std::string> classes_in_classloader = class_loader_->getAvailableClasses<T>();
		bool class_found = [&]{
			for (const auto& class_in_classloader : classes_in_classloader)
			{
				if (class_in_classloader == action_name)
				{
					return true;
				}
			}
			return false;
		}();

		if (!class_found)
		{
			throw CREATE_TEMOTO_ERROR_STACK("Plugin for action '" + action_name + "' not found");
		}
	}

	T& get()
	{
        return *plugin_instance_;
    }

	void load()
	{
		plugin_instance_ = class_loader_->createInstance<T>(action_name_);
	}

private:

	std::string camelToSnake(const std::string& str_camel_case) const
	{
		std::string str_snake_case = "";
		char c = std::tolower(str_camel_case[0]);
		str_snake_case+=(char(c));

		for (unsigned int i = 1; i < str_camel_case.length(); i++)
		{
			char ch = str_camel_case[i];
			if (std::isupper(ch))
			{
				str_snake_case = str_snake_case + '_';
				str_snake_case += char(std::tolower(ch));
			}
			else
			{
				str_snake_case = str_snake_case + ch;
			}
		}
		return str_snake_case;
	}

	std::string action_name_;
	std::shared_ptr<class_loader::ClassLoader> class_loader_;
	std::shared_ptr<T> plugin_instance_;
};
