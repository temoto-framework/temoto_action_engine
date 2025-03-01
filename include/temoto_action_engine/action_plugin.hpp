#pragma once

#include <boost/dll/import.hpp>
#include <string>
#include <memory>
#include <cstdlib>
#include <vector>
#include <filesystem>
#include <optional>

#include "temoto_action_engine/temoto_error.h"

template <typename T>
class ActionPlugin
{
	using loader_t = boost::dll::detail::library_function<boost::shared_ptr<T> ()>;

public:

	ActionPlugin(const std::string& action_name)
	: action_name_{action_name}
	{
		std::string lib_name = std::string("lib") + camelToSnake(action_name_) + ".so";
		auto action_path = findLibrary(lib_name);

		if (action_path)
		{
			action_full_path_ = *action_path + "/" + lib_name;
		}
		else
		{
			throw CREATE_TEMOTO_ERROR_STACK("Plugin for action '" + action_name + "' not found");
		}
	}

	~ActionPlugin()
	{
		plugin_instance_.reset();
		create_plugin_.reset();
	}

	boost::shared_ptr<T> get()
	{
        return plugin_instance_;
    }

	void load()
	try
	{
		create_plugin_ = std::make_shared<loader_t>(boost::dll::import_alias<boost::shared_ptr<T>()>(
        	boost::dll::fs::path(action_full_path_),
			action_name_,
			boost::dll::load_mode::append_decorations
		));

		plugin_instance_ = (*create_plugin_)();
	}
	catch(const std::exception& e)
	{
		std::cout << __func__ << ": " << e.what() << std::endl;
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

	std::optional<std::string> findLibrary(const std::string& library_name) const
	{
		std::string paths(std::getenv("LD_LIBRARY_PATH"));

		if (paths.empty())
		{
			throw CREATE_TEMOTO_ERROR_STACK("LD_LIBRARY_PATH environment variable is not set.");
		}

		for (const auto& dir : split(paths, ":"))
		{
			if (std::filesystem::exists(std::filesystem::path(dir) / library_name))
			{
				return dir;
			}
		}

		return std::nullopt;
	}

	std::vector<std::string> split(std::string s, const std::string& delimiter) const
	{
		std::vector<std::string> tokens;
		size_t pos = 0;
		std::string token;

		while ((pos = s.find(delimiter)) != std::string::npos)
		{
			token = s.substr(0, pos);
			tokens.push_back(token);
			s.erase(0, pos + delimiter.length());
		}
		tokens.push_back(s);

		return tokens;
	}

	std::string action_name_;
	std::string action_full_path_;
	boost::shared_ptr<T> plugin_instance_;
	std::shared_ptr<loader_t> create_plugin_;
};
