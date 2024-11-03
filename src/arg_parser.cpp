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

#include "temoto_action_engine/arg_parser.h"
#include <exception>
#include <boost/algorithm/string.hpp>

namespace action_engine
{
namespace po = boost::program_options;

ArgParser::ArgParser(int argc, char** argv)
: description_(po::options_description("Allowed options"))
{
	description_.add_options()
		("actor-name", po::value<std::string>(), "Required. Main wake word.")
		("actions-path", po::value<std::string>(), "Required. Action packages base path")
		("extra-wake-words", po::value<std::string>(), "Optional. Additional wake words. Indicates to which wake words the action engine will respond to.");

	po::store(po::parse_command_line(argc, argv, description_), vm_);
	po::notify(vm_);

	parseArgs();
}

bool ArgParser::parseActorName()
{
	if (vm_.count("actor-name"))
	{
		actor_name_ = vm_["actor-name"].as<std::string>();
		wake_words_.push_back(actor_name_);
		return true;
	}
	return false;
}

bool ArgParser::parseExtraWakeWords()
{
	if (vm_.count("extra-wake-words"))
	{
		std::string wake_words_str = vm_["extra-wake-words"].as<std::string>();
		boost::replace_all(wake_words_str, " ", "");
		boost::split(wake_words_, wake_words_str, boost::is_any_of(","));
		return true;
	}
	return false;
}

bool ArgParser::parseActionsPath()
{
	if (vm_.count("actions-path"))
	{
		action_paths_.push_back(vm_["actions-path"].as<std::string>());
		return true;
	}
	return false;
}

void ArgParser::parseArgs()
{
	if (!parseExtraWakeWords())
	{
		// No extra wake words
	}
	if (!parseActorName())
	{
		std::stringstream ss;
		ss << "Missing the main wake word\n" << description_;
		throw std::runtime_error(ss.str());
	}
	if(!parseActionsPath())
	{
		std::stringstream ss;
		ss << "Missing base path to TeMoto actions\n" << description_;
		throw std::runtime_error(ss.str());
	}
}

const std::vector<std::string>& ArgParser::getWakeWords() const
{
	return wake_words_;
}

const std::vector<std::string>& ArgParser::getActionPaths() const
{
	return action_paths_;
}
}
