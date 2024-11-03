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

#ifndef TEMOTO_ACTION_ENGINE__ARG_PARSER_H
#define TEMOTO_ACTION_ENGINE__ARG_PARSER_H

#include <string>
#include <vector>
#include <boost/program_options.hpp>

namespace action_engine
{
namespace po = boost::program_options;

class ArgParser
{
public:
	ArgParser(int argc, char** argv);

	bool parseActorName();

	bool parseExtraWakeWords();

	bool parseActionsPath();

	void parseArgs();

	const std::vector<std::string>& getWakeWords() const;

	const std::vector<std::string>& getActionPaths() const;

private:
	std::string actor_name_;
	std::vector<std::string> wake_words_;
	std::vector<std::string> action_paths_;
	po::variables_map vm_;
	po::options_description description_;
};
}

#endif
