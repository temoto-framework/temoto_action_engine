#ifndef UMRF_JSON_CONVERTER_NLJ
#define UMRF_JSON_CONVERTER_NLJ

#include "temoto_action_engine/umrf_node.h"
#include "temoto_action_engine/umrf_graph.h"

namespace action_engine
{

UmrfNode fromUmrfJsonStr(const std::string& umrf_json_str);

UmrfGraph fromUmrfGraphJsonStr(const std::string& ug_json_str);

std::string toUmrfJsonStr(const UmrfNode& u);

std::string toUmrfGraphJsonStr(const UmrfGraph& ug);

// UmrfNode fromUmrfJsonStr(const std::string& umrf_json_str, bool as_descriptor = false);

// UmrfGraph fromUmrfGraphJsonStr(const std::string& umrf_graph_json_str);

// ActionParameters::Parameters fromUmrfParametersJsonStr(const std::string umrf_param_json_str);

// std::string toUmrfJsonStr(const UmrfNode& umrf_node, bool as_descriptor = false);

// std::string toUmrfGraphJsonStr(const UmrfGraph& umrf_graph);

}

#endif