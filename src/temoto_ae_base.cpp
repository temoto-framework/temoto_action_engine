#include <iostream>
#include <future>
#include <vector>
#include <map>
#include <mutex>
#include <sstream>
#include <assert.h>
#include <fstream>
#include <string>

#include "temoto_action_engine/action_executor.h"
#include "temoto_action_engine/temoto_error.h"
#include "temoto_action_engine/umrf_json_converter.h"
#include "temoto_action_engine/messaging.h"

const std::string app_name = "AE_test";

int main(int argc, char** argv)
{
  if (!(argc == 2 || argc == 4))
  {
    std::cout << "Missing argument for json base path\n";
    return 1;
  }

  std::string base_path(argv[1]);
  std::vector<std::string> umrf_names = {"umrf_0.json", "umrf_1.json", "umrf_2.json", "umrf_3.json"};
  std::vector<Umrf> umrf_vec;

  /*
   * Read the UMRF jsons and parse them to Umrf
   */ 
  bool parsing_success = true;
  for (const auto& json_name : umrf_names)
  {
    std::string umrf_full_path = base_path  + json_name;
    std::ifstream ifs(umrf_full_path);
    std::string umrf_json_str;
    umrf_json_str.assign(std::istreambuf_iterator<char>(ifs), std::istreambuf_iterator<char>());

    try
    {
      Umrf umrf = umrf_json_converter::fromUmrfJsonStr(umrf_json_str);
      TEMOTO_PRINT_OF("Parsed " + umrf.getFullName(), app_name);
      std::cout << umrf;
      umrf_vec.emplace_back(umrf);
    }
    catch(const std::exception& e)
    {
      TEMOTO_PRINT_OF("Parsing problems with " + umrf_full_path, app_name);
      std::cerr << e.what() << '\n';
      parsing_success = false;
    }
  }

  if (!parsing_success)
  {
    return 1;
  }

  TEMOTO_PRINT_OF("All UMRFs parsed successfully", app_name);

  /*
   * Execute the UMRFs via Action Executor
   */
  ActionExecutor ae;
  try
  {
    ae.addUmrfGraph("umrf_graph_0", umrf_vec);
    TEMOTO_PRINT_OF("UMRF graph initialized", app_name);

    ae.executeUmrfGraph("umrf_graph_0");
    TEMOTO_PRINT_OF("UMRF graph executed successfully", app_name);
  }
  catch(TemotoErrorStack e)
  {
    std::cout << e.what() << '\n';
  }

  /*
   * Just go to sleep and let the Action Engine do it's job
   */
  while (true)
  {
    TEMOTO_PRINT_OF("sleeping ...", app_name);
    std::this_thread::sleep_for(std::chrono::milliseconds(1000));
  }

  return 0;

  // /*
  //  * Check if the output of the executed action sequence is equal to the 
  //  * expected output
  //  */ 
  // bool test_failed = false;
  // // for (unsigned int i=0; i<expected_action_parameters.getParameterCount(); i++)
  // // {
  // //   if (expected_action_parameters.ge)
  // // }
  // for (const auto& action_parameter : aem.getLastActionParameters().getParameterMap())
  // {
  //   // Check if parameter is there
  //   int expected_parameter = expected_action_parameters.getParameter(action_parameter.first);

  //   std::cout << "exp: " << expected_parameter << "; got: " << action_parameter.second << std::endl;

  //   // Check if the parameters are equal
  //   if (action_parameter.second != expected_parameter)
  //   {
  //     test_failed = true;
  //   }
  // }

  // if (test_failed)
  // {
  //   return 1;
  // }
  // else
  // {
  //   /*
  //    * Return 3 (random unused exit value) because segfaults etc do not
  //    * return a process exit code, which is then ironically evaluated to 0 (success)
  //    */  
  //   return 3;
  // }
}
