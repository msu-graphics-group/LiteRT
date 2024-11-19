#include <fstream>
#include <filesystem>
#include <sstream>
#include <iostream>

#include <step.h>
#include <LiteMath.h>
#include "Timer.h"

using namespace STEP;
using namespace profiling;

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: nurbss_tofile <path_to_step>" << std::endl;
    return 0;
  }
  std::filesystem::path path = argv[1];
  std::cout << "Parsing started..." << std::endl;
  auto timer = Timer();

  bool exists;
  STEP::Parser parser(path, exists);
  if (!exists) {
      std::cout << "[Error] Parsing failed. File does not exist." << std::endl;
      return 1;
  }

  auto nurbsTable = parser.allIDNurbs();

  auto time = timer.getElapsedTime();
  std::cout << "Parsing finished successfully." << std::endl;
  std::cout << "Parsing time: " << time.asSeconds() << "s." << std::endl;

  for (auto &entry: nurbsTable) {
    uint ID = entry.first;
    auto nurbs = entry.second;

    std::stringstream ss;
    ss << path.filename().replace_extension("").c_str() << ID << ".nurbss";
    std::ofstream cout(ss.str());
    cout << nurbs;
  }
  
  return 0;
}
