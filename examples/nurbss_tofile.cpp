#include <fstream>
#include <filesystem>
#include <sstream>

#include <stp_parser.hpp>
#include <LiteMath.h>
#include "Timer.h"

using namespace STEP;
using namespace profiling;

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: parse_and_div <path_to_stp_file>" << std::endl;
    return 0;
  }
  std::filesystem::path stp_path = argv[1];
  std::cout << "Parsing started..." << std::endl;
  auto timer = Timer();

  STEP::Parser parser(stp_path);
  auto nurbsTable = parser.allIDNurbs();

  auto time = timer.getElapsedTime();
  std::cout << "Parsing finished successfully." << std::endl;
  std::cout << "Parsing time: " << time.asSeconds() << "s." << std::endl;

  for (auto &entry: nurbsTable) {
    uint ID = entry.first;
    auto nurbs = entry.second;

    std::stringstream ss;
    ss << stp_path.filename().replace_extension("").c_str() << ID << ".nurbss";
    std::ofstream cout(ss.str());
    cout << nurbs;
  }
  
  return 0;
}
