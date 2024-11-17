#include <filesystem>

#include <step.h>
#include <LiteMath.h>
#include "Timer.h"

using namespace STEP;
using namespace profiling;

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: parse_and_print <path_to_stp_file>" << std::endl;
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
    auto ID = entry.first;
    auto nurbs = entry.second;
    std::cout << "### NURBS " << ID << " ###" << std::endl;
    std::cout << nurbs << std::endl;
  }
  return 0;
}
