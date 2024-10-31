#include <filesystem>
#include <chrono>

#include <stp_parser.hpp>
#include <LiteMath.h>

using namespace STEP;

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: parse_and_print <path_to_stp_file>" << std::endl;
    return 0;
  }
  std::filesystem::path stp_path = argv[1];
  std::cout << "Parsing started..." << std::endl;
  auto tick_start = std::chrono::high_resolution_clock::now();

  STEP::Parser parser(stp_path);
  auto nurbsTable = parser.allIDNurbs();

  auto tick_end = std::chrono::high_resolution_clock::now();
  std::cout << "Parsing finished successfully." << std::endl;
  float time = 
      std::chrono::duration_cast<std::chrono::milliseconds>(tick_end-tick_start).count()/1000.0f;
  std::cout << "Parsing time: " << time << "s." << std::endl;

  for (auto &entry: nurbsTable) {
    auto ID = entry.first;
    auto nurbs = entry.second;
    std::cout << "### NURBS " << ID << " ###" << std::endl;
    std::cout << nurbs << std::endl;
  }
  return 0;
}
