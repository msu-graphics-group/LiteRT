#include <fstream>
#include <chrono>
#include <filesystem>
#include <sstream>
#include <map>

#include <stp_parser.hpp>
#include <LiteMath.h>

using namespace STEP;

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: parse_and_div <path_to_stp_file>" << std::endl;
    return 0;
  }
  std::filesystem::path stp_path = argv[1];
  std::cout << "Parsing started..." << std::endl;
  auto tick_start = std::chrono::high_resolution_clock::now();
  auto entities = STEP::parse(stp_path);
  std::map<uint, RawNURBS> nurbsTable = STEP::allIDNurbs(entities);
  auto tick_end = std::chrono::high_resolution_clock::now();
  std::cout << "Parsing finished successfully." << std::endl;
  float time = 
      std::chrono::duration_cast<std::chrono::milliseconds>(tick_end-tick_start).count()/1000.0f;
  std::cout << "Parsing time: " << time << "s." << std::endl;

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
