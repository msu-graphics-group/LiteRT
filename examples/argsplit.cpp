#include <filesystem>
#include <chrono>

#include <stp_parser.hpp>
#include <LiteMath.h>

using namespace STEP;

int main(int argc, const char **argv) {
  if (argc != 3) {
    std::cout << "Usage: parse_and_print <path_to_stp_file> <entity_ID>" << std::endl;
    return 0;
  }
  std::filesystem::path stp_path = argv[1];
  uint id = std::stoi(argv[2]);  

  std::cout << "Parsing started..." << std::endl;
  auto tick_start = std::chrono::high_resolution_clock::now();

  STEP::Parser parser(stp_path);
  Entity entity = parser.getEntity(id);

  auto tick_end = std::chrono::high_resolution_clock::now();
  std::cout << "Parsing finished successfully." << std::endl;
  float time = 
      std::chrono::duration_cast<std::chrono::milliseconds>(tick_end-tick_start).count()/1000.0f;
  std::cout << "Parsing time: " << time << "s." << std::endl;

  std::cout << "#" << entity.id << " = " << STEP::type2str(entity.type) << std::endl;
  for (auto arg : entity.args) {
      std::cout << arg << std::endl;
  }
  return 0;
}
