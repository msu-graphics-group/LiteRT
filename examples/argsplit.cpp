#include <filesystem>

#include <stp_parser.hpp>
#include <LiteMath.h>
#include "Timer.h"

using namespace STEP;
using namespace profiling;

int main(int argc, const char **argv) {
  if (argc != 3) {
    std::cout << "Usage: parse_and_print <path_to_stp_file> <entity_ID>" << std::endl;
    return 0;
  }
  std::filesystem::path stp_path = argv[1];
  uint id = std::stoi(argv[2]);  

  std::cout << "Parsing started..." << std::endl;
  auto timer = Timer();

  STEP::Parser parser(stp_path);
  Entity entity = parser.getEntity(id);

  auto time = timer.getElapsedTime();
  std::cout << "Parsing finished successfully." << std::endl;
  std::cout << "Parsing time: " << time.asSeconds() << "s." << std::endl;

  std::cout << "#" << entity.id << " = " << STEP::type2str(entity.type) << std::endl;
  for (auto arg : entity.args) {
      std::cout << arg << std::endl;
  }
  return 0;
}
