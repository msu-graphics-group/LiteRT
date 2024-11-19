#include <filesystem>
#include <iostream>

#include <step.h>
#include <LiteMath.h>
#include "Timer.h"

using namespace STEP;
using namespace profiling;

int main(int argc, const char **argv) {
  if (argc != 3) {
    std::cout << "Usage: argsplit <path_to_step> <entity_ID>" << std::endl;
    return 0;
  }
  std::filesystem::path path = argv[1];
  uint id = std::stoi(argv[2]);  

  std::cout << "Parsing started..." << std::endl;
  auto timer = Timer();

  bool exists;
  STEP::Parser parser(path, exists);
  if (!exists) {
      std::cout << "[Error] Parsing failed. File does not exist." << std::endl;
      return 1;
  }

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
