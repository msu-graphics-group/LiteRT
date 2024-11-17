#include <filesystem>
#include <iostream>

#include <stl.h>
#include "Timer.h"

using namespace profiling;

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: stl <path_to_stl>" << std::endl;
    return 0;
  }
  std::filesystem::path path = argv[1];

  std::cout << "Parsing started..." << std::endl;
  auto timer = Timer();

  stl_reader::StlMesh <float, unsigned int> mesh;
  try { mesh = stl_reader::StlMesh(path); }
  catch (std::exception& e) {
      std::cout << "[Error] Failed to read the file.\n" << e.what() << std::endl;
  }

  auto time = timer.getElapsedTime();
  std::cout << "Parsing finished successfully." << std::endl;
  std::cout << "Parsing time: " << time.asSeconds() << "s." << std::endl;

  for(size_t itri = 0; itri < mesh.num_tris(); ++itri) {
       std::cout << "coordinates of triangle " << itri << ": ";
       for(size_t icorner = 0; icorner < 3; ++icorner) {
         const float* c = mesh.tri_corner_coords(itri, icorner);
         std::cout << "(" << c[0] << ", " << c[1] << ", " << c[2] << ") ";
       }
       std::cout << std::endl;

       const float* n = mesh.tri_normal(itri);
       std::cout << "normal of triangle " << itri << ": "
                 << "(" << n[0] << ", " << n[1] << ", " << n[2] << ")\n";
  }

  return 0;
}
