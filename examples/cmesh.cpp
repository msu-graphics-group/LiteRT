#include <filesystem>
#include <iostream>

#include "cmesh4.h"
#include "Timer.h"
#include <LiteMath.h>

using namespace LiteMath;
using namespace profiling;

std::ostream& operator<<(std::ostream& out, float4 v) {
    return out << "(" << v.x << " " << v.y << " " << v.z << ")";
}

int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: stl <path_to_stl>" << std::endl;
    return 0;
  }
  const char *path = argv[1];

  std::cout << "Parsing started..." << std::endl;
  auto timer = Timer();

  bool success;
  auto mesh = cmesh4::LoadMeshFromSTL(path, success);
  if (!success) {
      std::cout << "[Error] Failed to read the file." << std::endl;
      return 1;
  }

  auto time = timer.getElapsedTime();
  std::cout << "Parsing finished successfully." << std::endl;
  std::cout << "Parsing time: " << time.asSeconds() << "s." << std::endl;

  std::cout << "VERTICES:" << std::endl;
  for (size_t i = 0; i < mesh.VerticesNum(); i++) {
      std::cout << i << " " << mesh.vPos4f[i] << std::endl;
  }
  std::cout << std::endl;

  std::cout << "NORMALS: " << std::endl;
  for (size_t i = 0; i < mesh.TrianglesNum(); i++) {
      std::cout << i << " " << mesh.vNorm4f[i] << std::endl;
  }
  std::cout << std::endl;

  std::cout << "INDICES:" << std::endl;
  for (size_t i = 0; i < mesh.VerticesNum(); i++) {
      std::cout << mesh.indices[i] << " ";
      if ((i + 1) % 3 == 0) std::cout << std::endl;
  }

  return 0;
}
