#include "radiance_field.h"
#include <cassert>
#include <fstream>
#include <iostream>

void load_rf_scene(RFScene &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);

  fs.read((char *)&scene.size, sizeof(int));
  fs.read((char *)&scene.scale, sizeof(float));
  
  scene.data.resize(scene.size *  scene.size * scene.size * CellSize);

  fs.read((char *)(scene.data.data()), scene.size *  scene.size * scene.size * CellSize * sizeof(float));
  fs.close();

  std::cout << "Done loading from disk" << std::endl;
}
