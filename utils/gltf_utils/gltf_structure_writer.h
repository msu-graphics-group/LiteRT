#pragma once
#include "gltf_structure.h"

namespace gltf
{
  class GltfStructureWriter
  {
  public:
    bool write_to_json(FullData &FullData, std::string name);

  private:
  };
}