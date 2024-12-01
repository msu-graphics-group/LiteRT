#pragma once
#include "mesh.h"

namespace cmesh4
{
  cmesh4::SimpleMesh LoadMeshFromPly(const std::string& path, bool verbose = false);
}