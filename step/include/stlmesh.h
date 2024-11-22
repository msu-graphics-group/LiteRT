#pragma once

#include "LiteScene/cmesh4.h"

namespace cmesh4 {
    SimpleMesh LoadMeshFromSTL(const char* filepath, bool& success);
};
