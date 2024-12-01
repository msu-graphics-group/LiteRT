#ifndef NURBS_SRC_DEBUG
#define NURBS_SRC_DEBUG

#include <iostream>
#include <vector>

#include "LiteMath.h"

using namespace LiteMath;

namespace debug {
  enum class LOG {
    INFO,
    PASSED,
    FAILED
  };

  void title(std::string name);
  void test(std::string name, bool statement);
  std::ostream& operator<<(std::ostream& cout, LOG message);
};

std::ostream& operator<<(std::ostream& cout, std::vector<float> vector);
std::ostream& operator<<(std::ostream& cout, uint4 vector);
std::ostream& operator<<(std::ostream& cout, int4 vector);
std::ostream& operator<<(std::ostream& cout, float4 vector);
std::ostream& operator<<(std::ostream& cout, uint3 vector);
std::ostream& operator<<(std::ostream& cout, int3 vector);
std::ostream& operator<<(std::ostream& cout, float3 vector);
std::ostream& operator<<(std::ostream& cout, uint2 vector);
std::ostream& operator<<(std::ostream& cout, int2 vector);
std::ostream& operator<<(std::ostream& cout, float2 vector);
std::ostream& operator<<(std::ostream& cout, complex vector);

#endif
