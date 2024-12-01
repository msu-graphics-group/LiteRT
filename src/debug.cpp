#include <iostream>
#include <vector>
#include <string>

#include "LiteMath.h"
#include "debug.hpp"

using namespace LiteMath;

namespace debug {
  
  void title(std::string name) {
    std::cout << LOG::INFO << name << std::endl;
  }

  void test(std::string name, bool statement) {
    LOG message;
    if (statement) message = LOG::PASSED;
    else message = LOG::FAILED;
    std::cout << message << name << std::endl;
  }

  std::ostream& operator<<(std::ostream& cout, LOG message) {
    if (message == LOG::INFO)
      return cout << "\x1B[37m[INFO]\033[0m\t ";
    else if (message == LOG::PASSED)
      return cout << "\x1B[32m[PASSED]\033[0m ";
    else if (message == LOG::FAILED)
      return cout << "\x1B[31m[FAILED]\033[0m ";
    return cout << "[?]\t\t";
  }
};

std::ostream& operator<<(std::ostream& cout, std::vector<float> vector) {
  cout << "{ ";
  for (auto x : vector)
    cout << x << " ";
  return cout << "}";
}

std::ostream& operator<<(std::ostream& cout, uint4 vector) {
  return cout << "(" << vector.x << " " << vector.y << " " << vector.z << " " << vector.w << ")";
}

std::ostream& operator<<(std::ostream& cout, int4 vector) {
  return cout << "(" << vector.x << " " << vector.y << " " << vector.z << " " << vector.w << ")";
}

std::ostream& operator<<(std::ostream& cout, float4 vector) {
  return cout << "(" << vector.x << " " << vector.y << " " << vector.z << " " << vector.w << ")";
}

std::ostream& operator<<(std::ostream& cout, uint3 vector) {
  return cout << "(" << vector.x << " " << vector.y << " " << vector.z << ")";
}

std::ostream& operator<<(std::ostream& cout, int3 vector) {
  return cout << "(" << vector.x << " " << vector.y << " " << vector.z << ")";
}

std::ostream& operator<<(std::ostream& cout, float3 vector) {
  return cout << "(" << vector.x << " " << vector.y << " " << vector.z << ")";
}

std::ostream& operator<<(std::ostream& cout, uint2 vector) {
  return cout << "(" << vector.x << " " << vector.y << ")";
}

std::ostream& operator<<(std::ostream& cout, int2 vector) {
  return cout << "(" << vector.x << " " << vector.y << ")";
}

std::ostream& operator<<(std::ostream& cout, float2 vector) {
  return cout << "(" << vector.x << " " << vector.y << ")";
}

std::ostream& operator<<(std::ostream& cout, complex vector) {
  return cout << "(" << vector.re << " " << vector.im << ")";
}
