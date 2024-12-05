#include <iostream>

static float urand(float from=0, float to=1)
{
  return ((double)std::rand() / RAND_MAX) * (to - from) + from;
}