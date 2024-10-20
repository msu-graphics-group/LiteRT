#ifndef NURBS_SRC_UTILS
#define NURBS_SRC_UTILS
#include "Surface.hpp"

LiteMath::float3 get_center_of_mass(const RBezierGrid &surf);
float get_sphere_bound(const RBezierGrid &surface, const LiteMath::float3 &center);

#endif 