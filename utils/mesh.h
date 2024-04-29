#pragma once
#include "LiteScene/cmesh4.h"

namespace cmesh4
{
  using LiteMath::float4;
  using LiteMath::float3;
  using LiteMath::uint3;

  struct TriangleListGrid
  {
    struct Node
    {
      std::vector<unsigned> triangleIds; //ids of all triangles to intersect this node's bbox
      uint32_t payload; //left empty by constructor, can be used in different algorithms
    };
    uint3 size;
    std::vector<Node> nodes;
    float3 min_pos, max_pos; //AABB for mesh
  };

  float3 closest_point_triangle(const float3 &p, const float3 &a, const float3 &b, const float3 &c);
  float get_signed_distance(const cmesh4::SimpleMesh &mesh, const TriangleListGrid &grid, const float3 &pos);
  void get_bbox(const cmesh4::SimpleMesh &mesh, float3 *min_pos, float3 *max_pos);
  bool triangle_aabb_intersect(const float3 &a, const float3 &b, const float3 &c, 
                               const float3 &aabb_center, const float3 &aabb_half_size);

  //rescales mesh with constant scale to fit it inside the given box and returns transform that does it
  LiteMath::float4x4 rescale_mesh(cmesh4::SimpleMesh &mesh, float3 min_pos, float3 max_pos);

  TriangleListGrid create_triangle_list_grid(const cmesh4::SimpleMesh &mesh, uint3 grid_size);
}