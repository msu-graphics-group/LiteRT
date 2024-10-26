#pragma once
#include "LiteScene/cmesh4.h"
#include <map>

namespace cmesh4
{
  using LiteMath::float4;
  using LiteMath::float3;
  using LiteMath::uint3;

  struct TriangleListGrid
  {
    struct Node
    {
      std::vector<uint32_t> triangle_ids; //ids of all triangles to intersect this node's bbox
      uint32_t payload; //left empty by constructor, can be used in different algorithms
    };
    uint3 size;
    std::vector<Node> nodes;
    float3 min_pos, max_pos; //AABB for mesh
  };

  //Octree always represents unit cube [-1,1]^3
  struct TriangleListOctree 
  {
    struct Node
    {
      uint32_t offset;// offset for children (they are stored together). 0 offset means it's a leaf
      uint32_t tid_offset;// start of Node's triangles ids in triangle_ids list, only for leaves
      uint32_t tid_count;// how many triangles intersect this node
    };
    std::vector<Node> nodes;
    std::vector<uint32_t> triangle_ids;
  };

  float3 closest_point_triangle(const float3 &p, const float3 &a, const float3 &b, const float3 &c);
  float3 barycentric(const float3 &p, const float3 &a, const float3 &b, const float3 &c);
  float get_signed_distance(const cmesh4::SimpleMesh &mesh, const TriangleListGrid &grid, const float3 &pos);
  void get_bbox(const cmesh4::SimpleMesh &mesh, float3 *min_pos, float3 *max_pos);
  bool triangle_aabb_intersect(const float3 &a, const float3 &b, const float3 &c, 
                               const float3 &aabb_center, const float3 &aabb_half_size);

  TriangleListGrid create_triangle_list_grid(const cmesh4::SimpleMesh &mesh, uint3 grid_size);
  TriangleListOctree create_triangle_list_octree(const cmesh4::SimpleMesh &mesh, unsigned max_depth, 
                                                 unsigned max_triangles_per_leaf = 4, float search_range_mult = 3);

  //rescales mesh with constant scale to fit it inside the given box and returns transform that does it
  LiteMath::float4x4 rescale_mesh(cmesh4::SimpleMesh &mesh, float3 min_pos, float3 max_pos);

  void transform_mesh(cmesh4::SimpleMesh &mesh, LiteMath::float4x4 transform);

  //checks if mesh is watertight (it is required to build proper SDF from it)
  bool check_watertight_mesh(const cmesh4::SimpleMesh& mesh, bool verbose = false);
  cmesh4::SimpleMesh removing_holes(cmesh4::SimpleMesh& mesh, int& ind, bool& fl);
  cmesh4::SimpleMesh before_removing_holes(cmesh4::SimpleMesh mesh, int& ind, bool& fl);
  void compress_close_vertices(cmesh4::SimpleMesh &mesh, double threshold, bool verbose);
  void fix_normals(cmesh4::SimpleMesh &mesh, bool verbose);

  //checks for mesh defects and issues, notifies about them and tries to fix (e.g. wrong normal direction) 
  //it also rescales mesh so that the AABB is fit inside [-1,1]^3, and returns transform that does it
  LiteMath::float4x4 normalize_mesh(cmesh4::SimpleMesh &mesh, bool verbose = false);
}