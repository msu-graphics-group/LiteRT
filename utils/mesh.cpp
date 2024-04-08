#include "mesh.h"

namespace cmesh4
{
  using namespace LiteMath;

  void get_bbox(const cmesh4::SimpleMesh &mesh, float3 *min_pos, float3 *max_pos)
  {
    *min_pos = float3(1e9,1e9,1e9);
    *max_pos = float3(-1e9,-1e9,-1e9);

    for (const float4 &p : mesh.vPos4f)
    {
      *min_pos = min(*min_pos, to_float3(p));
      *max_pos = max(*max_pos, to_float3(p));
    }
  }

  void rescale_mesh(cmesh4::SimpleMesh &mesh, float3 min_pos, float3 max_pos)
  {
    assert(mesh.vPos4f.size() >= 3);

    float3 mesh_min, mesh_max;
    get_bbox(mesh, &mesh_min, &mesh_max);

    float3 mesh_size = mesh_max - mesh_min;
    float3 target_size = max_pos - min_pos;
    float3 scale3 = target_size/mesh_size;
    float scale = min(scale3.x, min(scale3.y, scale3.z));
    
    float4 scale_4 = float4(scale,scale,scale,1);
    float4 min_4 = to_float4(min_pos, 0.0f);
    float4 mesh_min_4 = to_float4(mesh_min, 0.0f);

    //changing poditions, .w coord is preserved
    for (float4 &p : mesh.vPos4f)
      p = min_4 + scale_4*(p - mesh_min_4);

    //it is only move and rescale, so now changes to normals are required
  }
}