#include "iou.h"
#include "rand.h"
namespace IoU
{
  static float trilinear_interpolation(const float values[8], float3 dp)
  {
    return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
          (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
          (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
          (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
          (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
          (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
          (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
          (  dp.x)*(  dp.y)*(  dp.z)*values[7];
  }

  bool is_pos_inside(const std::vector<SdfFrameOctreeNode> &frame_nodes, float3 pos)
  {
    unsigned idx = 0;
    while (frame_nodes[idx].offset != 0)
    {
      unsigned vert = 0;

      if (pos.x > 0.5)
      {
        vert += 4;
        pos.x = (pos.x - 0.5) * 2.0;
      }
      else pos.x *= 2.0;

      if (pos.y > 0.5)
      {
        vert += 2;
        pos.y = (pos.y - 0.5) * 2.0;
      }
      else pos.y *= 2.0;

      if (pos.z > 0.5)
      {
        vert += 1;
        pos.z = (pos.z - 0.5) * 2.0;
      }
      else pos.z *= 2.0;

      idx = frame_nodes[idx].offset + vert;
    }
    return trilinear_interpolation(frame_nodes[idx].values, pos) <= 0;
  }

  float IoU_frame_octree(const std::vector<SdfFrameOctreeNode> &frame_nodes, sdf_converter::MultithreadedDistanceFunction sdf, unsigned points)
  {
    unsigned inter = 0, uni = 0;
    for (unsigned i = 0; i < points; ++i)
    {
      float3 pos = float3(urand(-1.0, 1.0), urand(-1.0, 1.0), urand(-1.0, 1.0));
      float3 coeff = (pos + 1) / 2.0;
      if (sdf(pos, 0) > 0) 
      {
        if (is_pos_inside(frame_nodes, coeff)) ++uni;
      }
      else
      {
        ++uni;
        if (is_pos_inside(frame_nodes, coeff)) ++inter;
        else printf("%f %f %f\n", pos.x, pos.y, pos.z);
      }
    }
    if (uni > 0)
    {
      return float(inter) / float(uni);
    }
    return 1;
  }
}