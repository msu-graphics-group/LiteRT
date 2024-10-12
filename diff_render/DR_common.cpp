#include "DR_common.h"
#include "MultiRendererDR.h"
#include "../utils/sparse_octree_2.h"

#include <functional>
#include <cassert>
#include <chrono>

namespace dr
{
  static double urand(double from=0, double to=1)
  {
    return ((double)rand() / RAND_MAX) * (to - from) + from;
  }

  float circle_sdf(float3 center, float radius, float3 p)
  {
    return length(p - center) - radius;
  }
  float3 gradient_color(float3 p)
  {
    return  (1-p.x)*(1-p.y)*(1-p.z)*float3(1,0,0) + 
            (1-p.x)*(1-p.y)*(  p.z)*float3(1,0,0) + 
            (1-p.x)*(  p.y)*(1-p.z)*float3(0,1,0) + 
            (1-p.x)*(  p.y)*(  p.z)*float3(0,1,0) + 
            (  p.x)*(1-p.y)*(1-p.z)*float3(1,0,0) + 
            (  p.x)*(1-p.y)*(  p.z)*float3(1,0,0) + 
            (  p.x)*(  p.y)*(1-p.z)*float3(0,1,0) + 
            (  p.x)*(  p.y)*(  p.z)*float3(0,1,0);
  }

  float3 single_color(float3 p)
  {
    return float3(1,0,0);
  }

  //creates SBS where all nodes are present, i.e.
  //it is really a regular grid, but with more indexes
  //distance and color fields must be given
  //it is for test purposes only
  //SBS is created in [-1,1]^3 cube, as usual
  SdfSBS create_grid_sbs(unsigned brick_count, unsigned brick_size, 
                        std::function<float(float3)>  sdf_func,
                        std::function<float3(float3)> color_func)
  {
    unsigned v_size = brick_size+1;
    unsigned dist_per_node = v_size*v_size*v_size;
    unsigned colors_per_node = 8;
    unsigned p_count = brick_count*brick_size + 1u;
    unsigned c_count = brick_count + 1u;
    unsigned c_offset = p_count*p_count*p_count;

    SdfSBS scene;
    scene.header.brick_size = brick_size;
    scene.header.brick_pad  = 0;
    scene.header.bytes_per_value = 4;
    scene.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F;

    scene.values_f.resize(p_count*p_count*p_count + 3*c_count*c_count*c_count);
    scene.values.resize(brick_count*brick_count*brick_count*(dist_per_node+colors_per_node));
    scene.nodes.resize(brick_count*brick_count*brick_count);

    //fill the distances
    for (unsigned x = 0; x < p_count; x++)
    {
      for (unsigned y = 0; y < p_count; y++)
      {
        for (unsigned z = 0; z < p_count; z++)
        {
          unsigned idx = x*p_count*p_count + y*p_count + z;
          float3 p = 2.0f*(float3(x, y, z) / float3(brick_count*brick_size)) - 1.0f;
          scene.values_f[idx] = sdf_func(p);
        }
      }
    }

    //fill the colors
    for (unsigned x = 0; x < c_count; x++)
    {
      for (unsigned y = 0; y < c_count; y++)
      {
        for (unsigned z = 0; z < c_count; z++)
        {
          unsigned idx = x*c_count*c_count + y*c_count + z;
          float3 dp = float3(x, y, z) / float3(brick_count);
          float3 color = color_func(2.0f*dp - 1.0f);
          scene.values_f[c_offset + 3*idx + 0] = color.x;
          scene.values_f[c_offset + 3*idx + 1] = color.y;
          scene.values_f[c_offset + 3*idx + 2] = color.z;
        }
      }
    }

    //fill the nodes and indices
    for (unsigned bx = 0; bx < brick_count; bx++)
    {
      for (unsigned by = 0; by < brick_count; by++)
      {
        for (unsigned bz = 0; bz < brick_count; bz++)
        {

          //nodes
          unsigned n_idx = bx*brick_count*brick_count + by*brick_count + bz;
          unsigned offset = n_idx*(dist_per_node+colors_per_node);
          scene.nodes[n_idx].pos_xy = (bx << 16) | by;
          scene.nodes[n_idx].pos_z_lod_size = (bz << 16) | brick_count;
          scene.nodes[n_idx].data_offset = offset;

          //indices for distances
          for (unsigned x = 0; x < v_size; x++)
          {
            for (unsigned y = 0; y < v_size; y++)
            {
              for (unsigned z = 0; z < v_size; z++)
              {
                unsigned idx = x*v_size*v_size + y*v_size + z;
                unsigned val_idx = (bx*brick_size + x)*p_count*p_count + (by*brick_size + y)*p_count + (bz*brick_size + z);
                scene.values[offset + idx] = val_idx;
              }
            }
          }

          //indices for colors
          for (unsigned x = 0; x < 2; x++)
          {
            for (unsigned y = 0; y < 2; y++)
            {
              for (unsigned z = 0; z < 2; z++)
              {
                unsigned idx = x*2*2 + y*2 + z;
                unsigned val_idx = c_offset + 3*((bx + x)*c_count*c_count + (by + y)*c_count + (bz + z));
                scene.values[offset + dist_per_node + idx] = val_idx;
              }
            }
          }
        }
      }
    }

    return sdf_converter::SBS_ind_to_SBS_ind_with_neighbors(scene);  
  }

  SdfSBS circle_one_brick_scene()
  {
    return create_grid_sbs(1, 8, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);
  }

  SdfSBS circle_small_scene()
  {
    return create_grid_sbs(2, 4, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);
  }

  SdfSBS circle_medium_scene()
  {
    return create_grid_sbs(16, 4, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);
  }

  SdfSBS circle_smallest_scene()
  {
    return create_grid_sbs(1, 2, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          single_color);
  }

  SdfSBS circle_smallest_scene_colored()
  {
    return create_grid_sbs(1, 2, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);    
  }

  SdfSBS circle_smallest_scene_colored_2()
  {
    return create_grid_sbs(1, 4, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);    
  }

   SdfSBS two_circles_scene()
  {
    return create_grid_sbs(8, 4, 
                           [&](float3 p){return std::min(circle_sdf(float3(0,0.3,0.5), 0.5f, p),
                                                         circle_sdf(float3(0,-0.3,-0.5), 0.5f, p));}, 
                           [](float3 p){return circle_sdf(float3(0,0.3,0.5), 0.5f, p) > circle_sdf(float3(0,-0.3,-0.5), 0.5f, p) ? float3(1,0,0) : float3(0,0,1);});
  }

  std::vector<float4x4> get_cameras_uniform_sphere(int count, float3 center, float radius)
  {
    std::vector<float4x4> cameras;
    for (int i = 0; i < count; i++)
    {
      float phi = 2 * LiteMath::M_PI * urand();
      float psi = (LiteMath::M_PI / 2) * (1 - sqrtf(urand()));
      if (urand() > 0.5)
        psi = -psi;

      float3 view_dir = float3(cos(psi) * sin(phi), sin(psi), cos(psi) * cos(phi));
      float3 tangent = normalize(cross(view_dir, -float3(0, 1, 0)));
      float3 new_up = normalize(cross(view_dir, tangent));
      cameras.push_back(LiteMath::lookAt(center - radius * view_dir, center, new_up));
    }

    return cameras;
  }

  std::vector<float4x4> get_cameras_turntable(int count, float3 center, float radius, float height)
  {
    std::vector<float4x4> cameras;
    for (int i = 0; i < count; i++)
    {
      float phi = (2.0f * LiteMath::M_PI * i) / count;

      float3 view_dir = float3(sin(phi), -height/radius, cos(phi));
      float3 tangent = normalize(cross(view_dir, -float3(0, 1, 0)));
      float3 new_up = normalize(cross(view_dir, tangent));
      cameras.push_back(LiteMath::lookAt(center - radius * view_dir, center, new_up));
    }

    return cameras;
  }

  void randomize_color(SdfSBS &sbs)
  {
    int v_size = sbs.header.brick_size + 2 * sbs.header.brick_pad + 1;
    int dist_per_node = v_size * v_size * v_size;
    for (auto &n : sbs.nodes)
    {
      for (int i = 0; i < 8; i++)
      {
        unsigned off = sbs.values[n.data_offset + dist_per_node + i];
        for (int j = 0; j < 3; j++)
          sbs.values_f[off + j] = urand();
      }
    }
  }

  void randomize_distance(SdfSBS &sbs, float delta)
  {
    int v_size = sbs.header.brick_size + 2 * sbs.header.brick_pad + 1;
    int dist_per_node = v_size * v_size * v_size;
    for (auto &n : sbs.nodes)
    {
      for (int i = 0; i < dist_per_node; i++)
      {
        sbs.values_f[sbs.values[n.data_offset + i]] += delta*urand(-1,1);
      }
    }
  }
}