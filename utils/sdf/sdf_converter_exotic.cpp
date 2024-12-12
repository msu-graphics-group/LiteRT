#include "sdf_converter.h"
#include "utils/mesh/mesh_bvh.h"
#include "sparse_octree_builder.h"
#include "utils/mesh/mesh.h"
#include "omp.h"

#include <set>
#include <chrono>
#include <unordered_map>
#include <atomic>
#include <functional>

namespace sdf_converter
{
  struct PositionHasher;
  struct PositionEqual;
  
  static bool is_leaf(unsigned offset)
  {
    return (offset == 0) || (offset & INVALID_IDX);
  }

  void mesh_octree_to_psdf_frame_octree_rec(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree,
                                         std::vector<SdfFrameOctreeNode> &frame,
                                         unsigned idx, float3 p, float d)
  {
    unsigned ofs = tl_octree.nodes[idx].offset;
    frame[idx].offset = ofs;

    if (is_leaf(ofs)) 
    {
      float3 pos = 2.0f*(d*p) - 1.0f;
      bool is_dif_sgn = false;
      int8_t first_sgn = 0;

      float min_mid_dist_sq = 1000;
      int min_mid_ti = -1;
      float3 mid_pos = pos + d*float3(1, 1, 1);
      for (int i = 0; i < 8; i++)
      {
        float3 ch_pos = pos + 2*d*float3((i >> 2) & 1, (i >> 1) & 1, i & 1);
        float min_dist_sq = 1000;
        int min_ti = -1;
        for (int j=0; j<tl_octree.nodes[idx].tid_count; j++)
        {
          int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+j];

          float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
          float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
          float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
          float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
          float dist_sq = LiteMath::dot(vt, vt);

          if (dist_sq < min_dist_sq)
          {
            min_dist_sq = dist_sq; 
            min_ti = t_i;
          }

          if (i == 0)
          {
            vt = mid_pos - cmesh4::closest_point_triangle(mid_pos, a, b, c);
            dist_sq = LiteMath::dot(vt, vt);

            if (dist_sq < min_mid_dist_sq)
            {
              min_mid_dist_sq = dist_sq; 
              min_mid_ti = t_i;
            }
          }
        }

        if (min_ti >= 0)
        {
          float3 a = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+0]]);
          float3 b = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+1]]);
          float3 c = to_float3(mesh.vPos4f[mesh.indices[3*min_ti+2]]);
          float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
          float3 n = (1.0f/3.0f)*(to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+0]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+1]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_ti+2]]));

          frame[idx].values[i] = dot(normalize(n), vt) > 0 ? sqrt(min_dist_sq) : -sqrt(min_dist_sq);
          if (i == 0)
          {
            first_sgn = dot(normalize(n), vt) > 0 ? 1 : -1;
          }
          else if (!is_dif_sgn && first_sgn * dot(normalize(n), vt) <= 0)
          {
            is_dif_sgn = true;
            //printf("%d %f\n", i, first_sgn * dot(normalize(n), vt));
          }
        }
        else
        {
          frame[idx].values[i] = 1000;
          is_dif_sgn = true;
        }
      }
      if (!is_dif_sgn)
      {
        for (int i = 0; i < 8; ++i)
        {
          float3 ch_pos = pos + 2*d*float3((i >> 2) & 1, (i >> 1) & 1, i & 1);
          float3 a = to_float3(mesh.vPos4f[mesh.indices[3*min_mid_ti+0]]);
          float3 b = to_float3(mesh.vPos4f[mesh.indices[3*min_mid_ti+1]]);
          float3 c = to_float3(mesh.vPos4f[mesh.indices[3*min_mid_ti+2]]);
          float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
          float3 n = (1.0f/3.0f)*(to_float3(mesh.vNorm4f[mesh.indices[3*min_mid_ti+0]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_mid_ti+1]]) + 
                                  to_float3(mesh.vNorm4f[mesh.indices[3*min_mid_ti+2]]));
          if (frame[idx].values[i] * dot(normalize(n), vt) < 0)
          {
            frame[idx].values[i] = -frame[idx].values[i];
          }
          //printf("checked\n");
        }
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        mesh_octree_to_psdf_frame_octree_rec(mesh, tl_octree, frame, ofs + i, ch_p, ch_d);
      }
    }
  }

  void mesh_octree_to_psdf_frame_octree(const cmesh4::SimpleMesh &mesh,
                                       const cmesh4::TriangleListOctree &tl_octree, 
                                       std::vector<SdfFrameOctreeNode> &out_frame)
  {
    out_frame.resize(tl_octree.nodes.size());
    mesh_octree_to_psdf_frame_octree_rec(mesh, tl_octree, out_frame, 0, float3(0,0,0), 1);
  }

  std::vector<SdfFrameOctreeNode> create_psdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
    std::vector<SdfFrameOctreeNode> frame;
    mesh_octree_to_psdf_frame_octree(mesh, tlo, frame);
    // frame_octree_limit_nodes(frame, settings.nodes_limit, true);
    return frame;
  }

  SdfSBS create_sdf_SBS_col(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                            const std::vector<MultiRendererMaterial> &materials_lib, 
                            const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib, bool noisy)
  {
    SdfSBS sbs = create_sdf_SBS_tex(settings, header, mesh, noisy);

    //same header, except for layout
    //same nodes and offsets (UV and RGB occupy the same space)
    //same distance data, but instead of 16-bit UV, use 8-bit RGB (32 bit with padding, padding is 0xFF, so it can be used as alpha)
    sbs.header.aux_data = SDF_SBS_NODE_LAYOUT_DX_RGB8;


    auto &tex = textures_lib[mat_id];
    for (auto &n : sbs.nodes)
    {
      unsigned vals_per_int = 4/header.bytes_per_value;
      unsigned v_size = header.brick_size + 2*header.brick_pad + 1;
      unsigned dist_size = (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;
      unsigned off = n.data_offset + dist_size;
      for (unsigned i = 0; i < 8; i++)
      {
        unsigned uv_comp = sbs.values[off + i];
        float u = (1.0/65535.0) * (uv_comp >> 16);
        float v = (1.0/65535.0) * (uv_comp & 0xFFFF);
        //printf("id = %u, u = %f, v = %f\n", n.data_offset, u, v);
        float4 f_color = tex->sample(float2(u, v));
        uint4  u_color = uint4(0xFF * clamp(f_color, float4(0, 0, 0, 0), float4(1, 1, 1, 1)));
        sbs.values[off + i] = (0xFF << 24) | (u_color.z << 16) | (u_color.y << 8) | u_color.x;
      }
    }

    return sbs;
  }

  uint32_t sbs_adapt_node_metric(SdfSBSAdaptNode &node, SdfSBSAdaptHeader &header)
  {
    uint32_t metric = sizeof(node);
    uint32_t elem = header.bytes_per_value;
    if ((header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_DX_UV16) metric += 8 * 4;
    else if ((header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_DX_RGB8) metric += 8 * 4;
    else if ((header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F) 
    {
      metric += 8 * sizeof(float); 
      elem = sizeof(float);
    }
    else if ((header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN)
    {
      metric += 8 * sizeof(float); 
      elem = sizeof(float);
    }
    uint32_t x = ((node.vox_count_xyz_pad & 0xFF000000) >> 24) + 1;
    uint32_t y = ((node.vox_count_xyz_pad & 0x00FF0000) >> 16) + 1;
    uint32_t z = ((node.vox_count_xyz_pad & 0x0000FF00) >> 8) + 1;
    metric += x * y * z * elem;
    return metric;
  }

  uint32_t metrica_f(uint8_t x_size, uint8_t y_size, uint8_t z_size)
  {
    return (x_size + 1) * (y_size + 1) * (z_size + 1) * (sizeof(float) + sizeof(uint32_t)) + sizeof(SdfSBSAdaptNode);
  }

  struct PositionHasher
  {
    std::size_t operator()(const float3& k) const
    {
      // Compute individual hash values for first,
      // second and third and combine them using XOR
      // and bit shifting:

      return (  (std::hash<float>()(k.x)
              ^ (std::hash<float>()(k.y) << 1)) >> 1)
              ^ (std::hash<float>()(k.z) << 1);
    }
  };

  struct PositionEqual
  {
    bool operator()(const float3& lhs, const float3& rhs) const
    {
      return std::abs(lhs.x - rhs.x) < 1e-12f && std::abs(lhs.y - rhs.y) < 1e-12f && std::abs(lhs.z - rhs.z) < 1e-12f;
    }
  };

  struct SizeCmp
{
  bool operator()(int3 lhs, int3 rhs) const
  {
    int f_l = lhs.x, s_l = lhs.y, t_l = lhs.z, f_r = rhs.x, s_r = rhs.y, t_r = rhs.z;
    if (f_l < s_l)
    {
      f_l = f_l ^ s_l;
      s_l = f_l ^ s_l;
      f_l = f_l ^ s_l;
    }
    if (f_l < t_l)
    {
      f_l = f_l ^ t_l;
      t_l = f_l ^ t_l;
      f_l = f_l ^ t_l;
    }
    if (s_l < t_l)
    {
      t_l = t_l ^ s_l;
      s_l = t_l ^ s_l;
      t_l = t_l ^ s_l;
    }

    if (f_r < s_r)
    {
      f_r = f_r ^ s_r;
      s_r = f_r ^ s_r;
      f_r = f_r ^ s_r;
    }
    if (f_r < t_r)
    {
      f_r = f_r ^ t_r;
      t_r = f_r ^ t_r;
      f_r = f_r ^ t_r;
    }
    if (s_r < t_r)
    {
      t_r = t_r ^ s_r;
      s_r = t_r ^ s_r;
      t_r = t_r ^ s_r;
    }
    if (f_l != f_r) return f_l < f_r;
    if (s_l != s_r) return s_l < s_r;
    return t_l < t_r;
  }
};

  void div_block(SdfSBSAdapt &sbs, MultithreadedDistanceFunction sdf, 
                 std::unordered_map<float3, float, PositionHasher, PositionEqual> &pos_to_val,
                 uint16_t x_b, uint16_t y_b, uint16_t z_b,
                 uint16_t x_st, uint16_t y_st, uint16_t z_st,
                 uint8_t x_sz, uint8_t y_sz, uint8_t z_sz)
  {
    uint16_t max_sz = std::max(std::max(x_sz, y_sz), z_sz) * x_st;
    float d_max = 2.0 * sqrt(3.0) * ((float)max_sz / (float)SDF_SBS_ADAPT_MAX_UNITS);
    uint32_t max_val = sbs.header.bytes_per_value == 4 ? 0xFFFFFFFF : (1 << (8 * sbs.header.bytes_per_value)) - 1;
    uint8_t x_min = x_sz, x_max = 0, y_min = y_sz, y_max = 0, z_min = z_sz, z_max = 0;
    uint32_t cnt = 0;
    std::vector<uint8_t> x_imps(x_sz), y_imps(y_sz), z_imps(z_sz);
    std::fill(x_imps.begin(), x_imps.end(), 0);
    std::fill(y_imps.begin(), y_imps.end(), 0);
    std::fill(z_imps.begin(), z_imps.end(), 0);
    for (uint8_t x = 0; x < x_sz; ++x)
    {
      for (uint8_t y = 0; y < y_sz; ++y)
      {
        for (uint8_t z = 0; z < z_sz; ++z)
        {
          bool is_first = true, is_vox_imp = false;
          float sgn = 0;

          for (uint16_t x_neigh = 0; x_neigh <= 1; ++x_neigh)
          {
            for (uint16_t y_neigh = 0; y_neigh <= 1; ++y_neigh)
            {
              for (uint16_t z_neigh = 0; z_neigh <= 1; ++z_neigh)
              {
                auto key = 2.0 * float3{(x_b + (x + x_neigh) * x_st) / (float)SDF_SBS_ADAPT_MAX_UNITS, 
                                   (y_b + (y + y_neigh) * y_st) / (float)SDF_SBS_ADAPT_MAX_UNITS, 
                                   (z_b + (z + z_neigh) * z_st) / (float)SDF_SBS_ADAPT_MAX_UNITS} - 1.0;
                if (pos_to_val.find(key) == pos_to_val.end())
                {
                  pos_to_val[key] = sdf(key, 0);
                  //printf("%.10f %.10f %.10f %.10f %.10f\n", key.x, key.y, key.z, pos_to_val[key], sdf(key, 0));
                }
                if (is_first)
                {
                  sgn = pos_to_val[key];
                  //printf("%f %f\n", pos_to_val[key], sdf(key, 0));
                  is_first = false;
                }
                else if (sgn * pos_to_val[key] <= 0)
                {
                  //printf("%f %f %f - %f %f %f\n", sgn, pos_to_val[key], sdf(key, 0), key.x, key.y, key.z);
                  is_vox_imp = true;
                  break;
                }
              }
              if (is_vox_imp) break;
            }
            if (is_vox_imp) break;
          }
          if (is_vox_imp)
          {
            if (x_min > x) x_min = x;
            if (x_max < x) x_max = x;
            if (y_min > y) y_min = y;
            if (y_max < y) y_max = y;
            if (z_min > z) z_min = z;
            if (z_max < z) z_max = z;
            ++x_imps[x];
            ++y_imps[y];
            ++z_imps[z];
            ++cnt;
          }

        }
      }
    }
    /*x_min = 0;
    y_min = 0;
    z_min = 0;
    x_max = 7;
    y_max = 7;
    z_max = 7;*/
    if (cnt == 0) 
    {
      /*printf("-----------------\n");
      for (uint8_t x = 0; x <= x_sz; ++x)
      {
        for (uint8_t y = 0; y <= y_sz; ++y)
        {
          for (uint8_t z = 0; z <= z_sz; ++z)
          {
            auto key = 2.0 * float3{(x_b + (x) * x_st) / (float)SDF_SBS_ADAPT_MAX_UNITS, 
                                   (y_b + (y) * y_st) / (float)SDF_SBS_ADAPT_MAX_UNITS, 
                                   (z_b + (z) * z_st) / (float)SDF_SBS_ADAPT_MAX_UNITS} - 1.0;
            printf("%f ", pos_to_val[key]);
          }
          printf("\n");
        }
        printf("\n");
      }
      printf("-----------------\n");*/
      return;
    }

    uint32_t metric_min = metrica_f(x_max - x_min + 1, y_max - y_min + 1, z_max - z_min + 1);
    bool is_div = false;
    struct brick_data
    {
      uint16_t x_b, y_b, z_b;
      uint8_t x_sz, y_sz, z_sz;
    };
    brick_data b1, b2;

    for (uint8_t idx = x_min + 1; idx < x_max + 1; ++idx)
    {
      uint8_t idx2 = idx;
      if (x_imps[idx - 1] != 0)
      {
        while (idx2 < x_imps.size() && x_imps[idx2] == 0)
        {
          ++idx2;
        }
        uint32_t new_m = metrica_f(x_max - idx2 + 1, y_max - y_min + 1, z_max - z_min + 1) + 
                         metrica_f(idx - x_min, y_max - y_min + 1, z_max - z_min + 1);
        if (new_m < metric_min)
        {
          metric_min = new_m;
          is_div = true;
          b1.x_b = x_min;
          b1.y_b = y_min;
          b1.z_b = z_min;
          b1.x_sz = idx - x_min;
          b1.y_sz = y_max - y_min + 1;
          b1.z_sz = z_max - z_min + 1;

          b2.x_b = idx2;
          b2.y_b = y_min;
          b2.z_b = z_min;
          b2.x_sz = x_max - idx2 + 1;
          b2.y_sz = y_max - y_min + 1;
          b2.z_sz = z_max - z_min + 1;
        }
      }
    }

    for (uint8_t idx = y_min + 1; idx < y_max + 1; ++idx)
    {
      uint8_t idx2 = idx;
      if (y_imps[idx - 1] != 0)
      {
        while (idx2 < y_imps.size() && y_imps[idx2] == 0)
        {
          ++idx2;
        }
        uint32_t new_m = metrica_f(x_max - x_min + 1, y_max - idx2 + 1, z_max - z_min + 1) + 
                         metrica_f(x_max - x_min + 1, idx - y_min, z_max - z_min + 1);
        if (new_m < metric_min)
        {
          metric_min = new_m;
          is_div = true;
          b1.x_b = x_min;
          b1.y_b = y_min;
          b1.z_b = z_min;
          b1.x_sz = x_max - x_min + 1;
          b1.y_sz = idx - y_min;
          b1.z_sz = z_max - z_min + 1;

          b2.x_b = x_min;
          b2.y_b = idx2;
          b2.z_b = z_min;
          b2.x_sz = x_max - x_min + 1;
          b2.y_sz = y_max - idx2 + 1;
          b2.z_sz = z_max - z_min + 1;
        }
      }
    }

    for (uint8_t idx = z_min + 1; idx < z_max + 1; ++idx)
    {
      uint8_t idx2 = idx;
      if (z_imps[idx - 1] != 0)
      {
        while (idx2 < z_imps.size() && z_imps[idx2] == 0)
        {
          ++idx2;
        }
        uint32_t new_m = metrica_f(x_max - x_min + 1, y_max - y_min + 1, z_max - idx2 + 1) + 
                         metrica_f(x_max - x_min + 1, y_max - y_min + 1, idx - z_min);
        if (new_m < metric_min)
        {
          metric_min = new_m;
          is_div = true;
          b1.x_b = x_min;
          b1.y_b = y_min;
          b1.z_b = z_min;
          b1.x_sz = x_max - x_min + 1;
          b1.y_sz = y_max - y_min + 1;
          b1.z_sz = idx - z_min;

          b2.x_b = x_min;
          b2.y_b = y_min;
          b2.z_b = idx2;
          b2.x_sz = x_max - x_min + 1;
          b2.y_sz = y_max - y_min + 1;
          b2.z_sz = z_max - idx2 + 1;
        }
      }
    }
    //if (x_sz == 8 && y_sz == 8 && z_sz == 8) printf("-------\n");
    if (is_div)
    {
      div_block(sbs, sdf, pos_to_val, 
                x_b + b1.x_b * x_st, y_b + b1.y_b * y_st, z_b + b1.z_b * z_st, 
                x_st, y_st, z_st, b1.x_sz, b1.y_sz, b1.z_sz);
      div_block(sbs, sdf, pos_to_val, 
                x_b + b2.x_b * x_st, y_b + b2.y_b * y_st, z_b + b2.z_b * z_st, 
                x_st, y_st, z_st, b2.x_sz, b2.y_sz, b2.z_sz);
      //if (x_sz == 8 && y_sz == 8 && z_sz == 8) printf("-------\n");
      return;
    }
    SdfSBSAdaptNode node;
    node.data_offset = sbs.values.size();
    node.pos_xy = ((uint32_t)(x_b + x_min * x_st) << 16) | (uint32_t)(y_b + y_min * y_st);
    node.pos_z_vox_size = ((uint32_t)(z_b + z_min * z_st) << 16) | x_st;
    node.vox_count_xyz_pad = ((uint32_t)(x_max - x_min + 1) << 16) | 
                             ((uint32_t)(y_max - y_min + 1) << 8) | 
                              (uint32_t)(z_max - z_min + 1);
    for (uint16_t x_off = x_min; x_off <= x_max + 1; ++x_off)
    {
      for (uint16_t y_off = y_min; y_off <= y_max + 1; ++y_off)
      {
        for (uint16_t z_off = z_min; z_off <= z_max + 1; ++z_off)
        {
          auto key = 2.0 * float3{(x_b + (x_off) * x_st) / (float)SDF_SBS_ADAPT_MAX_UNITS, 
                                  (y_b + (y_off) * y_st) / (float)SDF_SBS_ADAPT_MAX_UNITS, 
                                  (z_b + (z_off) * z_st) / (float)SDF_SBS_ADAPT_MAX_UNITS} - 1.0;
          if (pos_to_val.find(key) == pos_to_val.end())
          {
            //printf("~~~ %.10f %.10f %.10f %.10f\n", key.x, key.y, key.z, sdf(key, 0));
            pos_to_val[key] = sdf(key, 0);
          }
          //printf("%.10f %.10f %.10f %.10f %.10f\n", key.x, key.y, key.z, pos_to_val[key], sdf(key, 0));
          unsigned d_compressed = std::max(0.0f, max_val*((pos_to_val[key]+d_max)/(2*d_max)));
          d_compressed = std::min(d_compressed, max_val);
          sbs.values.push_back(d_compressed);
          //printf("%f %f %u ~ %u %f %u %u\n", ((float)pos_to_val[key] + (float)d_max) / (2.0 * (float)d_max), ((float)max_val * ((float)pos_to_val[key] + (float)d_max) / (2.0 * (float)d_max)), (uint32_t)((float)max_val * ((float)pos_to_val[key] + (float)d_max) / (2.0 * (float)d_max)), d_max, 2.0 * sqrt(3.0) * ((float)max_sz / (float)SDF_SBS_ADAPT_MAX_UNITS), max_sz, SDF_SBS_ADAPT_MAX_UNITS);
          //printf("%.8f %u ~~ %u %u %u -- %u %u %u\n", sbs.values[sbs.values.size() - 1], pos_to_val[key], x_b + x_off * x_st, y_b + y_off * y_st, z_b + z_off * z_st, x_max - x_min + 1, y_max - y_min + 1, z_max - z_min + 1);
        }
      }
    }
    sbs.nodes.push_back(node);
    //if (x_sz == 8 && y_sz == 8 && z_sz == 8) printf("-------\n");
    return;
  }

  SdfSBSAdapt greed_sbs_adapt(MultithreadedDistanceFunction sdf, uint8_t depth)
  {
    SdfSBSAdapt sbs;
    sbs.header.aux_data = SDF_SBS_NODE_LAYOUT_DX;
    sbs.header.bytes_per_value = 4;
    sbs.header.brick_pad = 0;

    sbs.nodes = {};
    sbs.values = {};
    sbs.values_f = {};

    std::vector<uint32_t> imp_vox_cnt_nodes;
    //std::map<std::pair<std::pair<uint16_t, uint16_t>, uint16_t>, float> pos_to_val;
    std::unordered_map<float3, float, PositionHasher, PositionEqual> pos_to_val;
    //std::map<std::pair<std::pair<uint16_t, uint16_t>, uint16_t>, uint32_t> pos_to_idx;
    std::map<int3, unsigned, SizeCmp> different_nodes;
    if (depth > 12) depth = 12;
    uint16_t vox_size = (1u << (12 - depth));

    for (uint16_t x = 0; x < SDF_SBS_ADAPT_MAX_UNITS; x += 16 * vox_size)
    {
      for (uint16_t y = 0; y < SDF_SBS_ADAPT_MAX_UNITS; y += 16 * vox_size)
      {
        for (uint16_t z = 0; z < SDF_SBS_ADAPT_MAX_UNITS; z += 16 * vox_size)
        {
          /*uint32_t imp_vox_cnt = 0;
          for (uint16_t x_off = 0; x_off < 8; ++x_off)
          {
            for (uint16_t y_off = 0; y_off < 8; ++y_off)
            {
              for (uint16_t z_off = 0; z_off < 8; ++z_off)
              {
                float sgn = 0;
                bool is_first = true;
                bool is_vox_imp = false;
                float3 pos = float3((float)(x + x_off) / (float)0x8000, (float)(y + y_off) / (float)0x8000, (float)(z + z_off) / (float)0x8000);


                for (uint16_t x_neigh = 0; x_neigh <= 1; ++x_neigh)
                {
                  for (uint16_t y_neigh = 0; y_neigh <= 1; ++y_neigh)
                  {
                    for (uint16_t z_neigh = 0; z_neigh <= 1; ++z_neigh)
                    {
                      key = std::pair(std::pair(x + x_off + x_neigh, y + y_off + y_neigh), z + z_off + z_neigh);
                      if (pos_to_val.find(key) == pos_to_val.end())
                      {
                        float3 pos = float3((float)(x + x_off + x_neigh) / (float)0x8000, 
                                            (float)(y + y_off + y_neigh) / (float)0x8000, 
                                            (float)(z + z_off + z_neigh) / (float)0x8000);
                        pos = pos * 2.0f - 1.0f;
                        pos_to_val[key] = sdf(pos, 0);
                      }
                      if (is_first)
                      {
                        sgn = pos_to_val[key];
                        is_first = false;
                      }
                      else if (sgn * pos_to_val[key] <= 0)
                      {
                        is_vox_imp = true;
                        break;
                      }
                    }
                    if (is_vox_imp) break;
                  }
                  if (is_vox_imp) break;
                }


              }
            }
          }

          if (imp_vox_cnt > 0)
          {
            bool is_div = false;

            //

            //using seq vectors

            if (!is_div)
            {
              SdfSBSAdaptNode node;
              node.data_offset = sbs.values.size();
              node.pos_xy = ((uint32_t)x << 16) | (uint32_t)y;
              node.pos_z_vox_size = ((uint32_t)z << 16) | 1;//maybe change vox size later
              node.vox_count_xyz_pad = ((uint32_t)8 << 24) | ((uint32_t)8 << 16) | ((uint32_t)8 << 8);
              for (uint16_t x_off = 0; x_off <= 8; ++x_off)
              {
                for (uint16_t y_off = 0; y_off <= 8; ++y_off)
                {
                  for (uint16_t z_off = 0; z_off <= 8; ++z_off)
                  {
                    key = std::pair(std::pair(x + x_off, y + y_off), z + z_off);
                    if (pos_to_idx.find(key) == pos_to_idx.end())
                    {
                      sbs.values.push_back(sbs.values_f.size());
                      pos_to_idx[key] = sbs.values_f.size();
                      sbs.values_f.push_back(pos_to_val[key]);
                    }
                    else
                    {
                      sbs.values.push_back(pos_to_idx[key]);
                    }
                  }
                }
              }
              sbs.nodes.push_back(node);
            }
          }*/
          //check all slices and find empties
          //if empty slice near border or have another one empty slice -> divide
          //if empty slice lonely -> check if we have better dividing after that dividing -> divide
          //we should delete this node and create different smaller nodes
          unsigned last_sz = sbs.nodes.size();
          div_block(sbs, sdf, pos_to_val, x, y, z, vox_size, vox_size, vox_size, 16, 16, 16);
          for (unsigned d = last_sz; d < sbs.nodes.size(); ++d)
          {
            int3 sz = int3{((sbs.nodes[d].vox_count_xyz_pad >> 16) & 0xFF), ((sbs.nodes[d].vox_count_xyz_pad >> 8) & 0xFF), (sbs.nodes[d].vox_count_xyz_pad & 0xFF)};
            if (different_nodes.find(sz) != different_nodes.end()) different_nodes[sz] += 1;
            else different_nodes[sz] = 1;
          }
        }
      }
    }


    for (auto i : different_nodes)
    {
      printf("NODE: x - %u, y - %u, z - %u, c - %u\n", i.first.x, i.first.y, i.first.z, i.second);
    }

    return sbs;
  }

  bool eq_float3(const float3& lhs, const float3& rhs)
  {
    return std::abs(lhs.x - rhs.x) < 1e-12f && std::abs(lhs.y - rhs.y) < 1e-12f && std::abs(lhs.z - rhs.z) < 1e-12f;
  }

  bool is_tri_have_same_edge(float3 a[3], float3 b[3], 
                             bool &norm_fixed)
  {
    for (int i = 0; i < 3; ++i)
    {
      for (int j = 0; j < 3; ++j)
      {
        if (eq_float3(a[i], b[j]) && eq_float3(a[(i + 1) % 3], b[(j + 1) % 3]))
        {
          norm_fixed = false;
          return true;
        }
        if (eq_float3(a[i], b[(j + 1) % 3]) && eq_float3(a[(i + 1) % 3], b[j]))
        {
          norm_fixed = true;
          return true;
        }
      }
    }
    norm_fixed = false;
    return false;
  }

  void mesh_octree_to_vmpdf_rec(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree,
                                         /*std::vector<SdfFrameOctreeNode> &frame,*/
                                         const std::function<void(unsigned idx, unsigned offset, 
                                                          float3 p, float d, 
                                                          std::vector<std::vector<float3>> groups, 
                                                          bool is_leaf, bool is_norm_fixed)> &creating,
                                         unsigned idx, float3 p, float d)
  {
    unsigned ofs = tl_octree.nodes[idx].offset;
    //frame[idx].offset = ofs;
    std::vector<std::vector<float3>> groups;
    bool norm_broken = false;
    if (is_leaf(ofs)) 
    {
      float3 pos = 2.0f*(d*p) - 1.0f;
      for (int j=0; j<tl_octree.nodes[idx].tid_count; j++)
      {
        int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+j];
        float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
        float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
        float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
        float3 t[3] = {a, b, c};
        bool is_find_group = false;
        int real_group = -1;
        bool first_check = false;
        std::vector<int> del_groups = {};
        std::vector<bool> del_checks = {};
        for (int i = 0; i < groups.size(); ++i)
        {
          //if (norm_broken) break;
          
          bool is_correct_norm = true;
          bool is_first = true;
          for (int u = 0; u < groups[i].size(); u += 3)
          {
            //if (norm_broken) break;

            bool check = false;
            float3 x[3] = {groups[i][u], groups[i][u+1], groups[i][u+2]};
            if (is_find_group)
            {
              if (is_tri_have_same_edge(t, x, check))
              {
                if (is_first)
                {
                  is_first = false;
                  is_correct_norm = check;
                  del_groups.push_back(i);
                  del_checks.push_back(check);
                }
                else if ((is_correct_norm && !check) || (!is_correct_norm && check))
                {
                  norm_broken = true;
                }
              }
            }
            else
            {
              if (is_tri_have_same_edge(t, x, check))
              {
                is_find_group = true;
                is_correct_norm = check;
                first_check = check;
                is_first = false;
                real_group = i;
              }
            }
          }
        }
        //if (norm_broken) break;

        if (!is_find_group)
        {
          groups.push_back({a, b, c});
        }
        else
        {
          if (first_check)
          {
            groups[real_group].push_back(a);
            groups[real_group].push_back(b);
            groups[real_group].push_back(c);
          }
          else
          {
            groups[real_group].push_back(b);
            groups[real_group].push_back(a);
            groups[real_group].push_back(c);
          }

          for (int i = del_checks.size() - 1; i >= 0; --i)
          {
            for (int k = 0; k < groups[del_groups[i]].size(); k += 3)
            {
              if ((del_checks[i] && first_check) || (!del_checks[i] && !first_check))
              {
                groups[real_group].push_back(groups[del_groups[i]][k+0]);
                groups[real_group].push_back(groups[del_groups[i]][k+1]);
                groups[real_group].push_back(groups[del_groups[i]][k+2]);
              }
              else
              {
                groups[real_group].push_back(groups[del_groups[i]][k+1]);
                groups[real_group].push_back(groups[del_groups[i]][k+0]);
                groups[real_group].push_back(groups[del_groups[i]][k+2]);
              }
            }
            groups.erase(groups.begin() + del_groups[i]);
          }
        }
      }

      creating(idx, ofs, p, d, groups, is_leaf(ofs), norm_broken);
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);

        creating(idx, ofs, p, d, groups, is_leaf(ofs), norm_broken);

        mesh_octree_to_vmpdf_rec(mesh, tl_octree, creating, ofs + i, ch_p, ch_d);
      }
    }
    //creating(idx, ofs, p, d, groups, is_leaf(ofs), norm_broken);
  }

  void mesh_octree_to_vmpdf(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree, 
                                         std::vector<SdfFrameOctreeNode> &out_frame)
  {
    out_frame.resize(tl_octree.nodes.size());
    std::function lambda = [&out_frame](unsigned idx, unsigned offset, 
                               float3 p, float d, 
                               std::vector<std::vector<float3>> groups, 
                               bool is_leaf, bool is_norm_broken) -> void
    {
      out_frame[idx].offset = offset;
      float3 pos = 2.0f*(d*p) - 1.0f;
      if (is_norm_broken)
      {
        for (int i = 0; i < 8; ++i)
        {
          out_frame[idx].values[i] = 0;
        }
        //printf("AAA\n");
      }
      else if (is_leaf)
      {
        /*if (groups.size() > 1)
        {
          for (auto x : groups)
          {
            for (int p = 0; p < x.size(); p += 3)
            {
              printf("%f %f %f\n", x[p+0].x, x[p+0].y, x[p+0].z);
              printf("%f %f %f\n", x[p+1].x, x[p+1].y, x[p+1].z);
              printf("%f %f %f\n", x[p+2].x, x[p+2].y, x[p+2].z);
              printf("\n");
            }
            printf("----\n");
          }
        }*/
        for (int i = 0; i < 8; i++)
        {
          float3 ch_pos = pos + 2*d*float3((i >> 2) & 1, (i >> 1) & 1, i & 1);
          float global_sign = 1, global_min_dist_sq = 1000;

          for (auto j : groups)
          {
            float local_sign = 1, local_min_dist_sq = 1000;
            for (int k = 0; k < j.size(); k += 3)
            {
              float3 a = j[k+0], b = j[k+1], c = j[k+2];
              float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
              float dst_sq = LiteMath::dot(vt, vt);
              if (local_min_dist_sq > dst_sq)
              {
                local_min_dist_sq = dst_sq;
                if (LiteMath::dot(LiteMath::cross(a - b, a - c), vt) < 0)
                {
                  local_sign = -1;
                }
                else
                {
                  local_sign = 1;
                }
              }
            }
            global_sign *= local_sign;
            if (global_min_dist_sq > local_min_dist_sq) global_min_dist_sq = local_min_dist_sq;
          }
          out_frame[idx].values[i] = global_sign * sqrt(global_min_dist_sq);
        }
      }
    };
    mesh_octree_to_vmpdf_rec(mesh, tl_octree, lambda, 0, float3(0,0,0), 1);
  }

  std::vector<SdfFrameOctreeNode> create_vmpdf_frame_octree(SparseOctreeSettings settings, 
                                                                     const cmesh4::SimpleMesh &mesh)
  {
    auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
    std::vector<SdfFrameOctreeNode> frame;
    mesh_octree_to_vmpdf(mesh, tlo, frame);
    //frame_octree_limit_nodes(frame, settings.nodes_limit, true);
    return frame;
  }
  
  SdfSBS SBS_col_to_SBS_ind(const SdfSBS &sbs)
  {
    SdfSBS sbs_ind;
    //same header, except for layout
    //same nodes and offsets (UV and RGB occupy the same space)
    //completely different data structure (it is now only indices) + 
    //additional array for actual values (floats)
    sbs_ind.header = sbs.header;
    sbs_ind.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F;

    sbs_ind.nodes = sbs.nodes;

    unsigned vals_per_int = 4/sbs.header.bytes_per_value;
    unsigned v_size = sbs.header.brick_size + 2*sbs.header.brick_pad + 1;
    unsigned dist_size = (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;
    unsigned bits = 8*sbs.header.bytes_per_value;
    unsigned max_val = sbs.header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);

    sbs_ind.values.resize(sbs.nodes.size()*(v_size*v_size*v_size + 8)); //v_size^3 indices for distances + 8 for textures
    unsigned f_offset = 0;
    unsigned i_offset = 0;

    std::unordered_map<float3, unsigned, PositionHasher, PositionEqual> distance_indices;
    std::unordered_map<float3, unsigned, PositionHasher, PositionEqual> color_indices;

    for (int n_idx = 0; n_idx < sbs.nodes.size(); n_idx++)
    {
      const SdfSBSNode &n = sbs.nodes[n_idx];
      float px = n.pos_xy >> 16;
      float py = n.pos_xy & 0x0000FFFF;
      float pz = n.pos_z_lod_size >> 16;
      float sz = n.pos_z_lod_size & 0x0000FFFF;
      float sz_inv = 2.0f/sz;
      float d = 2.0f/(sz*sbs.header.brick_size);

      sbs_ind.nodes[n_idx].data_offset = i_offset;

      unsigned col_off = n.data_offset + dist_size;
      for (unsigned i = 0; i < v_size*v_size*v_size; i++)
      {
        uint32_t v_off = n.data_offset;
        float d_max = 2*1.73205081f/sz;
        float mult = 2*d_max/max_val;
        float val = -d_max + mult*((sbs.values[n.data_offset + i/vals_per_int] >> (bits*(i%vals_per_int))) & max_val);

        uint3 voxelPos = uint3(i/(v_size*v_size), i/v_size%v_size, i%v_size);
        float3 pos = float3(-1,-1,-1) + 2.0f*(float3(px,py,pz)/sz + float3(voxelPos)/(sz*sbs.header.brick_size));
                      
        //printf("%u %u val = %f pos = %f %f %f\n", n.data_offset, i, val, pos.x, pos.y, pos.z);
        //printf("min_pos, pos, sz = %f %f %f %f %f %f %f\n", px, py, pz, pos.x, pos.y, pos.z, sz);
        auto it = distance_indices.find(pos);
        if (it == distance_indices.end())
        {
          sbs_ind.values_f.push_back(val);
          distance_indices[pos] = sbs_ind.values_f.size() - 1;
          sbs_ind.values[i_offset + i] = sbs_ind.values_f.size() - 1;
        }
        else
        {
          sbs_ind.values[i_offset + i] = it->second;
        }
      }
      for (unsigned i = 0; i < 8; i++)
      {
        unsigned col_packed = sbs.values[col_off + i];
        float3 f_color = float3(col_packed & 0xFF, (col_packed >> 8) & 0xFF, (col_packed >> 16) & 0xFF)/255.0f;

        float3 pos = float3(px, py, pz) + float3((i & 4) >> 2, (i & 2) >> 1, i & 1)/sz;
        auto it = distance_indices.find(pos);
        if (it == color_indices.end())
        {
          sbs_ind.values_f.push_back(f_color.x);
          sbs_ind.values_f.push_back(f_color.y);
          sbs_ind.values_f.push_back(f_color.z);
          color_indices[pos] = sbs_ind.values_f.size() - 3;
          sbs_ind.values[i_offset + v_size*v_size*v_size + i] = sbs_ind.values_f.size() - 3;
        }
        else
        {
          sbs_ind.values[i_offset + v_size*v_size*v_size + i] = it->second;
        }
      }
      i_offset += v_size*v_size*v_size + 8;
    }

    return sbs_ind;
  }

  SdfSBS create_sdf_SBS_indexed(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                                const std::vector<MultiRendererMaterial> &materials_lib, 
                                const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib, bool noisy)
  {
    SdfSBS sbs = create_sdf_SBS_col(settings, header, mesh, mat_id, materials_lib, textures_lib, noisy);
    return SBS_col_to_SBS_ind(sbs);
  }


  SdfSBS SBS_ind_to_SBS_ind_with_neighbors(const SdfSBS &sbs)
  {
    unsigned vals_per_int = 4/sbs.header.bytes_per_value;
    unsigned v_size = sbs.header.brick_size + 2*sbs.header.brick_pad + 1;
    unsigned dist_size = (v_size*v_size*v_size+vals_per_int-1)/vals_per_int;
    unsigned bits = 8*sbs.header.bytes_per_value;
    unsigned max_val = sbs.header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);

    unsigned values_per_node_old = v_size*v_size*v_size + 8;
    unsigned values_per_node_new = v_size*v_size*v_size + 8 + 27;
    unsigned nbr_offset =          v_size*v_size*v_size + 8;

    //check if given sbs in valid and has the required layout
    assert((sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F);
    assert(sbs.nodes.size() > 0);
    assert(sbs.values.size() == sbs.nodes.size()*values_per_node_old);
    assert(sbs.values_f.size() > 0);

    SdfSBS sbs_n;
    //same header, except for layout
    //same nodes, but different data offsets
    //different offset (sbs_ind.values)
    //same distance values (sbs_ind.values_f)
    sbs_n.header = sbs.header;
    sbs_n.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN;
    sbs_n.nodes = sbs.nodes;
    sbs_n.values_f = sbs.values_f;

    sbs_n.values.resize(sbs.nodes.size()*values_per_node_new);
    //change data_offsets in nodes, copy indices from sbs.values, leave neighbor indices empty
    for (int n = 0; n < sbs.nodes.size(); n++)
    {
      unsigned offset = sbs.nodes[n].data_offset;
      unsigned new_offset = n*values_per_node_new;

      sbs_n.nodes[n].data_offset = new_offset;
      for (int i = 0; i < values_per_node_old; i++)
        sbs_n.values[new_offset + i] = sbs.values[offset + i];

      for (int i = 0; i < 27; i++)
        sbs_n.values[new_offset + values_per_node_old + i] = INVALID_IDX;
    }

    struct AdjList
    {
      unsigned count = 0;
      unsigned node_ids[8] = {INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX, INVALID_IDX};
    };
    std::vector<AdjList> adj_lists(sbs.values_f.size());

    //for every corner of every node, find all other nodes with the same corner
    for (int n = 0; n < sbs.nodes.size(); n++)
    {
      unsigned offset = sbs.nodes[n].data_offset;
      //iterate only corners
      for (int r_id=0; r_id<8; r_id++)
      {
        uint3 idx = (v_size-1)*uint3((r_id & 4) >> 2, (r_id & 2) >> 1, r_id & 1);
        unsigned corner_id = sbs.values[sbs.nodes[n].data_offset + idx.x*v_size*v_size + idx.y*v_size + idx.z];
        adj_lists[corner_id].node_ids[r_id] = n;
        adj_lists[corner_id].count++;
      }
    }

    //if the corner is shared between two or more node, they are all neighbors of each other
    //currently we only consider nodes as neighbors if they have the same size
    for (int i = 0; i < sbs.values_f.size(); i++)
    {
      if (adj_lists[i].count <= 1)
        continue;
      for (int r_id1 = 0; r_id1 < 8; r_id1++)
      {
        if (adj_lists[i].node_ids[r_id1] == INVALID_IDX)
          continue;
        for (int r_id2 = 0; r_id2 < 8; r_id2++)
        {
          if (adj_lists[i].node_ids[r_id2] == INVALID_IDX || r_id1 == r_id2)
            continue;
          
          unsigned size_1 = sbs_n.nodes[r_id1].pos_z_lod_size & 0x0000FFFF;
          unsigned size_2 = sbs_n.nodes[r_id2].pos_z_lod_size & 0x0000FFFF;
          if (size_1 != size_2)
            continue;

          int3 idx1((r_id1 & 4) >> 2, (r_id1 & 2) >> 1, r_id1 & 1);
          int3 idx2((r_id2 & 4) >> 2, (r_id2 & 2) >> 1, r_id2 & 1);

          int3 d_idx = idx2 - idx1 + int3(1,1,1);
          unsigned neighbor_n = 3*3*d_idx.x + 3*d_idx.y + d_idx.z;
          unsigned node_id = adj_lists[i].node_ids[r_id2];
          unsigned neighbor_node_id = adj_lists[i].node_ids[r_id1];

          sbs_n.values[sbs_n.nodes[node_id].data_offset + nbr_offset + neighbor_n] = neighbor_node_id;
        }
      }
    }

    return sbs_n;
  }
  
  SdfSBS create_sdf_SBS_indexed_with_neighbors(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                                               const std::vector<MultiRendererMaterial> &materials_lib, 
                                               const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib)
  {
    SdfSBS sbs = create_sdf_SBS_indexed(settings, header, mesh, mat_id, materials_lib, textures_lib);
    return SBS_ind_to_SBS_ind_with_neighbors(sbs);
  }
 
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

  float node_RMSE_linear(const float node_values[8], MultithreadedDistanceFunction sdf, float3 corner, float3 offset)
  {
    float rmse = 0.0f;
    for (int x = 0; x <= 2; x += 1)
    {
      for (int y = 0; y <= 2; y += 1)
      {
        for (int z = 0; z <= 2; z += 1)
        {
          float3 off_idx = float3((float)x / 2.0, (float)y / 2.0, (float)z / 2.0);
          float coeff = 1.0;
          if (x == 1) coeff *= 2.0;
          if (y == 1) coeff *= 2.0;
          if (z == 1) coeff *= 2.0;
          float sdf_val = sdf(corner + offset * off_idx, 0);//TODO save it to cash and take
          rmse += pow((sdf_val - trilinear_interpolation(node_values, off_idx)), 2);
        }
      }
    }
    return sqrt(rmse / 64.0);
    
  }

  void fill_frame(SdfFrameOctreeNode &node, MultithreadedDistanceFunction sdf, float3 p, float d)//change for take interpolate values if it needs
  {
    float3 pos = 2.0f * (d * p) - 1.0f;
    for (int i = 0; i < 8; i++)
    {
      float3 ch_pos = pos + 2 * d * float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      node.values[i] = sdf(ch_pos, 0);
    }
  }

  struct dirs {unsigned neighbour[3*2]; float3 p; float d; unsigned parent;};//x+x-y+y-z+z-

  struct vert_casche {std::set<unsigned> voxels; float dist; bool is_interp; std::map<unsigned, unsigned> idx_2_vert;};

  void fill_neighbours_til_one_dir(std::vector<struct dirs> &directions, 
                                   const std::vector<SdfFrameOctreeNode> &result,
                                   unsigned idx, unsigned num, int dir, unsigned counted_idx, unsigned another_idx)
  {
    directions[result[idx].offset + num].neighbour[counted_idx] = result[idx].offset + num + dir;
    if (result[directions[idx].neighbour[another_idx]].offset != 0 && directions[idx].neighbour[another_idx] != 0) //was changed need to check
        directions[result[idx].offset + num].neighbour[another_idx] = 
        result[directions[idx].neighbour[another_idx]].offset + num + dir;
    else
        directions[result[idx].offset + num].neighbour[another_idx] = directions[idx].neighbour[another_idx];
  }

  void fill_all_neighbours_til_one_dir(std::vector<struct dirs> &directions, 
                                       const std::vector<SdfFrameOctreeNode> &result,
                                       unsigned idx, unsigned num, int dir, unsigned counted_idx, unsigned another_idx)
  {
    fill_neighbours_til_one_dir(directions, result, idx, num, dir, counted_idx, another_idx); 
    unsigned udir = 0, sgn = 0;
    if (dir < 0)
    {
      udir = -dir;
      sgn = 0;
    }
    else
    {
      udir = dir;
      sgn = udir;
    }
    if (directions[directions[result[idx].offset + num].neighbour[another_idx]].d <= directions[result[idx].offset + num].d)
    {
      directions[directions[result[idx].offset + num].neighbour[another_idx]].neighbour[counted_idx] = result[idx].offset + num;
      std::vector<unsigned> nodes = {directions[result[idx].offset + num].neighbour[another_idx]};
      for (unsigned n = 0; n < nodes.size(); ++n)
      {
        if (result[nodes[n]].offset != 0)
        {
          //printf("%u ", nodes[n]);
          for (unsigned number = 0; number < 8; ++number)
          {
            if ((number & udir) == sgn) 
            {
              directions[result[nodes[n]].offset + number].neighbour[counted_idx] = result[idx].offset + num;
              nodes.push_back(result[nodes[n]].offset + number);
              //printf("%u ", result[nodes[n]].offset + number);
            }
          }
          //printf("\n");
        }
      }
      //printf("\n");
    }
  }

  void interpolate_vertex(const std::vector<struct dirs> &directions, 
                                   std::vector<SdfFrameOctreeNode> &result,
                                   unsigned idx, unsigned num, unsigned dir, unsigned vert)
  {
    float3 p = directions[directions[result[idx].offset + num].neighbour[dir]].p;
    float d = directions[directions[result[idx].offset + num].neighbour[dir]].d;
    float3 ch_pos_1 = 2.0f * (d * p) - 1.0f;
    float3 ch_pos_2 = ch_pos_1 + 2 * d;

    p = directions[result[idx].offset + num].p;
    d = directions[result[idx].offset + num].d;
    float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);

    float3 coeff = 1.0f - (ch_pos_2 - ch_pos) / (ch_pos_2 - ch_pos_1);
    
    result[result[idx].offset + num].values[vert] = 
      trilinear_interpolation(result[directions[result[idx].offset + num].neighbour[dir]].values, coeff);
  }

  void interpolate_vertex(const std::vector<struct dirs> &directions, 
                                   std::vector<SdfFrameOctreeNode> &result,
                                   unsigned idx, unsigned num, unsigned dir1, unsigned dir2, unsigned vert)
  {
    float3 p = directions[directions[directions[result[idx].offset + num].neighbour[dir1]].neighbour[dir2]].p;
    float d = directions[directions[directions[result[idx].offset + num].neighbour[dir1]].neighbour[dir2]].d;
    float3 ch_pos_1 = 2.0f * (d * p) - 1.0f;
    float3 ch_pos_2 = ch_pos_1 + 2 * d;

    p = directions[result[idx].offset + num].p;
    d = directions[result[idx].offset + num].d;
    float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);

    float3 coeff = 1.0f - (ch_pos_2 - ch_pos) / (ch_pos_2 - ch_pos_1);
    
    result[result[idx].offset + num].values[vert] = 
      trilinear_interpolation(result[directions[directions[result[idx].offset + num].neighbour[dir1]].neighbour[dir2]].values, coeff);
  }

  void sdf_artefacts_fix(std::vector<SdfFrameOctreeNode> &frame, std::vector<struct dirs> &directions, 
                         std::unordered_map<float3, vert_casche, PositionHasher, PositionEqual> &distance_cache, MultithreadedDistanceFunction sdf, 
                         float eps, unsigned max_threads/*while unused*/, unsigned depth, std::set<unsigned> bad_nodes, bool is_smooth)
  {
    for (auto node : bad_nodes)
    {
      /*bool chk = true, chk2 = false;
      unsigned x = node;
      while (directions[x].parent != 0)
      {
        printf("%u, ", x);
        x = directions[x].parent;
      }
      printf("%u 0\n", x);*/
      for (int i = 0 ; i < 6; ++i)
      {
        unsigned neigh = directions[node].neighbour[i];
        if (frame[neigh].offset == 0)
        {
          int j = i + 1;
          if (i % 2 == 1) j = i - 1;
          while (directions[neigh].neighbour[j] != node)
          {
            frame[neigh].offset = frame.size();
            frame.resize(frame.size() + 8);
            directions.resize(directions.size() + 8);
            for (unsigned num = 0; num < 8; ++num)
            {
              //initialize
              frame[frame[neigh].offset + num].offset = 0;
              directions[frame[neigh].offset + num].p = 2 * directions[neigh].p + float3((num & 4) >> 2, (num & 2) >> 1, num & 1);
              directions[frame[neigh].offset + num].d = directions[neigh].d / 2.0;
              directions[frame[neigh].offset + num].parent = neigh;

              if ((num & 4) != 0) 
              {
                fill_all_neighbours_til_one_dir(directions, frame, neigh, num, -4, 1, 0); 
                /*if (directions[directions[frame[neigh].offset + num].neighbour[0]].d <= directions[frame[neigh].offset + num].d)
                {
                  directions[directions[frame[neigh].offset + num].neighbour[0]].neighbour[1] = frame[neigh].offset + num;
                } //TODO recount all children inside directions[frame[neigh].offset + num].neighbour[0]*/
                
              }
              else 
              {
                fill_all_neighbours_til_one_dir(directions, frame, neigh, num, 4, 0, 1);
                /*if (directions[directions[frame[neigh].offset + num].neighbour[1]].d <= directions[frame[neigh].offset + num].d)
                {
                  directions[directions[frame[neigh].offset + num].neighbour[1]].neighbour[0] = frame[neigh].offset + num;
                } */
              }

              if ((num & 2) != 0) 
              {
                fill_all_neighbours_til_one_dir(directions, frame, neigh, num, -2, 3, 2);
                /*if (directions[directions[frame[neigh].offset + num].neighbour[2]].d <= directions[frame[neigh].offset + num].d)
                {
                  directions[directions[frame[neigh].offset + num].neighbour[2]].neighbour[3] = frame[neigh].offset + num;
                } */
              }
              else 
              {
                fill_all_neighbours_til_one_dir(directions, frame, neigh, num, 2, 2, 3);
                /*if (directions[directions[frame[neigh].offset + num].neighbour[3]].d <= directions[frame[neigh].offset + num].d)
                {
                  directions[directions[frame[neigh].offset + num].neighbour[3]].neighbour[2] = frame[neigh].offset + num;
                } */
              }

              if ((num & 1) != 0) 
              {
                fill_all_neighbours_til_one_dir(directions, frame, neigh, num, -1, 5, 4);
                /*if (directions[directions[frame[neigh].offset + num].neighbour[4]].d <= directions[frame[neigh].offset + num].d)
                {
                  directions[directions[frame[neigh].offset + num].neighbour[4]].neighbour[5] = frame[neigh].offset + num;
                } */
              }
              else 
              {
                fill_all_neighbours_til_one_dir(directions, frame, neigh, num, 1, 4, 5);
                /*if (directions[directions[frame[neigh].offset + num].neighbour[5]].d <= directions[frame[neigh].offset + num].d)
                {
                  directions[directions[frame[neigh].offset + num].neighbour[5]].neighbour[4] = frame[neigh].offset + num;
                } */
              }
            }
            for (unsigned num = 0; num < 8; ++num)
            {
              /*for (int y = 0; y < 6; ++y)
              {
                printf("%u ", directions[frame[neigh].offset + num].neighbour[y]);
              }
              printf("\n");*/

              for (unsigned vert = 0; vert < 8; ++vert)
              {
                unsigned x_dir = (vert & 4) >> 2;
                unsigned y_dir = 2 + ((vert & 2) >> 1);
                unsigned z_dir = 4 + (vert & 1);

                float3 v_pos = 2.0 * directions[frame[neigh].offset + num].d * directions[frame[neigh].offset + num].p - 1.0 + 
                  2 * directions[frame[neigh].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);

                if (distance_cache.find(v_pos) != distance_cache.end() && !distance_cache[v_pos].is_interp)
                {
                  frame[frame[neigh].offset + num].values[vert] = distance_cache[v_pos].dist;
                  distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                  distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                }
                else if (is_smooth && directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].d > directions[frame[neigh].offset + num].d && directions[frame[neigh].offset + num].neighbour[x_dir ^ 1] != 0)
                {
                  //take vertex from x_dir neighbour
                  interpolate_vertex(directions, frame, neigh, num, x_dir ^ 1, vert);
                  
                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    if (frame[frame[neigh].offset + num].values[vert] != distance_cache[v_pos].dist)
                    {
                      distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }
                else if (is_smooth && directions[directions[frame[neigh].offset + num].neighbour[y_dir ^ 1]].d > directions[frame[neigh].offset + num].d && directions[frame[neigh].offset + num].neighbour[y_dir ^ 1] != 0)
                {
                  //take vertex from y_dir neighbour
                  interpolate_vertex(directions, frame, neigh, num, y_dir ^ 1, vert);
                  
                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    if (frame[frame[neigh].offset + num].values[vert] != distance_cache[v_pos].dist)
                    {
                      distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }
                else if (is_smooth && directions[directions[frame[neigh].offset + num].neighbour[z_dir ^ 1]].d > directions[frame[neigh].offset + num].d && directions[frame[neigh].offset + num].neighbour[z_dir ^ 1] != 0)
                {
                  //take vertex from z_dir neighbour
                  interpolate_vertex(directions, frame, neigh, num, z_dir ^ 1, vert);
                  
                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    if (frame[frame[neigh].offset + num].values[vert] != distance_cache[v_pos].dist)
                    {
                      distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }//TODO add diagonal checks and cache
                else if (is_smooth && directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].d <= directions[frame[neigh].offset + num].d && directions[frame[neigh].offset + num].neighbour[x_dir ^ 1] != 0 &&
                         directions[directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].neighbour[y_dir ^ 1]].d > directions[frame[neigh].offset + num].d && directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].neighbour[y_dir ^ 1] != 0)
                {
                  //take vertex from xy_dir neighbour
                  interpolate_vertex(directions, frame, neigh, num, x_dir ^ 1, y_dir ^ 1, vert);
                  
                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    if (frame[frame[neigh].offset + num].values[vert] != distance_cache[v_pos].dist)
                    {
                      distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }
                else if (is_smooth && directions[directions[frame[neigh].offset + num].neighbour[y_dir ^ 1]].d <= directions[frame[neigh].offset + num].d && directions[frame[neigh].offset + num].neighbour[y_dir ^ 1] != 0 &&
                         directions[directions[directions[frame[neigh].offset + num].neighbour[y_dir ^ 1]].neighbour[z_dir ^ 1]].d > directions[frame[neigh].offset + num].d && directions[directions[frame[neigh].offset + num].neighbour[y_dir ^ 1]].neighbour[z_dir ^ 1] != 0)
                {
                  //take vertex from yz_dir neighbour
                  interpolate_vertex(directions, frame, neigh, num, y_dir ^ 1, z_dir ^ 1, vert);
                  
                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    if (frame[frame[neigh].offset + num].values[vert] != distance_cache[v_pos].dist)
                    {
                      distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }
                else if (is_smooth && directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].d <= directions[frame[neigh].offset + num].d && directions[frame[neigh].offset + num].neighbour[x_dir ^ 1] != 0 &&
                         directions[directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].neighbour[z_dir ^ 1]].d > directions[frame[neigh].offset + num].d && directions[directions[frame[neigh].offset + num].neighbour[x_dir ^ 1]].neighbour[z_dir ^ 1] != 0)
                {
                  //take vertex from xz_dir neighbour
                  interpolate_vertex(directions, frame, neigh, num, x_dir ^ 1, z_dir ^ 1, vert);
                  
                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    if (frame[frame[neigh].offset + num].values[vert] != distance_cache[v_pos].dist)
                    {
                      distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = true;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }
                else// all neighbours on the last level
                {
                  float3 p = directions[frame[neigh].offset + num].p;
                  float d = directions[frame[neigh].offset + num].d;
                  float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
                  frame[frame[neigh].offset + num].values[vert] = sdf(ch_pos, 0);

                  if (distance_cache.find(v_pos) != distance_cache.end())
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    if (distance_cache[v_pos].is_interp)
                    {
                      for (auto recount : distance_cache[v_pos].voxels)
                      {
                        frame[recount].values[distance_cache[v_pos].idx_2_vert[recount]] = distance_cache[v_pos].dist;
                      }
                    }
                    distance_cache[v_pos].is_interp = false;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                  else
                  {
                    distance_cache[v_pos].dist = frame[frame[neigh].offset + num].values[vert];
                    distance_cache[v_pos].is_interp = false;
                    distance_cache[v_pos].voxels.insert(frame[neigh].offset + num);
                    distance_cache[v_pos].idx_2_vert[frame[neigh].offset + num] = vert;
                  }
                }
              }
            }
            //init end
            /*for (int num = 0; num < 8; ++num)
            {
              unsigned child = frame[neigh].offset + num;
              int dirs[3] = {((num & 4) >> 2), ((num & 2) >> 1) + 2, (num & 1) + 4};
              int another_dirs[3] = {(((num & 4) >> 2) ^ 1), (((num & 2) >> 1) ^ 1) + 2, ((num & 1) ^ 1) + 4};
              int vert_mask[3] = {4, 2, 1};
              int off[3] = {2, 1, 0};
              for (int step = 0; step < 3; ++step)
              {
                if (directions[directions[child].neighbour[another_dirs[step]]].d <= directions[child].d)
                {//need check all nodes
                  std::vector<unsigned> recount_nodes(0);
                  recount_nodes.push_back(directions[child].neighbour[another_dirs[step]]);
                  for (int k = 0; k < recount_nodes.size(); ++k)
                  {
                    //if (recount_nodes[k] == node) printf("ya\n");
                    //if (dirs[step] == i || dirs[step] == j) {printf("%u - %u  %u\n", recount_nodes[k], directions[recount_nodes[k]].neighbour[dirs[step]], child); chk2 = true;}//DEBUGGING
                    directions[recount_nodes[k]].neighbour[dirs[step]] = child;
                    for (int vert = 0; vert < 8; ++vert)
                    {
                      if ((vert & vert_mask[step]) >> off[step] == another_dirs[step] % 2)
                      {
                        if (frame[recount_nodes[k]].offset != 0) recount_nodes.push_back(frame[recount_nodes[k]].offset + vert);
                        if (is_smooth) 
                        {
                          if (directions[recount_nodes[k]].d < directions[directions[recount_nodes[k]].neighbour[another_dirs[step]]].d) 
                          {
                            interpolate_vertex(directions, frame, directions[recount_nodes[k]].parent, 
                                               recount_nodes[k] - frame[directions[recount_nodes[k]].parent].offset, 
                                               another_dirs[step], vert);
                          }
                          else
                          {
                            float3 p = directions[recount_nodes[k]].p;
                            float d = directions[recount_nodes[k]].d;
                            float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
                            frame[recount_nodes[k]].values[vert] = sdf(ch_pos, 0);
                          }
                        }
                      }
                    }
                  }
                }
              }
              
            }*/
            /*if (chk2 && chk) {chk = false; printf("\n");}
            if (neigh != directions[node].neighbour[i]) printf("AAA\n");
            //else if (directions[directions[node].neighbour[i]].neighbour[j])
            printf("%u %u -- %u %u\n", neigh, directions[node].neighbour[i], directions[directions[node].neighbour[i]].neighbour[j], node);

            x = node;
            while (directions[x].parent != 0)
            {
              printf("%u, %u, %u\n", x, frame[x].offset, directions[x].neighbour[i]);
              x = directions[x].parent;
            }
            printf("%u, %u, %u\n", x, frame[x].offset, directions[x].neighbour[i]);

            x = neigh;

            while (directions[x].parent != 0)
            {
              printf("%u, %u, %u\n", x, frame[x].offset, directions[x].neighbour[j]);
              x = directions[x].parent;
            }
            printf("%u, %u, %u\n", x, frame[x].offset, directions[x].neighbour[j]);*/

            neigh = directions[node].neighbour[i];
          }
        }
      }
    }
  }

  struct compar
  { 
    bool operator()(const float &a, const float &b) const
    {
      return fabs(a - b) > 1e-7;
    }
  };

  std::vector<SdfFrameOctreeNode> construct_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, 
                                                             unsigned max_threads/*while unused*/, bool is_smooth, bool fix_artefacts)
  {
    std::unordered_map<float3, vert_casche, PositionHasher, PositionEqual> distance_cache;
    //struct dirs {unsigned neighbour[3*2]; float3 p; float d; unsigned parent;};//x+x-y+y-z+z-
    std::vector<SdfFrameOctreeNode> result(1);
    fill_frame(result[0], sdf, float3(0, 0, 0), 1);
    for (unsigned v = 0; v < 8; ++v)
    {
      float3 ch_pos = float3(-1, -1, -1) + 2 * float3((v & 4) >> 2, (v & 2) >> 1, v & 1);
      distance_cache[ch_pos].dist = result[0].values[v];
      distance_cache[ch_pos].is_interp = false;
      distance_cache[ch_pos].voxels.insert(0);
      distance_cache[ch_pos].idx_2_vert[0] = v;
    }
    std::vector<struct dirs> directions(1);
    directions[0].d = 1.0f;
    directions[0].p = float3(0, 0, 0);
    directions[0].parent = 0;
    memset(directions[0].neighbour, 0, sizeof(unsigned) * 6);
    std::set<unsigned> divided = {};
    std::set<unsigned> last_level = {0};

    std::map<std::pair<std::pair<int, int>, int>, std::set<float, compar>> data;//debug

    for (int i = 1; i < settings.depth; ++i)
    {
      //TODO
      bool is_end = true;
      for (auto node_idx : last_level)
      {
        float3 corner = 2.0 * directions[node_idx].p * directions[node_idx].d - 1.0;
        float min_vert = std::abs(result[node_idx].values[0]);
        float max_dist = std::sqrt(3) * directions[node_idx].d;
        for (unsigned vert = 1; vert < 8; ++vert)
        {
          min_vert = std::min(min_vert, std::abs(result[node_idx].values[vert]));
        }
        if (node_RMSE_linear(result[node_idx].values, sdf, corner, 2 * directions[node_idx].d * float3(1, 1, 1)) >= eps && max_dist > min_vert)
        {
          divided.insert(node_idx);
          is_end = false;
        }
      }
      if (is_end) break;
      last_level.clear();
      for (auto div_idx : divided)
      {
        result[div_idx].offset = result.size();
        result.resize(result.size() + 8);
        directions.resize(directions.size() + 8);
        for (unsigned num = 0; num < 8; ++num)
        {
          last_level.insert(result[div_idx].offset + num);
          //initialize
          result[result[div_idx].offset + num].offset = 0;
          directions[result[div_idx].offset + num].p = 2 * directions[div_idx].p + float3((num & 4) >> 2, (num & 2) >> 1, num & 1);
          directions[result[div_idx].offset + num].d = directions[div_idx].d / 2.0;
          directions[result[div_idx].offset + num].parent = div_idx;
        }

      }
      for (auto div_idx : divided)
      {
        for (unsigned num = 0; num < 8; ++num)
        {
          if ((num & 4) != 0) fill_neighbours_til_one_dir(directions, result, div_idx, num, -4, 1, 0);
          else fill_neighbours_til_one_dir(directions, result, div_idx, num, 4, 0, 1);

          if ((num & 2) != 0) fill_neighbours_til_one_dir(directions, result, div_idx, num, -2, 3, 2);
          else fill_neighbours_til_one_dir(directions, result, div_idx, num, 2, 2, 3);

          if ((num & 1) != 0) fill_neighbours_til_one_dir(directions, result, div_idx, num, -1, 5, 4);
          else fill_neighbours_til_one_dir(directions, result, div_idx, num, 1, 4, 5);
        }
      }
      for (auto div_idx : divided)
      {
        for (unsigned num = 0; num < 8; ++num)
        {
          /*if ((num & 4) != 0) fill_neighbours_til_one_dir(directions, result, div_idx, num, -4, 1, 0);
          else fill_neighbours_til_one_dir(directions, result, div_idx, num, 4, 0, 1);

          if ((num & 2) != 0) fill_neighbours_til_one_dir(directions, result, div_idx, num, -2, 3, 2);
          else fill_neighbours_til_one_dir(directions, result, div_idx, num, 2, 2, 3);

          if ((num & 1) != 0) fill_neighbours_til_one_dir(directions, result, div_idx, num, -1, 5, 4);
          else fill_neighbours_til_one_dir(directions, result, div_idx, num, 1, 4, 5);*///move to previous for and change function

          /*printf("%u", result[div_idx].offset + num);
          for (int y = 0; y < 6; ++y)
          {
            printf(" %u", directions[result[div_idx].offset + num].neighbour[y]);
          }
          printf("\n");*/

          for (unsigned vert = 0; vert < 8; ++vert)
          {
            unsigned x_dir = (vert & 4) >> 2;
            unsigned y_dir = 2 + ((vert & 2) >> 1);
            unsigned z_dir = 4 + (vert & 1);

            float3 v_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
            if (distance_cache.find(v_pos) != distance_cache.end())
            {
              result[result[div_idx].offset + num].values[vert] = distance_cache[v_pos].dist;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;
              //if (!distance_cache[v_pos].is_interp) printf("%u %f %f\n", result[div_idx].offset + num, distance_cache[v_pos].dist, sdf(v_pos, 0));

              float3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]) == last_level.end() && directions[result[div_idx].offset + num].neighbour[x_dir ^ 1] != 0)
            {
              //take vertex from x_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, x_dir ^ 1, vert);
              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = true;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*float3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
                if (data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() > 1 || ((int)(ch_pos.x * 1024) == -640 && (int)(ch_pos.y * 1024) == 0 && (int)(ch_pos.z * 1024) == -256))
                {
                  printf("x %f %f %f %f ", directions[result[div_idx].offset + num].d, directions[result[div_idx].offset + num].p.x, 
                                            directions[result[div_idx].offset + num].p.y, directions[result[div_idx].offset + num].p.z);
                  for (auto a : data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}])
                  {
                    printf("%f ", a);
                  }
                  printf("\n");
                  printf("X %f %u %u --", result[result[div_idx].offset + num].values[vert], num, vert);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[div_idx].values[n]);
                  }
                  printf("\n");
                  auto tmp = directions[result[div_idx].offset + num].neighbour[x_dir ^ 1];
                  printf("N %f %f %f %f --", directions[tmp].d, directions[tmp].p.x, directions[tmp].p.y, directions[tmp].p.z);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[tmp].values[n]);
                  }
                  printf("\n");
                }
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[y_dir ^ 1]) == last_level.end() && directions[result[div_idx].offset + num].neighbour[y_dir ^ 1] != 0)
            {
              //take vertex from y_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, y_dir ^ 1, vert);
              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = true;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*float3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
                if (data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() > 1 || ((int)(ch_pos.x * 1024) == -640 && (int)(ch_pos.y * 1024) == 0 && (int)(ch_pos.z * 1024) == -256))
                {
                  printf("y %f %f %f %f ", directions[result[div_idx].offset + num].d, directions[result[div_idx].offset + num].p.x, 
                                            directions[result[div_idx].offset + num].p.y, directions[result[div_idx].offset + num].p.z);
                  for (auto a : data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}])
                  {
                    printf("%f ", a);
                  }
                  printf("\n");
                  printf("Y %f %u %u --", result[result[div_idx].offset + num].values[vert], num, vert);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[div_idx].values[n]);
                  }
                  printf("\n");
                  auto tmp = directions[result[div_idx].offset + num].neighbour[y_dir ^ 1];
                  printf("N %f %f %f %f --", directions[tmp].d, directions[tmp].p.x, directions[tmp].p.y, directions[tmp].p.z);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[tmp].values[n]);
                  }
                  printf("\n");
                }
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[z_dir ^ 1]) == last_level.end() && directions[result[div_idx].offset + num].neighbour[z_dir ^ 1] != 0)
            {
              //take vertex from z_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, z_dir ^ 1, vert);
              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = true;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*float3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x, ch_pos.y}, ch_pos.z}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
                if (data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z}].size() > 1 || ((int)(ch_pos.x * 1024) == -640 && (int)(ch_pos.y * 1024) == 0 && (int)(ch_pos.z * 1024) == -256))
                {
                  printf("z %f %f %f %f ", directions[result[div_idx].offset + num].d, directions[result[div_idx].offset + num].p.x, 
                                            directions[result[div_idx].offset + num].p.y, directions[result[div_idx].offset + num].p.z);
                  for (auto a : data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}])
                  {
                    printf("%f ", a);
                  }
                  printf("\n");
                  printf("Z %f %u %u --", result[result[div_idx].offset + num].values[vert], num, vert);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[div_idx].values[n]);
                  }
                  printf("\n");
                  auto tmp = directions[result[div_idx].offset + num].neighbour[z_dir ^ 1];
                  printf("N %f %f %f %f --", directions[tmp].d, directions[tmp].p.x, directions[tmp].p.y, directions[tmp].p.z);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[tmp].values[n]);
                  }
                  printf("\n");
                }
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]) != last_level.end() && directions[result[div_idx].offset + num].neighbour[x_dir ^ 1] != 0 &&
                     last_level.find(directions[directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]].neighbour[y_dir ^ 1]) == last_level.end() && directions[directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]].neighbour[y_dir ^ 1] != 0)
            {
              //take vertex from xy_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, x_dir ^ 1, y_dir ^ 1, vert);
              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = true;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*float3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[y_dir ^ 1]) != last_level.end() && directions[result[div_idx].offset + num].neighbour[y_dir ^ 1] != 0 &&
                     last_level.find(directions[directions[result[div_idx].offset + num].neighbour[y_dir ^ 1]].neighbour[z_dir ^ 1]) == last_level.end() && directions[directions[result[div_idx].offset + num].neighbour[y_dir ^ 1]].neighbour[z_dir ^ 1] != 0)
            {
              //take vertex from yz_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, y_dir ^ 1, z_dir ^ 1, vert);
              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = true;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*float3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
            else if (is_smooth && last_level.find(directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]) != last_level.end() && directions[result[div_idx].offset + num].neighbour[x_dir ^ 1] != 0 &&
                     last_level.find(directions[directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]].neighbour[z_dir ^ 1]) == last_level.end() && directions[directions[result[div_idx].offset + num].neighbour[x_dir ^ 1]].neighbour[z_dir ^ 1] != 0)
            {
              //take vertex from xz_dir neighbour
              interpolate_vertex(directions, result, div_idx, num, x_dir ^ 1, z_dir ^ 1, vert);
              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = true;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*loat3 ch_pos = 2.0 * directions[result[div_idx].offset + num].d * directions[result[div_idx].offset + num].p - 1.0 + 
                  2 * directions[result[div_idx].offset + num].d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
            else// all neighbours on the last level
            {
              float3 p = directions[result[div_idx].offset + num].p;
              float d = directions[result[div_idx].offset + num].d;
              float3 ch_pos = 2.0f * (d * p) - 1.0f + 2 * d * float3((vert & 4) >> 2, (vert & 2) >> 1, vert & 1);
              result[result[div_idx].offset + num].values[vert] = sdf(ch_pos, 0);

              distance_cache[v_pos].dist = result[result[div_idx].offset + num].values[vert];
              distance_cache[v_pos].is_interp = false;
              distance_cache[v_pos].voxels.insert(result[div_idx].offset + num);
              distance_cache[v_pos].idx_2_vert[result[div_idx].offset + num] = vert;

              /*if (data.find({{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}) != data.end() && data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() < 2)
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].insert(result[result[div_idx].offset + num].values[vert]);
                if (data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}].size() > 1 || ((int)(ch_pos.x * 1024) == -640 && (int)(ch_pos.y * 1024) == 0 && (int)(ch_pos.z * 1024) == -256))
                {
                  printf("s %f %f %f %f ", directions[result[div_idx].offset + num].d, directions[result[div_idx].offset + num].p.x, 
                                            directions[result[div_idx].offset + num].p.y, directions[result[div_idx].offset + num].p.z);
                  for (auto a : data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}])
                  {
                    printf("%f ", a);
                  }
                  printf("\n");
                  printf("S %f %u %u --", result[result[div_idx].offset + num].values[vert], num, vert);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[div_idx].values[n]);
                  }
                  printf("\n");
                  auto tmp = directions[result[div_idx].offset + num].neighbour[x_dir ^ 1];
                  printf("Nx %f %f %f %f --", directions[tmp].d, directions[tmp].p.x, directions[tmp].p.y, directions[tmp].p.z);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[tmp].values[n]);
                  }
                  printf("\n");
                  tmp = directions[result[div_idx].offset + num].neighbour[y_dir ^ 1];
                  printf("Ny %f %f %f %f --", directions[tmp].d, directions[tmp].p.x, directions[tmp].p.y, directions[tmp].p.z);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[tmp].values[n]);
                  }
                  printf("\n");
                  tmp = directions[result[div_idx].offset + num].neighbour[z_dir ^ 1];
                  printf("Nz %f %f %f %f --", directions[tmp].d, directions[tmp].p.x, directions[tmp].p.y, directions[tmp].p.z);
                  for (int n = 0; n < 8; ++n)
                  {
                    printf(" %f", result[tmp].values[n]);
                  }
                  printf("\n");
                }
              }
              else
              {
                data[{{ch_pos.x * 1024, ch_pos.y * 1024}, ch_pos.z * 1024}] = {result[result[div_idx].offset + num].values[vert]};
              }*/
            }
          }
        }
      }
      if (i < settings.depth - 1) divided.clear();

      //printf("%d\n", last_level.size());
    }
    //printf("%d ", result.size());
    if (fix_artefacts) sdf_artefacts_fix(result, directions, distance_cache, sdf, eps, max_threads, settings.depth, divided, is_smooth);
    //printf("%d\n", result.size());
    return result;
  }

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, bool is_smooth, bool fix_artefacts)
  {
    assert(settings.depth > 1);
    return construct_sdf_frame_octree(settings, sdf, eps, 1, is_smooth, fix_artefacts);
  }
}