#include "sdf_converter.h"
#include "mesh_bvh.h"
#include "omp.h"
#include "sparse_octree_builder.h"
#include <chrono>
#include "mesh.h"

namespace sdf_converter
{
  SdfGrid create_sdf_grid(GridSettings settings, DistanceFunction sdf)
  {
    MultithreadedDistanceFunction sdf_multi = [&](const float3 &p, unsigned idx) -> float { return sdf(p); };
    return create_sdf_grid(settings, sdf_multi, 1);
  }

  SdfGrid create_sdf_grid(GridSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    assert(settings.size >= 1);

    unsigned sz = settings.size;
    SdfGrid grid;
    grid.size = uint3(sz, sz, sz);
    //printf("creating grid: %ux%ux%u\n", grid.size.x, grid.size.y, grid.size.z);

    unsigned long total_size = grid.size.x * grid.size.y * grid.size.z;
    if (total_size > 512 * 1000 * 1000)
      printf("Warning: large grid size: %lu\n", total_size);

    grid.data.resize(total_size);

    omp_set_num_threads(max_threads);
    #pragma omp parallel
    for (int i = 0; i < sz; i++)
      for (int j = 0; j < sz; j++)
        for (int k = 0; k < sz; k++)
          grid.data[i*sz*sz + j*sz + k] = sdf(2.0f*(float3(k + 0.5, j + 0.5, i + 0.5)/float(sz/* - 1*/)) - 1.0f, omp_get_thread_num());
    omp_set_num_threads(omp_get_max_threads());

    return grid;
  }

  SdfGrid create_sdf_grid(GridSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    unsigned max_threads = omp_get_max_threads();

    std::vector<MeshBVH> bvh(max_threads);
    for (unsigned i = 0; i < max_threads; i++)
      bvh[i].init(mesh);
    
    return create_sdf_grid(settings, [&](const float3 &p, unsigned idx) -> float 
                           { return bvh[idx].get_signed_distance(p); }, max_threads);
  }

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, DistanceFunction sdf)
  {
    assert(settings.remove_thr >= 0);
    assert(settings.depth > 1);
    assert(settings.build_type == SparseOctreeBuildType::DEFAULT); //MESH_TLO available only when building from mesh

    return create_sdf_frame_octree(settings, [&](const float3 &p, unsigned idx) -> float { return sdf(p); }, 1);
  }

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, float eps, bool is_smooth, bool fix_artefacts)
  {
    assert(settings.remove_thr >= 0);//copy from another functions
    assert(settings.depth > 1);
    assert(settings.build_type == SparseOctreeBuildType::DEFAULT); //MESH_TLO available only when building from mesh

    return construct_sdf_frame_octree(settings, sdf, eps, 1, is_smooth, fix_artefacts);
  }

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    auto nodes = construct_sdf_frame_octree(settings, sdf, max_threads);
    frame_octree_limit_nodes(nodes, settings.nodes_limit, false);
    return nodes;
  }

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    if (settings.build_type == SparseOctreeBuildType::MESH_TLO)
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 1, 2.0f);
      std::vector<SdfFrameOctreeNode> nodes;
      mesh_octree_to_sdf_frame_octree(mesh, tlo, nodes);
      frame_octree_limit_nodes(nodes, settings.nodes_limit, false);
      return nodes;
    }
    else
    {
      unsigned max_threads = omp_get_max_threads();

      std::vector<MeshBVH> bvh(max_threads);
      for (unsigned i = 0; i < max_threads; i++)
        bvh[i].init(mesh);
      
      return create_sdf_frame_octree(settings, [&](const float3 &p, unsigned idx) -> float 
                                     { return bvh[idx].get_signed_distance(p); }, max_threads);
    }
  }

  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, DistanceFunction sdf)
  {
    assert(settings.remove_thr >= 0);
    assert(settings.depth > 1);
    assert(settings.build_type == SparseOctreeBuildType::DEFAULT); //MESH_TLO available only when building from mesh

    return create_sdf_SVS(settings, [&](const float3 &p, unsigned idx) -> float { return sdf(p); }, 1);
  }
  
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    auto frame = construct_sdf_frame_octree(settings, sdf, max_threads);
    frame_octree_limit_nodes(frame, settings.nodes_limit, true);
    std::vector<SdfSVSNode> nodes;
    frame_octree_to_SVS_rec(frame, nodes, 0, uint3(0,0,0), 1);
    return nodes;
  }

  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    if (settings.build_type == SparseOctreeBuildType::MESH_TLO)
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 1, 2.0f);
      std::vector<SdfFrameOctreeNode> frame;
      std::vector<SdfSVSNode> nodes;
      mesh_octree_to_sdf_frame_octree(mesh, tlo, frame);
      frame_octree_limit_nodes(frame, settings.nodes_limit, true);
      frame_octree_to_SVS_rec(frame, nodes, 0, uint3(0,0,0), 1);
      return nodes;
    }
    else
    {
      unsigned max_threads = omp_get_max_threads();

      std::vector<MeshBVH> bvh(max_threads);
      for (unsigned i = 0; i < max_threads; i++)
        bvh[i].init(mesh);
      
      return create_sdf_SVS(settings, [&](const float3 &p, unsigned idx) -> float 
                            { return bvh[idx].get_signed_distance(p); }, max_threads);
    }
  }

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, DistanceFunction sdf)
  {
    assert(settings.build_type == SparseOctreeBuildType::DEFAULT); //MESH_TLO available only when building from mesh

    return create_sdf_SBS(settings, header, [&](const float3 &p, unsigned idx) -> float { return sdf(p); }, 1);
  }

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    assert(settings.remove_thr >= 0);
    assert(settings.depth > 1);
    assert(header.brick_size >= 1 && header.brick_size <= 16);
    assert(header.brick_pad == 0 || header.brick_pad == 1);
    assert(header.bytes_per_value == 1 || header.bytes_per_value == 2 || header.bytes_per_value == 4);

    auto frame = construct_sdf_frame_octree(settings, sdf, max_threads);
    frame_octree_limit_nodes(frame, settings.nodes_limit, true);
    auto sbs = frame_octree_to_SBS(sdf, max_threads, frame, header);
    return sbs;
  }

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh)
  {
    unsigned max_threads = omp_get_max_threads();

    std::vector<MeshBVH> bvh(max_threads);
    for (unsigned i = 0; i < max_threads; i++)
      bvh[i].init(mesh);
      
    return create_sdf_SBS(settings, header, [&](const float3 &p, unsigned idx) -> float 
                          { return bvh[idx].get_signed_distance(p); }, max_threads);
  }

  std::vector<SdfFrameOctreeTexNode> create_sdf_frame_octree_tex(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    if (settings.build_type == SparseOctreeBuildType::MESH_TLO)
    {
      //we set max_triangles_per_leaf = 0 to prevent issues with big, textured triangles (see test with textured cube)
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 2.0f);
      std::vector<SdfFrameOctreeTexNode> frame;
      mesh_octree_to_sdf_frame_octree_tex(mesh, tlo, frame);
      //frame_octree_limit_nodes(frame, settings.nodes_limit, true); //TODO
      return frame;
    }
    else
    {
      printf("Frame octree with texture can be built only from mesh with MESH_TLO build type\n");
      return {};
    }
  }

  std::vector<SdfFrameOctreeTexNode> conv_octree_2_tex(const std::vector<SdfFrameOctreeNode> &frame)
  {
    std::vector<SdfFrameOctreeTexNode> ans(frame.size());
    for (int i = 0; i < frame.size(); ++i)
    {
      ans[i].offset = frame[i].offset;
      for (int j = 0; j < 8; ++j)
      {
        ans[i].values[j] = frame[i].values[j];
        ans[i].tex_coords[j * 2] = 0;
        ans[i].tex_coords[j * 2 + 1] = 0;
      }
      ans[i].material_id = 0;
    }
    return ans;
  }

  SdfSBS create_sdf_SBS_tex(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, bool noisy)
  {

    unsigned max_threads = omp_get_max_threads();

    std::vector<MeshBVH> bvh(max_threads);
    for (unsigned i = 0; i < max_threads; i++)
      bvh[i].init(mesh);

    if (noisy)
    {    
      MultithreadedDistanceFunction mt_sdf = [&, noisy](const float3 &p, unsigned idx) -> float 
                                            {auto x = (fmod(p.x * 153 + p.y * 427 + p.z * 311, 2.0) - 1) * 0.003;
                                             return bvh[idx].get_signed_distance(p) + x; };
    
      std::vector<SdfFrameOctreeTexNode> frame = conv_octree_2_tex(create_sdf_frame_octree(settings, mt_sdf, max_threads));
      auto a = frame_octree_to_SBS_tex(mt_sdf, max_threads, frame, header);
      //fclose(f);
      return a;
    }
    else
    {
      MultithreadedDistanceFunction mt_sdf = [&](const float3 &p, unsigned idx) -> float 
                                           { return bvh[idx].get_signed_distance(p); };
  
      std::vector<SdfFrameOctreeTexNode> frame = create_sdf_frame_octree_tex(settings, mesh);
      return frame_octree_to_SBS_tex(mt_sdf, max_threads, frame, header);
    }
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


  SdfSBS create_sdf_SBS_indexed(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                                const std::vector<MultiRendererMaterial> &materials_lib, 
                                const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib, bool noisy)
  {
    SdfSBS sbs = create_sdf_SBS_col(settings, header, mesh, mat_id, materials_lib, textures_lib, noisy);
    return SBS_col_to_SBS_ind(sbs);
  }

  SdfSBS create_sdf_SBS_indexed_with_neighbors(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, unsigned mat_id,
                                               const std::vector<MultiRendererMaterial> &materials_lib, 
                                               const std::vector<std::shared_ptr<ICombinedImageSampler>> &textures_lib)
  {
    SdfSBS sbs = create_sdf_SBS_indexed(settings, header, mesh, mat_id, materials_lib, textures_lib);
    return SBS_ind_to_SBS_ind_with_neighbors(sbs);
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
}