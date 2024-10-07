#include "sdf_converter.h"
#include "mesh_bvh.h"
#include "omp.h"
#include "sparse_octree_2.h"
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

  std::vector<SdfOctreeNode> create_sdf_octree(SparseOctreeSettings settings, DistanceFunction sdf)
  {
    assert(settings.remove_thr >= 0);
    assert(settings.depth > 1);
    assert(settings.build_type == SparseOctreeBuildType::DEFAULT); //MESH_TLO available only when building from mesh

    auto nodes = construct_sdf_octree(settings, [&](const float3 &p, unsigned idx) -> float { return sdf(p); }, 1);
    octree_limit_nodes(nodes, settings.nodes_limit);
    return nodes;
  }

  std::vector<SdfOctreeNode> create_sdf_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    auto nodes = construct_sdf_octree(settings, sdf, max_threads);
    octree_limit_nodes(nodes, settings.nodes_limit);
    return nodes;
  }

  std::vector<SdfOctreeNode> create_sdf_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    unsigned max_threads = omp_get_max_threads();

    std::vector<MeshBVH> bvh(max_threads);
    for (unsigned i = 0; i < max_threads; i++)
      bvh[i].init(mesh);
    
    return create_sdf_octree(settings, [&](const float3 &p, unsigned idx) -> float 
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
    auto raw_nodes = construct_sdf_octree(settings, sdf, max_threads);
    auto nodes = convert_to_frame_octree(settings, sdf, max_threads, raw_nodes);
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
    auto raw_nodes = construct_sdf_octree(settings, sdf, max_threads);
    auto frame = convert_to_frame_octree(settings, sdf, max_threads, raw_nodes);
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

    auto raw_nodes = construct_sdf_octree(settings, sdf, max_threads);
    auto frame = convert_to_frame_octree(settings, sdf, max_threads, raw_nodes);
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

  SdfSBSAdapt greed_sbs_adapt(MultithreadedDistanceFunction sdf)//TODO
  {
    SdfSBSAdapt sbs;
    sbs.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F;
    sbs.header.bytes_per_value = 4;
    sbs.header.brick_pad = 0;

    sbs.nodes = {};
    sbs.values = {};
    sbs.values_f = {};

    std::vector<uint32_t> imp_vox_cnt_nodes;
    std::map<std::pair<std::pair<uint16_t, uint16_t>, uint16_t>, float> pos_to_val;
    std::map<std::pair<std::pair<uint16_t, uint16_t>, uint16_t>, uint32_t> pos_to_idx;
    std::pair<std::pair<uint16_t, uint16_t>, uint16_t> key;
    std::vector<uint8_t> x_imps(8), y_imps(8), z_imps(8);


    if ((sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      for (uint16_t x = 0; x < 0x8000; x += 8)
      {
        for (uint16_t y = 0; y < 0x8000; y += 8)
        {
          for (uint16_t z = 0; z < 0x8000; z += 8)
          {

            std::fill(x_imps.begin(), x_imps.end(), 0);
            std::fill(y_imps.begin(), y_imps.end(), 0);
            std::fill(z_imps.begin(), z_imps.end(), 0);
            uint32_t imp_vox_cnt = 0;
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

                  if (is_vox_imp) 
                  {
                    ++imp_vox_cnt;
                    ++x_imps[x_off];
                    ++y_imps[y_off];
                    ++z_imps[z_off];
                  }
                }
              }
            }

            if (imp_vox_cnt > 0)
            {
              bool is_div = false;

              std::vector<uint8_t> x_idx_seq(0), y_idx_seq(0), z_idx_seq(0);
              for (uint8_t idx = 0; idx < 8; ++idx)
              {
                if ((x_idx_seq.size() % 2 == 0 && x_imps[idx] == 0) || (x_idx_seq.size() % 2 != 0 && x_imps[idx] != 0))
                {
                  x_idx_seq.push_back(idx);
                }
                if ((y_idx_seq.size() % 2 == 0 && y_imps[idx] == 0) || (y_idx_seq.size() % 2 != 0 && y_imps[idx] != 0))
                {
                  y_idx_seq.push_back(idx);
                }
                if ((z_idx_seq.size() % 2 == 0 && z_imps[idx] == 0) || (z_idx_seq.size() % 2 != 0 && z_imps[idx] != 0))
                {
                  z_idx_seq.push_back(idx);
                }
              }//maybe need recursive function for checking better divs for block array

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
            }

            //check all slices and find empties
            //if empty slice near border or have another one empty slice -> divide
            //if empty slice lonely -> check if we have better dividing after that dividing -> divide
            //we should delete this node and create different smaller nodes

          }
        }
      }
    }


    

    return sbs;
  }
}