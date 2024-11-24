#include "similarity_compression.h"
#include <cassert>

namespace scom
{
  void initialize_rot_transforms(std::vector<float4x4> &rot_transforms, int v_size)
  {
    rot_transforms.resize(ROT_COUNT, float4x4());
    for (int i=0; i<ROT_COUNT; i++)
    {
      // GENERATE ROTATIONS
      int code = i;
      int index_1 = code % 3; code /= 3;
      int sign_1  = code % 2; code /= 2;
      int index_2 = (index_1 + code % 2 + 1) % 3; 
      int index_3 = (index_1 + (code+1) % 2 + 1) % 3; code /= 2;
      int sign_2  = (code % 2) % 2; code /= 2;
      int sign_3  = (code % 2) % 2; code /= 2;
      float3 e1(0,0,0), e2(0,0,0), e3(0,0,0);
      e1[index_1] = sign_1 ? -1 : 1;
      e2[index_2] = sign_2 ? -1 : 1;
      e3[index_3] = sign_3 ? -1 : 1;
      assert(dot(e1, e2) < 1e-6f && dot(e1, e3) < 1e-6f && dot(e2, e3) < 1e-6f);
      float4x4 rot = LiteMath::float4x4(e1.x, e2.x, e3.x, 0, 
                                        e1.y, e2.y, e3.y, 0, 
                                        e1.z, e2.z, e3.z, 0, 
                                        0, 0, 0, 1);
      rot_transforms[i] = LiteMath::translate4x4(float3(v_size/2.0f - 0.5f + 1e-3f)) * rot * LiteMath::translate4x4(-float3(v_size/2.0f - 0.5f));
    }
  }

  struct TransformCompact
  {
    unsigned rotation_id;
    float add;
  };

  TransformCompact get_identity_transform()
  {
    TransformCompact t;
    t.rotation_id = 0;
    t.add = 0.0f;
    return t;
  }

  uint32_t get_transform_code(const TransformCompact &t)
  {
    assert(t.rotation_id < ROT_COUNT);
    assert(t.add > -0.5 && t.add < 0.5);

    uint32_t rot_code = t.rotation_id;
    uint32_t add_code = (t.add + 0.5f) * float(0x007FFFFFu);

    float add_restored = (float((add_code << 8) & 0x7FFFFF00u) / float(0x7FFFFF00u) - 0.5f);
    //printf("add, restored: %f, %f\n", t.add, add_restored);

    return VALID_TRANSFORM_CODE_BIT | (add_code << 8) | rot_code;
  }

  uint32_t get_identity_transform_code()
  {
    return get_transform_code(get_identity_transform());
  }

  //brute force + replacement
  void similarity_compression_naive(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, CompressionOutput &output)
  {
    constexpr float invalid_dist = 1000;
    constexpr int num_bins = 100;
    float max_val = 0.01f;
    int valid_nodes = 0;
    int surface_nodes = 0;
    float dist_thr = 5e-6f;

    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    std::vector<float> average_brick_val(g_octree.nodes.size(), 0);
    std::vector<float> closest_dist(g_octree.nodes.size(), invalid_dist);
    std::vector<int> closest_idx(g_octree.nodes.size(), -1);
    std::vector<unsigned> remap(g_octree.nodes.size());
    std::vector<TransformCompact> remap_transforms(g_octree.nodes.size());
    std::vector<bool> surface_node(g_octree.nodes.size(), false);
    std::vector<int> hist(num_bins+1, 0);
    std::array<std::vector<float>, ROT_COUNT> brick_rotations; 
    std::vector<float4x4> rotations4;

    initialize_rot_transforms(rotations4, v_size);

    for (int i=0; i<ROT_COUNT; i++)
     brick_rotations[i] = std::vector<float>(dist_count, 0);

    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      remap[i] = i;
      remap_transforms[i] = get_identity_transform();
    }
    
    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      int off = g_octree.nodes[i].val_off;
      float min_val = 1000;
      float max_val = -1000;

      double sum_sq = 0;
      double sum = 0;

      for (int k=0; k<dist_count; k++)
      {
        min_val = std::min(min_val, g_octree.values_f[off+k]);
        max_val = std::max(max_val, g_octree.values_f[off+k]);
        sum_sq += g_octree.values_f[off+k]*g_octree.values_f[off+k];
        sum += g_octree.values_f[off+k];
      }

      average_brick_val[i] = sum/dist_count;

      float mult = 1.0/std::max(1e-6, sqrt(sum_sq));
      if (g_octree.nodes[i].offset == 0 && g_octree.nodes[i].is_not_void)
      {
        surface_node[i] = true;
        surface_nodes++;
      }
    }

    int remapped = 0;

    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      int off_a = g_octree.nodes[i].val_off;
      for (int r_id = 0; r_id < ROT_COUNT; r_id++)
      {
        for (int x = 0; x < v_size; x++)
        {
          for (int y = 0; y < v_size; y++)
          {
            for (int z = 0; z < v_size; z++)
            {
              int idx = x * v_size * v_size + y * v_size + z;
              int3 rot_vec2 = int3(LiteMath::mul4x3(rotations4[r_id], float3(x, y, z)));
              int rot_idx = rot_vec2.x * v_size * v_size + rot_vec2.y * v_size + rot_vec2.z;
              brick_rotations[r_id][idx] = g_octree.values_f[off_a + rot_idx];
            }
          }
        }
      }

      if (surface_node[i] == false || remap[i] != i)
        continue;

#pragma omp parallel for schedule(static)
      for (int j = i + 1; j < g_octree.nodes.size(); j++)
      {
        if (surface_node[j] == false)
          continue;

        int off_b = g_octree.nodes[j].val_off;
        float add = 0; //TODO: fix addition
        float min_dist = 1000;
        int min_r_id = 0;
        for (int r_id = 0; r_id < ROT_COUNT; r_id++)
        {
          float diff = 0;
          float dist = 0;
          for (int k = 0; k < dist_count; k++)
          {
            diff = brick_rotations[r_id][k] + add - g_octree.values_f[off_b + k];
            dist += diff * diff;
          }
          dist = dist / dist_count;
          if (dist < min_dist)
          {
            min_dist = dist;
            min_r_id = r_id;
          }
        }
        if (min_dist < dist_thr)
        {
#pragma omp critical
          {
            //printf("min_dist = %f\n", min_dist);
            // printf("min_r_id = %d\n", min_r_id);
            remapped += remap[j] == j;
            remap[j] = i;
            remap_transforms[j].rotation_id = min_r_id;
            remap_transforms[j].add = add;
            closest_dist[j] = min_dist;
          }
        }
      }
    }

    printf("remapped %d/%d nodes\n", remapped, surface_nodes);

    output.node_id_cleaf_id_remap.resize(g_octree.nodes.size(), 0);
    output.tranform_codes.resize(g_octree.nodes.size(), 0);
    output.compressed_data.reserve(surface_nodes * dist_count);

    std::vector<unsigned> remap_2(g_octree.nodes.size(), 0);
    int unique_node_id = 0;
    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (!surface_node[i])
        continue;
      if (remap[i] == i)
      {
        unsigned offset = g_octree.nodes[i].val_off;
        remap_2[i] = unique_node_id;
        output.compressed_data.insert(output.compressed_data.end(), g_octree.values_f.begin() + offset, g_octree.values_f.begin() + offset + dist_count);
        output.node_id_cleaf_id_remap[i] = unique_node_id;
        output.tranform_codes[i] = get_identity_transform_code();
        unique_node_id++;
      }
    }

    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (!surface_node[i])
        continue;
      if (remap[i] != i)
      {
        output.node_id_cleaf_id_remap[i] = remap_2[remap[i]];
        output.tranform_codes[i] = get_transform_code(remap_transforms[i]);
      }
    }
    output.leaf_count = unique_node_id;
    output.compressed_data.shrink_to_fit();
  }

  void similarity_compression(const sdf_converter::GlobalOctree &octree, const Settings &settings, CompressionOutput &output)
  {
    if (settings.clustering_algorithm == ClusteringAlgorithm::REPLACEMENT &&
        settings.search_algorithm == SearchAlgorithm::BRUTE_FORCE)
    {
      similarity_compression_naive(octree, settings, output);
    }
    else
    {
      printf("Only naive (brute force + replacement) similarity compression is supported\n");
    }
  }
}