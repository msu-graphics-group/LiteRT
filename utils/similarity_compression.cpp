#include "similarity_compression.h"
#include "similarity_compression_impl.h"
#include "ball_tree.h"
#include <cassert>
#include <atomic>
#include <chrono>

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

  std::vector<unsigned> generate_inverse_transform_indices(std::vector<float4x4> &rot_transforms)
  {
    std::vector<unsigned> inverse_indices(rot_transforms.size());
    int inverses_found = 0;
    for (int i=0; i<rot_transforms.size(); i++)
    {
      float4x4 inv = LiteMath::inverse4x4(rot_transforms[i]);
      for (int j=0; j<rot_transforms.size(); j++)
      {
        float diff = 0;
        for (int k=0; k<4; k++)
          for (int l=0; l<4; l++)
            diff += abs(rot_transforms[j][k][l] - inv[k][l]);
        if (diff < 1e-6f)
        {
          inverse_indices[i] = j;
          inverses_found++;
          break;
        }
      }
    }
    printf("inverse indices = {");
    for (int i=0; i<rot_transforms.size(); i++)
      printf("%d, ", inverse_indices[i]);
    printf("}\n");
    assert(inverses_found == rot_transforms.size());
    return inverse_indices;
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

  //replacement
  void similarity_compression_replacement(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, CompressionOutput &output)
  {
    constexpr int num_bins = 100;
    float max_val = 0.01f;
    int valid_nodes = 0;
    int surface_nodes = 0;

    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    std::vector<float> average_brick_val(g_octree.nodes.size(), 0);
    std::vector<float> closest_dist(g_octree.nodes.size(), settings.similarity_threshold);
    std::vector<int> closest_idx(g_octree.nodes.size(), -1);
    std::vector<unsigned> remap(g_octree.nodes.size());
    std::vector<TransformCompact> remap_transforms(g_octree.nodes.size());
    std::vector<bool> surface_node(g_octree.nodes.size(), false);
    std::vector<int> hist(num_bins+1, 0);
    std::array<std::vector<float>, ROT_COUNT> brick_rotations; 
    std::vector<float4x4> rotations4;

    initialize_rot_transforms(rotations4, v_size);

    //if you need to regenerate them, call generate_inverse_transform_indices
    //std::vector<unsigned> inverse_indices = generate_inverse_transform_indices(rotations4);
    std::vector<unsigned> inverse_indices = {0, 2, 1, 3, 14, 25, 6, 7, 
                                             8, 9, 19, 32, 12, 26, 4, 15, 
                                             38, 28, 30, 10, 20, 33, 22, 44, 
                                             24, 5, 13, 27, 17, 37, 18, 31, 
                                             11, 21, 43, 35, 36, 29, 16, 39, 
                                             41, 40, 42, 34, 23, 45, 46, 47, };

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

    std::vector<float> distance_mult_mask(dist_count, 1.0f);
    float mult_sum = 0.0f;
    for (int x = 0; x < v_size; x++)
    {
      for (int y = 0; y < v_size; y++)
      {
        for (int z = 0; z < v_size; z++)
        {
          float mult = 1.0f;
          if (x < g_octree.header.brick_pad || x >= v_size - g_octree.header.brick_pad ||
              y < g_octree.header.brick_pad || y >= v_size - g_octree.header.brick_pad ||
              z < g_octree.header.brick_pad || z >= v_size - g_octree.header.brick_pad)
          {
            mult = settings.distance_importance.x; //padding distance
          }
          else if (x < g_octree.header.brick_pad + 1 || x >= v_size - g_octree.header.brick_pad - 1 ||
                   y < g_octree.header.brick_pad + 1 || y >= v_size - g_octree.header.brick_pad - 1 ||
                   z < g_octree.header.brick_pad + 1 || z >= v_size - g_octree.header.brick_pad - 1)
          {
            mult = settings.distance_importance.y; //border distance
          }
          else
          {
            mult = settings.distance_importance.z; //internal distance
          }

          mult_sum += mult;
          distance_mult_mask[x*v_size*v_size + y*v_size + z] = mult;
        }
      }
    }

    Dataset dataset;
    dataset.all_points.resize(dist_count * ROT_COUNT * surface_nodes);
    dataset.data_points.resize(surface_nodes * ROT_COUNT);

    float mask_norm = dist_count/mult_sum;
    std::atomic<uint32_t> cur_point_group(0);
    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (!surface_node[i])
        continue;
      
      int off_a = g_octree.nodes[i].val_off;
      
#pragma omp parallel for schedule(static)
      for (int r_id = 0; r_id < ROT_COUNT; r_id++)
      {
        int off_dataset = (cur_point_group*ROT_COUNT + r_id)*dist_count;

        dataset.data_points[cur_point_group*ROT_COUNT + r_id].average_val = average_brick_val[i];
        dataset.data_points[cur_point_group*ROT_COUNT + r_id].data_offset = off_dataset;
        dataset.data_points[cur_point_group*ROT_COUNT + r_id].original_id = i;
        dataset.data_points[cur_point_group*ROT_COUNT + r_id].rotation_id = r_id;
        for (int x = 0; x < v_size; x++)
        {
          for (int y = 0; y < v_size; y++)
          {
            for (int z = 0; z < v_size; z++)
            {
              int idx = x * v_size * v_size + y * v_size + z;
              int3 rot_vec2 = int3(LiteMath::mul4x3(rotations4[r_id], float3(x, y, z)));
              int rot_idx = rot_vec2.x * v_size * v_size + rot_vec2.y * v_size + rot_vec2.z;
              dataset.all_points[off_dataset + idx] = mask_norm * distance_mult_mask[idx] * (g_octree.values_f[off_a + rot_idx] - average_brick_val[i]);
            }
          }
        }
      }
      cur_point_group++;
    }

    std::unique_ptr<INNSearchAS> NN_search_AS;

    if (settings.search_algorithm == SearchAlgorithm::LINEAR_SEARCH)
    {
      NN_search_AS.reset(new LinearSearchAS());
      NN_search_AS->build(dataset, 1);
    }
    else if (settings.search_algorithm == SearchAlgorithm::BALL_TREE)
    {
      NN_search_AS.reset(new BallTree());
      NN_search_AS->build(dataset, 32);
    }

    int remapped = 0;

    auto t1 = std::chrono::high_resolution_clock::now();
    for (int point_a = 0; point_a < dataset.data_points.size(); point_a+=ROT_COUNT)
    {
      int i  = dataset.data_points[point_a].original_id;
      if (surface_node[i] == false || remap[i] != i)
        continue;
      
      int off_a = dataset.data_points[point_a].data_offset;

      if (settings.search_algorithm == SearchAlgorithm::BRUTE_FORCE)
      {
        #pragma omp parallel for schedule(static)
        for (int point_b = point_a + ROT_COUNT; point_b < dataset.data_points.size(); point_b++)
        {
          int j = dataset.data_points[point_b].original_id;
          int off_b = dataset.data_points[point_b].data_offset;

          float diff = 0;
          float dist = 0;
          for (int k = 0; k < dist_count; k++)
          {
            diff = dataset.all_points[off_a + k] - dataset.all_points[off_b + k];
            dist += diff * diff;
          }
          dist = sqrtf(dist);

          if (dist < settings.similarity_threshold)
          {
            #pragma omp critical
            if (dist < closest_dist[j])
            {
              remapped += remap[j] == j;
              remap[j] = i;
              remap_transforms[j].rotation_id = inverse_indices[dataset.data_points[point_b].rotation_id];
              remap_transforms[j].add = dataset.data_points[point_b].average_val - dataset.data_points[point_a].average_val;
              closest_dist[j] = dist;
            }
          }
        }
      }
      else
      {
        NN_search_AS->scan_near(dataset.all_points.data() + off_a, settings.similarity_threshold,
          [&](float dist, unsigned point_b, const DataPoint &, const float *)
          {
            if (point_b < point_a + ROT_COUNT)
              return;
            int j = dataset.data_points[point_b].original_id;

            #pragma omp critical
            {
              if (dist < closest_dist[j])
              {
                remapped += remap[j] == j;
                remap[j] = i;
                remap_transforms[j].rotation_id = inverse_indices[dataset.data_points[point_b].rotation_id];
                remap_transforms[j].add = dataset.data_points[point_b].average_val - dataset.data_points[point_a].average_val;
                closest_dist[j] = dist;
              }
            }
          });
      }
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    float time = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    printf("remapping took %.1f s (%.1f ms/leaf)\n", 1e-6f*time, 1e-3f*time/surface_nodes);
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
    if (settings.clustering_algorithm == ClusteringAlgorithm::REPLACEMENT)
    {
      similarity_compression_replacement(octree, settings, output);
    }
    else
    {
      printf("Only replacement similarity compression is supported\n");
    }
  }
}