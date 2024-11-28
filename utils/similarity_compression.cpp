#include "similarity_compression.h"
#include "similarity_compression_impl.h"
#include "ball_tree.h"
#include "omp.h"

#include <cassert>
#include <atomic>
#include <chrono>
#include <algorithm>

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

  std::vector<unsigned> get_inverse_transform_indices()
  {
    //if you need to regenerate them, call generate_inverse_transform_indices
    //    std::vector<float4x4> rotations4;
    // initialize_rot_transforms(rotations4, v_size); v_size can vbe arbitrary
    //std::vector<unsigned> inverse_indices = generate_inverse_transform_indices(rotations4);
    std::vector<unsigned> inverse_indices = {0, 2, 1, 3, 14, 25, 6, 7, 
                                             8, 9, 19, 32, 12, 26, 4, 15, 
                                             38, 28, 30, 10, 20, 33, 22, 44, 
                                             24, 5, 13, 27, 17, 37, 18, 31, 
                                             11, 21, 43, 35, 36, 29, 16, 39, 
                                             41, 40, 42, 34, 23, 45, 46, 47, };
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

  void create_dataset(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, const std::vector<bool> &surface_node,
                      unsigned surface_node_count, const std::vector<float> &average_brick_val,
                      /*out*/ Dataset &dataset)
  {
    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;

    std::vector<float4x4> rotations4;
    initialize_rot_transforms(rotations4, v_size);

    std::vector<float> distance_mult_mask(dist_count, 1.0f);
    float mult_sum = 0.0f;
    assert(settings.distance_importance.x == 1.0f || settings.distance_importance.y == 1.0f || settings.distance_importance.z == 1.0f);
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
            mult = settings.distance_importance.x; // padding distance
          }
          else if (x < g_octree.header.brick_pad + 1 || x >= v_size - g_octree.header.brick_pad - 1 ||
                   y < g_octree.header.brick_pad + 1 || y >= v_size - g_octree.header.brick_pad - 1 ||
                   z < g_octree.header.brick_pad + 1 || z >= v_size - g_octree.header.brick_pad - 1)
          {
            mult = settings.distance_importance.y; // border distance
          }
          else
          {
            mult = settings.distance_importance.z; // internal distance
          }

          mult_sum += mult;
          distance_mult_mask[x * v_size * v_size + y * v_size + z] = mult;
        }
      }
    }

    dataset.all_points.resize(dist_count * ROT_COUNT * surface_node_count);
    dataset.data_points.resize(surface_node_count * ROT_COUNT);

    float mask_norm = dist_count / mult_sum;
    uint32_t cur_point_group = 0;
    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (!surface_node[i])
        continue;

      int off_a = g_octree.nodes[i].val_off;

#pragma omp parallel for schedule(static)
      for (int r_id = 0; r_id < ROT_COUNT; r_id++)
      {
        int off_dataset = (cur_point_group * ROT_COUNT + r_id) * dist_count;

        dataset.data_points[cur_point_group * ROT_COUNT + r_id].average_val = average_brick_val[i];
        dataset.data_points[cur_point_group * ROT_COUNT + r_id].data_offset = off_dataset;
        dataset.data_points[cur_point_group * ROT_COUNT + r_id].original_id = i;
        dataset.data_points[cur_point_group * ROT_COUNT + r_id].rotation_id = r_id;
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
  }

  void prepare_output(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, const std::vector<unsigned> &remap,
                      const std::vector<unsigned> &remap_2, const std::vector<bool> &surface_node, unsigned surface_node_count,
                      const std::vector<TransformCompact> &remap_transforms, const Dataset &dataset,
                      /*out*/ CompressionOutput &output)
  {
    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    auto inverse_indices = get_inverse_transform_indices();

    output.node_id_cleaf_id_remap.resize(g_octree.nodes.size(), 0);
    output.tranform_codes.resize(g_octree.nodes.size(), 0);
    output.compressed_data.reserve(surface_node_count * dist_count);

    int unique_node_id = 0;

    std::vector<int> group_indices(surface_node_count, -1);
    std::vector<float> centroid(dist_count, 0);
    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (!surface_node[i])
        continue;
      if (remap[i] == i)
      {
        int group_indices_size = 0;
        std::fill_n(centroid.data(), dist_count, 0);
        for (int j = i; j < g_octree.nodes.size(); j++)
        {
          if (remap[j] == i)
          {
            group_indices[group_indices_size] = j;
            for (int k = 0; k < dist_count; k++)
              centroid[k] += dataset.all_points[(remap_2[j]*ROT_COUNT + inverse_indices[remap_transforms[j].rotation_id])*dist_count + k];
            group_indices_size++;
          }
        }

        for (int k = 0; k < dist_count; k++)
          centroid[k] /= group_indices_size;
        
        int p_size = output.compressed_data.size();
        output.compressed_data.resize(p_size + dist_count);
        for (int k = 0; k < dist_count; k++)
          output.compressed_data[p_size + k] = centroid[k];

        for (int j = 0; j < group_indices_size; j++)
        {
          int k = group_indices[j];
          output.node_id_cleaf_id_remap[k] = unique_node_id;
          output.tranform_codes[k] = get_transform_code(remap_transforms[k]);
        }
        unique_node_id++;
      }
    }
    output.leaf_count = unique_node_id;
    output.compressed_data.shrink_to_fit();
  }

  void similarity_compression_hierarchical(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, CompressionOutput &output)
  {
    auto tt1 = std::chrono::high_resolution_clock::now();
 
    int total_node_count = g_octree.nodes.size();
    int surface_node_count = 0;

    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    std::vector<float> average_brick_val(g_octree.nodes.size(), 0);
    std::vector<float> closest_dist(g_octree.nodes.size(), settings.similarity_threshold);
    std::vector<unsigned> remap(g_octree.nodes.size());
    std::vector<unsigned> remap_2(g_octree.nodes.size(), 0);
    std::vector<TransformCompact> remap_transforms(g_octree.nodes.size());
    std::vector<bool> surface_node(g_octree.nodes.size(), false);

    auto inverse_indices = get_inverse_transform_indices();

    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      int off = g_octree.nodes[i].val_off;

      remap[i] = i;
      remap_transforms[i] = get_identity_transform();
      remap_2[i] = surface_node_count;

      if (g_octree.nodes[i].offset == 0 && g_octree.nodes[i].is_not_void)
      {
        surface_node[i] = true;
        surface_node_count++;
      }

      double sum = 0;
      for (int k=0; k<dist_count; k++)
        sum += g_octree.values_f[off+k];
      average_brick_val[i] = sum/dist_count;
    }

    Dataset dataset;
    create_dataset(g_octree, settings, surface_node, surface_node_count, average_brick_val, dataset);

    auto tt2 = std::chrono::high_resolution_clock::now();
    printf("prepare data %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(tt2-tt1).count());  

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

    auto tt3 = std::chrono::high_resolution_clock::now();
    printf("build NN search AS %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(tt3-tt2).count());  

    struct Link
    {
      int target = -1; //original node id (i.e. from global octree)
      int rotation = -1;
      float dist = 1e6f;
    };

    struct ComponentInfo
    {
      int lead_id = -1;
      int count = 0;
    };

    constexpr int LABEL_UNVISITED = -1;
    constexpr int LABEL_ISOLATED  = -2;

    int max_threads = omp_get_max_threads();
    std::vector<std::vector<Link>> sim_graph(total_node_count); //sparse adjacency matrix

    std::vector<std::vector<Link>> tmp_links(max_threads);
    for (int i=0; i<max_threads; i++)
      tmp_links[i].resize(surface_node_count, Link());
  

  auto t1 = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int point_a = 0; point_a < dataset.data_points.size(); point_a+=ROT_COUNT)
    {
      int thread_id = omp_get_thread_num();
      int link_count = 0;
      int i  = dataset.data_points[point_a].original_id;
      if (surface_node[i] == false || remap[i] != i)
        continue;

      const DataPoint &A = dataset.data_points[point_a];
      NN_search_AS->scan_near(dataset.all_points.data() + A.data_offset, settings.similarity_threshold,
                              [&](float dist, unsigned point_b, const DataPoint &B, const float *)
                              {
                                if (A.original_id == B.original_id)
                                  return;

                                tmp_links[thread_id][link_count].target = B.original_id;
                                tmp_links[thread_id][link_count].rotation = B.rotation_id;
                                tmp_links[thread_id][link_count].dist = dist;

                                link_count++;
                              });
    
      //dataset can contain duplicates with different rotations, we should find the link with the smallest distance
      //and remove other
      //we also should sort by target id to use it with binary search later

      std::sort(tmp_links[thread_id].begin(), tmp_links[thread_id].begin() + link_count, [](const Link &a, const Link &b)
      {
        return a.target != b.target ? (a.target < b.target) : (a.dist < b.dist);
      });

      int real_link_count = 0;
      int prev_target_id = -1;
      for (int j=0; j<link_count; j++)
      {
        if (prev_target_id != tmp_links[thread_id][j].target)
        {
          tmp_links[thread_id][real_link_count] = tmp_links[thread_id][j];
          prev_target_id = tmp_links[thread_id][j].target;
          real_link_count++;
        }
      }

      sim_graph[i] = std::vector<Link>(tmp_links[thread_id].begin(), tmp_links[thread_id].begin() + real_link_count);
    }

    auto t2 = std::chrono::high_resolution_clock::now();
    printf("build links %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count());

    //find connected components
    std::vector<int> labels(total_node_count, LABEL_UNVISITED);
    std::vector<ComponentInfo> component_infos;
    std::vector<std::vector<int>> components;
    std::vector<unsigned> stack(2*total_node_count);
    int head = 0;
    for (int lead_id=0;lead_id<total_node_count;lead_id++)
    {
      if (labels[lead_id] != LABEL_UNVISITED)
        continue;
      else 
      {
        int total_count = 0;
        if (sim_graph[lead_id].empty())
        {
          total_count = 1;
          labels[lead_id] = LABEL_ISOLATED;
          components.push_back(std::vector<int>{lead_id});
        }
        else
        {
          head = 0;
          stack[head] = lead_id;
          labels[lead_id] = component_infos.size();
          components.emplace_back();

          while (head >= 0)
          {
            int node_id = stack[head];
            head--;
            total_count++;
            components.back().push_back(node_id);
            for (int i=0; i<sim_graph[node_id].size(); i++)
            {
              int target_id = sim_graph[node_id][i].target;
              if (labels[target_id] == LABEL_UNVISITED)
              {
                stack[++head] = target_id;
                assert(head < stack.size());
                labels[target_id] = labels[node_id];
              }
            }
          }
        }
        component_infos.emplace_back();
        component_infos.back().lead_id = lead_id;
        component_infos.back().count = total_count;
      }
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    printf("find connected components %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count());

    int remapped = 0;
    if (settings.clustering_algorithm == ClusteringAlgorithm::COMPONENTS_MERGE)
    {
      for (int i=0; i<component_infos.size(); i++)
      {
        //printf("[%d] lead = %d %d nodes\n", i, component_infos[i].lead_id, component_infos[i].count);

        int lead_id = component_infos[i].lead_id;
        if (component_infos[i].count == 1)
        {
          remap[lead_id] = lead_id;
          remap_transforms[lead_id].rotation_id = 0;
          remap_transforms[lead_id].add = average_brick_val[lead_id];
        }
        else
        {
          remapped += component_infos[i].count - 1;

          int lead_off = remap_2[lead_id]*ROT_COUNT*dist_count;
          float max_dist = 0;
          for (int j=0; j<components[i].size(); j++)
          {
            float min_dist_sq = 1e6f;
            int min_rot_id = 0;
            for (int k=0; k<ROT_COUNT; k++)
            {
              int target_off = remap_2[components[i][j]]*ROT_COUNT*dist_count + k*dist_count;
              float dist_sq = 0;
              for (int l=0; l<dist_count; l++)
                dist_sq += (dataset.all_points[lead_off + l] - dataset.all_points[target_off + l])*(dataset.all_points[lead_off + l] - dataset.all_points[target_off + l]);
              if (dist_sq < min_dist_sq)
              {
                min_dist_sq = dist_sq;
                min_rot_id = k;
              }
            }

            max_dist = std::max(max_dist, min_dist_sq);
            
            remap[components[i][j]] = lead_id;
            remap_transforms[components[i][j]].rotation_id = inverse_indices[min_rot_id];
            remap_transforms[components[i][j]].add = average_brick_val[components[i][j]];
          }
          printf("%d) %d nodes, max dist = %f/%f\n", component_infos[i].lead_id, component_infos[i].count, max_dist, settings.similarity_threshold);
        }
      }     
    }
    auto t4 = std::chrono::high_resolution_clock::now();
    printf("clustering %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count()); 

    prepare_output(g_octree, settings, remap, remap_2, surface_node, surface_node_count, remap_transforms, dataset, output);
    auto t5 = std::chrono::high_resolution_clock::now();
    printf("prepare output %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count());  
    
    printf("remapped %d/%d nodes\n", remapped, surface_node_count);
  }

  //replacement
  void similarity_compression_replacement(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, CompressionOutput &output)
  {
    int surface_node_count = 0;

    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    std::vector<float> average_brick_val(g_octree.nodes.size(), 0);
    std::vector<float> closest_dist(g_octree.nodes.size(), settings.similarity_threshold);
    std::vector<unsigned> remap(g_octree.nodes.size());
    std::vector<unsigned> remap_2(g_octree.nodes.size(), 0);
    std::vector<TransformCompact> remap_transforms(g_octree.nodes.size());
    std::vector<bool> surface_node(g_octree.nodes.size(), false);

    auto inverse_indices = get_inverse_transform_indices();

    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      int off = g_octree.nodes[i].val_off;

      remap[i] = i;
      remap_transforms[i] = get_identity_transform();
      remap_2[i] = surface_node_count;

      if (g_octree.nodes[i].offset == 0 && g_octree.nodes[i].is_not_void)
      {
        surface_node[i] = true;
        surface_node_count++;
      }

      double sum = 0;
      for (int k=0; k<dist_count; k++)
        sum += g_octree.values_f[off+k];
      average_brick_val[i] = sum/dist_count;
    }

    Dataset dataset;
    create_dataset(g_octree, settings, surface_node, surface_node_count, average_brick_val, dataset);

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
      
      remap_transforms[i].rotation_id = 0;
      remap_transforms[i].add = dataset.data_points[point_a].average_val;
      
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
              remap_transforms[j].add = dataset.data_points[point_b].average_val;
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
                remap_transforms[j].add = dataset.data_points[point_b].average_val;
                closest_dist[j] = dist;
              }
            }
          });
      }
    }
    auto t2 = std::chrono::high_resolution_clock::now();

    float time = std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
    printf("remapping took %.1f s (%.1f ms/leaf)\n", 1e-6f*time, 1e-3f*time/surface_node_count);
    printf("remapped %d/%d nodes\n", remapped, surface_node_count);

    prepare_output(g_octree, settings, remap, remap_2, surface_node, surface_node_count, remap_transforms, dataset, output);
  }

  void similarity_compression(const sdf_converter::GlobalOctree &octree, const Settings &settings, CompressionOutput &output)
  {
    if (settings.clustering_algorithm == ClusteringAlgorithm::HIERARCHICAL || 
        settings.clustering_algorithm == ClusteringAlgorithm::COMPONENTS_MERGE)
    {
      similarity_compression_hierarchical(octree, settings, output);
      //similarity_compression_replacement(octree, settings, output);
    }
    else if (settings.clustering_algorithm == ClusteringAlgorithm::REPLACEMENT)
    {
      similarity_compression_replacement(octree, settings, output);
    }
    else
    {
      printf("Only replacement similarity compression is supported\n");
    }
  }
}