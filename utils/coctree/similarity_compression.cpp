#include "similarity_compression.h"
#include "similarity_compression_impl.h"
#include "ball_tree.h"
#include "omp.h"

#include <cassert>
#include <atomic>
#include <chrono>
#include <algorithm>
#include <set>

namespace scom
{
  void save_dataset(const Dataset &dataset, const std::string &filename)
  {
  std::ofstream fs(filename, std::ios::binary);
  size_t points_size = dataset.data_points.size();
  size_t all_points_size = dataset.all_points.size();
  fs.write((const char *)&points_size, sizeof(size_t));
  fs.write((const char *)dataset.data_points.data(), points_size * sizeof(DataPoint));
  fs.write((const char *)&all_points_size, sizeof(size_t));
  fs.write((const char *)dataset.all_points.data(), all_points_size * sizeof(float));
  
  fs.flush();
  fs.close();
  }
  
  void load_dataset(const std::string &filename, Dataset &dataset)
  {
  std::ifstream fs(filename, std::ios::binary);
  size_t points_size = 0;
  size_t all_points_size = 0;
  fs.read((char *)&points_size, sizeof(size_t));
  dataset.data_points.resize(points_size);
  fs.read((char *)dataset.data_points.data(), points_size * sizeof(DataPoint));
  
  fs.read((char *)&all_points_size, sizeof(size_t));
  dataset.all_points.resize(all_points_size);
  fs.read((char *)dataset.all_points.data(), all_points_size * sizeof(float));
  
  assert(all_points_size % points_size == 0);
  dataset.dim = all_points_size / points_size;

  fs.close();
  }

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

  TransformCompact get_identity_transform()
  {
    TransformCompact t;
    t.rotation_id = 0;
    t.add = 0.0f;
    return t;
  }

  struct Link
  {
    int target = -1; // original node id (i.e. from global octree)
    int rotation = -1;
    float dist = 1e6f;
  };

  struct ComponentInfo
  {
    int lead_id = -1;
    int count = 0;
  };

  struct Cluster
  {
    int lead_id = -1;
    int count = 0;
    std::vector<int> point_ids; // original node ids (i.e. from global octree)
  };

  static constexpr int HC_VALID = 0x7fffffff;
  static constexpr int MAX_STEPS = HC_VALID - 1;
  static constexpr float MAX_DISTANCE = 1.0f;
  static constexpr int LABEL_UNVISITED = -1;
  static constexpr int LABEL_ISOLATED = -2;

  struct HCluster
  {
    HCluster() = default;
    HCluster(int _U, int _V, int _invalidation_step, int _size) : U(_U), V(_V), invalidation_step(_invalidation_step), size(_size) {}
    int U = -1;
    int V = -1;
    int invalidation_step = HC_VALID;
    int size = 0;
  };

  struct Dist
  {
    Dist() = default;
    Dist(int _U, int _V, float _dist) : U(_U), V(_V), dist(_dist) {}
    int U = -1;
    int V = -1;
    float dist = 1000.0f;
  };

  struct CCluster
  {
    int offset = -1;
    int size = 0;
    float creation_min_dist = MAX_DISTANCE;
    int creation_step = -1;
    int invalidation_step = HC_VALID;
    int cluster_id = -1;
  };

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
    //assert(settings.distance_importance.x == 1.0f || settings.distance_importance.y == 1.0f || settings.distance_importance.z == 1.0f);
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

    dataset.dim = ((dist_count + 7) / 8)*8; //to support AVX2 vectorization
    printf("creating dataset %.1f Mb\n", float(dataset.dim * ROT_COUNT * ((size_t)surface_node_count) * sizeof(float)) / 1024.0f / 1024.0f);
    dataset.all_points.resize(dataset.dim * ROT_COUNT * ((size_t)surface_node_count));
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
        size_t off_dataset = (size_t)(cur_point_group * ROT_COUNT + r_id) * dataset.dim;

        dataset.data_points[cur_point_group * ROT_COUNT + r_id].average_val = average_brick_val[i];
        dataset.data_points[cur_point_group * ROT_COUNT + r_id].data_offset = off_dataset;
        dataset.data_points[cur_point_group * ROT_COUNT + r_id].original_id = i;
        dataset.data_points[cur_point_group * ROT_COUNT + r_id].rotation_id = r_id;

        float sum = 0.0f;
        for (int x = 0; x < v_size; x++)
        {
          for (int y = 0; y < v_size; y++)
          {
            for (int z = 0; z < v_size; z++)
            {
              int idx = x * v_size * v_size + y * v_size + z;
              int3 rot_vec2 = int3(LiteMath::mul4x3(rotations4[r_id], float3(x, y, z)));
              int rot_idx = rot_vec2.x * v_size * v_size + rot_vec2.y * v_size + rot_vec2.z;
              float t = (g_octree.values_f[off_a + rot_idx] - average_brick_val[i]);
              sum += t*t;
              dataset.all_points[off_dataset + idx] = mask_norm * distance_mult_mask[idx] * t;
            }
          }
        }

        sum = sqrtf(sum);
        for (int i=0;i<dataset.dim;i++)
          dataset.all_points[off_dataset + i] /= sum;
      }
      cur_point_group++;
    }
  }

  void prepare_output(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, const Dataset &dataset,
                      const std::vector<float> &average_brick_val, const std::vector<Cluster> &clusters, 
              /*out*/ CompressionOutput &output)
  {
    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    auto inverse_indices = get_inverse_transform_indices();

    std::vector<float4x4> rotations4;
    initialize_rot_transforms(rotations4, v_size);


    //dataset has group with ROT_COUNT vectors (dataset.dim values each) for each surface node
    std::vector<size_t> node_id_to_dataset_group_off(g_octree.nodes.size());
    int surface_node_count = 0;
    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      if (g_octree.nodes[i].offset == 0 && g_octree.nodes[i].is_not_void)
      {
        node_id_to_dataset_group_off[i] = surface_node_count * ROT_COUNT * dataset.dim;
        surface_node_count++;
      }
    }

    //Find the best rotation id (for every brick in non-trivial cluster
    //find the rotation that gives the closest match to leading brick).
    //This is the place when we actually need the leading brick, so
    //the final quality can depend on what brick will be chosen, but not much.
    //Note, that we use dataset here (i.e. R^n projections of bricks), because we
    //should use the same metric here as with clustering.
    std::vector<int> rotation_ids(g_octree.nodes.size(), 0);
    for (auto &cluster : clusters)
    {
      size_t lead_off = node_id_to_dataset_group_off[cluster.lead_id];
      for (int node_id : cluster.point_ids)
      {
        if (node_id == cluster.lead_id)
        {
          rotation_ids[node_id] = 0;
        }
        else
        {
          float min_dist_sq = 1e6f;
          int min_rot_id = 0;
          for (int k = 0; k < ROT_COUNT; k++)
          {
            size_t target_off = node_id_to_dataset_group_off[node_id] + k * dataset.dim;
            float dist_sq = 0;
            for (int l = 0; l < dataset.dim; l++)
              dist_sq += (dataset.all_points[lead_off + l] - dataset.all_points[target_off + l]) * (dataset.all_points[lead_off + l] - dataset.all_points[target_off + l]);
            if (dist_sq < min_dist_sq)
            {
              min_dist_sq = dist_sq;
              min_rot_id = k;
            }
          }

          rotation_ids[node_id] = min_rot_id;
        }
      }
    }

    output.node_id_cleaf_id_remap.resize(g_octree.nodes.size(), 0);
    output.tranforms.resize(g_octree.nodes.size(), get_identity_transform());
    output.compressed_data.resize(clusters.size() * dist_count);

    //Main cycle to prepare the similarity compression output.
    //We replace each cluster with it's centroid. It is not perfect from
    //math standpoint, because we are in brick metric space, not R^n, but
    //it almost always gives better result that picking leading brick
    int unique_node_id = 0;
    std::vector<float> centroid(dist_count, 0);
    for (const Cluster &cluster : clusters)
    {
      int lead_id = cluster.lead_id;
      std::fill_n(centroid.data(), dist_count, 0);

      //calculate centroid
      //Note, that we use actual brick data and not dataset
      //because we want the centroid to be AN ACTUAL BRICK
      //(we'll actually render it, although with some transformations)
      for (int node_id : cluster.point_ids)
      {
        int off_a = g_octree.nodes[node_id].val_off;
        for (int x = 0; x < v_size; x++)
        {
          for (int y = 0; y < v_size; y++)
          {
            for (int z = 0; z < v_size; z++)
            {
              int idx = x * v_size * v_size + y * v_size + z;
              int3 rot_vec2 = int3(LiteMath::mul4x3(rotations4[rotation_ids[node_id]], float3(x, y, z)));
              int rot_idx = rot_vec2.x * v_size * v_size + rot_vec2.y * v_size + rot_vec2.z;
              centroid[idx] += g_octree.values_f[off_a + rot_idx];
            }
          }
        }
      }

      float centroid_average = 0;
      for (int k = 0; k < dist_count; k++)
      {
        centroid[k] /= cluster.count;
        centroid_average += centroid[k];
      }
      centroid_average /= dist_count;

      //Save centroid to output
      //Note: centroid[k] - centroid_average + average_brick_val[lead_id]
      //This line shifts centroid average to lead brick average
      //This is not necessary, but gives better result during brick dynamic compression
      //In some cases centroid can have weird average values that will be compressed with
      //large error. It might be bug, idk.
      for (int k = 0; k < dist_count; k++)
        output.compressed_data[unique_node_id*dist_count + k] = centroid[k] - centroid_average + average_brick_val[lead_id];

      for (int node_id : cluster.point_ids)
      {
        TransformCompact tc;
        tc.rotation_id = inverse_indices[rotation_ids[node_id]];
        tc.add = (average_brick_val[node_id] - average_brick_val[lead_id]);
        output.node_id_cleaf_id_remap[node_id] = unique_node_id;
        output.tranforms[node_id] = tc;
      }
      unique_node_id++;
    }
    output.leaf_count = unique_node_id;
  }

  void clustering_recursive_fill(std::vector<Cluster> &final_clusters, const std::vector<Cluster> &components,
                                 const std::vector<std::vector<Link>> &sim_graph, int total_node_count)
  {
    final_clusters.reserve(std::min<int>(2 * components.size(), total_node_count));

    std::vector<int> visited(total_node_count);
    std::vector<int> cur_cluster(total_node_count);
    int cur_cluster_size = 0;

    for (int component_id = 0; component_id < components.size(); component_id++)
    {
      const Cluster &component = components[component_id];
      if (component.count <= 2)
      {
        // printf("single cluster %d = [%d]\n", (int)final_clusters.size(), component.lead_id);
        // components with 2 or less points are already guaranteed to have distance < threshold
        // between any of their points
        final_clusters.push_back(component);
      }
      else
      {
        int prev_fc_size = final_clusters.size();
        std::fill_n(visited.data(), visited.size(), LABEL_ISOLATED);
        for (int i = 0; i < component.count; i++)
        {
          visited[component.point_ids[i]] = LABEL_UNVISITED;
        }
        int visited_count = 0;
        while (visited_count < component.count)
        {
          cur_cluster_size = 0;
          int start_id = 0; // rand() % component.count;
          int lead_id = -1;
          for (int i = 0; i < component.count; i++)
          {
            int id = component.point_ids[(start_id + i) % component.count];
            if (visited[id] == LABEL_UNVISITED)
            {
              lead_id = id;
              break;
            }
          }
          assert(lead_id != -1);
          visited[lead_id] = final_clusters.size();
          cur_cluster[cur_cluster_size++] = lead_id;

          // add all points from cluster that are reachable from lead_id but not yet visited
          for (int i = 0; i < sim_graph[lead_id].size(); i++)
          {
            int target_id = sim_graph[lead_id][i].target;
            if (visited[target_id] == LABEL_UNVISITED)
            {
              visited[target_id] = final_clusters.size();
              cur_cluster[cur_cluster_size++] = target_id;
            }
          }
          visited_count += cur_cluster_size;
          // printf("added cluster with %d points, %d/%d visited\n", cur_cluster_size, visited_count, component.count);

          final_clusters.emplace_back();
          final_clusters.back().lead_id = lead_id;
          final_clusters.back().count = cur_cluster_size;
          final_clusters.back().point_ids = std::vector<int>(cur_cluster.begin(), cur_cluster.begin() + cur_cluster_size);
        }

        // printf("added %d clusters\n", (int)(final_clusters.size() - prev_fc_size));
        // printf("old cluster [%d][", component.lead_id);
        // for (int i=0; i<component.count; i++)
        //   printf("%d, ", component.point_ids[i]);
        // printf("]\n");
        // printf("new clusters:\n");
        // for (int i=prev_fc_size; i<final_clusters.size(); i++)
        // {
        //   printf("cluster %d = [%d][", i, final_clusters[i].lead_id);
        //   for (int j=0; j<final_clusters[i].count; j++)
        //     printf("%d, ", final_clusters[i].point_ids[j]);
        //   printf("]\n");
        // }
      }
    }

    final_clusters.shrink_to_fit();
  }

  void clustering_hierarchical(const Settings &settings, std::vector<Cluster> &final_clusters,
                               const std::vector<Cluster> &components, const std::vector<std::vector<Link>> &sim_graph,
                               int total_node_count, int surface_node_count)
  {
    auto t00 = std::chrono::high_resolution_clock::now();

    std::atomic<unsigned> time_init_ns(0);
    std::atomic<unsigned> time_table_init_ns(0);
    std::atomic<unsigned> time_new_distances_ns(0);
    std::atomic<unsigned> time_find_min_ns(0);
    std::atomic<unsigned> time_ddg_traverse_ns(0);
    std::atomic<unsigned> time_ddg_sort_ns(0);
    std::atomic<unsigned> time_all_clusters_iterate_ns(0);
    std::atomic<unsigned> time_prepare_output_ns(0);

    final_clusters.reserve(std::min<int>(2 * components.size(), total_node_count));

    int max_component_size = 0;
    for (int i = 0; i < components.size(); i++)
      max_component_size = std::max(max_component_size, components[i].count);

    std::vector<std::vector<int>> all_cluster_contents(components.size());
    std::vector<std::vector<CCluster>> all_clusters(components.size());
    std::atomic<int> min_clusters_count(0);

    std::vector<int> stack(surface_node_count);
    std::vector<int> global_id_to_component_id(total_node_count, -1);
    std::vector<Dist> absolute_min_history(surface_node_count);

    int max_clusters_count = 2 * max_component_size;
    std::vector<HCluster> line_clusters(max_clusters_count); //each line in the distance matrix is a cluster
    std::vector<Dist> line_min(max_clusters_count, Dist(-1, -1, MAX_DISTANCE));
    std::vector<float> distance_matrix(max_clusters_count * max_clusters_count, MAX_DISTANCE);
    
    auto t0 = std::chrono::high_resolution_clock::now();
    time_init_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(t0-t00).count();

    // Main cycle. Iterate over connected compinents and perform hierarchical clustering
    // for each component independently
    
    //You can turn it on, but I haven't got any performance boost from multithreading here
    //Probably because of the fast that we have one largest component that that will be the bottleneck
    //#pragma omp parallel for schedule(dynamic)
    for (int component_id = 0; component_id < components.size(); component_id++)
    {
      const Cluster &component = components[component_id];
      if (component.count == 1) //there is nothing to do here
      {
        #pragma omp critical
        {
          final_clusters.push_back(component);
        }
        min_clusters_count++;
      }
      else
      {        
        printf("Hierarchical clustering, %d points\n", component.count);
        auto t1 = std::chrono::high_resolution_clock::now();

        Dist absolute_min(-1, -1, MAX_DISTANCE);
        std::fill_n(line_min.data(), max_clusters_count, Dist(-1, -1, MAX_DISTANCE));

        //build global_id_to_component_id remap, will be needed only in the next cycle
        for (int i = 0; i < component.count; i++)
          global_id_to_component_id[component.point_ids[i]] = i;

        //create initial clusters, each contain only one point from the component
        for (int i = 0; i < component.count; i++)
        {
          line_clusters[i] = HCluster(i, -1, HC_VALID, 1);

          distance_matrix[i * max_clusters_count + i] = 0.0f;
          for (auto &link : sim_graph[component.point_ids[i]])
          {
            int target_id = global_id_to_component_id[link.target];
            assert(target_id != -1);
            distance_matrix[i * max_clusters_count + target_id] = link.dist;
            if (link.dist < line_min[i].dist)
              line_min[i] = Dist(i, target_id, link.dist);
          }
          if (line_min[i].dist < absolute_min.dist)
            absolute_min = line_min[i];
        }

        auto t2 = std::chrono::high_resolution_clock::now();
        time_table_init_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();

        int clusters_left = component.count;
        int next_cluster_pos = component.count;
        int step = 0;

        //ok, there is a main cycle of hierarchical clustering
        //on each step we find the closest pair of clusters and merge them
        //and find the next closest pair
        //we maintain the closest pair in absolute_min
        //and closest pair for each cluster in line_min
        while (absolute_min.dist < settings.similarity_threshold &&
               clusters_left >= 2)
        {
          // printf("merge clusters U=%d V=%d with d = %f\n", absolute_min.U, absolute_min.V, absolute_min.dist);
          auto t3 = std::chrono::high_resolution_clock::now();

          //TODO: fix the underlying issue and uncomment this assert
          //assert(step == 0 || absolute_min_history[step-1].dist <= absolute_min.dist);
          absolute_min_history[step] = absolute_min;

          // merge clusters to create new one
          line_clusters[next_cluster_pos] = HCluster(absolute_min.U, absolute_min.V, HC_VALID,
                                                     line_clusters[absolute_min.U].size + line_clusters[absolute_min.V].size); // merge U and V
          line_clusters[absolute_min.U].invalidation_step = step+1;                                                              // invalidate U
          line_clusters[absolute_min.V].invalidation_step = step+1;                                                              // invalidate V

          // find distances from this cluster to all other valid clusters

          for (int i = 0; i < next_cluster_pos; i++)
          {
            if (line_clusters[i].invalidation_step == HC_VALID)
            {
              float d = std::max(distance_matrix[i * max_clusters_count + absolute_min.U],
                                 distance_matrix[i * max_clusters_count + absolute_min.V]);

              distance_matrix[i * max_clusters_count + next_cluster_pos] = d;
              distance_matrix[next_cluster_pos * max_clusters_count + i] = d;

              if (d < line_min[i].dist)
                line_min[i] = Dist(i, next_cluster_pos, d);
              if (d < line_min[next_cluster_pos].dist)
                line_min[next_cluster_pos] = Dist(next_cluster_pos, i, d);
            }
          }

          auto t4 = std::chrono::high_resolution_clock::now();
          time_new_distances_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t4 - t3).count();

          // recalculate minimums in lines where it was invalidated and find new absolute minimum
          absolute_min = Dist(-1, -1, MAX_DISTANCE);
          for (int i = 0; i <= next_cluster_pos; i++)
          {
            if (line_clusters[i].invalidation_step == HC_VALID)
            {
              if (line_min[i].dist < absolute_min.dist)
              {
                // printf("%d abs = %f\n", i, line_min[i].dist);
                // this minimum is a candidate to be the new absolute minimum and at the same time
                // it was invalidated. We have to recalculate it
                if (line_clusters[line_min[i].V].invalidation_step != HC_VALID)
                {
                  line_min[i] = Dist(-1, -1, MAX_DISTANCE);
                  for (int j = 0; j <= next_cluster_pos; j++)
                  {
                    if (i != j && line_clusters[j].invalidation_step == HC_VALID)
                    {
                      float d = distance_matrix[i * max_clusters_count + j];
                      if (d < line_min[i].dist)
                        line_min[i] = Dist(i, j, d);
                    }
                  }
                }
                assert(line_min[i].U == -1 || line_clusters[line_min[i].V].invalidation_step == HC_VALID);

                // line min can be changed above
                if (line_min[i].dist < absolute_min.dist)
                  absolute_min = line_min[i];
              }
            }
          }

          auto t5 = std::chrono::high_resolution_clock::now();
          time_find_min_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t5 - t4).count();

          // printf("line min = {\n");
          // for (int i=0; i<=next_cluster_pos; i++)
          // {
          //   if (line_clusters[line_min[i].U].invalidation_step == HC_VALID && line_clusters[line_min[i].V].invalidation_step == HC_VALID)
          //     printf("(%d, %d) %f\n", line_min[i].U, line_min[i].V, line_min[i].dist);
          //   else
          //     printf("(%d, %d) X\n", line_min[i].U, line_min[i].V);
          // }
          // printf("}\n");

          next_cluster_pos++;
          step++;
          clusters_left--;
        }

        //Now, after we finish clustering process, we build a clustering dendrogram
        //inside line_clusters, like every non-trivial cluster in this array has links to it's childeren
        //However, we are limited by similarity_threshold, so we probably haven't finished our dendrogram 
        //We also may want to get the desired number of clusters for the whole model, not the minimum
        //in each component.
        //To do all of this, we traverse this dendromgram (which is a forest, set of trees, actually)
        //And fill all_cluster_contents and all_clusters arrays
        //all_cluster_contents contains all the node ids and all_clusters have the cluster infos
        int top = 0;
        int cur_content_size = 0;
        int clusters_count = next_cluster_pos;

        auto t6 = std::chrono::high_resolution_clock::now();
        for (int i = 0; i < clusters_count; i++)
        {
          if (line_clusters[i].invalidation_step == HC_VALID)
          {
            stack[top] = i;
            top++;
          }
        }

        min_clusters_count += top; // these are clusters with HC_VALID invalidation_step
                                   // it means they cannot be merged together without breaking similarity threshold

        all_cluster_contents[component_id].resize(component.count, -1);
        all_clusters[component_id].resize(clusters_count);

        //Here we traverse the dendrogram and fill the arrays 
        //Dealing with explicit stack as it's faster than recursion
        while (top > 0)
        {
          int cur = stack[top - 1];
          top--;
          if (line_clusters[cur].size == 1) // it is a one-point cluster
          {
            all_clusters[component_id][cur].offset = cur_content_size;
            all_clusters[component_id][cur].size = 1;
            all_clusters[component_id][cur].creation_min_dist = 0.0f;
            all_clusters[component_id][cur].creation_step = 0;
            all_clusters[component_id][cur].invalidation_step = line_clusters[cur].invalidation_step;
            all_clusters[component_id][cur].cluster_id = cur;

            all_cluster_contents[component_id][cur_content_size] = component.point_ids[line_clusters[cur].U];
            cur_content_size++;
          }
          else
          {
            assert(cur >= component.count);
            int creation_step = cur - component.count;
            all_clusters[component_id][cur].offset = cur_content_size;
            all_clusters[component_id][cur].size = line_clusters[cur].size;
            all_clusters[component_id][cur].creation_min_dist = absolute_min_history[creation_step].dist;
            all_clusters[component_id][cur].creation_step = creation_step + 1;
            all_clusters[component_id][cur].invalidation_step = line_clusters[cur].invalidation_step;
            all_clusters[component_id][cur].cluster_id = cur;

            stack[top] = line_clusters[cur].U;
            stack[top + 1] = line_clusters[cur].V;
            top += 2;
          }
        }

        auto t7 = std::chrono::high_resolution_clock::now();
        time_ddg_traverse_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t7 - t6).count();

        //Sort the clusters by creation_min_dist
        //Remind that all_clusters have all the clusters from the dendrogram (incl. overlapping ones)
        //We then can iterate over this array to determine at which level of dendrogram we should stop
        std::sort(all_clusters[component_id].begin(), all_clusters[component_id].end(),
                  [](const CCluster &a, const CCluster &b)
                  { return a.creation_min_dist < b.creation_min_dist; });

        auto t8 = std::chrono::high_resolution_clock::now();
        time_ddg_sort_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t8 - t7).count();

        // printf("added %d clusters\n", (int)(final_clusters.size() - prev_fc_size));
        // printf("old cluster [%d][", component.lead_id);
        // for (int i=0; i<component.count; i++)
        //    printf("%d, ", component.point_ids[i]);
        // printf("]\n");
        // printf("new clusters:\n");

        // printf("indices = {");
        // for (int i=0;i<component.count;i++)
        //   printf("%d, ", all_cluster_contents[component_id][i]);
        // printf("}\n");

        // for (int i=0; i<all_clusters[component_id].size(); i++)
        // {
        //   printf("%d cluster %f data:[%d - %d] time:[%d - %d)\n",
        //          all_clusters[component_id][i].cluster_id,
        //          all_clusters[component_id][i].creation_min_dist,
        //          all_clusters[component_id][i].offset,
        //          all_clusters[component_id][i].offset + all_clusters[component_id][i].size,
        //          all_clusters[component_id][i].creation_step,
        //          all_clusters[component_id][i].invalidation_step);
        // }
        // for (int i=prev_fc_size; i<final_clusters.size(); i++)
        // {
        //   printf("cluster %d = [%d][", i, final_clusters[i].lead_id);
        //   for (int j=0; j<final_clusters[i].count; j++)
        //     printf("%d, ", final_clusters[i].point_ids[j]);
        //   printf("]\n");
        // }
      }
    }

    //Here we iterate all the cluster data from different components to determine at which level of dendrogram 
    //we should stop in each of them to met the target leaf count with the lowest error
    auto t9 = std::chrono::high_resolution_clock::now();
    std::vector<int> end_step_per_cluster(all_clusters.size(), MAX_STEPS);
    if (settings.target_leaf_count <= 0 || min_clusters_count >= settings.target_leaf_count)
    {
      // We have to take the last level of the clustering dendrogram in each component
      if (settings.target_leaf_count > 0)
        printf("Warning: settings.target_leaf_count is too low to influence the compression.\n");
    }
    else
    {
      // We should determine which level of dendrogram to take in each component

      // a.x is the component id, a.y is the current position
      auto list_cmp = [&all_clusters](int2 a, int2 b)
      {
        return all_clusters[a.x][a.y].creation_min_dist < all_clusters[b.x][b.y].creation_min_dist;
      };
      std::set<int2, decltype(list_cmp)> list_ends(list_cmp);

      int cur_nodes = surface_node_count;
      // first, we need to add to the list the first actual clusters
      for (int i = 0; i < all_clusters.size(); i++)
      {
        end_step_per_cluster[i] = 0;
        for (int j = 0; j < all_clusters[i].size(); j++)
        {
          if (all_clusters[i][j].size > 1) // it is a cluster, not a single point
            list_ends.emplace(int2(i, j));
        }
      }

      while (cur_nodes > settings.target_leaf_count+1)
      {
        assert(list_ends.size() > 0);
        int2 end = *list_ends.begin();
        list_ends.erase(list_ends.begin());
        end_step_per_cluster[end.x] = all_clusters[end.x][end.y].creation_step;
        if (end.y + 1 < all_clusters[end.x].size())
          list_ends.emplace(int2(end.x, end.y + 1));
        cur_nodes--;
      }
    }

    auto t10 = std::chrono::high_resolution_clock::now();
    time_all_clusters_iterate_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t10 - t9).count();

    //Finally we transform the data into the format expected by the output
    for (int i = 0; i < all_clusters.size(); i++)
    {
      for (int j = 0; j < all_clusters[i].size(); j++)
      {
        CCluster &clust = all_clusters[i][j];
        if (clust.creation_step <= end_step_per_cluster[i] && clust.invalidation_step > end_step_per_cluster[i])
        {
          final_clusters.emplace_back();
          final_clusters.back().lead_id = all_cluster_contents[i][clust.offset + 0];
          final_clusters.back().count = clust.size;
          final_clusters.back().point_ids = std::vector<int>(all_cluster_contents[i].begin() + clust.offset, all_cluster_contents[i].begin() + clust.offset + clust.size);
        }
      }
    }

    auto t11 = std::chrono::high_resolution_clock::now();
    time_prepare_output_ns += std::chrono::duration_cast<std::chrono::nanoseconds>(t11 - t10).count();

    unsigned time_A = std::chrono::duration_cast<std::chrono::nanoseconds>(t9 - t0).count();

    printf("time_init                 (ms) = %.3f\n", 1e-6f*time_init_ns);
    printf("time_table_init           (ms) = %.3f\n", 1e-6f*time_table_init_ns);
    printf("time_new_distances        (ms) = %.3f\n", 1e-6f*time_new_distances_ns);
    printf("time_find_min             (ms) = %.3f\n", 1e-6f*time_find_min_ns);
    printf("time_ddg_traverse         (ms) = %.3f\n", 1e-6f*time_ddg_traverse_ns);
    printf("time_ddg_sort             (ms) = %.3f\n", 1e-6f*time_ddg_sort_ns);
    printf("time_all_clusters_iterate (ms) = %.3f\n", 1e-6f*time_all_clusters_iterate_ns);
    printf("time_prepare_output       (ms) = %.3f\n", 1e-6f*time_prepare_output_ns);
    printf("TOTAL                     (ms) = %.3f\n", 1e-6f*(time_table_init_ns + time_new_distances_ns + time_find_min_ns + 
                                                             time_ddg_traverse_ns + time_ddg_sort_ns + time_all_clusters_iterate_ns + 
                                                             time_prepare_output_ns + time_init_ns));
    //printf("A                         (ms) = %.3f\n", 1e-6f*time_A);
    final_clusters.shrink_to_fit();
  }

  void similarity_compression_advanced(const sdf_converter::GlobalOctree &g_octree, const Settings &settings, CompressionOutput &output)
  {
    auto tt1 = std::chrono::high_resolution_clock::now();
 
    int total_node_count = g_octree.nodes.size();
    int surface_node_count = 0;

    int v_size = g_octree.header.brick_size + 2 * g_octree.header.brick_pad + 1;
    int dist_count = v_size * v_size * v_size;
    std::vector<float> average_brick_val(g_octree.nodes.size(), 0);
    std::vector<float> closest_dist(g_octree.nodes.size(), settings.similarity_threshold);
    std::vector<bool> surface_node(g_octree.nodes.size(), false);

    auto inverse_indices = get_inverse_transform_indices();

    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      int off = g_octree.nodes[i].val_off;

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

    if (settings.search_algorithm == SearchAlgorithm::LINEAR_SEARCH ||
        settings.search_algorithm == SearchAlgorithm::BRUTE_FORCE)
    {
      NN_search_AS.reset(new LinearSearchAS());
      NN_search_AS->build(dataset, 1);
    }
    else if (settings.search_algorithm == SearchAlgorithm::BALL_TREE)
    {
      switch (dataset.dim)
      {
        case  2*2*2: NN_search_AS.reset(new BallTreeFD<8>());  break;
        case  32:    NN_search_AS.reset(new BallTreeFD<32>());  break; //3*3*3 + padding
        case  4*4*4: NN_search_AS.reset(new BallTreeFD<64>()); break;
        case  128:   NN_search_AS.reset(new BallTreeFD<128>());break;  //5*5*5 + padding
        case  6*6*6: NN_search_AS.reset(new BallTreeFD<216>());break;
        case  344:   NN_search_AS.reset(new BallTreeFD<344>());break;  //7*7*7 + padding
        case  8*8*8: NN_search_AS.reset(new BallTreeFD<512>());break;
        default: NN_search_AS.reset(new BallTree());       break;
      }
      
      NN_search_AS->build(dataset, 32);
    }

    auto tt3 = std::chrono::high_resolution_clock::now();
    printf("build NN search AS %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(tt3-tt2).count());  

    int max_threads = omp_get_max_threads();
    std::vector<std::vector<Link>> sim_graph(total_node_count); //sparse adjacency matrix

    std::vector<std::vector<Link>> tmp_links(max_threads);
    for (int i=0; i<max_threads; i++)
      tmp_links[i].resize(ROT_COUNT*surface_node_count, Link());
  

  auto t1 = std::chrono::high_resolution_clock::now();
    #pragma omp parallel for schedule(dynamic)
    for (int point_a = 0; point_a < dataset.data_points.size(); point_a+=ROT_COUNT)
    {
      int thread_id = omp_get_thread_num();
      int link_count = 0;
      int i  = dataset.data_points[point_a].original_id;
      if (surface_node[i] == false)
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
    std::vector<Cluster> components;
    std::vector<unsigned> stack(2*total_node_count);
    int head = 0;
    for (int lead_id=0;lead_id<total_node_count;lead_id++)
    {
      if (labels[lead_id] != LABEL_UNVISITED || surface_node[lead_id] == false)
        continue;
      else 
      {
        int cur_component_id = components.size();
        components.emplace_back();

        int total_count = 0;
        if (sim_graph[lead_id].empty())
        {
          total_count = 1;
          labels[lead_id] = LABEL_ISOLATED;
          components[cur_component_id].point_ids.push_back(lead_id);
        }
        else
        {
          head = 0;
          stack[head] = lead_id;
          labels[lead_id] = cur_component_id;

          while (head >= 0)
          {
            int node_id = stack[head];
            head--;
            total_count++;
            components[cur_component_id].point_ids.push_back(node_id);
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
        components[cur_component_id].lead_id = lead_id;
        components[cur_component_id].count = total_count;
      }
    }

    auto t3 = std::chrono::high_resolution_clock::now();
    printf("find connected components %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count());

    //clustering
    std::vector<Cluster> final_clusters;
    if (settings.clustering_algorithm == ClusteringAlgorithm::COMPONENTS_MERGE)
    {
      final_clusters = std::move(components);
    }
    else if (settings.clustering_algorithm == ClusteringAlgorithm::COMPONENTS_RECURSIVE_FILL)
    {
      clustering_recursive_fill(final_clusters, components, sim_graph, total_node_count);
    }
    else if (settings.clustering_algorithm == ClusteringAlgorithm::HIERARCHICAL)
    {
      clustering_hierarchical(settings, final_clusters, components, sim_graph, total_node_count, surface_node_count);
    }

    auto t4 = std::chrono::high_resolution_clock::now();
    printf("clustering %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t4-t3).count()); 

    prepare_output(g_octree, settings, dataset, average_brick_val, final_clusters, output);
    auto t5 = std::chrono::high_resolution_clock::now();
    printf("prepare output %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t5-t4).count());  
    
    printf("total time = %.2f ms\n", 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t5-tt1).count());
    
    int remapped = 0;
    for (int i = 0; i < final_clusters.size(); i++)
      remapped += final_clusters[i].count - 1;

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
    std::vector<TransformCompact> remap_transforms(g_octree.nodes.size());
    std::vector<bool> surface_node(g_octree.nodes.size(), false);

    auto inverse_indices = get_inverse_transform_indices();

    for (int i=0; i<g_octree.nodes.size(); i++)
    {
      int off = g_octree.nodes[i].val_off;

      remap[i] = i;
      remap_transforms[i] = get_identity_transform();

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

    if (settings.search_algorithm == SearchAlgorithm::LINEAR_SEARCH ||
        settings.search_algorithm == SearchAlgorithm::BRUTE_FORCE)
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
      
      size_t off_a = dataset.data_points[point_a].data_offset;

      if (settings.search_algorithm == SearchAlgorithm::BRUTE_FORCE)
      {
        #pragma omp parallel for schedule(static)
        for (int point_b = point_a + ROT_COUNT; point_b < dataset.data_points.size(); point_b++)
        {
          int j = dataset.data_points[point_b].original_id;
          size_t off_b = dataset.data_points[point_b].data_offset;

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

    std::vector<Cluster> clusters;
    std::vector<int> lead_node_id_to_cluster_id(g_octree.nodes.size(), -1);

    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (surface_node[i] && remap[i] == i)
      {
        clusters.emplace_back();
        clusters.back().lead_id = i;
        lead_node_id_to_cluster_id[i] = clusters.size() - 1;
      }
    }
    for (int i = 0; i < g_octree.nodes.size(); i++)
    {
      if (surface_node[i])
      {
        clusters[lead_node_id_to_cluster_id[remap[i]]].point_ids.push_back(i);
        clusters[lead_node_id_to_cluster_id[remap[i]]].count++;
      }
    }
    prepare_output(g_octree, settings, dataset, average_brick_val, clusters, output);
  }

  void similarity_compression(const sdf_converter::GlobalOctree &octree, const Settings &settings, CompressionOutput &output)
  {
    if (settings.clustering_algorithm == ClusteringAlgorithm::HIERARCHICAL || 
        settings.clustering_algorithm == ClusteringAlgorithm::COMPONENTS_MERGE ||
        settings.clustering_algorithm == ClusteringAlgorithm::COMPONENTS_RECURSIVE_FILL)
    {
      similarity_compression_advanced(octree, settings, output);
    }
    else if (settings.clustering_algorithm == ClusteringAlgorithm::REPLACEMENT)
    {
      similarity_compression_replacement(octree, settings, output);
    }
    else
    {
      printf("[similarity_compression::ERROR] unknown clustering algorithm\n");
    }
  }
}