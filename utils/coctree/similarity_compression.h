#pragma once
#include "utils/sdf/sdf_scene.h"
#include "utils/sdf/sparse_octree_builder.h"
#include <vector>

namespace scom
{
  static constexpr uint32_t VALID_TRANSFORM_CODE_BIT = 0x80000000u;
  static constexpr int ROT_COUNT = 48;
  static constexpr int ROT_ID_IDENTITY = 0;

  enum class ClusteringAlgorithm
  {
    REPLACEMENT,               //fast, guarantees that no replacement with loss > similarity_threshold will happen
                               //can ofter miss possible merges, does not use target_leaf_count 
    COMPONENTS_MERGE,          //faster than REPLACEMENT (because of multithreading) with stronger compression
                               //creates one cluster for each component in thr-distance graph
                               //can possible merge very dissimilar brick if they are connected with a long chain
                               //does not use target_leaf_count 
    COMPONENTS_RECURSIVE_FILL, //faster than REPLACEMENT (because of multithreading) with similar quality
                               //guarantees that no replacement with loss > similarity_threshold will happen
                               //does not use target_leaf_count 
    HIERARCHICAL               //the most theoretically sound, but slower that COMPONENTS_MERGE or REPLACEMENT
                               //guarantees that no replacement with loss > similarity_threshold will happen
                               //can use target_leaf_count for more fine-tured compression to the exact size
  };

  enum class SearchAlgorithm
  {
    BRUTE_FORCE,   //for cycle for all nodes
    LINEAR_SEARCH, //even slower than BRUTE_FORCE, for debug only
    BALL_TREE      //use ball tree acceleration structure
  };

  struct Settings
  {
    ClusteringAlgorithm clustering_algorithm = ClusteringAlgorithm::REPLACEMENT;
    SearchAlgorithm search_algorithm = SearchAlgorithm::BRUTE_FORCE;
    float similarity_threshold = 0.0f; //set negative value to disable and use only target_leaf_count
    int   target_leaf_count = -1;      //set negative value to disable and use only similarity_threshold
    float3 distance_importance = float3(1,1,1); // importance of padding, border, internal distances respectively
  };

  struct TransformCompact
  {
    unsigned rotation_id;
    float add;
  };

  struct CompressionOutput
  {
    int leaf_count;
    std::vector<float> compressed_data;           // data for all leaf nodes remained after similarity compression
    std::vector<uint32_t> node_id_cleaf_id_remap; // ids of leaf node in list of leaves left after similarity compression,
                                                  // for each node from global octree (0 for non-leaf nodes)
    std::vector<TransformCompact> tranforms;      // similarity compression transform codes for each node from global octree
  };
  
  void similarity_compression(const sdf_converter::GlobalOctree &octree, const Settings &settings, CompressionOutput &output);
  void initialize_rot_transforms(std::vector<float4x4> &rot_transforms, int v_size);
  void initialize_rot_modifiers(std::vector<int4> &rot_modifiers, int v_size);
  TransformCompact get_identity_transform();
}