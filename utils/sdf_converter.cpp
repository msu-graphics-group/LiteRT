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
          grid.data[i*sz*sz + j*sz + k] = sdf(2.0f*(float3(k+0.5, j+0.5, i+0.5)/float(sz)) - 1.0f, omp_get_thread_num());
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

    header.v_size = header.brick_size + 2*header.brick_pad + 1;

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

  SdfHPOctree create_sdf_hp_octree(HPOctreeBuilder::BuildSettings settings, DistanceFunction sdf)
  {
    return create_sdf_hp_octree(settings, [&](const float3 &p, unsigned idx) -> float { return sdf(p); }, 1);
  }

  SdfHPOctree create_sdf_hp_octree(HPOctreeBuilder::BuildSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    HPOctreeBuilder builder;
    builder.construct(sdf, settings);
    return builder.octree;
  }

  SdfHPOctree create_sdf_hp_octree(HPOctreeBuilder::BuildSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    HPOctreeBuilder builder;
    builder.construct(mesh, settings);
    return builder.octree;
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
}