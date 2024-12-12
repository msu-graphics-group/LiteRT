#include "sdf_converter.h"
#include "utils/mesh/mesh_bvh.h"
#include "sparse_octree_builder.h"
#include "utils/mesh/mesh.h"
#include "utils/coctree/similarity_compression.h"
#include "omp.h"
#include <chrono>

namespace sdf_converter
{
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

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    GlobalOctree g;
    g.header.brick_size = 1;
    g.header.brick_pad = 0;
    std::vector<SdfFrameOctreeNode> frame;
    sdf_to_global_octree(settings, sdf, max_threads, g);
    global_octree_to_frame_octree(g, frame);

    return frame;
  }

  std::vector<SdfFrameOctreeNode> create_sdf_frame_octree(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    GlobalOctree g;
    g.header.brick_size = 1;
    g.header.brick_pad = 0;
    std::vector<SdfFrameOctreeNode> frame;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
      mesh_octree_to_global_octree(mesh, tlo, g);
    }
    global_octree_to_frame_octree(g, frame);

    return frame;
  }
  
  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    GlobalOctree g;
    g.header.brick_size = 1;
    g.header.brick_pad = 0;
    std::vector<SdfSVSNode> svs;
    sdf_to_global_octree(settings, sdf, max_threads, g);
    global_octree_to_SVS(g, svs);

    return svs;
  }

  std::vector<SdfSVSNode> create_sdf_SVS(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    GlobalOctree g;
    g.header.brick_size = 1;
    g.header.brick_pad = 0;
    std::vector<SdfSVSNode> svs;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
      mesh_octree_to_global_octree(mesh, tlo, g);
    }
    global_octree_to_SVS(g, svs);

    return svs;
  }

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, MultithreadedDistanceFunction sdf, unsigned max_threads)
  {
    GlobalOctree g;
    g.header.brick_size = header.brick_size;
    g.header.brick_pad = header.brick_pad;
    SdfSBS sbs;
    sbs.header = header;
    sdf_to_global_octree(settings, sdf, max_threads, g);
    global_octree_to_SBS(g, sbs);

    return sbs;
  }

  SdfSBS create_sdf_SBS(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh)
  {
    assert(settings.depth > 1);
    assert(header.brick_size >= 1 && header.brick_size <= 16);
    assert(header.brick_pad == 0 || header.brick_pad == 1);
    assert(header.bytes_per_value == 1 || header.bytes_per_value == 2 || header.bytes_per_value == 4);

    GlobalOctree g;
    g.header.brick_size = header.brick_size;
    g.header.brick_pad = header.brick_pad;
    SdfSBS sbs;
    sbs.header = header;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
      mesh_octree_to_global_octree(mesh, tlo, g);
    }
    global_octree_to_SBS(g, sbs);

    return sbs;
  }

  std::vector<SdfFrameOctreeTexNode> create_sdf_frame_octree_tex(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    if (settings.build_type == SparseOctreeBuildType::MESH_TLO)
    {
      //we set max_triangles_per_leaf = 0 to prevent issues with big, textured triangles (see test with textured cube)
      GlobalOctree g;
      g.header.brick_size = 1;
      g.header.brick_pad = 0;
      {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
      mesh_octree_to_global_octree(mesh, tlo, g);
      }
      std::vector<SdfFrameOctreeTexNode> frame;
      global_octree_to_frame_octree_tex(g, frame);
      return frame;
    }
    else
    {
      printf("Frame octree with texture can be built only from mesh with MESH_TLO build type\n");
      return {};
    }
  }

  SdfSBS create_sdf_SBS_tex(SparseOctreeSettings settings, SdfSBSHeader header, const cmesh4::SimpleMesh &mesh, bool noisy)
  {
    header.aux_data = SDF_SBS_NODE_LAYOUT_DX_UV16;

    unsigned max_threads = omp_get_max_threads();
    GlobalOctree g;
    g.header.brick_size = header.brick_size;
    g.header.brick_pad = header.brick_pad;
    auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);

    if (noisy)
    {    
      std::vector<MeshBVH> bvh(max_threads);
      for (unsigned i = 0; i < max_threads; i++)
        bvh[i].init(mesh);
      MultithreadedDistanceFunction mt_sdf = [&, noisy](const float3 &p, unsigned idx) -> float 
                                            {auto x = (fmod(p.x * 153 + p.y * 427 + p.z * 311, 2.0) - 1) * 0.003;
                                             return bvh[idx].get_signed_distance(p) + x; };
      sdf_to_global_octree(settings, mt_sdf, max_threads, g);
    }
    else
    {
      mesh_octree_to_global_octree(mesh, tlo, g);
    }

    SdfSBS sbs;
    sbs.header = header;
    global_octree_to_SBS(g, sbs);

    return sbs;
  }

  COctreeV2 create_COctree_v2(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh)
  {
    COctreeV2 coctree;
    
    GlobalOctree g;
    g.header.brick_size = 1;
    g.header.brick_pad = 0;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
      mesh_octree_to_global_octree(mesh, tlo, g);
    }
    global_octree_to_COctreeV2(g, coctree);

    return coctree;
  }

  COctreeV3 create_COctree_v3(SparseOctreeSettings settings, 
                              COctreeV3Settings co_settings, 
                              const cmesh4::SimpleMesh &mesh)
  {
    COctreeV3 coctree;
    GlobalOctree g;
    g.header.brick_size = co_settings.brick_size;
    g.header.brick_pad = co_settings.brick_pad;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f + 0.5f*co_settings.brick_pad/(float)co_settings.brick_size);
      mesh_octree_to_global_octree(mesh, tlo, g);
    }
    int types[4] = {0,0,0,0};
    for (int i = 0; i < g.nodes.size(); i++)
    {
      types[(int)g.nodes[i].type] += 1;
    }
    printf("types: %d %d %d %d\n", types[0], types[1], types[2], types[3]);
    global_octree_to_COctreeV3(g, coctree, co_settings);
    
    return coctree;
  }

  COctreeV3 create_COctree_v3(SparseOctreeSettings settings, 
                              COctreeV3Settings co_settings, 
                              scom::Settings  scom_settings, 
                              const cmesh4::SimpleMesh &mesh)
  {
    COctreeV3 coctree;

    GlobalOctree g;
    g.header.brick_size = co_settings.brick_size;
    g.header.brick_pad = co_settings.brick_pad;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
      mesh_octree_to_global_octree(mesh, tlo, g);
    }
    global_octree_to_COctreeV3(g, coctree, co_settings, scom_settings);

    return coctree;
  }

  GlobalOctree create_global_octree_by_mesh(SparseOctreeSettings settings, const cmesh4::SimpleMesh &mesh, 
                                            GlobalOctreeHeader &header)
  {
    assert(header.brick_size != 0);
    if (settings.build_type != SparseOctreeBuildType::MESH_TLO)
    {
      printf("WRONG FUNCTION\n");
      return {};
    }
    float mult = (float) (header.brick_pad * 2 + header.brick_size);
    mult /= (float) header.brick_size;
    GlobalOctree glob;
    glob.header = header;
    {
      auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, mult);//change 1.0f to another mult depend of the pad
      mesh_octree_to_global_octree(mesh, tlo, glob);
    }
    return glob;
  }
}