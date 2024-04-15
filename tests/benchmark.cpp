#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "../utils/sparse_octree.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"

#include <functional>
#include <cassert>
#include <chrono>
#include <map>

void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset);

struct BenchmarkResult
{
  unsigned iters;
  float render_average_time_ms;
  std::string scene_name;
  std::string preset_name;
};

void benchmark_framed_octree_intersection()
{
  constexpr unsigned iters = 100;
  std::vector<BenchmarkResult> results;

  std::vector<std::string> scene_names = {"Teapot"};
  std::vector<std::string> scene_paths = {"scenes/01_simple_scenes/data/teapot.vsgf"}; 


  std::vector<unsigned> presets_ob = {SDF_OCTREE_BLAS_NO, SDF_OCTREE_BLAS_DEFAULT,
                                      SDF_OCTREE_BLAS_DEFAULT, SDF_OCTREE_BLAS_DEFAULT,
                                      SDF_OCTREE_BLAS_DEFAULT};

  std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_DEFAULT, SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                                      SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                                      SDF_OCTREE_NODE_INTERSECT_NEWTON};

  std::vector<std::string> preset_names = {"no_bvh_traversal", "bvh_traversal", "bvh_sphere_tracing", "bvh_analytic", "bvh_newton"};


  for (int scene_n = 0; scene_n < scene_names.size(); scene_n++)
  {
    auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+scene_paths[scene_n]).c_str());

    float3 mb1,mb2, ma1,ma2;
    cmesh4::get_bbox(mesh, &mb1, &mb2);
    cmesh4::rescale_mesh(mesh, float3(-0.9,-0.9,-0.9), float3(0.9,0.9,0.9));
    cmesh4::get_bbox(mesh, &ma1, &ma2);
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    SparseOctreeBuilder builder;
    SparseOctreeSettings settings{8, 4, 0.0f};
    std::vector<SdfFrameOctreeNode> frame_nodes;

    builder.construct([&mesh_bvh](const float3 &p) { return mesh_bvh.get_signed_distance(p); }, settings);
    builder.convert_to_frame_octree(frame_nodes);

    unsigned W = 1024, H = 1024;
    LiteImage::Image2D<uint32_t> image(W, H);

    for (int i=0; i<presets_ob.size(); i++)
    {
      MultiRenderPreset preset = getDefaultPreset();
      preset.mode = MULTI_RENDER_MODE_LAMBERT;
      preset.sdf_frame_octree_blas = presets_ob[i];
      preset.sdf_frame_octree_intersect = presets_oi[i];

      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene({(unsigned)frame_nodes.size(), 
                        frame_nodes.data()});

      double sum_ms = 0.0;
      render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
      for (int iter = 0; iter<iters; iter++)
      {
        float timings[4] = {0,0,0,0};
    auto t1 = std::chrono::steady_clock::now();
        pRender->Render(image.data(), image.width(), image.height(), "color"); 
        pRender->GetExecutionTime("CastRaySingleBlock", timings);
    auto t2 = std::chrono::steady_clock::now();
        float time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
        printf("%s rendered in %.1f ms. %d kRays/s\n", "SDF Framed Octree", time_ms, (int)((W * H) / time_ms));
        printf("CastRaySingleBlock took %.1f ms\n", timings[0]);
        sum_ms += timings[0];
      }

      results.emplace_back();
      results.back().scene_name = scene_names[scene_n];
      results.back().preset_name = preset_names[i];
      results.back().iters = iters;
      results.back().render_average_time_ms = sum_ms/iters;
    }
  }

  for (auto &res : results)
  {
    printf("[%s + %s] %.2f ms/frame \n", res.scene_name.c_str(), res.preset_name.c_str(), res.render_average_time_ms);
  }
}