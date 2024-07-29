#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"
#include "../utils/sdf_converter.h"
#include "../utils/sparse_octree_2.h"
#include "../utils/marching_cubes.h"
#include "../utils/sdf_smoother.h"
#include "../utils/demo_meshes.h"
#include "../utils/image_metrics.h"
#include "../diff_render/MultiRendererDR.h"

#include <functional>
#include <cassert>
#include <chrono>

SdfSBS circle_smallest_scene();
std::vector<float4x4> get_cameras_uniform_sphere(int count, float3 center, float radius);
void randomize_distance(SdfSBS &sbs, float delta);

void benchmark_iteration_time()
{
  // create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_smallest_scene();

  unsigned W = 512, H = 512;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  preset.spp = 16;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(1, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0, 0, W, H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
  }

  // put random colors to SBS
  auto indexed_SBS = circle_smallest_scene();
  // randomize_color(indexed_SBS);
  randomize_distance(indexed_SBS, 0.25f);

  dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

  dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_render_mode = dr::DR_RENDER_MODE_MASK;
  dr_preset.dr_reconstruction_type = dr::DR_RECONSTRUCTION_TYPE_GEOMETRY;
  dr_preset.opt_iterations = 10;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 16;

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  std::vector<float> grad_dr(param_count, 0);
  std::vector<float> grad_ref(param_count, 0);

  dr::MultiRendererDR dr_render;
  dr_render.SetReference(images_ref, view, proj);

  //there can be some tricky stuff, like multithreading, GPU and so on, so we need wall time clock
  //launch this multiple time with now background tasks on your PC to get maximum precision
  for (int i = 0; i < 3; i++)
  {
  auto t1 = std::chrono::system_clock::now();
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
  auto t2 = std::chrono::system_clock::now();

    double time = 1e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    double rays = W*H*dr_preset.opt_iterations*dr_preset.spp;
    printf("benchmark_iteration_time: %.3f sec, %.3f MRays/sec\n", time, 1e-6*rays/time);
  }
}