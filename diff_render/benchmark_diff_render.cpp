#include "../tests/tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"
#include "../utils/sdf_converter.h"
#include "../utils/sparse_octree_builder.h"
#include "../utils/marching_cubes.h"
#include "../utils/sdf_smoother.h"
#include "../utils/demo_meshes.h"
#include "../utils/image_metrics.h"
#include "../diff_render/MultiRendererDR.h"

#include <functional>
#include <cassert>
#include <chrono>

using namespace dr;

void benchmark_iteration_time(MultiRendererDRPreset dr_preset)
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
    auto pRender = CreateMultiRenderer(DEVICE_CPU);
    pRender->SetPreset(preset);
    pRender->SetViewport(0, 0, W, H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
  }

  // put random colors to SBS
  auto indexed_SBS = circle_smallest_scene();
  // randomize_color(indexed_SBS);
  randomize_distance(indexed_SBS, 0.25f);

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  std::vector<float> grad_dr(param_count, 0);
  std::vector<float> grad_ref(param_count, 0);

  MultiRendererDR dr_render;
  dr_render.SetReference(images_ref, view, proj);

  //there can be some tricky stuff, like multithreading, GPU and so on, so we need wall time clock
  //launch this multiple time with now background tasks on your PC to get maximum precision
  for (int i = 0; i < 3; i++)
  {
  auto t1 = std::chrono::system_clock::now();
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
  auto t2 = std::chrono::system_clock::now();

    double time = 1e-9*std::chrono::duration_cast<std::chrono::nanoseconds>(t2 - t1).count();
    double rays = W*H*dr_preset.opt_iterations*dr_preset.spp;
    printf("benchmark_iteration_time: %.3f sec, %.3f MRays/sec\n", time, 1e-6*rays/time);
  }
}

void benchmark_iteration_time()
{
  MultiRendererDRPreset dr_preset = getDefaultPresetDR();

  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.opt_iterations = 10;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 16;

  printf("DR_RENDER_MODE_LAMBERT + DR_RECONSTRUCTION_FLAG_COLOR\n");
  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_COLOR;
  benchmark_iteration_time(dr_preset);

  printf("DR_RENDER_MODE_MASK + DR_RECONSTRUCTION_FLAG_GEOMETRY\n");
  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  benchmark_iteration_time(dr_preset);

  printf("DR_RENDER_MODE_DIFFUSE + DR_RECONSTRUCTION_FLAG_GEOMETRY\n");
  dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  benchmark_iteration_time(dr_preset);

  printf("DR_RENDER_MODE_LAMBERT + DR_RECONSTRUCTION_FLAG_GEOMETRY\n");
  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  benchmark_iteration_time(dr_preset);
}

void benchmark_dr_optimization()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 1024, H = 1024;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 16;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);
  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");

  std::vector<float4x4> view = get_cameras_turntable(8, float3(0, 0, 0), 3.0f, 1.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer(DEVICE_GPU);
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);

    pRender->SetScene(mesh);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/bunny_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();
    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
    dr_preset.border_relax_eps = 0.005f;
    dr_preset.opt_iterations = 501;
    dr_preset.opt_lr = 0.03f;
    dr_preset.spp = 16;
    dr_preset.border_spp = 1024;
    dr_preset.image_batch_size = 2;
    dr_preset.render_width = 128;
    dr_preset.render_height = 128;

    dr_preset.debug_print = true;
    dr_preset.debug_print_interval = 1;
    dr_preset.debug_progress_images = MULTI_RENDER_MODE_LAMBERT;
    dr_preset.debug_progress_interval = 10;

    MultiRendererDRPreset dr_preset_2 = dr_preset;
    dr_preset_2.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset_2.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
    dr_preset_2.render_width = 2*128;
    dr_preset_2.render_height = 2*128;
    dr_preset_2.opt_lr = 0.01f;
    dr_preset_2.opt_iterations = 201;
    dr_preset_2.spp = 16;

    MultiRendererDRPreset dr_preset_3 = dr_preset;
    dr_preset_3.render_width = 3*128;
    dr_preset_3.render_height = 3*128;
    dr_preset_3.opt_lr = 0.005f;
    dr_preset_3.opt_iterations = 101;
    dr_preset_3.spp = 16;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeGrid(8, false, {dr_preset, dr_preset_2, dr_preset_3});
    dr_render.SetViewport(0, 0, W, H);
    dr_render.UpdateCamera(view[0], proj[0]);
    dr_render.Clear(W, H, "color");
    dr_render.RenderFloat(image_res.data(), W, H, "color", 1);
    LiteImage::SaveImage<float4>("saves/bunny_res.bmp", image_res);
  }

  float psnr = image_metrics::PSNR(image_res, images_ref[0]);
  printf("optimization finished. PSNR: %f\n", psnr);
}