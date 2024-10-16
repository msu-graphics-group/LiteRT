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
#include "../utils/stat_utils.h"
#include "../render_common.h"

#include <functional>
#include <cassert>
#include <chrono>

using namespace dr;

void render(LiteImage::Image2D<float4> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum = 1)
{
  float fov_degrees = 60;
  float z_near = 0.1f;
  float z_far = 100.0f;
  float aspect   = 1.0f;
  auto proj      = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
  auto worldView = LiteMath::lookAt(pos, target, up);

  pRender->RenderFloat(image.data(), image.width(), image.height(), worldView, proj, preset, a_passNum);
}

void diff_render_test_1_forward_pass()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
  preset.spp = 16;
  
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 5);

  SdfSBSHeader header;
  header.brick_size = 4;
  header.brick_pad = 0;
  header.bytes_per_value = 1;
  SdfSBS indexed_SBS;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");

  float4x4 view, proj;
  LiteImage::Image2D<float4> image_mesh(W, H);
  LiteImage::Image2D<float4> image_SBS(W, H);
  LiteImage::Image2D<float4> image_SBS_depth(W, H);
  LiteImage::Image2D<float4> image_SBS_prim(W, H);

  LiteImage::Image2D<float4> image_SBS_dr(W, H);
  LiteImage::Image2D<float4> image_SBS_dr_2(W, H);
  LiteImage::Image2D<float4> image_SBS_dr_depth(W, H);
  LiteImage::Image2D<float4> image_SBS_dr_prim(W, H);

  LiteImage::Image2D<float4> image_SBS_diff(W, H);

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);

    pRender->SetScene(mesh);
    render(image_mesh, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    view = pRender->getWorldView();
    proj = pRender->getProj();
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);  

    indexed_SBS = sdf_converter::create_sdf_SBS_indexed_with_neighbors(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures());
    pRender->SetScene(indexed_SBS);

    preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
    preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;
    pRender->RenderFloat(image_SBS.data(), image_SBS.width(), image_SBS.height(), view, proj, preset);   

    preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
    pRender->RenderFloat(image_SBS_depth.data(), image_SBS.width(), image_SBS.height(), view, proj, preset);   

    preset.render_mode = MULTI_RENDER_MODE_PRIMITIVE;
    pRender->RenderFloat(image_SBS_prim.data(), image_SBS.width(), image_SBS.height(), view, proj, preset);   
  }

  {
    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_COLOR;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0;
    dr_preset.spp = 16;

    dr_render.SetReference({image_mesh}, {view}, {proj});

    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_SBS_dr = dr_render.getLastImage(0);

    dr_preset.dr_render_mode = DR_RENDER_MODE_LINEAR_DEPTH;
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_SBS_dr_depth = dr_render.getLastImage(0);

    dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_PRIMITIVE;
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_SBS_dr_prim = dr_render.getLastDebugImage(0);
  }

  {
    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0;
    dr_preset.spp = 16;

    dr_render.SetReference({image_mesh}, {view}, {proj});
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_dr_2 = dr_render.getLastImage(0);
  }

  for (int i = 0; i < W * H; i++)
  {
    float4 diff = image_SBS.data()[i] - image_SBS_dr.data()[i];
    image_SBS_diff.data()[i] = float4(100*diff.x, 100*diff.y, 100*diff.z, 1.0f);
  }

  LiteImage::SaveImage<float4>("saves/test_dr_1_mesh.bmp", image_mesh); 

  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs.bmp", image_SBS);
  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_depth.bmp", image_SBS_depth);
  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_prim.bmp", image_SBS_prim);

  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_dr.bmp", image_SBS_dr);
  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_dr_depth.bmp", image_SBS_dr_depth);
  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_dr_prim.bmp", image_SBS_dr_prim);
  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_dr_2.bmp", image_SBS_dr_2);

  LiteImage::SaveImage<float4>("saves/test_dr_1_sbs_dr_diff.bmp", image_SBS_diff);

  float psnr_0 = image_metrics::PSNR(image_SBS_depth, image_SBS_dr_depth);
  float psnr_1 = image_metrics::PSNR(image_SBS_prim, image_SBS_dr_prim);
  float psnr_2 = image_metrics::PSNR(image_SBS, image_SBS_dr);
  float psnr_3 = image_metrics::PSNR(image_SBS, image_SBS_dr_2);

  printf("TEST 1. Differentiable render forward pass\n");

  printf(" 1.1. %-64s", "Diff render (depth mode) of SBS match regular render");
  if (psnr_0 >= 50)
    printf("passed    (%.2f)\n", psnr_0);
  else
    printf("FAILED, psnr = %f\n", psnr_0);

  printf(" 1.2. %-64s", "Diff render (primitive id mode) of SBS match regular render");
  if (psnr_1 >= 35)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);
  printf(" 1.3. %-64s", "Diff render (color mode) of SBS match regular render");
  if (psnr_2 >= 50)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);

  printf(" 1.4. %-64s", "Diff render (geometry mode) of SBS match regular render");
  if (psnr_3 >= 50)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);
}

void diff_render_test_2_check_color_derivatives()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_medium_scene();

  unsigned W = 128, H = 128;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 16;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(8, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_2_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 4;

    unsigned color_param_count = 3*8*indexed_SBS.nodes.size();
    unsigned color_param_offset = indexed_SBS.values_f.size() - color_param_count;

    std::vector<float> grad_dr(color_param_count, 0);
    std::vector<float> grad_ref(color_param_count, 0);

    {
    MultiRendererDR dr_render;
    dr_render.SetReference({images_ref[0]}, {view[0]}, {proj[0]});
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    grad_dr = std::vector<float>(dr_render.getLastdLoss_dS() + color_param_offset, 
                                 dr_render.getLastdLoss_dS() + color_param_offset + color_param_count);
    }
    {
    MultiRendererDR dr_render;
    dr_preset.dr_diff_mode = DR_DIFF_MODE_FINITE_DIFF;
    dr_render.SetReference({images_ref[0]}, {view[0]}, {proj[0]});
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    grad_ref = std::vector<float>(dr_render.getLastdLoss_dS() + color_param_offset, 
                                  dr_render.getLastdLoss_dS() + color_param_offset + color_param_count);
    }

    long double average_1 = 0;
    long double abs_average_1 = 0;
    long double average_2 = 0;
    long double abs_average_2 = 0;
    long double diff = 0;
    long double abs_diff = 0;
    for (int i = 0; i < color_param_count; i++)
    {
      average_1 += grad_ref[i];
      abs_average_1 += abs(grad_ref[i]);
      average_2 += grad_dr[i];
      abs_average_2 += abs(grad_dr[i]);
      diff += abs(grad_ref[i] - grad_dr[i]);
      abs_diff += abs(grad_ref[i] - grad_dr[i]);
    }
    average_1 /= color_param_count;
    abs_average_1 /= color_param_count;
    average_2 /= color_param_count;
    abs_average_2 /= color_param_count;
    diff /= color_param_count;
    abs_diff /= color_param_count;

    /*
    printf("[");
    for (int i = 0; i < color_param_count; i++)
      printf("%f ", grad_ref[i]);
    printf("]\n");

    printf("[");
    for (int i = 0; i < color_param_count; i++)
      printf("%f ", grad_dr[i]);
    printf("]\n");
    */

    //image_SBS_single = dr_render.getLastImage(0);
    float average_error = abs(abs_average_2 - abs_average_1) / (abs_average_1 + abs_average_2);
    float average_bias = abs(average_2 - average_1) / (abs_average_1 + abs_average_2);
    float max_error = 0;
    float average_error_2 = 0;
    float average_bias_2 = 0;
    for (int i = 0; i < color_param_count; i++)
    {
      float error = abs(grad_dr[i] - grad_ref[i]) / (abs_average_1 + abs_average_2);
      float bias = (grad_dr[i] - grad_ref[i]) / (abs_average_1 + abs_average_2);
      max_error = max(max_error, error);
      average_error_2 += error;
      average_bias_2 += bias;
    }
    average_error_2 /= color_param_count;
    average_bias_2 /= color_param_count;

    printf("TEST 2. Check color derivatives\n");
  
    printf(" 2.1. %-64s", "Relative difference in average PD value");
    if (average_error <= 0.05f)
      printf("passed    (%f)\n", average_error);
    else
      printf("FAILED, average_error = %f\n", average_error);
    
    printf(" 2.2. %-64s", "Relative bias in average PD");
    if (average_bias <= 0.05f)
      printf("passed    (%f)\n", average_bias);
    else
      printf("FAILED, average_bias = %f\n", average_bias);
    
    printf(" 2.3. %-64s", "Max relative difference in PD value");
    if (max_error <= 0.2f)
      printf("passed    (%f)\n", max_error);
    else
      printf("FAILED, max_error = %f\n", max_error);
    
    printf(" 2.4. %-64s", "Average relative difference in PD value");
    if (average_error_2 <= 0.05f)
      printf("passed    (%f)\n", average_error_2);
    else
      printf("FAILED, max_bias = %f\n", average_error_2);
    
    printf(" 2.5. %-64s", "Average relative bias in PD");
    if (average_bias_2 <= 0.05f)
      printf("passed    (%f)\n", average_bias_2);
    else
      printf("FAILED, average_bias = %f\n", average_bias_2);
  }
}


void test_position_derivatives(const SdfSBS &SBS, unsigned render_node, unsigned diff_render_mode,
                               unsigned border_sampling, bool close_view, bool empty_reference)
{
  static unsigned off = 1;
  srand(time(nullptr));

  unsigned W = 64, H = 64;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 256;

  float4x4 base_proj = LiteMath::perspectiveMatrix(close_view ? 15 : 60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view; 
  if (close_view)
    view = std::vector<float4x4>{LiteMath::lookAt(float3(0.55, 0.55, 2.0), float3(0.55, 0.55, 0), float3(0, 1, 0))};
  else
    view = std::vector<float4x4>{LiteMath::lookAt(float3(0.3, 0, 2.5), float3(0, 0, 0), float3(0, 1, 0))};

  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS);
    if (!empty_reference)
      pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_4_"+std::to_string(off)+"_ref.bmp").c_str(), images_ref[i]); 
  }

  //change SBS slightly
  auto indexed_SBS = SBS;
  if (!empty_reference)
  {
    randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.1f);
  }

  MultiRendererDRPreset dr_preset = getDefaultPresetDR();

  dr_preset.dr_render_mode = diff_render_mode;
  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.dr_input_type = diff_render_mode == DR_RENDER_MODE_LINEAR_DEPTH ? DR_INPUT_TYPE_LINEAR_DEPTH : DR_INPUT_TYPE_COLOR;
  dr_preset.opt_iterations = 1;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 64;
  dr_preset.border_spp = 1024;
  dr_preset.debug_pd_brightness = 1.0f;
  dr_preset.border_relax_eps = 1e-5f;
  dr_preset.finite_diff_delta = 0.005f;
  dr_preset.finite_diff_brightness = 0.25f;
  dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_BORDER_DETECTION;
  dr_preset.debug_progress_images = DEBUG_PROGRESS_RAW;
  dr_preset.debug_forced_border = false;
  dr_preset.debug_border_save_normals = true;
  bool debug_pd_images = true;

  std::ofstream hist_data_txt("saves_border_rays/hist_data.txt", std::cout.out | std::cout.app);
  bool fprint_hist_data = dr_preset.debug_border_save_normals;

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  //std::vector<float> deltas = {0.04f, 0.02f, 0.01f, 0.005f, 0.0025f, 0.001f, 0.0005f, 0.00025f, 0.0001f};
  //for (float delta : deltas)
  {
    std::array<std::vector<double>, 3> grad_mean;
    std::array<std::vector<double>, 3> grad_conf;

    for (int T = 0; T < 2; T++)
    {
    unsigned samples = 25;
    std::vector<std::vector<float>> grads(samples);


    if (!T && fprint_hist_data)
      hist_data_txt << "Hist " << dr_preset.border_relax_eps << ' ' << (off-1) << " 0\n";

    for (unsigned i = 0; i < samples; i++)
    {
      srand(time(nullptr) + i);
      MultiRendererDR dr_render;
      dr_preset.dr_diff_mode = T == 1 ? DR_DIFF_MODE_FINITE_DIFF : DR_DIFF_MODE_DEFAULT;
      dr_preset.dr_border_sampling = border_sampling;
      //dr_preset.debug_pd_images = i == 0;
      //dr_preset.debug_border_samples_mega_image = T == 0 && i == 0;
      //dr_preset.debug_border_samples = T == 0 && i == 0;

      dr_render.SetReference(images_ref, view, proj);
      dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
      BVHDR* bvh_as = dynamic_cast<BVHDR*>(dr_render.GetAccelStruct().get());

      if (!T && fprint_hist_data)
      {
        assert((bvh_as->m_GraphicsPrimPoints.size() % 3) == 0);
        for (uint32_t b = 0u; b < bvh_as->m_GraphicsPrimPoints.size(); b += 3)
          hist_data_txt << bvh_as->m_GraphicsPrimPoints[b + 1].w << ", ";
        hist_data_txt << "\n";
      }
      grads[i] = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                    dr_render.getLastdLoss_dS() + param_offset + param_count);

      //printf("border_rays hit chance = %f\n", dr_render.border_rays_hit/(dr_preset.border_spp*W*H + 1e-6f));
      image_res = dr_render.getLastImage(0);
      LiteImage::SaveImage<float4>(("saves/test_dr_4_"+std::to_string(off)+"_res.bmp").c_str(), image_res); 
    }
    if (!T && fprint_hist_data)
    {
      hist_data_txt << "\n";
    }

    grad_mean[T] = stat::mean<float>(grads);
    grad_conf[T] = stat::confidence<float>(grads, 0.95f);
    }

    //increase confidence intervals for finite differences if they are too small
    for (auto &stddev : grad_conf[0])
      stddev = std::max(stddev, 1e-6);
    for (auto &stddev : grad_conf[1])
      stddev = std::max(stddev, 1e-6);
    for (int i=0;i<param_count;i++)
    {
      if (abs(grad_mean[0][i]) < 1e-3f)
      {
        grad_mean[0][i] = 0;
        grad_mean[1][i] = 0;
      }
    }

    {
      long double std_dev_mult = 0;
      long double average_error = 0;
      long double max_error = 0;
      long double relative_error = 0;
      long double relative_bias  = 0;
      long double total_sum      = 1e-6f;

      for (int i = 0; i < param_count; i++)
      {
        std_dev_mult += grad_conf[0][i] / grad_conf[1][i];
        double error = std::abs(grad_mean[0][i] - grad_mean[1][i]) / std::max(grad_conf[1][i], grad_conf[0][i]);
        average_error += error;
        max_error = std::max<double>(max_error, error);
        relative_error += std::abs(grad_mean[0][i] - grad_mean[1][i]);
        relative_bias  +=         (grad_mean[0][i] - grad_mean[1][i]);
        total_sum += std::abs(grad_mean[0][i]) + std::abs(grad_mean[1][i]);
      }
      std_dev_mult /= param_count;
      average_error /= param_count;
      relative_error /= 0.5f*total_sum;
      relative_bias  /= 0.5f*total_sum;
    
      bool passed = true;

      printf(" 4.%d.1 %-63s", off, "Precision");
      if (relative_error <= 0.15f || average_error <= 1.0f)
        printf("passed    (rel. err. = %f, average %f stddev)\n", (float)relative_error, (float)average_error);
      else
      {
        passed = false;
        printf("FAILED, rel. err. = %f, average %f stddev\n", (float)relative_error, (float)average_error);
      }

      printf(" 4.%d.2 %-63s", off, "Bias");
      if (std::abs(relative_bias) <= 0.15)
        printf("passed    (%f)\n", (float)relative_bias);
      else
      {
        passed = false;
        printf("FAILED, overall bias = %f\n", (float)relative_bias);
      }
      
      if (!passed)
      {
        for (int T=0;T<2;T++)
        {
          printf("MEAN = [");
          for (unsigned i = 0; i < param_count; i++)
          {
            printf("%9.2f ", grad_mean[T][i]);
          }
          printf("]\n");
        }
        printf("BIAS = [");
        for (unsigned i = 0; i < param_count; i++)
        {
          float bias = std::abs(grad_mean[1][i]) > 1e-6f ? std::abs(grad_mean[0][i]) / std::abs(grad_mean[1][i]) : 1.0f;
          printf("%9.2f ", bias);
        }
        printf("]\n");
      }
    }
  }
  off++;
}

void diff_render_test_3_optimize_color()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 512, H = 512;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 16;
  
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 7);

  SdfSBSHeader header;
  header.brick_size = 2;
  header.brick_pad = 0;
  header.bytes_per_value = 1;
  SdfSBS indexed_SBS;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");

  float4x4 view, proj;
  LiteImage::Image2D<float4> image_mesh(W, H);
  LiteImage::Image2D<float4> image_SBS(W, H);
  LiteImage::Image2D<float4> image_SBS_dr(W, H);

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);

    pRender->SetScene(mesh);
    render(image_mesh, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    view = pRender->getWorldView();
    proj = pRender->getProj();
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);  

    indexed_SBS = sdf_converter::create_sdf_SBS_indexed(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures());
    pRender->SetScene(indexed_SBS);
    pRender->RenderFloat(image_SBS.data(), image_SBS.width(), image_SBS.height(), view, proj, preset);   
  }

  {
    //put random colors to SBS
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 200;
    dr_preset.opt_lr = 0.25;
    dr_preset.spp = 1;

    dr_render.SetReference({image_mesh}, {view}, {proj});
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_dr = dr_render.getLastImage(0);
  }

  LiteImage::SaveImage<float4>("saves/test_dr_3_mesh.bmp", image_mesh); 
  LiteImage::SaveImage<float4>("saves/test_dr_3_sbs.bmp", image_SBS);
  LiteImage::SaveImage<float4>("saves/test_dr_3_sbs_dr.bmp", image_SBS_dr);

  //float psnr_1 = image_metrics::PSNR(image_mesh, image_SBS);
  float psnr_2 = image_metrics::PSNR(image_mesh, image_SBS_dr);

  printf("TEST 3. Differentiable render optimize color\n");

  printf(" 3.1. %-64s", "Diff render for color reconstruction");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
}

void diff_render_test_4_check_position_derivatives()
{
  printf("TEST 4. Check position derivatives\n");

  printf("4.1 Mask, random border sampling, empty reference\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, DR_BORDER_SAMPLING_RANDOM,
                            false, true);
  printf("4.2 Mask, SVM border sampling, empty reference\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, DR_BORDER_SAMPLING_SVM,
                            false, true);
  printf("4.3 Mask, random border sampling, larger model\n");
  test_position_derivatives(circle_smallest_scene_colored_2(), 
                            MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, DR_BORDER_SAMPLING_RANDOM,
                            false, true);
  printf("4.4 Mask, random border sampling\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, DR_BORDER_SAMPLING_RANDOM,
                            false, false);
  printf("4.5 Diffuse, random border sampling\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_DIFFUSE, DR_RENDER_MODE_DIFFUSE, DR_BORDER_SAMPLING_RANDOM,
                            false, true);
  printf("4.6 Lambert, random border sampling, close view\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, DR_BORDER_SAMPLING_RANDOM,
                            true, true);
  printf("4.7 Lambert, random border sampling, empty reference\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, DR_BORDER_SAMPLING_RANDOM,
                            false, true);
  printf("4.8 Lambert, random border sampling\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, DR_BORDER_SAMPLING_RANDOM,
                            false, false);
  printf("4.9 Lambert, SVM border sampling\n");
  test_position_derivatives(circle_smallest_scene_colored(), 
                            MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, DR_BORDER_SAMPLING_SVM,
                            false, false); 
  printf("4.10 Depth, random border sampling, close view\n");
  test_position_derivatives(circle_smallest_scene_colored(),
                            MULTI_RENDER_MODE_LINEAR_DEPTH, DR_RENDER_MODE_LINEAR_DEPTH, DR_BORDER_SAMPLING_RANDOM,
                            true, true);
  printf("4.11 Depth, random border sampling\n");
  test_position_derivatives(circle_smallest_scene_colored(),
                            MULTI_RENDER_MODE_LINEAR_DEPTH, DR_RENDER_MODE_LINEAR_DEPTH, DR_BORDER_SAMPLING_RANDOM,
                            false, true);
}

void optimization_stand_common(uint32_t num, uint32_t sub_num, const SdfSBS &SBS_ref, const SdfSBS &SBS_initial, 
                               MultiRendererDRPreset dr_preset, std::string name, uint32_t view_count = 8)
{
  //create renderers for SDF scene and mesh scene
  srand(time(nullptr));

  unsigned W = dr_preset.render_height, H = dr_preset.render_width;

  MultiRenderPreset preset = getDefaultPreset();
  if (dr_preset.dr_render_mode == DR_RENDER_MODE_MASK)
    preset.render_mode = MULTI_RENDER_MODE_MASK;
  else if (dr_preset.dr_render_mode == DR_RENDER_MODE_DIFFUSE)
    preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  else if (dr_preset.dr_render_mode == DR_RENDER_MODE_LAMBERT)
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  else if (dr_preset.dr_render_mode == DR_RENDER_MODE_LINEAR_DEPTH)
    preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
  preset.spp = 64;
  preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_turntable(view_count, float3(0, 0, 0), 4.0f, 1.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/osc_"+std::to_string(sub_num)+"_ref_"+std::to_string(i)+".png").c_str(), images_ref[i]); 
  }
  LiteImage::SaveImage<float4>(("saves/test_dr_"+std::to_string(num)+"_"+std::to_string(sub_num)+"_ref.png").c_str(), images_ref[0]); 

  auto indexed_SBS = SBS_initial;

  MultiRendererDR dr_render;
  dr_render.SetLights({create_direct_light(float3(0.7, 0.7, 0.7), float3(1,1,1)), 
                       create_ambient_light(float3(0.25, 0.25, 0.25)),
                       create_direct_light(float3(0.7, 0.7, 0.7), float3(-1,1,-1)),});
  dr_render.SetReference(images_ref, view, proj);

  auto t1 = std::chrono::high_resolution_clock::now();
  dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
  auto t2 = std::chrono::high_resolution_clock::now();
  image_res = dr_render.getLastImage(0);
  LiteImage::SaveImage<float4>(("saves/test_dr_"+std::to_string(num)+"_"+std::to_string(sub_num)+"_res.bmp").c_str(), image_res);

  float psnr_sum = 0.0f;
  for (int i = 0; i < view.size(); i++)
  {
    psnr_sum += image_metrics::PSNR(images_ref[i], dr_render.getLastImage(i));
  }
  psnr_sum /= view.size();

  float time_ms = std::chrono::duration<float, std::milli>(t2 - t1).count();
  float time_per_iter = time_ms/dr_preset.opt_iterations;

  printf("%2u.%u. %-64s", num, sub_num, name.c_str());
  if (psnr_sum >= 25)
    printf("passed    (%.2f)    took %.1f s, %.1f ms/iter\n", psnr_sum, time_ms/1000.0f, time_per_iter);
  else
    printf("FAILED, psnr = %.2f took %.1f s, %.1f ms/iter\n", psnr_sum, time_ms/1000.0f, time_per_iter);
}


MultiRendererDRPreset optimization_stand_common_preset()
{
  MultiRendererDRPreset dr_preset = getDefaultPresetDR();

  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_RANDOM;
  dr_preset.opt_iterations = 500;
  dr_preset.opt_lr = 0.01f;
  dr_preset.spp = 16;
  dr_preset.border_spp = 1024;
  dr_preset.image_batch_size = 2;
  dr_preset.render_height = 128;
  dr_preset.render_width = 128;

  dr_preset.debug_print = true;
  dr_preset.debug_print_interval = 1;
  dr_preset.debug_progress_interval = 25;
  dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL;

  return dr_preset;
}


void diff_render_test_5_optimize_sdf_finite_derivatives()
{
  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();

  SdfSBS smallest_scene = create_grid_sbs(1, 2, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);    
  SdfSBS smallest_initial = create_grid_sbs(1, 2, 
                          [&](float3 p){return circle_sdf(float3(-0.2,0.15,-0.1), 0.6f, p);}, 
                          gradient_color);  

  printf("TEST 5. Optimization with finite derivatives\n");

  dr_preset.dr_diff_mode = DR_DIFF_MODE_FINITE_DIFF;
  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.opt_iterations = 500;
  optimization_stand_common(5, 1, smallest_scene, smallest_initial, dr_preset, "Smallest scene. Mask. Finite differences.");
}

void diff_render_test_6_optimization_stand()
{
  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();

  SdfSBS smallest_scene = create_grid_sbs(1, 2, 
                          [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                          gradient_color);    
  SdfSBS smallest_initial = create_grid_sbs(1, 2, 
                          [&](float3 p){return circle_sdf(float3(-0.2,0.15,-0.1), 0.6f, p);}, 
                          gradient_color);  
  
  SdfSBS small_scene = create_grid_sbs(1, 4, 
                       [&](float3 p){return circle_sdf(float3(0,0.2,0.2), 0.6f, p);}, 
                       gradient_color); 
  SdfSBS small_initial = create_grid_sbs(1, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         gradient_color);

  SdfSBS medium_scene = create_grid_sbs(4, 4, 
                       [&](float3 p){return circle_sdf(float3(0,0.2,0.2), 0.6f, p);}, 
                       single_color); 
  SdfSBS medium_scene_colored = create_grid_sbs(4, 4, 
                       [&](float3 p){return circle_sdf(float3(0,0.2,0.2), 0.6f, p);}, 
                       gradient_color); 
  SdfSBS medium_initial = create_grid_sbs(4, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         single_color);

  SdfSBS ds_scene = create_grid_sbs(1, 16, 
                       [&](float3 p){return std::max(circle_sdf(float3(0,0,0), 0.8f, p), -circle_sdf(float3(0.6,0,0), 0.4f, p));}, 
                       single_color); 
  SdfSBS ds_initial = create_grid_sbs(1, 16, 
                         [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                         [&](float3 p){ return float3(0,0.7,0); });

  SdfSBS circle_scene = create_grid_sbs(4, 4, 
                                        [&](float3 p){return circle_sdf(float3(0,0,0), 0.75f, p);}, 
                                        single_color); 

  SdfSBS ts_scene = two_circles_scene();

  printf("TEST 6. Optimization on a set of simple scenes\n");

  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.opt_iterations = 500;
  optimization_stand_common(6, 1, smallest_scene, smallest_initial, dr_preset, "Smallest scene. Mask.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.opt_iterations = 500;
  optimization_stand_common(6, 2, medium_scene, medium_initial, dr_preset, "Sphere. Mask.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.opt_iterations = 1000;
  optimization_stand_common(6, 3, ts_scene, medium_initial, dr_preset, "Two spheres. Mask.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 500;
  optimization_stand_common(6, 4, medium_scene_colored, medium_initial, dr_preset, "Sphere. Diffuse. Monochrome.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 500;
  optimization_stand_common(6, 5, ts_scene, medium_initial, dr_preset, "Two spheres. Diffuse. Colored.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 500;
  optimization_stand_common(6, 6, medium_scene_colored, medium_initial, dr_preset, "Sphere. Lambert. Monochrome.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 1000;
  optimization_stand_common(6, 8, ds_scene, medium_initial, dr_preset, "Death Star. Lambert. Colored.");

  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 1000;
  optimization_stand_common(6, 9, ts_scene, medium_initial, dr_preset, "Two spheres. Lambert. Colored.");
}

void diff_render_test_7_SVM_sampling()
{
  SdfSBS medium_initial = create_grid_sbs(4, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         single_color);
  SdfSBS ts_scene = two_circles_scene();

  printf("TEST 8. Optimization with SVM border sampling\n");

  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();
  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 1000;
  dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
  optimization_stand_common(7, 1, ts_scene, medium_initial, dr_preset, "Two spheres. Lambert. Colored.");
}

void diff_render_test_8_redistancing()
{
  SdfSBS medium_initial = create_grid_sbs(4, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         single_color);
  SdfSBS ts_scene = two_circles_scene();

  printf("TEST 8. Optimization with redistancing\n");

  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();
  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 1000;
  dr_preset.redistancing_enable = true;
  dr_preset.redistancing_interval = 1;
  optimization_stand_common(8, 1, ts_scene, medium_initial, dr_preset, "Two spheres. Lambert. Colored.");
}

void diff_render_test_9_regularization()
{
  SdfSBS medium_initial = create_grid_sbs(4, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         single_color);
  SdfSBS ts_scene = two_circles_scene();

  printf("TEST 9. Optimization with regularization\n");

  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();
  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 1000;
  dr_preset.reg_function = DR_REG_FUNCTION_LK_DENOISING;
  dr_preset.reg_power = 2.0f;
  dr_preset.reg_lambda = 0.1f;
  optimization_stand_common(9, 1, ts_scene, medium_initial, dr_preset, "Two spheres. Lambert. Colored.");
}

void diff_render_test_10_ray_casting_mask()
{
  SdfSBS medium_initial = create_grid_sbs(4, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         single_color);
  SdfSBS ts_scene = two_circles_scene();

  printf("TEST 10. Optimization with ray casing mask\n");

  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();
  dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
  dr_preset.opt_iterations = 1000;
  dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;
  optimization_stand_common(10, 1, ts_scene, medium_initial, dr_preset, "Two spheres. Lambert. Colored.");
}

void diff_render_test_11_expanding_grid()
{
  SdfSBS ds_scene = create_grid_sbs(16, 4, 
                                    [&](float3 p){return std::max(circle_sdf(float3(0,0,0), 0.8f, p), -circle_sdf(float3(0.6,0,0), 0.4f, p));},
                                    [&](float3 p){ return length(p) < 0.65f ? float3(1,0,0) : float3(0,1,0);});

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
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(ds_scene);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_11_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
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
    dr_preset.debug_print_interval = 10;
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
    LiteImage::SaveImage<float4>("saves/test_dr_11_res.bmp", image_res);
  }

  printf("TEST 11. Optimize with expanding grid\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("11.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_12_depth()
{
  SdfSBS medium_initial = create_grid_sbs(4, 4, 
                         [&](float3 p){return circle_sdf(float3(-0.2,0,-0.2), 0.6f, p);}, 
                         single_color);
  SdfSBS ts_scene = two_circles_scene();

  printf("TEST 12. Optimization using depth reference images\n");

  MultiRendererDRPreset dr_preset = optimization_stand_common_preset();
  dr_preset.dr_render_mode = DR_RENDER_MODE_LINEAR_DEPTH;
  dr_preset.dr_input_type = DR_INPUT_TYPE_LINEAR_DEPTH;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.opt_iterations = 1000;
  dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;
  optimization_stand_common(12, 1, ts_scene, medium_initial, dr_preset, "Two spheres. Depth.");
}

// #ifdef USE_ENZYME
// int enzyme_const, enzyme_dup, enzyme_out; // must be global
// double __enzyme_autodiff(void*, ...);

// double litert_test_28_enzyme_ad_sqr_1(double  x) { return  x *  x; }
// double litert_test_28_enzyme_ad_sqr_2(double* x) { return *x * *x; }
// double litert_test_28_enzyme_ad_mul(double k, double x) { return k * x; }
// #endif

void diff_render_test_1_enzyme_ad()
{
//   printf("TEST 1. Enzyme AD\n");
// #ifdef USE_ENZYME
//   double x = 5.;
//   double df_dx_res = 2*x;

//   // Output argument
//   printf(" 1.1. %-64s", "Basic square derivative");
//   double df_dx_out = __enzyme_autodiff((void*)litert_test_28_enzyme_ad_sqr_1, x);
//   if ((df_dx_out - df_dx_res) < 1e-7 && (df_dx_out - df_dx_res) > -1e-7) // actually even (df_dx_out == df_dx_res) works
//     printf("passed    (d(x^2) = %.1f, x = %.1f)\n", df_dx_out, x);
//   else
//     printf("FAILED,   (df_dx_out = %f, must be %.1f)\n", df_dx_out, df_dx_res);

//   // Duplicated argument
//   printf(" 1.2. %-64s", "Square derivative, dx is stored in an argument");
//   double df_dx_arg = 0.;
//   __enzyme_autodiff((void*)litert_test_28_enzyme_ad_sqr_2, &x, &df_dx_arg);
//   if ((df_dx_arg - df_dx_res) < 1e-7 && (df_dx_arg - df_dx_res) > -1e-7) // actually even (df_dx_arg == df_dx_res) works
//     printf("passed    (d(x^2) = %.1f, x = %.1f)\n", df_dx_arg, x);
//   else
//     printf("FAILED,   (df_dx_arg = %f, must be %.1f)\n", df_dx_arg, df_dx_res);

//   // Inactive (const) argument (explicit)
//   printf(" 1.3. %-64s", "Derivative d(k*x)/dx, k - parameter");
//   double k = 7;
//   df_dx_out = 0.; // only to verify the result
//   df_dx_out = __enzyme_autodiff((void*)litert_test_28_enzyme_ad_mul, enzyme_const, k, x); // enzyme_const not needed if k is int
//   if ((df_dx_out - k) < 1e-7 && (df_dx_out - k) > -1e-7) // actually even (df_dx_out == k) works
//     printf("passed    (d(k*x)/dx = %.1f, k = %.1f)\n", df_dx_out, k);
//   else
//     printf("FAILED,   (df_dx_out = %f, must be %.1f)\n", df_dx_out, k);

// #else
//   printf("  Enzyme AD is not used.\n");

// #endif
}

void diff_render_test_4_render_simple_scenes()
{
  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;

  float4x4 view, proj;
  LiteImage::Image2D<float4> image_med(W, H);
  LiteImage::Image2D<float4> image_small(W, H);
  LiteImage::Image2D<float4> image_one_brick(W, H);
  LiteImage::Image2D<float4> image_small_dr(W, H);
  LiteImage::Image2D<float4> image_one_brick_dr(W, H);

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    auto scene = circle_medium_scene();
    pRender->SetScene(scene);
    render(image_med, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    view = pRender->getWorldView();
    proj = pRender->getProj();
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    auto scene = circle_small_scene();
    pRender->SetScene(scene);
    render(image_small, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    auto scene = circle_one_brick_scene();
    pRender->SetScene(scene);
    render(image_one_brick, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }

  {
    auto scene = circle_one_brick_scene();
    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 1;

    dr_render.SetReference({image_med}, {view}, {proj});
    dr_render.OptimizeFixedStructure(dr_preset, scene);
    
    image_one_brick_dr = dr_render.getLastImage(0);
  }

  {
    auto scene = circle_small_scene();
    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 1;

    dr_render.SetReference({image_med}, {view}, {proj});
    dr_render.OptimizeFixedStructure(dr_preset, scene);
    
    image_small_dr = dr_render.getLastImage(0);
  }

  LiteImage::SaveImage<float4>("saves/test_dr_4_medium.bmp", image_med); 
  LiteImage::SaveImage<float4>("saves/test_dr_4_small.bmp", image_small);
  LiteImage::SaveImage<float4>("saves/test_dr_4_one_brick.bmp", image_one_brick);
  LiteImage::SaveImage<float4>("saves/test_dr_4_one_brick_dr.bmp", image_one_brick_dr);
  LiteImage::SaveImage<float4>("saves/test_dr_4_small_dr.bmp", image_small_dr);

  float psnr_1 = image_metrics::PSNR(image_med, image_small);
  float psnr_2 = image_metrics::PSNR(image_med, image_one_brick);
  float psnr_3 = image_metrics::PSNR(image_small, image_one_brick);
  float psnr_4 = image_metrics::PSNR(image_one_brick, image_one_brick_dr);
  float psnr_5 = image_metrics::PSNR(image_small, image_small_dr);

  printf("TEST 4. Render simple scenes\n");

  printf(" 4.1. %-64s", "Small scene is ok");
  if (psnr_1 >= 30)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);
  
  printf(" 4.2. %-64s", "One brick scene is ok");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  
  printf(" 4.3. %-64s", "Small and one brick scene are equal");
  if (psnr_3 >= 40)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);
  
  printf(" 4.4. %-64s", "DR and common render for one brick scene are equal");
  if (psnr_4 >= 40)
    printf("passed    (%.2f)\n", psnr_4);
  else
    printf("FAILED, psnr = %f\n", psnr_4);
  
  printf(" 4.5. %-64s", "DR and common render for small scene are equal");
  if (psnr_5 >= 40)
    printf("passed    (%.2f)\n", psnr_5);
  else
    printf("FAILED, psnr = %f\n", psnr_5);
}

void diff_render_test_5_optimize_color_simpliest()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_medium_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(8, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_SBS_single(W, H);
  LiteImage::Image2D<float4> image_SBS_multi(W, H);
  LiteImage::Image2D<float4> image_SBS_multi_2(W, H);
  LiteImage::Image2D<float4> image_SBS_multi_mae(W, H);

  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_5_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 200;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 16;

    dr_render.SetReference({images_ref[0]}, {view[0]}, {proj[0]});
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_single = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_single.bmp", image_SBS_single);
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 400;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 16;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_multi = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_multi.bmp", image_SBS_multi);
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_small_scene();
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 400;
    dr_preset.opt_lr = 0.01f;
    dr_preset.spp = 16;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_multi_2 = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_multi_2.bmp", image_SBS_multi_2);

  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.opt_iterations = 400;
    dr_preset.opt_lr = 0.01f;
    dr_preset.spp = 16;
    dr_preset.dr_loss_function = DR_LOSS_FUNCTION_MAE;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_multi_mae = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_multi_mae.bmp", image_SBS_multi_mae);

  float psnr_1 = image_metrics::PSNR(images_ref[0], image_SBS_single);
  float psnr_2 = image_metrics::PSNR(images_ref[0], image_SBS_multi);
  float psnr_3 = image_metrics::PSNR(images_ref[0], image_SBS_multi_2);
  float psnr_4 = image_metrics::PSNR(images_ref[0], image_SBS_multi_mae);

  printf("TEST 5. Optimize color from multiple views\n");
  
  printf(" 5.1. %-64s", "1 view, 1-brick scene");
  if (psnr_1 >= 30)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);

  printf(" 5.2. %-64s", "8 views, 1-brick scene");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  
  printf(" 5.3. %-64s", "8 views, 8-brick scene");
  if (psnr_3 >= 30)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);

  printf(" 5.4. %-64s", "8 views, MAE loss function");
  if (psnr_4 >= 30)
    printf("passed    (%.2f)\n", psnr_4);
  else
    printf("FAILED, psnr = %f\n", psnr_4);
}

void diff_render_test_7_optimize_with_finite_diff()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_medium_scene();

  unsigned W = 128, H = 128;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 16;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(8, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_SBS_def(W, H);
  LiteImage::Image2D<float4> image_SBS_finite_diff(W, H);

  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_7_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.opt_iterations = 100;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_def = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_7_def.bmp", image_SBS_def);
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_FINITE_DIFF;
    dr_preset.opt_iterations = 100;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_finite_diff = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_7_finite_diff.bmp", image_SBS_finite_diff);


  float psnr_1 = image_metrics::PSNR(images_ref[0], image_SBS_finite_diff);
  float psnr_2 = image_metrics::PSNR(images_ref[0], image_SBS_def);

  printf("TEST 7. Optimize color with default and finite differences PD\n");
  
  printf(" 7.1. %-64s", "Diff render gives similar or better optimization result");
  if (psnr_2 - psnr_1 >= -3)
    printf("passed    (%.2f)\n", psnr_2 - psnr_1);
  else
    printf("FAILED, psnr diff = %f\n", psnr_2 - psnr_1);
}

void diff_render_test_8_optimize_with_lambert()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 16;
  
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 7);

  SdfSBSHeader header;
  header.brick_size = 2;
  header.brick_pad = 0;
  header.bytes_per_value = 1;
  SdfSBS indexed_SBS;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");
  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);
  std::vector<float4x4> view = get_cameras_uniform_sphere(4, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_SBS(W, H);
  LiteImage::Image2D<float4> image_SBS_dr(W, H);

  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
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
    LiteImage::SaveImage<float4>(("saves/test_dr_8_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);  

    indexed_SBS = sdf_converter::create_sdf_SBS_indexed(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures());
    pRender->SetScene(indexed_SBS);
    pRender->RenderFloat(image_SBS.data(), image_SBS.width(), image_SBS.height(), view[0], proj[0], preset);   
  }

  {
    //put random colors to SBS
    randomize_color(indexed_SBS);

    MultiRendererDR dr_render;
    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset.opt_iterations = 300;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 1;
    dr_preset.spp = 2;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    
    image_SBS_dr = dr_render.getLastImage(0);
  }

  LiteImage::SaveImage<float4>("saves/test_dr_8_mesh.bmp", images_ref[0]); 
  LiteImage::SaveImage<float4>("saves/test_dr_8_sbs.bmp", image_SBS);
  LiteImage::SaveImage<float4>("saves/test_dr_8_sbs_dr.bmp", image_SBS_dr);

  //float psnr_1 = image_metrics::PSNR(image_mesh, image_SBS);
  float psnr_2 = image_metrics::PSNR(images_ref[0], image_SBS_dr);

  printf("TEST 8. Differentiable render optimize lambert\n");

  printf(" 8.1. %-64s", "Diff render for color reconstruction with lambert shading");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
}

void diff_render_test_11_optimize_smallest_scene()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_smallest_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
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
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_11_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_smallest_scene();
    // randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.25f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.opt_iterations = 100;
    dr_preset.opt_lr = 0.01f;
    dr_preset.spp = 4;
    dr_preset.image_batch_size = 1;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_11_res.bmp", image_res);
  }

  printf("TEST 11. Differentiable render optimize smallest SDF scene\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("11.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_12_optimize_sphere_mask()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_small_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_MASK;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_12_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_small_scene();
    // randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.1f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.opt_iterations = 300;
    dr_preset.opt_lr = 0.0025f;
    dr_preset.spp = 4;
    dr_preset.border_spp = 1000;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
    dr_preset.image_batch_size = 4;
    dr_preset.debug_print = true;

    // dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_12_res.bmp", image_res);
  }

  printf("TEST 12. Optimize smallest SDF scene with mask\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("12.1. %-64s", "SDF is reconstructed");
  if (psnr >= 25)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_13_optimize_sphere_diffuse()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_small_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_13_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_small_scene();
    // randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.1f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.opt_iterations = 200;
    dr_preset.opt_lr = 0.005f;
    dr_preset.spp = 4;
    dr_preset.border_spp = 1000;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
    dr_preset.image_batch_size = 4;
    dr_preset.debug_print = true;

    // dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_13_res.bmp", image_res);
  }

  printf("TEST 13. Optimize smallest SDF scene with diffuse\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("13.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_14_optimize_sphere_lambert()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_small_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_14_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_small_scene();
    // randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.1f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.opt_iterations = 200;
    dr_preset.opt_lr = 0.005f;
    dr_preset.spp = 4;
    dr_preset.border_spp = 1000;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
    dr_preset.image_batch_size = 4;
    dr_preset.debug_print = true;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_14_res.bmp", image_res);
  }

  printf("TEST 14. Optimize smallest SDF scene with lambert\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("14.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_15_combined_reconstruction()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_small_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(8, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_15_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_small_scene();
    randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.15f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
    dr_preset.opt_iterations = 500;
    dr_preset.opt_lr = 0.01f;
    dr_preset.spp = 16;
    dr_preset.border_spp = 512;
    dr_preset.image_batch_size = 2;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_15_res.bmp", image_res);
  }

  printf("TEST 15. Combined optimization of small SDF scene\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("15.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void border_detection_test(uint32_t num, uint32_t multi_render_mode, uint32_t dr_render_mode, SdfSBS scene)
{
  srand(time(nullptr));
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_smallest_scene();

  unsigned W = 512, H = 512;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = multi_render_mode;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(1, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  LiteImage::Image2D<float4> image_bdet(W, H);
  LiteImage::Image2D<float4> image_bint(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
  }

  //we test only border detection here, so refernce if actually useless
  //LiteImage::SaveImage<float4>(("saves/test_dr_16_"+std::to_string(num)+"_ref.bmp").c_str(), images_ref[0]); 

  // put random colors to SBS
  auto indexed_SBS = scene;

  MultiRendererDRPreset dr_preset = getDefaultPresetDR();
  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.dr_render_mode = dr_render_mode;
  dr_preset.opt_iterations = 1;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 16;
  dr_preset.border_spp = 256;
  {
    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_NONE;
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>(("saves/test_dr_16_"+std::to_string(num)+"_res.bmp").c_str(), image_res);
  }
  {
    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_BORDER_DETECTION;
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_bdet = dr_render.getLastDebugImage(0);
    LiteImage::SaveImage<float4>(("saves/test_dr_16_"+std::to_string(num)+"_bdet.bmp").c_str(), image_bdet);
  }
  {
    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL;
    dr_preset.debug_forced_border = true;
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_bint = dr_render.getLastDebugImage(0);
    LiteImage::SaveImage<float4>(("saves/test_dr_16_"+std::to_string(num)+"_bint.bmp").c_str(), image_bint);
  }

  int match = 0;
  int miss = 0;
  int over = 0;
  for (int i = 0; i < W * H; i++)
  {
    bool border = length(to_float3(image_bdet.data()[i])) > 0;
    bool border_int = length(to_float3(image_bint.data()[i])) > 0;

    match += border && border_int;
    miss += !border && border_int;
    over += border && !border_int;
  }
  
  float om = float(over)/float(match);
  float miss_rate = float(miss)/float(match);

  printf("16.%u.1. %-64s", num, "detected border is small (over/match < 4)");
  if (om < 8)
    printf("passed    (%.2f)\n", om);
  else
    printf("FAILED, over/match = %f\n", om);
  
  printf("16.%u.2. %-64s", num, "a few pixels are missed (miss/match < 0.05)");
  if (miss_rate < 0.05)
    printf("passed    (%.2f)\n", miss_rate);
  else
    printf("FAILED, miss/match = %f\n", miss_rate);
}
void diff_render_test_16_borders_detection()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  auto bunny_SBS = create_grid_sbs(1, 32, [&](float3 p) -> float { return mesh_bvh.get_signed_distance(p);},
                                   single_color);

  printf("TEST 16. Check border detection\n");
  printf("16.1. Mask with two circles\n");
  border_detection_test(1, MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, two_circles_scene());

  printf("16.2. Mask with bunny\n");
  border_detection_test(2, MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, bunny_SBS);

  printf("16.3. Diffuse with circle\n");
  border_detection_test(3, MULTI_RENDER_MODE_DIFFUSE, DR_RENDER_MODE_DIFFUSE, circle_medium_scene());

  printf("16.4. Lambert with circle\n");
  border_detection_test(4, MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, circle_medium_scene());

  printf("16.5. Lambert with two circles\n");
  border_detection_test(5, MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, two_circles_scene());

  printf("16.6. Lambert with bunny\n");
  border_detection_test(6, MULTI_RENDER_MODE_LAMBERT, DR_RENDER_MODE_LAMBERT, bunny_SBS);
}

void diff_render_test_17_optimize_bunny()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 16;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);
  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");

  std::vector<float4x4> view = get_cameras_uniform_sphere(8, float3(0, 0, 0), 4.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
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
    LiteImage::SaveImage<float4>(("saves/test_dr_17_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS =  create_grid_sbs(4, 4, 
                                        [&](float3 p){return circle_sdf(float3(0,-0.15f,0), 0.7f, p);}, 
                                        [](float3 p){return float3(0.95,0.95,0.95);});

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY | DR_RECONSTRUCTION_FLAG_COLOR;
    dr_preset.opt_iterations = 1001;
    dr_preset.opt_lr = 0.01f;
    dr_preset.spp = 24;
    dr_preset.border_spp = 1024;
    dr_preset.image_batch_size = 2;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_RANDOM;
    dr_preset.debug_print = true;
    dr_preset.debug_print_interval = 1;
    dr_preset.debug_progress_interval = 10;
    dr_preset.render_height = 256;
    dr_preset.render_width = 256;
    dr_preset.border_relax_eps = 0.005f;
    dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL;

    // dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_17_res.bmp", image_res);
  }

  printf("TEST 17. Optimize bunny scene\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("17.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_18_sphere_depth()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_small_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_18_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_small_scene();
    // randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.1f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_LINEAR_DEPTH;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.dr_input_type = DR_INPUT_TYPE_LINEAR_DEPTH;
    dr_preset.opt_iterations = 500;
    dr_preset.opt_lr = 0.002f;
    dr_preset.spp = 16;
    dr_preset.border_spp = 512;
    dr_preset.image_batch_size = 4;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_18_res.bmp", image_res);
  }

  printf("TEST 18. Optimize smallest SDF scene with depth input\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("18.1. %-64s", "SDF is reconstructed");
  if (psnr >= 40)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_20_sphere_depth_with_redist()
{
  //create renderers for SDF scene and mesh scene
  auto SBS_ref = circle_small_scene();

  unsigned W = 256, H = 256;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
  //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.spp = 64;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS_ref);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_20_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    auto indexed_SBS = circle_small_scene();
    // randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.1f);

    MultiRendererDRPreset dr_preset = getDefaultPresetDR();

    dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_render_mode = DR_RENDER_MODE_LINEAR_DEPTH;
    dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
    dr_preset.dr_input_type = DR_INPUT_TYPE_LINEAR_DEPTH;
    dr_preset.opt_iterations = 500;
    dr_preset.opt_lr = 0.002f;
    dr_preset.spp = 16;
    dr_preset.border_spp = 512;
    dr_preset.image_batch_size = 4;

    // new compared to Test 18
    dr_preset.redistancing_enable = true;
    dr_preset.redistancing_interval = 1;
    // dr_preset.debug_print = true;
    // dr_preset.debug_progress_images = DEBUG_PROGRESS_RAW;
    // dr_preset.debug_progress_interval = 4;

    MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_20_res.bmp", image_res);
  }

  printf("TEST 20. Optimize smallest SDF scene with depth input, redistancing enabled\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("20.1. %-64s", "SDF is reconstructed");
  if (psnr >= 40)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void diff_render_test_22_border_sampling_accuracy_mask()
{
  srand(0);
  unsigned W = 512, H = 512;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_MASK;
  preset.spp = 4;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(1, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  LiteImage::Image2D<float4> image_ref(W, H);
  LiteImage::Image2D<float4> image_res(W, H);
  SdfSBS indexed_SBS;

  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);  

    pRender->SetScene(mesh);
    pRender->RenderFloat(image_ref.data(), image_ref.width(), image_ref.height(), view[0], proj[0], preset);
    LiteImage::SaveImage<float4>("saves/test_dr_22_ref.bmp", image_ref); 

    SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 2);

    SdfSBSHeader header;
    header.brick_size = 16;
    header.brick_pad = 0;
    header.bytes_per_value = 4;
    
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    indexed_SBS = create_grid_sbs(1, 16, [&](float3 p) -> float { return mesh_bvh.get_signed_distance(p);},
                                  single_color);
    //randomize_distance(indexed_SBS, 0.03);
  }

  MultiRendererDRPreset dr_preset = getDefaultPresetDR();

  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.dr_input_type = DR_INPUT_TYPE_COLOR;
  dr_preset.opt_iterations = 1;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 64;
  dr_preset.debug_render_mode =  DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL;
  dr_preset.debug_print = true;
  dr_preset.debug_progress_interval = 1;
  dr_preset.render_height = 128;
  dr_preset.render_width  = 128;

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  std::vector<float> grad_ref(param_count, 0);
  std::vector<float> grad_rnd(param_count, 0);
  std::vector<float> grad_svm(param_count, 0);

  {
    MultiRendererDR dr_render;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_RANDOM;
    dr_preset.border_spp = 40000;
    dr_render.SetReference({image_ref}, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    grad_ref = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                  dr_render.getLastdLoss_dS() + param_offset + param_count);
  }

  unsigned samples = 1000;
  {
    MultiRendererDR dr_render;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_RANDOM;
    dr_preset.border_spp = samples;
    //dr_preset.debug_border_samples_mega_image = true;
    dr_render.SetReference({image_ref}, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    grad_rnd = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                  dr_render.getLastdLoss_dS() + param_offset + param_count);
  }
  {
    MultiRendererDR dr_render;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
    dr_preset.border_spp = samples;
    //dr_preset.debug_border_samples_mega_image = true;
    dr_render.SetReference({image_ref}, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    grad_svm = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                  dr_render.getLastdLoss_dS() + param_offset + param_count);
  }

  long double l1 = 0, l2 = 0, l3 = 0;
  long double d1 = 0, d2 = 0, d3 = 0;
  long double loss_1 = 0, loss_2 = 0, loss_3 = 0;
  for (unsigned i = 0; i < param_count; i++)
  {
    l1 += grad_ref[i] * grad_ref[i];
    l2 += grad_rnd[i] * grad_rnd[i];
    l3 += grad_svm[i] * grad_svm[i];

    d1 += grad_ref[i] * grad_rnd[i];
    d2 += grad_ref[i] * grad_svm[i];
    d3 += grad_rnd[i] * grad_svm[i];

    loss_1 += abs(grad_ref[i] - grad_rnd[i]);
    loss_2 += abs(grad_ref[i] - grad_svm[i]);
    loss_3 += abs(grad_rnd[i] - grad_svm[i]);
  }

  float cosine_1 = d1 / (sqrt(l1) * sqrt(l2));
  float cosine_2 = d2 / (sqrt(l1) * sqrt(l3));
  float cosine_3 = d3 / (sqrt(l2) * sqrt(l3));

  printf("Reference - Random cosine similarity: %f\n", cosine_1);
  printf("Reference - SVM    cosine similarity: %f\n", cosine_2);
  printf("   Random - SVM    cosine similarity: %f\n", cosine_3);

  printf("Reference - Random loss: %Lf\n", loss_1/param_count);
  printf("Reference - SVM    loss: %Lf\n", loss_2/param_count);
  printf("   Random - SVM    loss: %Lf\n", loss_3/param_count);
}

void
diff_render_test_23_ray_casting_mask()
{
  float time1 = 0, psnr1 = 0, time2 = 0, psnr2 = 0;

  {
    //create renderers for SDF scene and mesh scene
    auto SBS_ref = circle_small_scene();

    unsigned W = 256, H = 256;

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
    //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
    preset.spp = 10;

    float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

    std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
    std::vector<float4x4> proj(view.size(), base_proj);

    std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
    LiteImage::Image2D<float4> image_res(W, H);
    for (int i = 0; i < view.size(); i++)
    {
      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetViewport(0,0,W,H);

      pRender->SetScene(SBS_ref);
      pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
      LiteImage::SaveImage<float4>(("saves/test_dr_13_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
    }

    {
      auto indexed_SBS = circle_small_scene();
      // randomize_color(indexed_SBS);
      randomize_distance(indexed_SBS, 0.1f);

      MultiRendererDRPreset dr_preset = getDefaultPresetDR();

      dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
      dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
      dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
      dr_preset.opt_iterations = 200;
      dr_preset.opt_lr = 0.005f;
      dr_preset.spp = 4;
      dr_preset.border_spp = 10;
      dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
      dr_preset.image_batch_size = 4;
      dr_preset.debug_print = true;

      // dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;

      MultiRendererDR dr_render;
      dr_render.SetReference(images_ref, view, proj);

      auto before = std::chrono::high_resolution_clock::now();

      dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);

      time1 = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - before).count()/1000.f) / 10;
      image_res = dr_render.getLastImage(0);
      LiteImage::SaveImage<float4>("saves/test_dr_23_res_without_mask.bmp", image_res);
    }

    // printf("TEST 23. Compare ray tracing with/without mask\n");
    
    psnr1 = image_metrics::PSNR(image_res, images_ref[0]);
  }

  {
    //create renderers for SDF scene and mesh scene
    auto SBS_ref = circle_small_scene();

    unsigned W = 256, H = 256;

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
    //preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
    preset.spp = 10;

    float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

    std::vector<float4x4> view = get_cameras_uniform_sphere(16, float3(0, 0, 0), 3.0f);
    std::vector<float4x4> proj(view.size(), base_proj);

    std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
    LiteImage::Image2D<float4> image_res(W, H);
    for (int i = 0; i < view.size(); i++)
    {
      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetViewport(0,0,W,H);

      pRender->SetScene(SBS_ref);
      pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
      LiteImage::SaveImage<float4>(("saves/test_dr_13_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
    }

    {
      auto indexed_SBS = circle_small_scene();
      // randomize_color(indexed_SBS);
      randomize_distance(indexed_SBS, 0.1f);

      MultiRendererDRPreset dr_preset = getDefaultPresetDR();

      dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
      dr_preset.dr_render_mode = DR_RENDER_MODE_DIFFUSE;
      dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
      dr_preset.opt_iterations = 200;
      dr_preset.opt_lr = 0.005f;
      dr_preset.spp = 4;
      dr_preset.border_spp = 10;
      dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_SVM;
      dr_preset.image_batch_size = 4;
      dr_preset.debug_print = true;

      dr_preset.dr_raycasting_mask = DR_RAYCASTING_MASK_ON;

      MultiRendererDR dr_render;
      dr_render.setBorderThickness(5);
      dr_render.SetReference(images_ref, view, proj);

      auto before = std::chrono::high_resolution_clock::now();

      dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
      
      time2 = (std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - before).count()/1000.f) / 10;
      
      dr_render.cleanMasks();

      image_res = dr_render.getLastImage(0);
      LiteImage::SaveImage<float4>("saves/test_dr_23_res_with_mask.bmp", image_res);
    }
    
    psnr2 = image_metrics::PSNR(image_res, images_ref[0]);
  }

  printf("\nTEST 23. Compare ray tracing with/without mask\n");
  printf("Without mask: psnr:%.2f, time:%.2f ms\n", psnr1, time1);
  printf("With    mask: psnr:%.2f, time:%.2f ms\n", psnr2, time2);
  printf("The ratio of the running time of the algorithm without mask to with mask: %.2f\n\n", time1 / time2);
}

void
diff_render_test_24_optimization_with_tricubic()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::normalize_mesh(mesh);
  unsigned W = 512, H = 512;
  
  {
    MultiRenderPreset preset = getDefaultPreset();
    preset.interpolation_type = 0;
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    LiteImage::Image2D<float4> image_1(W, H);

    auto grid = sdf_converter::create_sdf_grid(GridSettings(64), mesh);
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(grid);
    render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<float4>("saves/test_25_grid_trilinear.bmp", image_1);
  }

  {
    MultiRenderPreset preset = getDefaultPreset();
    preset.interpolation_type = TRICUBIC_INTERPOLATION_MODE;
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    LiteImage::Image2D<float4> image_1(W, H);

    auto grid = sdf_converter::create_sdf_grid(GridSettings(64), mesh);
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(grid);
    render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<float4>("saves/test_25_grid_tricubic.bmp", image_1);
  }
}

void
diff_render_test_25_tricubic_enzyme_derrivative()
{
  float grid[64] = {
    0.1, 0.2, 0.3, 0.4, 0.2, 0.3, 0.4, 0.5, 0.3, 0.4, 0.5, 0.6, 0.4, 0.5, 0.6, 0.7,
    0.2, 0.3, 0.4, 0.5, 0.3, 0.4, 0.5, 0.6, 0.4, 0.5, 0.6, 0.7, 0.5, 0.6, 0.7, 0.8,
    0.3, 0.4, 0.5, 0.6, 0.4, 0.5, 0.6, 0.7, 0.5, 0.6, 0.7, 0.8, 0.6, 0.7, 0.8, 0.9,
    0.4, 0.5, 0.6, 0.7, 0.5, 0.6, 0.7, 0.8, 0.6, 0.7, 0.8, 0.9, 0.7, 0.8, 0.9, 1.0
  };

  BVHRT bvhrt;
  bvhrt.m_SdfGridData = std::vector<float>(grid, grid + 64);

  float x[3] = {0.5, 0.5, 0.5}, d_x[3], d_grid[64];

  float interpolated_value = bvhrt.tricubicInterpolation(grid, x);
  std::cout << "INTERPOLATED VALUE IS: " << interpolated_value << std::endl;

  bvhrt.tricubicInterpolationDerrivative(grid, d_grid, x, d_x);
  printf("%f %f %f\n", d_x[0], d_x[1], d_x[2]);
}

void
diff_render_test_26_sbs_tricubic()
{
  
}

void diff_render_test_27_visualize_bricks()
{
  //this test is rendering voxels with different combination of positive and negative distances in it's corners
  srand(time(nullptr));

  SdfSBS sbs;
  sbs.header.brick_pad = 0;
  sbs.header.brick_size = 1;
  sbs.header.bytes_per_value = 1;
  sbs.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F;

  sbs.nodes.resize(1);
  sbs.nodes[0].data_offset = 0;
  sbs.nodes[0].pos_xy = (0 << 16) | 0;
  sbs.nodes[0].pos_z_lod_size = (0 << 16) | 1;
  
  sbs.values = {0,1,2,3,4,5,6,7,
                8, 11, 14, 17, 20, 23, 26, 29};

  sbs.values_f.resize(8 + 3*8);

  unsigned rnd = rand() % 256;
  //rnd = 1*1 + 0*2 + 1*4 + 0*8 + 1*16 + 0*32 + 1*64 + 0*128;
  for (int i = 0; i < 8; i++)
  {
    float rand_f = ((float)rand() / (float)RAND_MAX);
    sbs.values_f[i] = (rnd & (1 << i)) ? -rand_f: rand_f;
  }
  for (int i = 8; i < 8+3*8; i++)
  {
    sbs.values_f[i] = 1.0f;
  }

  unsigned W = 1024, H = 1024;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  preset.spp = 1;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);
  std::vector<float4x4> view = get_cameras_turntable(16, float3(0,0,0), 4, 1);
  std::vector<float4x4> proj(view.size(), base_proj);
  LiteImage::Image2D<float4> image_res(W, H);

  for (int i = 0; i < view.size(); i++)
  {
    MultiRendererDR pRender;
    pRender.SetPreset(preset);
    pRender.SetViewport(0,0,W,H);

    //dr::BVHDR* bvhdr_tree = dynamic_cast<dr::BVHDR*>(pRender.GetAccelStruct().get());
    //printf("BVH tree size: %lu\n", pRender.GetAccelStruct().get());
    //pRender.Redistance(sbs.values_f.data(), {1, 1, 1}, 2.f / 1, 2);

    pRender.SetScene(sbs);

    pRender.RenderFloat(image_res.data(), W, H, view[i], proj[i], preset);
    LiteImage::SaveImage<float4>(("saves/test_dr_24_"+std::to_string(i)+".bmp").c_str(), image_res); 
  }
}

void diff_render_test_28_border_sampling_normals_vis()
{
  // srand(0);
  unsigned W = 512, H = 512;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_MASK;
  preset.spp = 4;

  float4x4 base_proj = LiteMath::perspectiveMatrix(60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view = get_cameras_uniform_sphere(1, float3(0, 0, 0), 3.0f);
  std::vector<float4x4> proj(view.size(), base_proj);

  LiteImage::Image2D<float4> image_ref(W, H);
  LiteImage::Image2D<float4> image_res(W, H);
  SdfSBS indexed_SBS;

  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    uint32_t texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    uint32_t matId = pRender->AddMaterial(mat);
    pRender->SetMaterial(matId, 0);  

    pRender->SetScene(mesh);
    pRender->RenderFloat(image_ref.data(), image_ref.width(), image_ref.height(), view[0], proj[0], preset);
    LiteImage::SaveImage<float4>("saves/test_dr_27_ref.bmp", image_ref); 

    SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 2);

    SdfSBSHeader header;
    header.brick_size = 16;
    header.brick_pad = 0;
    header.bytes_per_value = 4;
    
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    indexed_SBS = create_grid_sbs(1, 16, [&](float3 p) -> float { return mesh_bvh.get_signed_distance(p);},
                                  single_color);
  }

  MultiRendererDRPreset dr_preset = getDefaultPresetDR();

  dr_preset.dr_render_mode = DR_RENDER_MODE_MASK;
  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.dr_input_type = DR_INPUT_TYPE_COLOR;
  dr_preset.opt_iterations = 1;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 64;
  dr_preset.debug_render_mode =  DR_DEBUG_RENDER_MODE_BORDER_INTEGRAL;
  dr_preset.debug_print = true;
  dr_preset.debug_progress_interval = 1;
  dr_preset.render_height = 128;
  dr_preset.render_width  = 128;

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  std::vector<float> grad_ref(param_count, 0);
  std::vector<float> grad_rnd(param_count, 0);
  std::vector<float> grad_svm(param_count, 0);

  LiteImage::Image2D<float4> image_norm(W, H);
  GraphicsPrim normals;
  normals.header.prim_type = GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR;

  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

  unsigned samples = 50;
  {
    MultiRendererDR dr_render;
    dr_preset.dr_border_sampling = DR_BORDER_SAMPLING_RANDOM;
    dr_preset.debug_border_save_normals = true;
    dr_preset.border_spp = samples;
    dr_render.SetReference({image_ref}, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);
    grad_svm = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                  dr_render.getLastdLoss_dS() + param_offset + param_count);
    BVHDR* bvh_dr = static_cast<BVHDR*>(dr_render.GetAccelStruct().get());
    normals.points = bvh_dr->m_GraphicsPrimPoints;
  }
  printf("Normals count: %d\n", (int)normals.points.size());
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->GetAccelStruct()->ClearGeom();
    pRender->GetAccelStruct()->ClearScene();
    BVHRT *bvhrt = dynamic_cast<BVHRT*>(pRender->GetAccelStruct()->UnderlyingImpl(0));

    unsigned norm_geomId = bvhrt->AddGeom_GraphicsPrim(normals, pRender->GetAccelStruct().get());
    //unsigned sdf_geomId = bvhrt->AddGeom_SdfSBS(indexed_SBS, pRender->GetAccelStruct().get(), true);

    unsigned model_geomId = pRender->GetAccelStruct()->AddGeom_Triangles3f(
                           (const float*)mesh.vPos4f.data(), mesh.vPos4f.size(), 
                            mesh.indices.data(), mesh.indices.size(), BUILD_HIGH, sizeof(float)*4);                                                          
    pRender->add_mesh_internal(mesh, model_geomId);
    

    pRender->AddInstance(norm_geomId, LiteMath::float4x4());
    pRender->AddInstance(model_geomId, LiteMath::float4x4());

    pRender->GetAccelStruct()->CommitScene();

    pRender->RenderFloat(image_norm.data(), image_norm.width(), image_norm.height(), view[0], proj[0], preset, 1);
    std::string norm_img_name = "saves/test_dr_28_normals_";
    norm_img_name += std::to_string(0) + ".bmp";
    LiteImage::SaveImage<float4>(norm_img_name.c_str(), image_norm);
  }
  printf("DONE\n");
}


void check_border_rays(const SdfSBS &SBS, unsigned render_node, unsigned diff_render_mode,
                       unsigned border_sampling, bool close_view)
{
  srand(time(nullptr));

  unsigned W = 64, H = 64;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;
  preset.spp = 256;

  float4x4 base_proj = LiteMath::perspectiveMatrix(close_view ? 15 : 60, 1.0f, 0.01f, 100.0f);

  std::vector<float4x4> view; 
  if (close_view)
    view = std::vector<float4x4>{LiteMath::lookAt(float3(0.6, 0.6, 2.0), float3(0.6, 0.6, 0), float3(0, 1, 0))};
  else
    view = std::vector<float4x4>{LiteMath::lookAt(float3(0, 0, 2.5), float3(0, 0, 0), float3(0, 1, 0))};

  std::vector<float4x4> proj(view.size(), base_proj);

  std::vector<LiteImage::Image2D<float4>> images_ref(view.size(), LiteImage::Image2D<float4>(W, H));
  LiteImage::Image2D<float4> image_res(W, H);
  for (int i = 0; i < view.size(); i++)
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(SBS);
    pRender->RenderFloat(images_ref[i].data(), images_ref[i].width(), images_ref[i].height(), view[i], proj[i], preset);
    LiteImage::SaveImage<float4>("saves/test_dr_35_0_ref.bmp", images_ref[i]); 
  }

  auto indexed_SBS = SBS;

  MultiRendererDRPreset dr_preset = getDefaultPresetDR();

  dr_preset.dr_render_mode = diff_render_mode;
  dr_preset.dr_diff_mode = DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_border_sampling = border_sampling;
  dr_preset.dr_reconstruction_flags = DR_RECONSTRUCTION_FLAG_GEOMETRY;
  dr_preset.dr_input_type = diff_render_mode == DR_RENDER_MODE_LINEAR_DEPTH ? DR_INPUT_TYPE_LINEAR_DEPTH : DR_INPUT_TYPE_COLOR;
  dr_preset.opt_iterations = 1;
  dr_preset.opt_lr = 0.0f;
  dr_preset.spp = 512;
  dr_preset.border_spp = 2048;
  dr_preset.debug_pd_brightness = 1.0f;
  dr_preset.border_relax_eps = 0.01f;
  dr_preset.finite_diff_delta = 0.005f;
  dr_preset.finite_diff_brightness = 0.25f;
  dr_preset.debug_render_mode = DR_DEBUG_RENDER_MODE_BORDER_DETECTION;
  dr_preset.debug_progress_images = DEBUG_PROGRESS_RAW;
  dr_preset.debug_forced_border = false;
  dr_preset.debug_border_samples_mega_image = true;
  dr_preset.debug_border_save_normals = true;


  std::ofstream hist_data_txt("saves_border_rays/hist_data.txt");
  bool fprint_hist_data = dr_preset.debug_border_save_normals; // hist - must enable normals (t is stored with the normals)

  if (fprint_hist_data)
    hist_data_txt << "Hist " << dr_preset.border_relax_eps << "\n";

  const unsigned samples = 16;
  for (unsigned i = 0; i < samples; i++)
  {
    srand(time(nullptr) + i);
    MultiRendererDR dr_render;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeFixedStructure(dr_preset, indexed_SBS);

    BVHDR* bvh_as = dynamic_cast<BVHDR*>(dr_render.GetAccelStruct().get());

    if (fprint_hist_data)
    {
      assert((bvh_as->m_GraphicsPrimPoints.size() % 3) == 0);
      for (uint32_t b = 0u; b < bvh_as->m_GraphicsPrimPoints.size(); b += 3)
        hist_data_txt << bvh_as->m_GraphicsPrimPoints[b + 1].w << ", ";
      hist_data_txt << "\n";
    }

    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_35_0_res.bmp", image_res); 
  }
  if (fprint_hist_data)
    hist_data_txt << "\n";

  hist_data_txt.close();
}

void diff_render_test_35_border_sampling_rays_hist()
{
  check_border_rays(circle_smallest_scene_colored(), 
                    MULTI_RENDER_MODE_MASK, DR_RENDER_MODE_MASK, DR_BORDER_SAMPLING_RANDOM,
                    true);
}

void perform_tests_diff_render(const std::vector<int> &test_ids)
{
  std::vector<int> tests = test_ids;

  std::vector<std::function<void(void)>> test_functions = {
      /*01*/diff_render_test_1_forward_pass, 
      /*02*/diff_render_test_2_check_color_derivatives, 
      /*03*/diff_render_test_3_optimize_color,
      /*04*/diff_render_test_4_check_position_derivatives, 
      /*05*/diff_render_test_5_optimize_sdf_finite_derivatives,
      /*06*/diff_render_test_6_optimization_stand,
      /*07*/diff_render_test_7_SVM_sampling,
      /*08*/diff_render_test_8_redistancing,
      /*09*/diff_render_test_9_regularization,
      /*10*/diff_render_test_10_ray_casting_mask,
      /*11*/diff_render_test_11_expanding_grid,
      /*12*/diff_render_test_12_depth,

      /*13*/diff_render_test_11_optimize_smallest_scene, 
      /*14*/diff_render_test_12_optimize_sphere_mask,
      /*15*/diff_render_test_13_optimize_sphere_diffuse, 
      /*16*/diff_render_test_14_optimize_sphere_lambert, 
      /*17*/diff_render_test_15_combined_reconstruction,
      /*18*/diff_render_test_16_borders_detection, 
      /*19*/diff_render_test_17_optimize_bunny, 
      /*20*/diff_render_test_18_sphere_depth,
      /*21*/diff_render_test_7_optimize_with_finite_diff, 
      /*22*/diff_render_test_20_sphere_depth_with_redist,
      /*23*/diff_render_test_22_border_sampling_accuracy_mask, 
      /*24*/diff_render_test_23_ray_casting_mask, 
      /*25*/diff_render_test_24_optimization_with_tricubic, 
      /*26*/diff_render_test_25_tricubic_enzyme_derrivative, 
      /*27*/diff_render_test_26_sbs_tricubic, 
      /*28*/diff_render_test_27_visualize_bricks, 
      /*29*/diff_render_test_28_border_sampling_normals_vis,
      /*30*/diff_render_test_1_enzyme_ad, 
      /*31*/diff_render_test_3_optimize_color,
      /*32*/diff_render_test_4_render_simple_scenes, 
      /*33*/diff_render_test_5_optimize_color_simpliest,
      /*34*/diff_render_test_8_optimize_with_lambert,
      /*35*/diff_render_test_35_border_sampling_rays_hist,
      /*36*/
      /*37*/
      /*38*/
      /*39*/
      /*40*/};

  if (tests.empty())
  {
    tests.resize(test_functions.size());
    for (int i = 0; i < test_functions.size(); i++)
      tests[i] = i + 1;
  }

  for (int i = 0; i < 80; i++)
    printf("#");
  printf("\nDIFF RENDER TESTS\n");
  for (int i = 0; i < 80; i++)
    printf("#");
  printf("\n");

  for (int i : tests)
  {
    assert(i > 0 && i <= test_functions.size());
    test_functions[i - 1]();
  }
}