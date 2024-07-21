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

static double urand(double from=0, double to=1)
{
  return ((double)rand() / RAND_MAX) * (to - from) + from;
}

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

#ifdef USE_ENZYME
int enzyme_const, enzyme_dup, enzyme_out; // must be global
double __enzyme_autodiff(void*, ...);

double litert_test_28_enzyme_ad_sqr_1(double  x) { return  x *  x; }
double litert_test_28_enzyme_ad_sqr_2(double* x) { return *x * *x; }
double litert_test_28_enzyme_ad_mul(double k, double x) { return k * x; }
#endif

void diff_render_test_1_enzyme_ad()
{
  printf("TEST 1. Enzyme AD\n");
#ifdef USE_ENZYME
  double x = 5.;
  double df_dx_res = 2*x;

  // Output argument
  printf(" 1.1. %-64s", "Basic square derivative");
  double df_dx_out = __enzyme_autodiff((void*)litert_test_28_enzyme_ad_sqr_1, x);
  if ((df_dx_out - df_dx_res) < 1e-7 && (df_dx_out - df_dx_res) > -1e-7) // actually even (df_dx_out == df_dx_res) works
    printf("passed    (d(x^2) = %.1f, x = %.1f)\n", df_dx_out, x);
  else
    printf("FAILED,   (df_dx_out = %f, must be %.1f)\n", df_dx_out, df_dx_res);

  // Duplicated argument
  printf(" 1.2. %-64s", "Square derivative, dx is stored in an argument");
  double df_dx_arg = 0.;
  __enzyme_autodiff((void*)litert_test_28_enzyme_ad_sqr_2, &x, &df_dx_arg);
  if ((df_dx_arg - df_dx_res) < 1e-7 && (df_dx_arg - df_dx_res) > -1e-7) // actually even (df_dx_arg == df_dx_res) works
    printf("passed    (d(x^2) = %.1f, x = %.1f)\n", df_dx_arg, x);
  else
    printf("FAILED,   (df_dx_arg = %f, must be %.1f)\n", df_dx_arg, df_dx_res);

  // Inactive (const) argument (explicit)
  printf(" 1.3. %-64s", "Derivative d(k*x)/dx, k - parameter");
  double k = 7;
  df_dx_out = 0.; // only to verify the result
  df_dx_out = __enzyme_autodiff((void*)litert_test_28_enzyme_ad_mul, enzyme_const, k, x); // enzyme_const not needed if k is int
  if ((df_dx_out - k) < 1e-7 && (df_dx_out - k) > -1e-7) // actually even (df_dx_out == k) works
    printf("passed    (d(k*x)/dx = %.1f, k = %.1f)\n", df_dx_out, k);
  else
    printf("FAILED,   (df_dx_out = %f, must be %.1f)\n", df_dx_out, k);

#else
  printf("  Enzyme AD is not used.\n");

#endif
}

void diff_render_test_2_forward_pass()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 1024, H = 1024;

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
    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0;
    dr_preset.spp = 16;

    dr_render.SetReference({image_mesh}, {view}, {proj});
    dr_render.OptimizeColor(dr_preset, indexed_SBS);
    
    image_SBS_dr = dr_render.getLastImage(0);
  }

  LiteImage::SaveImage<float4>("saves/test_dr_2_mesh.bmp", image_mesh); 
  LiteImage::SaveImage<float4>("saves/test_dr_2_sbs.bmp", image_SBS);
  LiteImage::SaveImage<float4>("saves/test_dr_2_sbs_dr.bmp", image_SBS_dr);

  //float psnr_1 = image_metrics::PSNR(image_mesh, image_SBS);
  float psnr_2 = image_metrics::PSNR(image_SBS, image_SBS_dr);

  printf("TEST 2. Differentiable render forward pass\n");

  printf(" 2.1. %-64s", "Diff render of SBS match regular render");
  if (psnr_2 >= 40)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
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
    int v_size = header.brick_size + 2*header.brick_pad + 1;
    int dist_per_node = v_size*v_size*v_size;
    for (auto &n : indexed_SBS.nodes)
    {
      for (int i=0;i<8;i++)
      {
        unsigned off = indexed_SBS.values[n.data_offset + dist_per_node + i];
        for (int j=0;j<3;j++)
          indexed_SBS.values_f[off + j] = urand();
      }
    }
    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 200;
    dr_preset.opt_lr = 0.25;
    dr_preset.spp = 1;

    dr_render.SetReference({image_mesh}, {view}, {proj});
    dr_render.OptimizeColor(dr_preset, indexed_SBS);
    
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

void perform_tests_diff_render(const std::vector<int> &test_ids)
{
  std::vector<int> tests = test_ids;

  std::vector<std::function<void(void)>> test_functions = {
      diff_render_test_1_enzyme_ad, diff_render_test_2_forward_pass, diff_render_test_3_optimize_color};

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