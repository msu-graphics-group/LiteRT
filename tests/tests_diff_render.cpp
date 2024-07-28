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

float circle_sdf(float3 center, float radius, float3 p)
{
  return length(p - center) - radius;
}
float3 gradient_color(float3 p)
{
  return  (1-p.x)*(1-p.y)*(1-p.z)*float3(1,0,0) + 
          (1-p.x)*(1-p.y)*(  p.z)*float3(1,0,0) + 
          (1-p.x)*(  p.y)*(1-p.z)*float3(0,1,0) + 
          (1-p.x)*(  p.y)*(  p.z)*float3(0,1,0) + 
          (  p.x)*(1-p.y)*(1-p.z)*float3(1,0,0) + 
          (  p.x)*(1-p.y)*(  p.z)*float3(1,0,0) + 
          (  p.x)*(  p.y)*(1-p.z)*float3(0,1,0) + 
          (  p.x)*(  p.y)*(  p.z)*float3(0,1,0);
}

float3 single_color(float3 p)
{
  return float3(1,0,0);
}

//creates SBS where all nodes are present, i.e.
//it is really a regular grid, but with more indexes
//distance and color fields must be given
//it is for test purposes only
//SBS is created in [-1,1]^3 cube, as usual
SdfSBS create_grid_sbs(unsigned brick_count, unsigned brick_size, 
                       std::function<float(float3)>  sdf_func,
                       std::function<float3(float3)> color_func)
{
  unsigned v_size = brick_size+1;
  unsigned dist_per_node = v_size*v_size*v_size;
  unsigned colors_per_node = 8;
  unsigned p_count = brick_count*brick_size + 1u;
  unsigned c_count = brick_count + 1u;
  unsigned c_offset = p_count*p_count*p_count;

  SdfSBS scene;
  scene.header.brick_size = brick_size;
  scene.header.brick_pad  = 0;
  scene.header.bytes_per_value = 4;
  scene.header.aux_data = SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F;

  scene.values_f.resize(p_count*p_count*p_count + 3*c_count*c_count*c_count);
  scene.values.resize(brick_count*brick_count*brick_count*(dist_per_node+colors_per_node));
  scene.nodes.resize(brick_count*brick_count*brick_count);

  //fill the distances
  for (unsigned x = 0; x < p_count; x++)
  {
    for (unsigned y = 0; y < p_count; y++)
    {
      for (unsigned z = 0; z < p_count; z++)
      {
        unsigned idx = x*p_count*p_count + y*p_count + z;
        float3 p = 2.0f*(float3(x, y, z) / float3(brick_count*brick_size)) - 1.0f;
        scene.values_f[idx] = sdf_func(p);
      }
    }
  }

  //fill the colors
  for (unsigned x = 0; x < c_count; x++)
  {
    for (unsigned y = 0; y < c_count; y++)
    {
      for (unsigned z = 0; z < c_count; z++)
      {
        unsigned idx = x*c_count*c_count + y*c_count + z;
        float3 dp = float3(x, y, z) / float3(brick_count);
        float3 color = color_func(dp);
        scene.values_f[c_offset + 3*idx + 0] = color.x;
        scene.values_f[c_offset + 3*idx + 1] = color.y;
        scene.values_f[c_offset + 3*idx + 2] = color.z;
      }
    }
  }

  //fill the nodes and indices
  for (unsigned bx = 0; bx < brick_count; bx++)
  {
    for (unsigned by = 0; by < brick_count; by++)
    {
      for (unsigned bz = 0; bz < brick_count; bz++)
      {

        //nodes
        unsigned n_idx = bx*brick_count*brick_count + by*brick_count + bz;
        unsigned offset = n_idx*(dist_per_node+colors_per_node);
        scene.nodes[n_idx].pos_xy = (bx << 16) | by;
        scene.nodes[n_idx].pos_z_lod_size = (bz << 16) | brick_count;
        scene.nodes[n_idx].data_offset = offset;

        //indices for distances
        for (unsigned x = 0; x < v_size; x++)
        {
          for (unsigned y = 0; y < v_size; y++)
          {
            for (unsigned z = 0; z < v_size; z++)
            {
              unsigned idx = x*v_size*v_size + y*v_size + z;
              unsigned val_idx = (bx*brick_size + x)*p_count*p_count + (by*brick_size + y)*p_count + (bz*brick_size + z);
              scene.values[offset + idx] = val_idx;
            }
          }
        }

        //indices for colors
        for (unsigned x = 0; x < 2; x++)
        {
          for (unsigned y = 0; y < 2; y++)
          {
            for (unsigned z = 0; z < 2; z++)
            {
              unsigned idx = x*2*2 + y*2 + z;
              unsigned val_idx = c_offset + 3*((bx + x)*c_count*c_count + (by + y)*c_count + (bz + z));
              scene.values[offset + dist_per_node + idx] = val_idx;
            }
          }
        }
      }
    }
  }

  return scene;  
}

SdfSBS circle_one_brick_scene()
{
  return create_grid_sbs(1, 8, 
                         [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                         gradient_color);
}

SdfSBS circle_small_scene()
{
  return create_grid_sbs(2, 4, 
                         [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                         gradient_color);
}

SdfSBS circle_medium_scene()
{
  return create_grid_sbs(16, 4, 
                         [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                         gradient_color);
}

SdfSBS circle_smallest_scene()
{
  return create_grid_sbs(1, 2, 
                         [&](float3 p){return circle_sdf(float3(0,0,0), 0.8f, p);}, 
                         single_color);
}

std::vector<float4x4> get_cameras_uniform_sphere(int count, float3 center, float radius)
{
  std::vector<float4x4> cameras;
  for (int i = 0; i < count; i++)
  {
    float phi = 2 * M_PI * urand();
    float psi = (M_PI / 2) * (1 - sqrtf(urand()));
    if (urand() > 0.5)
      psi = -psi;

    float3 view_dir = -float3(cos(psi) * sin(phi), sin(psi), cos(psi) * cos(phi));
    float3 tangent = normalize(cross(view_dir, float3(0, 1, 0)));
    float3 new_up = normalize(cross(view_dir, tangent));
    cameras.push_back(LiteMath::lookAt(center - radius * view_dir, center, new_up));
  }

  return cameras;
}

std::vector<float4x4> get_cameras_turntable(int count, float3 center, float radius, float height)
{
  std::vector<float4x4> cameras;
  for (int i = 0; i < count; i++)
  {
    float phi = 2 * M_PI * urand();

    float3 view_dir = -float3(sin(phi), height/radius, cos(phi));
    float3 tangent = normalize(cross(view_dir, float3(0, 1, 0)));
    float3 new_up = normalize(cross(view_dir, tangent));
    cameras.push_back(LiteMath::lookAt(center - radius * view_dir, center, new_up));
  }

  return cameras;
}

void randomize_color(SdfSBS &sbs)
{
  int v_size = sbs.header.brick_size + 2 * sbs.header.brick_pad + 1;
  int dist_per_node = v_size * v_size * v_size;
  for (auto &n : sbs.nodes)
  {
    for (int i = 0; i < 8; i++)
    {
      unsigned off = sbs.values[n.data_offset + dist_per_node + i];
      for (int j = 0; j < 3; j++)
        sbs.values_f[off + j] = urand();
    }
  }
}

void randomize_distance(SdfSBS &sbs, float delta)
{
  int v_size = sbs.header.brick_size + 2 * sbs.header.brick_pad + 1;
  int dist_per_node = v_size * v_size * v_size;
  for (auto &n : sbs.nodes)
  {
    for (int i = 0; i < dist_per_node; i++)
    {
      sbs.values_f[sbs.values[n.data_offset + i]] += delta*urand(-1,1);
    }
  }
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
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.ray_gen_mode = RAY_GEN_MODE_RANDOM;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
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
  LiteImage::Image2D<float4> image_SBS_dr_2(W, H);

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

    dr_preset.dr_render_mode = dr::DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_type = dr::DR_RECONSTRUCTION_TYPE_COLOR;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0;
    dr_preset.spp = 16;

    dr_render.SetReference({image_mesh}, {view}, {proj});
    dr_render.OptimizeColor(dr_preset, indexed_SBS);
    
    image_SBS_dr = dr_render.getLastImage(0);
  }

  {
    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.dr_render_mode = dr::DR_RENDER_MODE_LAMBERT;
    dr_preset.dr_reconstruction_type = dr::DR_RECONSTRUCTION_TYPE_GEOMETRY;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0;
    dr_preset.spp = 16;

    dr_render.SetReference({image_mesh}, {view}, {proj});
    dr_render.OptimizeColor(dr_preset, indexed_SBS);
    
    image_SBS_dr_2 = dr_render.getLastImage(0);
  }

  LiteImage::SaveImage<float4>("saves/test_dr_2_mesh.bmp", image_mesh); 
  LiteImage::SaveImage<float4>("saves/test_dr_2_sbs.bmp", image_SBS);
  LiteImage::SaveImage<float4>("saves/test_dr_2_sbs_dr.bmp", image_SBS_dr);
  LiteImage::SaveImage<float4>("saves/test_dr_2_sbs_dr_2.bmp", image_SBS_dr_2);

  //float psnr_1 = image_metrics::PSNR(image_mesh, image_SBS);
  float psnr_2 = image_metrics::PSNR(image_SBS, image_SBS_dr);
  float psnr_3 = image_metrics::PSNR(image_SBS, image_SBS_dr_2);

  printf("TEST 2. Differentiable render forward pass\n");

  printf(" 2.1. %-64s", "Diff render (color mode) of SBS match regular render");
  if (psnr_2 >= 40)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);

  printf(" 2.2. %-64s", "Diff render (geometry mode) of SBS match regular render");
  if (psnr_3 >= 40)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);
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
    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 1;

    dr_render.SetReference({image_med}, {view}, {proj});
    dr_render.OptimizeColor(dr_preset, scene, false);
    
    image_one_brick_dr = dr_render.getLastImage(0);
  }

  {
    auto scene = circle_small_scene();
    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 1;

    dr_render.SetReference({image_med}, {view}, {proj});
    dr_render.OptimizeColor(dr_preset, scene, false);
    
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
  preset.spp = 16;

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

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 200;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference({images_ref[0]}, {view[0]}, {proj[0]});
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    
    image_SBS_single = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_single.bmp", image_SBS_single);
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 400;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    
    image_SBS_multi = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_multi.bmp", image_SBS_multi);
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_small_scene();
    randomize_color(indexed_SBS);

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 400;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    
    image_SBS_multi_2 = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_5_rec_multi_2.bmp", image_SBS_multi_2);

  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.opt_iterations = 400;
    dr_preset.opt_lr = 0.01f;
    dr_preset.spp = 4;
    dr_preset.dr_loss_function = dr::DR_LOSS_FUNCTION_MAE;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    
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

void diff_render_test_6_check_color_derivatives()
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
    LiteImage::SaveImage<float4>(("saves/test_dr_5_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_DEFAULT;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 4;

    unsigned color_param_count = 3*8*indexed_SBS.nodes.size();
    unsigned color_param_offset = indexed_SBS.values_f.size() - color_param_count;

    std::vector<float> grad_dr(color_param_count, 0);
    std::vector<float> grad_ref(color_param_count, 0);

    {
    dr::MultiRendererDR dr_render;
    dr_render.SetReference({images_ref[0]}, {view[0]}, {proj[0]});
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    grad_dr = std::vector<float>(dr_render.getLastdLoss_dS() + color_param_offset, 
                                 dr_render.getLastdLoss_dS() + color_param_offset + color_param_count);
    }
    {
    dr::MultiRendererDR dr_render;
    dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_FINITE_DIFF;
    dr_render.SetReference({images_ref[0]}, {view[0]}, {proj[0]});
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
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

    printf("TEST 6. Check color derivatives\n");
  
    printf(" 6.1. %-64s", "Relative difference in average PD value");
    if (average_error <= 0.05f)
      printf("passed    (%f)\n", average_error);
    else
      printf("FAILED, average_error = %f\n", average_error);
    
    printf(" 6.2. %-64s", "Relative bias in average PD");
    if (average_bias <= 0.05f)
      printf("passed    (%f)\n", average_bias);
    else
      printf("FAILED, average_bias = %f\n", average_bias);
    
    printf(" 6.3. %-64s", "Max relative difference in PD value");
    if (max_error <= 0.2f)
      printf("passed    (%f)\n", max_error);
    else
      printf("FAILED, max_error = %f\n", max_error);
    
    printf(" 6.4. %-64s", "Average relative difference in PD value");
    if (average_error_2 <= 0.05f)
      printf("passed    (%f)\n", average_error_2);
    else
      printf("FAILED, max_bias = %f\n", average_error_2);
    
    printf(" 6.5. %-64s", "Average relative bias in PD");
    if (average_bias_2 <= 0.05f)
      printf("passed    (%f)\n", average_bias_2);
    else
      printf("FAILED, average_bias = %f\n", average_bias_2);
  }
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

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_DEFAULT;
    dr_preset.opt_iterations = 100;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    
    image_SBS_def = dr_render.getLastImage(0);
  }
  LiteImage::SaveImage<float4>("saves/test_dr_7_def.bmp", image_SBS_def);
  
  {
    //put random colors to SBS
    auto indexed_SBS = circle_one_brick_scene();
    randomize_color(indexed_SBS);

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_FINITE_DIFF;
    dr_preset.opt_iterations = 100;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 4;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    
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

    dr::MultiRendererDR dr_render;
    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.dr_render_mode = dr::DR_RENDER_MODE_LAMBERT;
    dr_preset.opt_iterations = 300;
    dr_preset.opt_lr = 0.1f;
    dr_preset.spp = 1;
    dr_preset.spp = 2;

    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS);
    
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

void diff_render_test_9_check_position_derivatives()
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
    LiteImage::SaveImage<float4>(("saves/test_dr_9_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  {
    //put random colors to SBS
    auto indexed_SBS = circle_smallest_scene();
    //randomize_color(indexed_SBS);
    randomize_distance(indexed_SBS, 0.25f);

    dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

    dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_DEFAULT;
    dr_preset.dr_reconstruction_type = dr::DR_RECONSTRUCTION_TYPE_GEOMETRY;
    dr_preset.opt_iterations = 1;
    dr_preset.opt_lr = 0.0f;
    dr_preset.spp = 16;

    unsigned param_count = indexed_SBS.values_f.size() - 3*8*indexed_SBS.nodes.size();
    unsigned param_offset = 0;

    std::vector<float> grad_dr(param_count, 0);
    std::vector<float> grad_ref(param_count, 0);

    {
    dr::MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    grad_dr = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                 dr_render.getLastdLoss_dS() + param_offset + param_count);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_9_res.bmp", image_res); 
    }
    {
    dr::MultiRendererDR dr_render;
    dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_FINITE_DIFF;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    grad_ref = std::vector<float>(dr_render.getLastdLoss_dS() + param_offset, 
                                  dr_render.getLastdLoss_dS() + param_offset + param_count);
    }

    long double average_1 = 0;
    long double abs_average_1 = 0;
    long double average_2 = 0;
    long double abs_average_2 = 0;
    long double diff = 0;
    long double abs_diff = 0;
    for (int i = 0; i < param_count; i++)
    {
      average_1 += grad_ref[i];
      abs_average_1 += abs(grad_ref[i]);
      average_2 += grad_dr[i];
      abs_average_2 += abs(grad_dr[i]);
      diff += abs(grad_ref[i] - grad_dr[i]);
      abs_diff += abs(grad_ref[i] - grad_dr[i]);
    }
    average_1 /= param_count;
    abs_average_1 /= param_count;
    average_2 /= param_count;
    abs_average_2 /= param_count;
    diff /= param_count;
    abs_diff /= param_count;

    printf("[");
    for (int i = 0; i < param_count; i++)
      printf("%8.3f ", grad_ref[i]);
    printf("]\n");

    printf("[");
    for (int i = 0; i < param_count; i++)
      printf("%8.3f ", grad_dr[i]);
    printf("]\n");

    //image_SBS_single = dr_render.getLastImage(0);
    float average_error = abs(abs_average_2 - abs_average_1) / (abs_average_1 + abs_average_2);
    float average_bias = abs(average_2 - average_1) / (abs_average_1 + abs_average_2);
    float max_error = 0;
    float average_error_2 = 0;
    float average_bias_2 = 0;
    for (int i = 0; i < param_count; i++)
    {
      float error = abs(grad_dr[i] - grad_ref[i]) / (abs_average_1 + abs_average_2);
      float bias = (grad_dr[i] - grad_ref[i]) / (abs_average_1 + abs_average_2);
      max_error = max(max_error, error);
      average_error_2 += error;
      average_bias_2 += bias;
    }
    average_error_2 /= param_count;
    average_bias_2 /= param_count;

    printf("TEST 9. Check position derivatives\n");
  
    printf(" 9.1. %-64s", "Relative difference in average PD value");
    if (average_error <= 0.1f)
      printf("passed    (%f)\n", average_error);
    else
      printf("FAILED, average_error = %f\n", average_error);
    
    printf(" 9.2. %-64s", "Relative bias in average PD");
    if (average_bias <= 0.1f)
      printf("passed    (%f)\n", average_bias);
    else
      printf("FAILED, average_bias = %f\n", average_bias);
    
    printf(" 9.3. %-64s", "Max relative difference in PD value");
    if (max_error <= 2)
      printf("passed    (%f)\n", max_error);
    else
      printf("FAILED, max_error = %f\n", max_error);
    
    printf(" 9.4. %-64s", "Average relative difference in PD value");
    if (average_error_2 <= 0.1f)
      printf("passed    (%f)\n", average_error_2);
    else
      printf("FAILED, average_error = %f\n", average_error_2);
    
    printf(" 9.5. %-64s", "Average relative bias in PD");
    if (abs(average_bias_2) <= 0.1f)
      printf("passed    (%f)\n", average_bias_2);
    else
      printf("FAILED, average_bias = %f\n", average_bias_2);
  }
}

void diff_render_test_10_optimize_sdf_finite_derivatives()
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
    LiteImage::SaveImage<float4>(("saves/test_dr_10_ref_"+std::to_string(i)+".bmp").c_str(), images_ref[i]); 
  }

  auto indexed_SBS = circle_smallest_scene();
  // randomize_color(indexed_SBS);
  randomize_distance(indexed_SBS, 0.25f);

  dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

  dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_FINITE_DIFF;
  dr_preset.dr_reconstruction_type = dr::DR_RECONSTRUCTION_TYPE_GEOMETRY;
  dr_preset.opt_iterations = 40;
  dr_preset.opt_lr = 0.02f;
  dr_preset.spp = 4;
  dr_preset.image_batch_size = 1;

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  std::vector<float> grad_dr(param_count, 0);
  std::vector<float> grad_ref(param_count, 0);

  {
    dr::MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
    image_res = dr_render.getLastImage(0);
    LiteImage::SaveImage<float4>("saves/test_dr_10_res.bmp", image_res);
  }

  printf("TEST 10. Differentiable render optimize SDF with finite derivatives\n");
  
  float psnr = image_metrics::PSNR(image_res, images_ref[0]);

  printf("10.1. %-64s", "SDF is reconstructed");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
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

  auto indexed_SBS = circle_smallest_scene();
  // randomize_color(indexed_SBS);
  randomize_distance(indexed_SBS, 0.25f);

  dr::MultiRendererDRPreset dr_preset = dr::getDefaultPresetDR();

  dr_preset.dr_diff_mode = dr::DR_DIFF_MODE_DEFAULT;
  dr_preset.dr_reconstruction_type = dr::DR_RECONSTRUCTION_TYPE_GEOMETRY;
  dr_preset.opt_iterations = 100;
  dr_preset.opt_lr = 0.01f;
  dr_preset.spp = 4;
  dr_preset.image_batch_size = 1;

  unsigned param_count = indexed_SBS.values_f.size() - 3 * 8 * indexed_SBS.nodes.size();
  unsigned param_offset = 0;

  std::vector<float> grad_dr(param_count, 0);
  std::vector<float> grad_ref(param_count, 0);

  {
    dr::MultiRendererDR dr_render;
    dr_render.SetReference(images_ref, view, proj);
    dr_render.OptimizeColor(dr_preset, indexed_SBS, false);
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


void perform_tests_diff_render(const std::vector<int> &test_ids)
{
  std::vector<int> tests = test_ids;

  std::vector<std::function<void(void)>> test_functions = {
      diff_render_test_1_enzyme_ad, diff_render_test_2_forward_pass, diff_render_test_3_optimize_color,
      diff_render_test_4_render_simple_scenes, diff_render_test_5_optimize_color_simpliest, diff_render_test_6_check_color_derivatives,
      diff_render_test_7_optimize_with_finite_diff, diff_render_test_8_optimize_with_lambert, diff_render_test_9_check_position_derivatives,
      diff_render_test_10_optimize_sdf_finite_derivatives, diff_render_test_11_optimize_smallest_scene};

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