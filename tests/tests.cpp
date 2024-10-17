#include "tests.h"
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
#include "../utils/iou.h"
#include "harmonic_function/any_polygon_common.h"
#include "render_settings.h"

#include <functional>
#include <cassert>
#include <chrono>
#include <optional>

std::string scenes_folder_path = "./";

void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum = 1)
{
  float fov_degrees = 60;
  float z_near = 0.1f;
  float z_far = 100.0f;
  float aspect   = 1.0f;
  auto proj      = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
  auto worldView = LiteMath::lookAt(pos, target, up);

  pRender->Render(image.data(), image.width(), image.height(), worldView, proj, preset, a_passNum);
}

void litert_test_1_framed_octree()
{
    auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());

    float3 mb1,mb2, ma1,ma2;
    cmesh4::get_bbox(mesh, &mb1, &mb2);
    cmesh4::rescale_mesh(mesh, float3(-0.9,-0.9,-0.9), float3(0.9,0.9,0.9));
    cmesh4::get_bbox(mesh, &ma1, &ma2);

    printf("total triangles %d\n", (int)mesh.TrianglesNum());
    printf("bbox [(%f %f %f)-(%f %f %f)] to [(%f %f %f)-(%f %f %f)]\n",
           mb1.x, mb1.y, mb1.z, mb2.x, mb2.y, mb2.z, ma1.x, ma1.y, ma1.z, ma2.x, ma2.y, ma2.z);

    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 9);
    std::vector<SdfFrameOctreeNode> frame_nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);

    unsigned W = 2048, H = 2048;
    LiteImage::Image2D<uint32_t> image(W, H);
    float timings[4] = {0,0,0,0};

    std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ST, 
                                        SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ST};

    std::vector<std::string> names = {"bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

    for (int i=0; i<presets_oi.size(); i++)
    {
      MultiRenderPreset preset = getDefaultPreset();
      preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
      preset.sdf_node_intersect = presets_oi[i];

      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene(frame_nodes);

  auto t1 = std::chrono::steady_clock::now();
      render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
      pRender->GetExecutionTime("CastRaySingleBlock", timings);
  auto t2 = std::chrono::steady_clock::now();

      float time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
      printf("%s rendered in %.1f ms. %d kRays/s\n", "SDF Framed Octree", time_ms, (int)((W * H) / time_ms));
      printf("CastRaySingleBlock took %.1f ms\n", timings[0]);

      LiteImage::SaveImage<uint32_t>(("saves/test_1_"+names[i]+".bmp").c_str(), image); 
    }
}

void litert_test_2_SVS()
{
    auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());

    float3 mb1,mb2, ma1,ma2;
    cmesh4::get_bbox(mesh, &mb1, &mb2);
    cmesh4::rescale_mesh(mesh, float3(-0.9,-0.9,-0.9), float3(0.9,0.9,0.9));
    cmesh4::get_bbox(mesh, &ma1, &ma2);

    printf("total triangles %d\n", (int)mesh.TrianglesNum());
    printf("bbox [(%f %f %f)-(%f %f %f)] to [(%f %f %f)-(%f %f %f)]\n",
           mb1.x, mb1.y, mb1.z, mb2.x, mb2.y, mb2.z, ma1.x, ma1.y, ma1.z, ma2.x, ma2.y, ma2.z);

    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 9);
    std::vector<SdfSVSNode> frame_nodes = sdf_converter::create_sdf_SVS(settings, mesh);

    unsigned W = 2048, H = 2048;
    LiteImage::Image2D<uint32_t> image(W, H);
    float timings[4] = {0,0,0,0};

    std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                                        SDF_OCTREE_NODE_INTERSECT_NEWTON, SDF_OCTREE_NODE_INTERSECT_BBOX};

    std::vector<std::string> names = {"bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

    for (int i=0; i<presets_oi.size(); i++)
    {
      MultiRenderPreset preset = getDefaultPreset();
      preset.render_mode = MULTI_RENDER_MODE_PHONG_NO_TEX;
      preset.sdf_node_intersect = presets_oi[i];

      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene(frame_nodes);

  auto t1 = std::chrono::steady_clock::now();
      render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
      pRender->GetExecutionTime("CastRaySingleBlock", timings);
  auto t2 = std::chrono::steady_clock::now();

      float time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
      printf("%s rendered in %.1f ms. %d kRays/s\n", "SDF Framed Octree", time_ms, (int)((W * H) / time_ms));
      printf("CastRaySingleBlock took %.1f ms\n", timings[0]);

      LiteImage::SaveImage<uint32_t>(("saves/test_2_"+names[i]+".bmp").c_str(), image); 
    }
}

void litert_test_3_SBS_verify()
{
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.9, -0.9, -0.9), float3(0.9, 0.9, 0.9));

  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
  SdfSBSHeader header_1_1{1,0,1,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_1_2{1,0,2,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_2_1{2,0,1,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_2_2{2,0,2,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_2_4{2,0,4,SDF_SBS_NODE_LAYOUT_DX};

  SdfSBS sbs_1_1 = sdf_converter::create_sdf_SBS(settings, header_1_1, mesh);
  SdfSBS sbs_1_2 = sdf_converter::create_sdf_SBS(settings, header_1_2, mesh);
  SdfSBS sbs_2_1 = sdf_converter::create_sdf_SBS(settings, header_2_1, mesh);
  SdfSBS sbs_2_2 = sdf_converter::create_sdf_SBS(settings, header_2_2, mesh);
  SdfSBS sbs_2_4 = sdf_converter::create_sdf_SBS(settings, header_2_4, mesh);

  std::vector<SdfSVSNode> svs_nodes = sdf_converter::create_sdf_SVS(settings, mesh);

  unsigned W = 1024, H = 1024;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);
  LiteImage::Image2D<uint32_t> svs_image(W, H);

  printf("TEST 3. SVS and SBS correctness\n");
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(mesh);
    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_reference.bmp", image); 
    ref_image = image;
  }
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(svs_nodes);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SVS.bmp", image); 
    svs_image = image;

    float psnr = image_metrics::PSNR(ref_image, image);

    printf("  3.1. %-64s", "[CPU] SVS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_1_1);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_1_1.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf("  3.2. %-64s", "[CPU] 1-voxel,1-byte SBS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);

    float svs_psnr = image_metrics::PSNR(svs_image, image);
    printf("  3.3. %-64s", "[CPU] 1-voxel,1-byte SBS matches SVS");
    if (svs_psnr >= 40)
      printf("passed\n");
    else
      printf("FAILED, psnr = %f\n", svs_psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_1_1);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_1_1.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf("  3.4. %-64s", "1-voxel,1-byte SBS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);

    float svs_psnr = image_metrics::PSNR(svs_image, image);
    printf("  3.5. %-64s", "1-voxel,1-byte SBS matches SVS");
    if (svs_psnr >= 40)
      printf("passed\n");
    else
      printf("FAILED, psnr = %f\n", svs_psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_1_2);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_1_2.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf("  3.6. %-64s", "1-voxel,2-byte SBS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_2_1);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_2_1.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf("  3.7. %-64s", "8-voxel,1-byte SBS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_2_2);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_2_2.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf("  3.8. %-64s", "8-voxel,2-byte SBS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_2_4);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_2_4.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf("  3.9. %-64s", "8-voxel,4-byte SBS and mesh image_metrics::PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
}

void litert_test_4_hydra_scene()
{
  printf("TEST 4. Rendering Hydra scene\n");
  printf("INSTANCING ON GPU IS BROKEN. SKIP\n");
  return;

  //create renderers for SDF scene and mesh scene
  const char *scene_name = "scenes/01_simple_scenes/instanced_objects.xml";
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  //const char *scene_name = "scenes/01_simple_scenes/bunny_cornell.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ANALYTIC;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("GPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->CreateSceneFromHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS, 
                                SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9));

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  //render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
  //render(ref_image, pRenderRef, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset);

  LiteImage::SaveImage<uint32_t>("saves/test_4_res.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_4_ref.bmp", ref_image);

  float psnr = image_metrics::PSNR(ref_image, image);

  printf("  4.1. %-64s", "mesh and SDF image_metrics::PSNR > 30 ");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void litert_test_5_interval_tracing()
{
  printf("TEST 5. Interval tracing\n");
  printf("INSTANCING ON GPU IS BROKEN. SKIP\n");
  return;

  //create renderers for SDF scene and mesh scene
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  const char *scene_name = "scenes/01_simple_scenes/instanced_objects.xml";
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  //const char *scene_name = "scenes/01_simple_scenes/bunny_cornell.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset_ref = getDefaultPreset();
  MultiRenderPreset preset_1 = getDefaultPreset();
  preset_1.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;
  MultiRenderPreset preset_2 = getDefaultPreset();
  preset_2.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_IT;
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("GPU");
  pRenderRef->SetPreset(preset_ref);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset_1);
  pRender->SetViewport(0,0,W,H);
  pRender->CreateSceneFromHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS,
                                SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9));

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  pRender->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset_1);
  pRender->Render(image_2.data(), image_2.width(), image_2.height(), m1, m2, preset_2);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset_ref);

  LiteImage::SaveImage<uint32_t>("saves/test_5_ST.bmp", image_1); 
  LiteImage::SaveImage<uint32_t>("saves/test_5_IT.bmp", image_2); 
  LiteImage::SaveImage<uint32_t>("saves/test_5_ref.bmp", ref_image);

  float psnr_1 = image_metrics::PSNR(image_1, image_2);
  float psnr_2 = image_metrics::PSNR(ref_image, image_2);

  printf("  5.1. %-64s", "mesh and SDF image_metrics::PSNR > 30 ");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  printf("  5.2. %-64s", "Interval and Sphere tracing image_metrics::PSNR > 45 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);
}

void litert_test_6_faster_bvh_build()
{
  //create renderers for SDF scene and mesh scene
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  const char *scene_name = "scenes/01_simple_scenes/teapot.xml";
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  //const char *scene_name = "scenes/01_simple_scenes/bunny_cornell.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset_ref = getDefaultPreset();
  MultiRenderPreset preset_1 = getDefaultPreset();
  preset_1.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
  MultiRenderPreset preset_2 = getDefaultPreset();
  preset_2.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("GPU");
  pRenderRef->SetPreset(preset_ref);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender_1 = CreateMultiRenderer("GPU");
  pRender_1->SetPreset(preset_1);
  pRender_1->SetViewport(0,0,W,H);
auto t1 = std::chrono::steady_clock::now();
  pRender_1->CreateSceneFromHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS,
                                  SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9));
auto t2 = std::chrono::steady_clock::now();
  float time_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

  auto pRender_2 = CreateMultiRenderer("GPU");
  pRender_2->SetPreset(preset_2);
  pRender_2->SetViewport(0,0,W,H);
auto t3 = std::chrono::steady_clock::now();
  pRender_2->CreateSceneFromHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS,
                                  SparseOctreeSettings(SparseOctreeBuildType::MESH_TLO, 9));
auto t4 = std::chrono::steady_clock::now();
  float time_2 = std::chrono::duration_cast<std::chrono::milliseconds>(t4 - t3).count();

  auto m1 = pRender_1->getWorldView();
  auto m2 = pRender_1->getProj();

  pRender_1->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset_1);
  pRender_2->Render(image_2.data(), image_2.width(), image_2.height(), m1, m2, preset_2);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset_ref);

  LiteImage::SaveImage<uint32_t>("saves/test_6_default.bmp", image_1); 
  LiteImage::SaveImage<uint32_t>("saves/test_6_mesh_tlo.bmp", image_2); 
  LiteImage::SaveImage<uint32_t>("saves/test_6_ref.bmp", ref_image);

  float psnr_1 = image_metrics::PSNR(image_1, image_2);
  float psnr_2 = image_metrics::PSNR(ref_image, image_2);
  printf("TEST 6. MESH_TLO SDF BVH build. default %.1f s, mesh TLO %.1f s\n", time_1/1000.0f, time_2/1000.0f);
  printf("  6.1. %-64s", "mesh and SDF image_metrics::PSNR > 30 ");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  printf("  6.2. %-64s", "DEFAULT and MESH_TLO image_metrics::PSNR > 45 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);
}

void litert_test_7_stub()
{

}

void litert_test_8_SDF_grid()
{
  //create renderers for SDF scene and mesh scene
  const char *scene_name = "scenes/01_simple_scenes/teapot.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ANALYTIC;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("GPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->CreateSceneFromHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_GRID, 
                                SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 7));

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset);

  LiteImage::SaveImage<uint32_t>("saves/test_8_res.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_8_ref.bmp", ref_image);

  float psnr = image_metrics::PSNR(ref_image, image);
  printf("TEST 8. Rendering SDF grid\n");
  printf("  8.1. %-64s", "mesh and SDF grid image_metrics::PSNR > 30 ");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void litert_test_9_mesh()
{
  //create renderers for SDF scene and mesh scene
  const char *scene_name = "scenes/01_simple_scenes/teapot.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("CPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();
  
  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset);

  LiteImage::SaveImage<uint32_t>("saves/test_9_res.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_9_ref.bmp", ref_image);

  float psnr = image_metrics::PSNR(ref_image, image);
  printf("TEST 9. Rendering simple mesh\n");
  printf("  9.1. %-64s", "CPU and GPU render image_metrics::PSNR > 45 ");
  if (psnr >= 45)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

// save and load octrees of all types
void litert_test_10_save_load()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::normalize_mesh(mesh);
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  std::vector<SdfFrameOctreeNode> frame_nodes;
  std::vector<SdfSVSNode> svs_nodes;
  SdfSBS sbs;

  load_sdf_frame_octree(frame_nodes, "saves/test_10_frame_octree.bin");
  load_sdf_SVS(svs_nodes, "saves/test_10_svs.bin");
  load_sdf_SBS(sbs, "saves/test_10_sbs.bin");

  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;

  LiteImage::Image2D<uint32_t> image_ref(W, H);
  auto pRender_ref = CreateMultiRenderer("GPU");
  pRender_ref->SetPreset(preset);
  pRender_ref->SetScene(mesh);
  render(image_ref, pRender_ref, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_ref.bmp", image_ref);

  LiteImage::Image2D<uint32_t> image_2(W, H);
  auto pRender_2 = CreateMultiRenderer("GPU");
  pRender_2->SetPreset(preset);
  pRender_2->SetScene(frame_nodes);
  render(image_2, pRender_2, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_frame_octree.bmp", image_2);

  LiteImage::Image2D<uint32_t> image_3(W, H);
  auto pRender_3 = CreateMultiRenderer("GPU");
  pRender_3->SetPreset(preset);
  pRender_3->SetScene(svs_nodes);
  render(image_3, pRender_3, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_svs.bmp", image_3);

  LiteImage::Image2D<uint32_t> image_4(W, H);
  auto pRender_4 = CreateMultiRenderer("GPU");
  pRender_4->SetPreset(preset);
  pRender_4->SetScene(sbs);
  render(image_4, pRender_4, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_sbs.bmp", image_4);

  LiteImage::Image2D<uint32_t> image_5(W, H);
  auto pRender_5 = CreateMultiRenderer("GPU");
  pRender_5->SetPreset(preset);
  pRender_5->LoadSceneHydra("scenes/02_sdf_scenes/test_10.xml");
  render(image_5, pRender_5, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_hydra_scene.bmp", image_5);

  float psnr_2 = image_metrics::PSNR(image_ref, image_2);
  float psnr_3 = image_metrics::PSNR(image_ref, image_3);
  float psnr_4 = image_metrics::PSNR(image_ref, image_4);
  float psnr_5 = image_metrics::PSNR(image_ref, image_5);

  printf(" 10.2. %-64s", "SDF Framed Octree ");
  if (psnr_2 >= 25)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);

  printf(" 10.3. %-64s", "SDF Sparse Voxel Set ");
  if (psnr_3 >= 25)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);

  printf(" 10.4. %-64s", "SDF Sparse Brick Set ");
  if (psnr_4 >= 25)
    printf("passed    (%.2f)\n", psnr_4);
  else
    printf("FAILED, psnr = %f\n", psnr_4);
  
  printf(" 10.5. %-64s", "SDF Scene loaded from Hydra scene");
  if (psnr_5 >= 25)
    printf("passed    (%.2f)\n", psnr_5);
  else
    printf("FAILED, psnr = %f\n", psnr_5);
}

static double urand(double from=0, double to=1)
{
  return ((double)rand() / RAND_MAX) * (to - from) + from;
}

void litert_test_11_stub()
{

}

void litert_test_12_stub()
{

}

void litert_test_13_stub()
{

}

void litert_test_14_octree_nodes_removal()
{

}

void litert_test_15_frame_octree_nodes_removal()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::normalize_mesh(mesh);
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  std::vector<SdfFrameOctreeNode> octree_nodes_ref;
  std::vector<SdfFrameOctreeNode> octree_nodes_7;
  std::vector<SdfFrameOctreeNode> octree_nodes_8;
  const unsigned level_6_nodes = 21603;

  {
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 6);
    octree_nodes_ref = sdf_converter::create_sdf_frame_octree(settings, mesh);
    sdf_converter::frame_octree_limit_nodes(octree_nodes_ref, level_6_nodes, false);
  }

  {
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 7);
    octree_nodes_7 = sdf_converter::create_sdf_frame_octree(settings, mesh);
    sdf_converter::frame_octree_limit_nodes(octree_nodes_7, level_6_nodes, false);
  }

  {
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
    octree_nodes_8 = sdf_converter::create_sdf_frame_octree(settings, mesh);
    sdf_converter::frame_octree_limit_nodes(octree_nodes_8, level_6_nodes, false);
  }

  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> image_3(W, H);

  {
    auto pRender_1 = CreateMultiRenderer("GPU");
    pRender_1->SetPreset(preset);
    pRender_1->SetScene(octree_nodes_ref);
    render(image_1, pRender_1, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_15_ref.bmp", image_1);
  }

  {
    auto pRender_2 = CreateMultiRenderer("GPU");
    pRender_2->SetPreset(preset);
    pRender_2->SetScene(octree_nodes_7);
    render(image_2, pRender_2, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_15_trimmed_7.bmp", image_2);
  }

  {
    auto pRender_3 = CreateMultiRenderer("GPU");
    pRender_3->SetPreset(preset);
    pRender_3->SetScene(octree_nodes_8);
    render(image_3, pRender_3, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_15_trimmed_8.bmp", image_3);
  }

  float psnr_1 = image_metrics::PSNR(image_1, image_2);
  float psnr_2 = image_metrics::PSNR(image_1, image_3);

  printf("TEST 15. Frame octree nodes removal\n");
  printf(" 15.1. %-64s", "octrees have the same node count ");
  if (octree_nodes_ref.size() == octree_nodes_7.size() && octree_nodes_ref.size() == octree_nodes_8.size())
    printf("passed\n");
  else
    printf("FAILED, %d, %d, %d\n", (int)octree_nodes_ref.size(), (int)octree_nodes_7.size(), (int)octree_nodes_8.size());
  printf(" 15.2. %-64s", "clear level 7 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);

  printf(" 15.3. %-64s", "clear levels 7 and 8 ");
  if (psnr_2 >= 45)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
}

void litert_test_16_SVS_nodes_removal()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::normalize_mesh(mesh);
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  std::vector<SdfSVSNode> octree_nodes_ref;
  std::vector<SdfSVSNode> octree_nodes_7;
  std::vector<SdfSVSNode> octree_nodes_8;
  const unsigned level_6_nodes = 11215;

  {
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 6);
    std::vector<SdfFrameOctreeNode> nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
    sdf_converter::frame_octree_limit_nodes(nodes, level_6_nodes, true);
    sdf_converter::frame_octree_to_SVS_rec(nodes, octree_nodes_ref, 0, uint3(0,0,0), 1);
  }

  {
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 7);
    std::vector<SdfFrameOctreeNode> nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
    sdf_converter::frame_octree_limit_nodes(nodes, level_6_nodes, true);
    sdf_converter::frame_octree_to_SVS_rec(nodes, octree_nodes_7, 0, uint3(0,0,0), 1);
  }

  {
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
    std::vector<SdfFrameOctreeNode> nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
    sdf_converter::frame_octree_limit_nodes(nodes, level_6_nodes, true);
    sdf_converter::frame_octree_to_SVS_rec(nodes, octree_nodes_8, 0, uint3(0,0,0), 1);
  }

  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> image_3(W, H);

  {
    auto pRender_1 = CreateMultiRenderer("GPU");
    pRender_1->SetPreset(preset);
    pRender_1->SetScene(octree_nodes_ref);
    render(image_1, pRender_1, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_16_ref.bmp", image_1);
  }

  {
    auto pRender_2 = CreateMultiRenderer("GPU");
    pRender_2->SetPreset(preset);
    pRender_2->SetScene(octree_nodes_7);
    render(image_2, pRender_2, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_16_trimmed_7.bmp", image_2);
  }

  {
    auto pRender_3 = CreateMultiRenderer("GPU");
    pRender_3->SetPreset(preset);
    pRender_3->SetScene(octree_nodes_8);
    render(image_3, pRender_3, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_16_trimmed_8.bmp", image_3);
  }

  float psnr_1 = image_metrics::PSNR(image_1, image_2);
  float psnr_2 = image_metrics::PSNR(image_1, image_3);

  printf("TEST 16. SVS nodes removal\n");
  printf(" 16.1. %-64s", "octrees have correct node count ");
  if (octree_nodes_ref.size() >= octree_nodes_7.size() && octree_nodes_ref.size() >= octree_nodes_8.size())
    printf("passed\n");
  else
    printf("FAILED, %d, %d, %d\n", (int)octree_nodes_ref.size(), (int)octree_nodes_7.size(), (int)octree_nodes_8.size());
  printf(" 16.2. %-64s", "clear level 7 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);

  printf(" 16.3. %-64s", "clear levels 7 and 8 ");
  if (psnr_2 >= 45)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
}

void litert_test_17_all_types_sanity_check()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::normalize_mesh(mesh);

  unsigned W = 512, H = 512;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;

  std::vector<LiteImage::Image2D<uint32_t>> image_ref(3, LiteImage::Image2D<uint32_t>(W, H));
  std::vector<LiteImage::Image2D<uint32_t>> image_1(3, LiteImage::Image2D<uint32_t>(W, H));
  std::vector<LiteImage::Image2D<uint32_t>> image_2(3, LiteImage::Image2D<uint32_t>(W, H));
  std::vector<LiteImage::Image2D<uint32_t>> image_3(3, LiteImage::Image2D<uint32_t>(W, H));
  std::vector<LiteImage::Image2D<uint32_t>> image_4(3, LiteImage::Image2D<uint32_t>(W, H));
  std::vector<LiteImage::Image2D<uint32_t>> image_5(3, LiteImage::Image2D<uint32_t>(W, H));
  
  std::vector<std::string> modes = {"CPU", "GPU", "RTX"};

  for (int m = 0; m < 3; m++)
  {
    {
      auto pRender = CreateMultiRenderer(modes[m].c_str());
      pRender->SetPreset(preset);
      pRender->SetScene(mesh);
      render(image_ref[m], pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
      LiteImage::SaveImage<uint32_t>("saves/test_17_ref.bmp", image_ref[m]);
    }

    {
      auto grid = sdf_converter::create_sdf_grid(GridSettings(64), mesh);
      auto pRender = CreateMultiRenderer(modes[m].c_str());
      pRender->SetPreset(preset);
      pRender->SetScene(grid);
      render(image_1[m], pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
      LiteImage::SaveImage<uint32_t>("saves/test_17_grid.bmp", image_1[m]);    
    }

    {
      auto octree = sdf_converter::create_sdf_frame_octree(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64*64*64), mesh);
      auto pRender = CreateMultiRenderer(modes[m].c_str());
      pRender->SetPreset(preset);
      pRender->SetScene(octree);
      render(image_3[m], pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
      LiteImage::SaveImage<uint32_t>("saves/test_17_frame_octree.bmp", image_3[m]);
    }

    {
      auto octree = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64*64*64), mesh);
      auto pRender = CreateMultiRenderer(modes[m].c_str());
      pRender->SetPreset(preset);
      pRender->SetScene(octree);
      render(image_4[m], pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
      LiteImage::SaveImage<uint32_t>("saves/test_17_SVS.bmp", image_4[m]);
    }

    {
      SdfSBSHeader header;
      header.brick_size = 2;
      header.brick_pad = 0;
      header.bytes_per_value = 1;
      header.aux_data = SDF_SBS_NODE_LAYOUT_DX;

      auto sbs = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64*64*64), header, mesh);
      auto pRender = CreateMultiRenderer(modes[m].c_str());
      pRender->SetPreset(preset);
      pRender->SetScene(sbs);
      render(image_5[m], pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
      LiteImage::SaveImage<uint32_t>("saves/test_17_SBS.bmp", image_5[m]);
    }
  }

  float psnr_1_gpu = image_metrics::PSNR(image_ref[1], image_ref[0]);
  float psnr_1_rtx = image_metrics::PSNR(image_ref[2], image_ref[0]);

  float psnr_2_0   = image_metrics::PSNR(image_1[0], image_ref[0]);
  float psnr_2_1   = image_metrics::PSNR(image_1[1], image_ref[0]);
  float psnr_2_2   = image_metrics::PSNR(image_1[2], image_ref[0]);
  float psnr_2_gpu = image_metrics::PSNR(image_1[1], image_1[0]);
  float psnr_2_rtx = image_metrics::PSNR(image_1[2], image_1[0]);

  float psnr_4_0   = image_metrics::PSNR(image_3[0], image_ref[0]);
  float psnr_4_1   = image_metrics::PSNR(image_3[1], image_ref[0]);
  float psnr_4_2   = image_metrics::PSNR(image_3[2], image_ref[0]);
  float psnr_4_gpu = image_metrics::PSNR(image_3[1], image_3[0]);
  float psnr_4_rtx = image_metrics::PSNR(image_3[2], image_3[0]);

  float psnr_5_0   = image_metrics::PSNR(image_4[0], image_ref[0]);
  float psnr_5_1   = image_metrics::PSNR(image_4[1], image_ref[0]);
  float psnr_5_2   = image_metrics::PSNR(image_4[2], image_ref[0]);
  float psnr_5_gpu = image_metrics::PSNR(image_4[1], image_4[0]);
  float psnr_5_rtx = image_metrics::PSNR(image_4[2], image_4[0]);

  float psnr_6_0   = image_metrics::PSNR(image_5[0], image_ref[0]);
  float psnr_6_1   = image_metrics::PSNR(image_5[1], image_ref[0]);
  float psnr_6_2   = image_metrics::PSNR(image_5[2], image_ref[0]);
  float psnr_6_gpu = image_metrics::PSNR(image_5[1], image_5[0]);
  float psnr_6_rtx = image_metrics::PSNR(image_5[2], image_5[0]);

  printf("TEST 17. all types sanity check\n");
  
  printf("17.1.1 %-64s", "Mesh CPU - GPU");
  if (psnr_1_gpu >= 50)
    printf("passed    (%.2f)\n", psnr_1_gpu);
  else
    printf("FAILED, psnr = %f\n", psnr_1_gpu);
  
  printf("17.1.2 %-64s", "Mesh CPU - RTX");
  if (psnr_1_gpu >= 50)
    printf("passed    (%.2f)\n", psnr_1_rtx);
  else
    printf("FAILED, psnr = %f\n", psnr_1_rtx);

  printf("17.2.1 %-64s", "Grid CPU - Reference");
  if (psnr_2_0 >= 25)
    printf("passed    (%.2f)\n", psnr_2_0);
  else
    printf("FAILED, psnr = %f\n", psnr_2_0);
  
  printf("17.2.2 %-64s", "Grid GPU - Reference");
  if (psnr_2_1 >= 25)
    printf("passed    (%.2f)\n", psnr_2_1);
  else
    printf("FAILED, psnr = %f\n", psnr_2_1);

  printf("17.2.3 %-64s", "Grid RTX - Reference");
  if (psnr_2_2 >= 25)
    printf("passed    (%.2f)\n", psnr_2_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2_2);
  
  printf("17.2.4 %-64s", "Grid CPU - GPU");
  if (psnr_2_gpu >= 50)
    printf("passed    (%.2f)\n", psnr_2_gpu);
  else
    printf("FAILED, psnr = %f\n", psnr_2_gpu);
  
  printf("17.2.5 %-64s", "Grid CPU - RTX");
  if (psnr_2_rtx >= 50)
    printf("passed    (%.2f)\n", psnr_2_rtx);
  else
    printf("FAILED, psnr = %f\n", psnr_2_rtx);
  
  printf("17.4.1 %-64s", "Frame Octree CPU - Reference");
  if (psnr_4_0 >= 30)
    printf("passed    (%.2f)\n", psnr_4_0);
  else 
    printf("FAILED, psnr = %f\n", psnr_4_0);
  
  printf("17.4.2 %-64s", "Frame Octree GPU - Reference");
  if (psnr_4_1 >= 30)
    printf("passed    (%.2f)\n", psnr_4_1);
  else
    printf("FAILED, psnr = %f\n", psnr_4_1);
  
  printf("17.4.3 %-64s", "Frame Octree RTX - Reference");
  if (psnr_4_2 >= 30)
    printf("passed    (%.2f)\n", psnr_4_2);
  else
    printf("FAILED, psnr = %f\n", psnr_4_2);
  
  printf("17.4.4 %-64s", "Frame Octree CPU - GPU");
  if (psnr_4_gpu >= 50)
    printf("passed    (%.2f)\n", psnr_4_gpu);
  else
    printf("FAILED, psnr = %f\n", psnr_4_gpu);
  
  printf("17.4.5 %-64s", "Frame Octree CPU - RTX");
  if (psnr_4_rtx >= 50)
    printf("passed    (%.2f)\n", psnr_4_rtx);
  else
    printf("FAILED, psnr = %f\n", psnr_4_rtx);

  printf("17.5.1 %-64s", "SVS CPU - Reference");
  if (psnr_5_0 >= 30)
    printf("passed    (%.2f)\n", psnr_5_0);
  else
    printf("FAILED, psnr = %f\n", psnr_5_0);
  
  printf("17.5.2 %-64s", "SVS GPU - Reference");
  if (psnr_5_1 >= 30)
    printf("passed    (%.2f)\n", psnr_5_1);
  else
    printf("FAILED, psnr = %f\n", psnr_5_1);
  
  printf("17.5.3 %-64s", "SVS RTX - Reference");
  if (psnr_5_2 >= 30)
    printf("passed    (%.2f)\n", psnr_5_2);
  else
    printf("FAILED, psnr = %f\n", psnr_5_2);
  
  printf("17.5.4 %-64s", "SVS CPU - GPU");
  if (psnr_5_gpu >= 50)
    printf("passed    (%.2f)\n", psnr_5_gpu);
  else
    printf("FAILED, psnr = %f\n", psnr_5_gpu);
  
  printf("17.5.5 %-64s", "SVS CPU - RTX");
  if (psnr_5_rtx >= 50)
    printf("passed    (%.2f)\n", psnr_5_rtx);
  else
    printf("FAILED, psnr = %f\n", psnr_5_rtx);

  printf("17.6.1 %-64s", "SBS CPU - Reference");
  if (psnr_6_0 >= 30)
    printf("passed    (%.2f)\n", psnr_6_0);
  else
    printf("FAILED, psnr = %f\n", psnr_6_0);
  
  printf("17.6.2 %-64s", "SBS GPU - Reference");
  if (psnr_6_1 >= 30)
    printf("passed    (%.2f)\n", psnr_6_1);
  else
    printf("FAILED, psnr = %f\n", psnr_6_1);
  
  printf("17.6.3 %-64s", "SBS RTX - Reference");
  if (psnr_6_2 >= 30)
    printf("passed    (%.2f)\n", psnr_6_2);
  else
    printf("FAILED, psnr = %f\n", psnr_6_2);
  
  printf("17.6.4 %-64s", "SBS CPU - GPU");
  if (psnr_6_gpu >= 50)
    printf("passed    (%.2f)\n", psnr_6_gpu);
  else
    printf("FAILED, psnr = %f\n", psnr_6_gpu);
  
  printf("17.6.5 %-64s", "SBS CPU - RTX");
  if (psnr_6_rtx >= 50) 
    printf("passed    (%.2f)\n", psnr_6_rtx);
  else
    printf("FAILED, psnr = %f\n", psnr_6_rtx);
}

void litert_test_18_mesh_normalization()
{
  //create renderers for SDF scene and mesh scene
  cmesh4::SimpleMesh mesh, mesh_filled, mesh_compressed, mesh_n_fixed, mesh_normalized;
  //mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "saves/dragon/mesh.vsgf").c_str());
  mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, 0.999f*float3(-1, -1, -1), 0.999f*float3(1, 1, 1));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();

  LiteImage::Image2D<uint32_t> ref_image(W, H);
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> image_3(W, H);
  LiteImage::Image2D<uint32_t> image_4(W, H);

  LiteImage::Image2D<uint32_t> ref_sdf(W, H);
  LiteImage::Image2D<uint32_t> sdf_1(W, H);
  LiteImage::Image2D<uint32_t> sdf_2(W, H);
  LiteImage::Image2D<uint32_t> sdf_3(W, H);
  LiteImage::Image2D<uint32_t> sdf_4(W, H);

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    render(ref_image, pRender, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_1ref.bmp", ref_image);

    auto sdf = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9), mesh);
    auto pRenderSdf = CreateMultiRenderer("GPU");
    pRenderSdf->SetPreset(preset);
    pRenderSdf->SetViewport(0,0,W,H);
    pRenderSdf->SetScene(sdf);
    render(ref_sdf, pRenderSdf, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_sdf_1ref.bmp", ref_sdf);
  }

  {
    int ind = -1;
    bool fl = false;
    mesh_filled = cmesh4::check_watertight_mesh(mesh, true) ? mesh : cmesh4::removing_holes(mesh, ind, fl);
    mesh_filled = mesh;
    printf("mesh_filled size = %d\n", (int)mesh_filled.TrianglesNum());

    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh_filled);
    render(image_1, pRender, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_2removed_holes.bmp", image_1);


    auto sdf = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9), mesh_filled);
    auto pRenderSdf = CreateMultiRenderer("GPU");
    pRenderSdf->SetPreset(preset);
    pRenderSdf->SetViewport(0,0,W,H);
    pRenderSdf->SetScene(sdf);
    render(sdf_1, pRenderSdf, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_sdf_2removed_holes.bmp", sdf_1);
  }

  {
    mesh_compressed = mesh_filled;
    cmesh4::compress_close_vertices(mesh_compressed, 1e-9f, true);
    printf("mesh_compressed size = %d\n", (int)mesh_compressed.TrianglesNum());

    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh_compressed);
    render(image_2, pRender, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_3compressed.bmp", image_2);


    auto sdf = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9), mesh_compressed);
    auto pRenderSdf = CreateMultiRenderer("GPU");
    pRenderSdf->SetPreset(preset);
    pRenderSdf->SetViewport(0,0,W,H);
    pRenderSdf->SetScene(sdf);
    render(sdf_2, pRenderSdf, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_sdf_3compressed.bmp", sdf_2);
  }

  {
    mesh_n_fixed = mesh_compressed;
    cmesh4::fix_normals(mesh_n_fixed, true);
    printf("mesh_compressed size = %d\n", (int)mesh_n_fixed.TrianglesNum());

    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh_n_fixed);
    render(image_3, pRender, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_4n_fixed.bmp", image_3);


    auto sdf = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9), mesh_n_fixed);
    auto pRenderSdf = CreateMultiRenderer("GPU");
    pRenderSdf->SetPreset(preset);
    pRenderSdf->SetViewport(0,0,W,H);
    pRenderSdf->SetScene(sdf);
    render(sdf_3, pRenderSdf, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_18_sdf_4n_fixed.bmp", sdf_3);
  }

  float psnr_1 = image_metrics::PSNR(ref_image, image_1);
  float psnr_2 = image_metrics::PSNR(ref_image, image_2);
  float psnr_3 = image_metrics::PSNR(ref_image, image_3);

  float psnr_sdf_1 = image_metrics::PSNR(ref_sdf, sdf_1);
  float psnr_sdf_2 = image_metrics::PSNR(ref_sdf, sdf_2);
  float psnr_sdf_3 = image_metrics::PSNR(ref_sdf, sdf_3);

  printf("TEST 18. Mesh normalization\n");

  printf(" 18.1. %-64s", "Removing holes left mesh intact");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1); 

  printf(" 18.2. %-64s", "Removing holes left SDF intact");
  if (psnr_sdf_1 >= 45)
    printf("passed    (%.2f)\n", psnr_sdf_1);
  else
    printf("FAILED, psnr = %f\n", psnr_sdf_1);   

  printf(" 18.1. %-64s", "Removing holes left mesh intact");
  if (psnr_2 >= 45)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2); 

  printf(" 18.2. %-64s", "Removing holes left SDF intact");
  if (psnr_sdf_2 >= 45)
    printf("passed    (%.2f)\n", psnr_sdf_2);
  else
    printf("FAILED, psnr = %f\n", psnr_sdf_2);   

  printf(" 18.1. %-64s", "Removing holes left mesh intact");
  if (psnr_3 >= 45)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3); 

  printf(" 18.2. %-64s", "Removing holes left SDF intact");
  if (psnr_sdf_3 >= 45)
    printf("passed    (%.2f)\n", psnr_sdf_3);
  else
    printf("FAILED, psnr = %f\n", psnr_sdf_3);
}

void litert_test_19_marching_cubes()
{
  printf("TEST 19. Marching cubes\n");

  cmesh4::MultithreadedDensityFunction sdf = [](const float3 &pos, unsigned idx) -> float
  {
    float3 rp = float3(pos.x, pos.y, pos.z);
    const float radius = 0.95f;
    const float max_A = 0.125f;
    float l = sqrt(rp.x * rp.x + rp.z * rp.z);
    float A = max_A*(1.0f - l / radius);
    float c = A*(cos(10*M_PI*l) + 1.1f) - std::abs(rp.y) - 1e-6f;
    return c;
  };
  cmesh4::MarchingCubesSettings settings;
  settings.size = LiteMath::uint3(256, 256, 256);
  settings.min_pos = float3(-1, -1, -1);
  settings.max_pos = float3(1, 1, 1);
  settings.iso_level = 0.0f;

  cmesh4::SimpleMesh mesh = cmesh4::create_mesh_marching_cubes(settings, sdf, 15);

  unsigned W = 4096, H = 4096;
  MultiRenderPreset preset = getDefaultPreset();
  preset.normal_mode = NORMAL_MODE_VERTEX;
  LiteImage::Image2D<uint32_t> image_1(W, H);
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    render(image_1, pRender, float3(3, 1, 0), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_19_1.bmp", image_1);
  }
}

void litert_test_20_radiance_fields()
{
  printf("TEST 20. Radiance fields\n");
  printf("DISABLED\n");
  return;

  const char *scene_name = "scenes/02_sdf_scenes/relu_fields.xml";
  unsigned W = 1024, H = 1024;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_RF;
  LiteImage::Image2D<uint32_t> image(W, H);

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->LoadScene((scenes_folder_path+scene_name).c_str());

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  preset.render_mode = MULTI_RENDER_MODE_RF;
  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  LiteImage::SaveImage<uint32_t>("saves/test_20_rf.bmp", image); 

  preset.render_mode = MULTI_RENDER_MODE_RF_DENSITY;
  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  LiteImage::SaveImage<uint32_t>("saves/test_20_rf_density.bmp", image); 

  preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  LiteImage::SaveImage<uint32_t>("saves/test_20_depth.bmp", image); 
}

void litert_test_21_rf_to_mesh()
{
  printf("TEST 21. Radiance field to mesh\n");
  printf("DISABLED\n");
  return;

  const char *scene_name = "scenes/02_sdf_scenes/relu_fields.xml";
  RFScene scene;
  load_rf_scene(scene, "scenes/02_sdf_scenes/model.dat");

  cmesh4::MultithreadedDensityFunction sdf = [&scene](const float3 &pos, unsigned idx) -> float
  {
    if (pos.x < 0.01f || pos.x > 0.99f || pos.y < 0.01f || pos.y > 0.99f || pos.z < 0.01f || pos.z > 0.99f)
      return -1.0f;
    float3 f_idx = scene.size*pos;
    uint3 idx0 = uint3(f_idx);
    float3 dp = f_idx - float3(idx0);
    float values[8];
    for (int i = 0; i < 8; i++)
    {
      uint3 idx = clamp(idx0 + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1), uint3(0u), uint3((unsigned)(scene.size - 1)));
      values[i] = scene.data[CellSize*(idx.z*scene.size*scene.size + idx.y*scene.size + idx.x)];
    }

    //bilinear sampling
    return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
           (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
           (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
           (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
           (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
           (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
           (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
           (  dp.x)*(  dp.y)*(  dp.z)*values[7];
  };
  cmesh4::MarchingCubesSettings settings;
  settings.size = LiteMath::uint3(1024, 1024, 1024);
  settings.min_pos = float3(-1, -1, -1);
  settings.max_pos = float3(1, 1, 1);
  settings.iso_level = 0.5f;

  cmesh4::SimpleMesh mesh = cmesh4::create_mesh_marching_cubes(settings, sdf, 15);

  unsigned W = 2048, H = 2048;
  MultiRenderPreset preset = getDefaultPreset();
  preset.normal_mode = NORMAL_MODE_VERTEX;
  float4x4 m1, m2;
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> image_3(W, H);

  {
    preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_RF;

    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->LoadScene((scenes_folder_path+scene_name).c_str());
    m1 = pRender->getWorldView();
    m2 = pRender->getProj();

    pRender->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset);
    LiteImage::SaveImage<uint32_t>("saves/test_21_rf.bmp", image_1); 
  }

  {
    preset = getDefaultPreset();
    preset.normal_mode = NORMAL_MODE_VERTEX;
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    pRender->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset);
    LiteImage::SaveImage<uint32_t>("saves/test_21_mesh.bmp", image_1);
  }

  {
    auto octree = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 10), mesh);
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(octree);
    pRender->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset);
    LiteImage::SaveImage<uint32_t>("saves/test_21_SVS.bmp", image_1);
  }
}

float2 get_quality(sdf_converter::MultithreadedDistanceFunction sdf, SdfGridView grid, unsigned points = 250000)
{
  long double sum = 0;
  long double sum_abs = 0;

  auto grid_sdf = get_SdfGridFunction(grid);

  uint4 e_mat = uint4(0.0f, 0.0f, 0.0f, 0.0f);

  for (unsigned i = 0; i < points; i++)
  {
    float3 p = float3(urand(), urand(), urand())*2.0f - 1.0f;
    float d1 = sdf(p, 0);
    float d2 = grid_sdf->eval_distance(p);

    sum += d1-d2;
    sum_abs += std::abs(d1-d2);

    e_mat.x += d1 > 0 && d2 > 0;
    e_mat.y += d1 > 0 && d2 < 0;
    e_mat.z += d1 < 0 && d2 > 0;
    e_mat.w += d1 < 0 && d2 < 0;
  }

  //printf("E: %u, %u, %u, %u\n", e_mat.x, e_mat.y, e_mat.z, e_mat.w);
  //printf("IoU: %f\n", float(e_mat.w)/float(e_mat.y+e_mat.z+e_mat.w));
  //printf("V: %f\n", float(e_mat.y+e_mat.w)/float(e_mat.z+e_mat.w));

  return float2(sum/points, sum_abs/points);
}

  float sdBox(float3 p)
  {
    float3 q = abs(p) - float3(1.0f);
    return length(LiteMath::max(q, float3(0.0f))) + LiteMath::min(LiteMath::max(q.x, LiteMath::max(q.y, q.z)), 0.0f);
  }

  float sdMengerSponge(float3 p)
  {
    float d = sdBox(p);
    float3 res = float3(d, 1.0f, 0.0f);

    int Iterations = 2;
    float s = 1.0f;
    for (int m = 0; m < Iterations; m++)
    {
      float3 a = mod(p * s, float3(2.0f)) - 1.0f;
      s *= 3.0f;
      float3 r = abs(1.0f - 3.0f * abs(a));

      float da = LiteMath::max(r.x, r.y);
      float db = LiteMath::max(r.y, r.z);
      float dc = LiteMath::max(r.z, r.x);
      float c = (LiteMath::min(da, LiteMath::min(db, dc)) - 1.0f) / s;

      d = std::max(d, c);
    }

    return d;
  }

void litert_test_22_sdf_grid_smoothing()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));
  unsigned W = 2048, H = 2048;
  MultiRenderPreset preset = getDefaultPreset();

    unsigned grid_size = 65;
    unsigned max_threads = 15;
    float noise = 0.03f;

    std::vector<MeshBVH> bvh(max_threads);
    for (unsigned i = 0; i < max_threads; i++)
      bvh[i].init(mesh);
    auto real_sdf = [&](const float3 &p, unsigned idx) -> float 
    /*{
      return sdMengerSponge(1.5f*float3((p.x+p.z)/sqrt(2), p.y, (p.x-p.z)/sqrt(2)))/1.5f;
    };*/
    { return bvh[idx].get_signed_distance(p); /*return sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - 0.3;*/ };

    //FILE *f = fopen("./hello_1.txt", "w");
    
    auto noisy_sdf = [&](const float3 &p, unsigned idx) -> float { 
      auto x = (fmod(p.x * 153 + p.y * 427 + p.z * 311, 2.0) - 1) * 0.003;
      //fprintf (f, "%f %f %f, %f - %f\n", p.x, p.y, p.z, bvh[idx].get_signed_distance(p), x);
      return real_sdf(p, idx) + x/*+ urand(-1 - 2.0f/grid_size, 1 - 2.0f/grid_size)*noise*/; };


  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> image_3(W, H);

  {
    preset = getDefaultPreset();
    preset.normal_mode = NORMAL_MODE_VERTEX;
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_22_mesh.bmp", image_1);
  } 

  auto grid = sdf_converter::create_sdf_grid(GridSettings(grid_size), real_sdf, max_threads);
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(grid);
    render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_22_grid.bmp", image_1);
  }
  float2 q_best = get_quality(real_sdf, grid);
  //printf("q_best = %f, %f\n", q_best.x, q_best.y);

  auto noisy_grid = sdf_converter::create_sdf_grid(GridSettings(grid_size), noisy_sdf, max_threads);
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(noisy_grid);
    render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_22_grid_noisy.bmp", image_1);
  }
  float2 q_noisy = get_quality(real_sdf, noisy_grid);
  //printf("q_noisy = %f, %f\n", q_noisy.x, q_noisy.y);

  float lambda = 0.3f;
  auto smoothed_grid = sdf_converter::sdf_grid_smoother(noisy_grid, 1, 0.001, lambda, 100);
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(smoothed_grid);
    render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_22_grid_smoothed.bmp", image_1);    
  }
  //printf("\nlambda = %f\n", lambda);
  float2 q_smoothed = get_quality(real_sdf, smoothed_grid);
  //printf("q_smoothed = %f, %f\n", q_smoothed.x, q_smoothed.y);
}

void litert_test_23_textured_sdf()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 9);
  auto textured_octree = sdf_converter::create_sdf_frame_octree_tex(settings, mesh);

  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);
  
  LiteImage::Image2D<uint32_t> image_tc(W, H);
  LiteImage::Image2D<uint32_t> ref_image_tc(W, H);

  LiteImage::Image2D<uint32_t> image_tex(W, H);
  LiteImage::Image2D<uint32_t> ref_image_tex(W, H);

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    render(ref_image, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }
  
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(textured_octree);
    render(image, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }
  
  preset.render_mode = MULTI_RENDER_MODE_TEX_COORDS;
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    render(ref_image_tc, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }
  
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(textured_octree);
    render(image_tc, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }
  
  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
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
    render(ref_image_tex, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
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

    pRender->SetScene(textured_octree);
    render(image_tex, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }

  LiteImage::SaveImage<uint32_t>("saves/test_23_mesh.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_23_textured_sdf.bmp", ref_image);
  LiteImage::SaveImage<uint32_t>("saves/test_23_tc_mesh.bmp", image_tc);
  LiteImage::SaveImage<uint32_t>("saves/test_23_tc_textured_sdf.bmp", ref_image_tc);
  LiteImage::SaveImage<uint32_t>("saves/test_23_tex_mesh.bmp", ref_image_tex);
  LiteImage::SaveImage<uint32_t>("saves/test_23_tex_textured_sdf.bmp", image_tex);

  float psnr = image_metrics::PSNR(ref_image, image);
  float psnr_tc = image_metrics::PSNR(ref_image_tc, image_tc);
  float psnr_tex = image_metrics::PSNR(ref_image_tex, image_tex);
  printf("TEST 23. Textured SDF\n");

  printf(" 23.1. %-64s", "Surface of textured SDF is close to mesh surface");
  if (psnr >= 35)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);  

  printf(" 23.2. %-64s", "Correct texture coordinates on textured SDF");
  if (psnr_tc >= 35)
    printf("passed    (%.2f)\n", psnr_tc);
  else
    printf("FAILED, psnr = %f\n", psnr_tc);
  
  printf(" 23.3. %-64s", "Correct texture on textured SDF");
  if (psnr_tex >= 35)
    printf("passed    (%.2f)\n", psnr_tex);
  else
    printf("FAILED, psnr = %f\n", psnr_tex);
}

void demo_meshes_set_textures(std::shared_ptr<MultiRenderer> pRender)
{
  for (int i=1;i<=8;i++)
  {
    LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>((std::string("scenes/textures/block_")+std::to_string(i)+".png").c_str());
    unsigned texId = pRender->AddTexture(texture);
    MultiRendererMaterial mat;
    mat.type = MULTI_RENDER_MATERIAL_TYPE_TEXTURED;
    mat.texId = texId;
    pRender->AddMaterial(mat);
  }
}

void litert_test_24_demo_meshes()
{
  unsigned W = 4096, H = 4096;

  MultiRenderPreset preset = getDefaultPreset();
  preset.normal_mode = NORMAL_MODE_VERTEX;
  preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;

  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  float3 p = float3(0,0,12);
  unsigned rots_x = 5;
  unsigned rots_y = 5;
  unsigned rots_z = 5;

  std::vector<std::string> type_names = {"mesh", "sdf_SVS"};

  std::vector<std::string> names = {"cube", "cube_textured", "cylinder_ug", "cylinder_uv", "cylinder_sv"};
  std::vector<cmesh4::SimpleMesh> meshes = {
    cmesh4_demo::create_cube(float3(1,1,1), false, cmesh4_demo::VerticesType::UNIQUE, cmesh4_demo::NormalsType::GEOMETRY),
    cmesh4_demo::create_cube(float3(1,1,1), true, cmesh4_demo::VerticesType::UNIQUE, cmesh4_demo::NormalsType::GEOMETRY),
    cmesh4_demo::create_cylinder(float3(0.4,1,0.4), 32, 32, cmesh4_demo::VerticesType::UNIQUE, cmesh4_demo::NormalsType::GEOMETRY),
    cmesh4_demo::create_cylinder(float3(0.4,1,0.4), 32, 32, cmesh4_demo::VerticesType::UNIQUE, cmesh4_demo::NormalsType::VERTEX),
    cmesh4_demo::create_cylinder(float3(0.4,1,0.4), 32, 32, cmesh4_demo::VerticesType::SHARED, cmesh4_demo::NormalsType::VERTEX),};

  int test_n = 1;
  printf("TEST 24. Demo meshes texturing tests\n");
  printf("INSTANCING ON GPU IS BROKEN. SKIP\n");
  return;

  for (int i=0;i<names.size();i++)
  {
    for (int type_i = 0; type_i < type_names.size(); type_i++)    
    {
      auto name = names[i];
      auto mesh = meshes[i];
      cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

      auto pRender = CreateMultiRenderer("GPU");
      demo_meshes_set_textures(pRender);
      pRender->SetPreset(preset);
      pRender->SetViewport(0,0,W,H);

      {
        pRender->SetPreset(preset);
        pRender->GetAccelStruct()->ClearGeom();
        unsigned geomId = 0;
        if (type_names[type_i] == "mesh")
        {
          auto BVH_RT = dynamic_cast<BVHRT*>(pRender->GetAccelStruct()->UnderlyingImpl(0));
          assert(BVH_RT);
          unsigned geomId = BVH_RT->AddGeom_Triangles3f((const float*)mesh.vPos4f.data(), (const float*)mesh.vNorm4f.data(), mesh.vPos4f.size(),
                                                        mesh.indices.data(), mesh.indices.size(), BUILD_HIGH, sizeof(float)*4);
          pRender->add_mesh_internal(mesh, geomId);
        }
        else if (type_names[type_i] == "sdf_SVS")
        {
          auto sdf_SVS = sdf_converter::create_sdf_frame_octree_tex(SparseOctreeSettings(SparseOctreeBuildType::MESH_TLO, 8), mesh);
          auto BVH_RT = dynamic_cast<BVHRT*>(pRender->GetAccelStruct()->UnderlyingImpl(0));
          assert(BVH_RT);
          unsigned geomId = BVH_RT->AddGeom_SdfFrameOctreeTex(sdf_SVS, pRender->GetAccelStruct().get());
          pRender->add_SdfFrameOctreeTex_internal(sdf_SVS, geomId);
        }
        pRender->GetAccelStruct()->ClearScene();

        for (unsigned x_i = 0; x_i < rots_x; x_i++)
        {
          LiteMath::float4x4 m = LiteMath::translate4x4(float3(3.0f*((float)x_i - rots_x/2.0f + 0.5f), 3.0f, 0)) * 
                                LiteMath::rotate4x4X(M_PI*float(x_i)/float(rots_x));
          pRender->AddInstance(geomId, m);
        }

        for (unsigned y_i = 0; y_i < rots_y; y_i++)
        {
          LiteMath::float4x4 m = LiteMath::translate4x4(float3(3.0f*((float)y_i - rots_y/2.0f + 0.5f), 0, 0)) * 
                                LiteMath::rotate4x4Y(M_PI*float(y_i)/float(rots_y));
          pRender->AddInstance(geomId, m);
        }

        for (unsigned z_i = 0; z_i < rots_z; z_i++)
        {
          LiteMath::float4x4 m = LiteMath::translate4x4(float3(3.0f*((float)z_i - rots_z/2.0f + 0.5f), -3.0f, 0)) * 
                                LiteMath::rotate4x4Z(M_PI*float(z_i)/float(rots_z));
          pRender->AddInstance(geomId, m);
        }

        pRender->GetAccelStruct()->CommitScene();
      }

      render(image, pRender, p, float3(0, 0, 0), 1.0f*cross(p, float3(1, 0, 0)), preset);
      LiteImage::SaveImage<uint32_t>((std::string("saves/test_24_") +name+"_"+type_names[type_i]+".bmp").c_str(), image);

      if (type_names[type_i] == "mesh")
      {
        ref_image = image;
      }
      else
      {
        float psnr = image_metrics::PSNR(ref_image, image);

        printf(" 24.%d. %-64s", test_n, name.c_str());
        if (psnr >= 30)
          printf("passed    (%.2f)\n", psnr);
        else
          printf("FAILED, psnr = %f\n", psnr);  

        test_n++;
      }
    }
  }
}

void litert_test_25_float_images()
{
  //create renderers for SDF scene and mesh scene
  const char *scene_name = "scenes/01_simple_scenes/teapot.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  LiteImage::Image2D<float4> image(W, H);
  LiteImage::Image2D<float4> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("CPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  pRender->RenderFloat(image.data(), image.width(), image.height(), m1, m2, preset);
  pRenderRef->RenderFloat(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset);

  LiteImage::SaveImage<float4>("saves/test_25_res.bmp", image); 
  LiteImage::SaveImage<float4>("saves/test_25_ref.bmp", ref_image);

  float psnr = image_metrics::PSNR(ref_image, image);
  printf("TEST 25. Rendering simple mesh with floating-point colors (32 bits per channel)\n");
  printf(" 25.1. %-64s", "CPU and GPU render image_metrics::PSNR > 45 ");
  if (psnr >= 45)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void litert_test_26_sbs_shallow_bvh()
{

}

void litert_test_27_textured_colored_SBS()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 9);

  SdfSBSHeader header;
  header.brick_size = 2;
  header.brick_pad = 0;
  header.bytes_per_value = 1;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/porcelain.png");

  LiteImage::Image2D<uint32_t> image_mesh(W, H);
  LiteImage::Image2D<uint32_t> image_SVS(W, H);
  LiteImage::Image2D<uint32_t> image_SBS_tex(W, H);
  LiteImage::Image2D<uint32_t> image_SBS_col(W, H);
  LiteImage::Image2D<uint32_t> image_SBS_ind(W, H);
  
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
  
    auto textured_octree = sdf_converter::create_sdf_frame_octree_tex(settings, mesh);
    pRender->SetScene(textured_octree);
    render(image_SVS, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
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

    auto textured_SBS = sdf_converter::create_sdf_SBS_tex(settings, header, mesh);
    pRender->SetScene(textured_SBS);
    render(image_SBS_tex, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
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

    auto colored_SBS = sdf_converter::create_sdf_SBS_col(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures());
    pRender->SetScene(colored_SBS);
    render(image_SBS_col, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
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

    auto indexed_SBS = sdf_converter::create_sdf_SBS_indexed(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures());
    pRender->SetScene(indexed_SBS);
    render(image_SBS_ind, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);    
  }

  LiteImage::SaveImage<uint32_t>("saves/test_27_mesh.bmp", image_mesh); 
  LiteImage::SaveImage<uint32_t>("saves/test_27_svs.bmp", image_SVS);
  LiteImage::SaveImage<uint32_t>("saves/test_27_sbs_tex.bmp", image_SBS_tex);
  LiteImage::SaveImage<uint32_t>("saves/test_27_sbs_col.bmp", image_SBS_col);
  LiteImage::SaveImage<uint32_t>("saves/test_27_sbs_ind.bmp", image_SBS_ind);

  float psnr_1 = image_metrics::PSNR(image_mesh, image_SBS_tex);
  float psnr_2 = image_metrics::PSNR(image_SVS, image_SBS_tex);
  float psnr_3 = image_metrics::PSNR(image_SBS_col, image_SBS_tex);
  float psnr_4 = image_metrics::PSNR(image_SBS_ind, image_SBS_col);

  printf("TEST 27. Teatured and colored SBS\n");

  printf(" 27.1. %-64s", "Correct texture on textured SBS");
  if (psnr_1 >= 35)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);  

  printf(" 27.2. %-64s", "Textured SBS is close to textured SVS");
  if (psnr_2 >= 35)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  
  printf(" 27.3. %-64s", "Colored SBS is realtively close to textured SBS");
  if (psnr_3 >= 30)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);

  printf(" 27.4. %-64s", "Indexed SBS perfectly match colored SBS");
  if (psnr_4 >= 50)
    printf("passed    (%.2f)\n", psnr_4);
  else
    printf("FAILED, psnr = %f\n", psnr_4);
}

void litert_test_28_sbs_reg()
{
  
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 5);

  SdfSBSHeader header;
  header.brick_size = 2;
  header.brick_pad = 0;
  header.bytes_per_value = 1;

  LiteImage::Image2D<float4> texture = LiteImage::LoadImage<float4>("scenes/Solid_white.png");

  LiteImage::Image2D<uint32_t> image_mesh(W, H);
  LiteImage::Image2D<uint32_t> image_SBS_ind(W, H);
  LiteImage::Image2D<uint32_t> image_orig_SBS_ind(W, H);
  LiteImage::Image2D<uint32_t> image_orig_SBS_ind_not(W, H);

  //LiteImage::Image2D<uint32_t> image_SBS_grid(W, H);
  //LiteImage::Image2D<uint32_t> image_SBS_grid_smooth(W, H);
  
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
  }

  printf("TEST 28. SBS regularization\n");
  
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

    auto indexed_SBS_not_noisy = sdf_converter::create_sdf_SBS_indexed(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures());
    
    auto indexed_SBS = sdf_converter::create_sdf_SBS_indexed(settings, header, mesh, matId, pRender->getMaterials(), pRender->getTextures(), true);
    
    
    auto sbs = sdf_converter::sdf_SBS_smoother(indexed_SBS, 1, 0.001, 0.3f, 100);
    
    pRender->SetScene(sbs);
    render(image_SBS_ind, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);    
    pRender->SetScene(indexed_SBS);
    render(image_orig_SBS_ind, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
    pRender->SetScene(indexed_SBS_not_noisy);
    render(image_orig_SBS_ind_not, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }
  
  LiteImage::SaveImage<uint32_t>("saves/test_28_mesh.bmp", image_mesh);
  LiteImage::SaveImage<uint32_t>("saves/test_28_sbs_ind.bmp", image_SBS_ind);
  LiteImage::SaveImage<uint32_t>("saves/test_28_noisy_sbs_ind.bmp", image_orig_SBS_ind);
  LiteImage::SaveImage<uint32_t>("saves/test_28_orig_sbs_ind.bmp", image_orig_SBS_ind_not);
  //LiteImage::SaveImage<uint32_t>("saves/test_28_sbs_grid.bmp", image_SBS_grid);
  //LiteImage::SaveImage<uint32_t>("saves/test_28_sbs_grid_smooth.bmp", image_SBS_grid_smooth);
  

  
}

void count_of_nodes_in_layers(const std::vector<SdfFrameOctreeNode> &frame_nodes, unsigned layer, std::vector<unsigned> &layers, unsigned idx)
{
  if (layer + 1 > layers.size())
  {
    layers.resize(layer + 1);
    layers[layer] = 0;
  }
  layers[layer] += 1;
  if (frame_nodes[idx].offset != 0)
  {
    for (unsigned num = 0; num < 8; ++num)
    {
      count_of_nodes_in_layers(frame_nodes, layer + 1, layers, frame_nodes[idx].offset + num);
    }
  }
}

void litert_test_29_smoothed_frame_octree()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());

  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 9);

  unsigned W = 2048, H = 2048;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  MultiRenderPreset preset = getDefaultPreset();

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);

    pRender->SetScene(mesh);
    render(image, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }

  std::vector<MeshBVH> bvh(1);
    for (unsigned i = 0; i < 1; i++)
      bvh[i].init(mesh);
    auto real_sdf = [&](const float3 &p, unsigned idx) -> float 
    { return bvh[idx].get_signed_distance(p); /*return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - 0.8;*/};

    printf("TEST 29. New frame octree creation\n");

  LiteImage::SaveImage<uint32_t>("saves/test_29_mesh.bmp", image);

  unsigned num_of_test = 1;
  for (float eps = 1e-3; eps > 1e-5; eps /= std::sqrt(10), ++num_of_test)
  {

    std::vector<SdfFrameOctreeNode> frame_nodes = sdf_converter::create_sdf_frame_octree(settings, real_sdf, eps, true, false);

    {
      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene(frame_nodes);
      render(image_1, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);

      float psnr = image_metrics::PSNR(image, image_1);

      printf("  29.%u.1 %-64s", num_of_test, "mesh and new frame octree PSNR > 30");
      if (psnr >= 30)
        printf("passed    (%.2f), eps = %f\n", psnr, eps);
      else
        printf("FAILED, psnr = %f, eps = %f\n", psnr, eps);

      float iou = IoU::IoU_frame_octree(frame_nodes, real_sdf, 1000000);

      printf("  29.%u.2 %-64s", num_of_test, "sdf and new frame octree IoU > 0.9");
      if (iou >= 0.9)
        printf("passed    (%.5f), eps = %f\n", iou, eps);
      else
        printf("FAILED, iou = %f, eps = %f\n", iou, eps);
      
    }

    settings.nodes_limit = frame_nodes.size();
    //printf("%d\n", frame_nodes.size());

    frame_nodes = sdf_converter::create_sdf_frame_octree(settings, real_sdf, 1);
    //std::vector<unsigned> buffer = {0};

    //count_of_nodes_in_layers(frame_nodes, 0, buffer, 0);

    /*for (auto i : buffer)
    {
      printf("%u ", i);
    }
    printf("\n");*/

    {
      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene(frame_nodes);
      render(image_2, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);

      float psnr = image_metrics::PSNR(image, image_2);

      printf("  29.%u.3 %-64s", num_of_test, "mesh and old frame octree PSNR > 30 (with same nodes count) ");
      if (psnr >= 30)
        printf("passed    (%.2f), eps = %f\n", psnr, eps);
      else
        printf("FAILED, psnr = %f, eps = %f\n", psnr, eps);

      float iou = IoU::IoU_frame_octree(frame_nodes, real_sdf, 1000000);

      printf("  29.%u.4 %-64s", num_of_test, "sdf and old frame octree IoU > 0.9 (with same nodes count) ");
      if (iou >= 0.9)
        printf("passed    (%.5f), eps = %f\n", iou, eps);
      else
        printf("FAILED, iou = %f, eps = %f\n", iou, eps);

      LiteImage::SaveImage<uint32_t>(("saves/test_29_our_octree_" + std::to_string(num_of_test) + ".bmp").c_str(), image_1);
      LiteImage::SaveImage<uint32_t>(("saves/test_29_octree_" + std::to_string(num_of_test) + ".bmp").c_str(), image_2);
    }

  }
}

void litert_test_30_verify_SBS_SBSAdapt()
{
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.9, -0.9, -0.9), float3(0.9, 0.9, 0.9));

  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
  SdfSBSHeader header_1_1{1,0,1,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_1_2{1,0,2,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_2_1{2,0,1,SDF_SBS_NODE_LAYOUT_DX};
  SdfSBSHeader header_2_2{2,0,2,SDF_SBS_NODE_LAYOUT_DX};

  SdfSBS sbs_1_1 = sdf_converter::create_sdf_SBS(settings, header_1_1, mesh);
  SdfSBS sbs_1_2 = sdf_converter::create_sdf_SBS(settings, header_1_2, mesh);
  SdfSBS sbs_2_1 = sdf_converter::create_sdf_SBS(settings, header_2_1, mesh);
  SdfSBS sbs_2_2 = sdf_converter::create_sdf_SBS(settings, header_2_2, mesh);
  SdfSBS &curr_sbs = sbs_2_2; // in all 4 cases (whether is_single_node is true or false) PSNR(sbs, sbs_adapted) == 100

  unsigned W = 1024, H = 1024;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);
  LiteImage::Image2D<uint32_t> sbs_image(W, H);

  printf("TEST 30. SBS and SBSAdapt correctness\n");
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(mesh);
    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_30_reference.bmp", image); 
    ref_image = image;
  }
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(curr_sbs);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_30_SBS.bmp", image); 
    sbs_image = image;

    float psnr = image_metrics::PSNR(ref_image, image);

    printf(" 30.1. %-64s", "[CPU] 2-voxel,2-byte SBS and reference image PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    SdfSBSAdapt sbsa_scene;
    SdfSBSAdaptView sbsa_view = convert_sbs_to_adapt(sbsa_scene, curr_sbs);
    pRender->SetScene(sbsa_view);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_30_SBSA.bmp", image); 

    float psnr = image_metrics::PSNR(ref_image, image);
    printf(" 30.2. %-64s", "[CPU] SBSAdapt and reference image PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);

    float sbsa_psnr = image_metrics::PSNR(sbs_image, image);
    printf(" 30.3. %-64s", "[CPU] 2-voxel,2-byte SBS matches SBSAdapt");
    if (sbsa_psnr >= 40)
      printf("passed    (%.2f)\n", sbsa_psnr);
    else
      printf("FAILED, psnr = %f\n", sbsa_psnr);
  }
}
void litert_test_31_fake_nurbs_render()
{
  unsigned W = 1024, H = 1024;

  MultiRenderPreset preset = getDefaultPreset();
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  RawNURBS nurbs;
  //TODO: nurbs = load_nurbs("my_nurbs.txt");

  auto pRenderRef = CreateMultiRenderer("CPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->SetScene(nurbs);

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->SetScene(nurbs);

  render(image, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  render(ref_image, pRenderRef, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);

  LiteImage::SaveImage<uint32_t>("saves/test_31_res.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_31_ref.bmp", ref_image);

  float psnr = image_metrics::PSNR(ref_image, image);
  printf("TEST 31. Rendering fake NURBS\n");
  printf(" 31.1. %-64s", "CPU and GPU render image_metrics::PSNR > 45 ");
  if (psnr <= 45)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void litert_test_32_smooth_sbs_normals()
{
  //create renderers for SDF scene and mesh scene
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/bunny.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 5);

  SdfSBSHeader header;
  header.brick_size = 4;
  header.brick_pad = 0;
  header.bytes_per_value = 1;

  LiteImage::Image2D<uint32_t> image_mesh(W, H);
  LiteImage::Image2D<uint32_t> image_SBS(W, H);
  LiteImage::Image2D<uint32_t> image_SBS_n(W, H);

  {
    auto pRender = CreateMultiRenderer("GPU");
    preset.normal_mode = NORMAL_MODE_VERTEX;
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    pRender->SetScene(mesh);
    render(image_mesh, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetViewport(0,0,W,H);
    auto indexed_SBS = sdf_converter::create_sdf_SBS_indexed_with_neighbors(settings, header, mesh, 0, pRender->getMaterials(), pRender->getTextures());
    pRender->SetScene(indexed_SBS);

    preset.normal_mode = NORMAL_MODE_GEOMETRY;
    //auto t1 = std::chrono::high_resolution_clock::now();
    render(image_SBS, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);    
    //auto t2 = std::chrono::high_resolution_clock::now();

    preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;
    //auto t3 = std::chrono::high_resolution_clock::now();
    render(image_SBS_n, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);  
    //auto t4 = std::chrono::high_resolution_clock::now();

    //printf("render times in ms: %f %f\n", std::chrono::duration<double, std::milli>(t2-t1).count(), std::chrono::duration<double, std::milli>(t4-t3).count());  
  }

  LiteImage::SaveImage<uint32_t>("saves/test_32_mesh.bmp", image_mesh); 
  LiteImage::SaveImage<uint32_t>("saves/test_32_sbs.bmp", image_SBS);
  LiteImage::SaveImage<uint32_t>("saves/test_32_sbs_n.bmp", image_SBS_n);

  float psnr_1 = image_metrics::PSNR(image_mesh, image_SBS);
  float psnr_2 = image_metrics::PSNR(image_mesh, image_SBS_n);

  printf("TEST 32. SBS with smooth normals\n");

  printf(" 32.1. %-64s", "Smooth normals increase PSNR");
  if (psnr_1 > 30 && psnr_2 >= psnr_1)
    printf("passed    (%.2f > %.2f)\n", psnr_2, psnr_1);
  else
    printf("FAILED    (%.2f < %.2f)\n", psnr_2, psnr_1);  
}

void litert_test_33_verify_SBS_SBSAdapt_split()
{
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.9, -0.9, -0.9), float3(0.9, 0.9, 0.9));

  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
  std::vector<SdfSBSHeader> headers = {{2,0,2,SDF_SBS_NODE_LAYOUT_DX}};
  // Note: when SBS brick size is not a power of 2, it cannot be identically converted to Adaptive SBS

  unsigned W = 1024, H = 1024;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);
  LiteImage::Image2D<uint32_t> sbs_image(W, H);

  printf("TEST 33. SBS and non-cubic SBSAdapt correctness\n");
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(mesh);
    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_33_reference.bmp", image); 
    ref_image = image;
  }

  for (int i = 0; i < headers.size(); ++i)
  {
    SdfSBS curr_sbs = sdf_converter::create_sdf_SBS(settings, headers[i], mesh);
    std::string voxel_count = std::to_string(headers[i].brick_size);
    std::string bytes_count = std::to_string(headers[i].bytes_per_value);

    {
      auto pRender = CreateMultiRenderer("CPU");
      pRender->SetPreset(preset);

      // SdfSBSAdapt sbsa_scene;
      // SdfSBSAdaptView sbsa_view = convert_sbs_to_adapt(sbsa_scene, curr_sbs);
      // pRender->SetScene(sbsa_view);
      pRender->SetScene(curr_sbs);

      render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
      std::string img_path = "saves/test_33_" + voxel_count + '_' + bytes_count + "_SBS.bmp";
      LiteImage::SaveImage<uint32_t>(img_path.c_str(), image);
      sbs_image = image;

      float psnr = image_metrics::PSNR(ref_image, image);

      std::string result = "[CPU] " + voxel_count + "-voxel," + bytes_count + "-byte SBS and reference image PSNR > 40 ";
      printf("\n 33.%d. %-64s", 3*i, result.c_str());
      if (psnr >= 40)
        printf("passed    (%.2f)\n", psnr);
      else
        printf("FAILED, psnr = %f\n", psnr);
    }
    {
      auto pRender = CreateMultiRenderer("CPU");
      pRender->SetPreset(preset);

      SdfSBSAdapt sbsa_scene;
      SdfSBSAdaptView sbsa_view = convert_sbs_to_adapt_with_split(sbsa_scene, curr_sbs);
      pRender->SetScene(sbsa_view);

      render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
      std::string img_path = "saves/test_33_" + voxel_count + '_' + bytes_count + "_SBSA_split.bmp";
      LiteImage::SaveImage<uint32_t>(img_path.c_str(), image); 

      float psnr = image_metrics::PSNR(ref_image, image);
      printf(" 33.%d. %-64s", 3*i+1, "[CPU] SBSAdapt with split and reference image PSNR > 40 ");
      if (psnr >= 40)
        printf("passed    (%.2f)\n", psnr);
      else
        printf("FAILED, psnr = %f\n", psnr);

      float sbsa_psnr = image_metrics::PSNR(sbs_image, image);
      std::string result = "[CPU] " + voxel_count + "-voxel," + bytes_count + "-byte SBS matches SBSAdapt with split ";
      printf(" 33.%d. %-64s", 3*i+2, result.c_str());
      if (sbsa_psnr >= 40)
        printf("passed    (%.2f)\n", sbsa_psnr);
      else
        printf("FAILED, psnr = %f\n", sbsa_psnr);
    }
  }
}

void
litert_test_34_tricubic_sbs()
{
  printf("TEST 34 DRAW SBS STRUCTURE WITH TRICUBIC INTERPOLATION\n");
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::normalize_mesh(mesh);

  unsigned W = 512, H = 512;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;

  LiteImage::Image2D<uint32_t> img(W, H);
  SdfSBSHeader header;
  header.brick_size = 2;
  header.brick_pad = 0;
  header.bytes_per_value = 1;
  header.aux_data = SDF_SBS_NODE_LAYOUT_DX;

  auto sbs = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64*64*64), header, mesh);
  auto pRender = CreateMultiRenderer("CPU");
  pRender->SetPreset(preset);
  pRender->SetScene(sbs);
  render(img, pRender, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_34_sbs_tricubic.bmp", img);
}

void litert_test_35_SBSAdapt_greed_creating()
{
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  //preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_BBOX;

  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());
  cmesh4::rescale_mesh(mesh, float3(-0.9, -0.9, -0.9), float3(0.9, 0.9, 0.9));

  std::vector<MeshBVH> bvh(1);
    for (unsigned i = 0; i < 1; i++)
      bvh[i].init(mesh);
    auto real_sdf = [&](const float3 &p, unsigned idx) -> float 
    { return bvh[idx].get_signed_distance(p); /*return std::sqrt(p.x * p.x + p.y * p.y + p.z * p.z) - 0.8;*/};

  //SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
  //std::vector<SdfSBSHeader> headers = {{2,0,2,SDF_SBS_NODE_LAYOUT_DX}};
  // Note: when SBS brick size is not a power of 2, it cannot be identically converted to Adaptive SBS

  unsigned W = 1024, H = 1024;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> image_sbs(W, H);
  LiteImage::Image2D<uint32_t> image_sbs_adapt(W, H);
  //LiteImage::Image2D<uint32_t> sbs_image(W, H);

  printf("TEST 35. SBSAdapt greed creating\n");
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(mesh);
    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_35_mesh.bmp", image);
  }

  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 4);
  SdfSBSHeader header_1_1{8,0,4,SDF_SBS_NODE_LAYOUT_DX};

  SdfSBS sbs_1_1 = sdf_converter::create_sdf_SBS(settings, header_1_1, mesh);
  SdfSBSAdapt sbsa_scene;
  SdfSBSAdaptView sbsa_view = convert_sbs_to_adapt(sbsa_scene, sbs_1_1);

  SdfSBSAdapt sbs_adapt = sdf_converter::greed_sbs_adapt(real_sdf, 4);

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_1_1);

    render(image_sbs, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_35_sbs_adapt.bmp", image_sbs);
  }

  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(sbs_adapt);

    render(image_sbs_adapt, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_35_sbs_greed.bmp", image_sbs_adapt);
  }

  printf("TEST 35. Greedy SBS adapt builder\n");

  float psnr_1 = image_metrics::PSNR(image_sbs_adapt, image);
  float psnr_2 = image_metrics::PSNR(image_sbs, image_sbs_adapt);

  printf(" 35.1. %-64s", "Adapt SBS is correct");
  if (psnr_1 > 30)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED    (%.2f)\n", psnr_1);

  printf(" 35.2. %-64s", "Adapt SBS exactly match original SBS");
  if (psnr_2 > 50)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED    (%.2f)\n", psnr_2);  
}

void litert_test_36_primitive_visualization()
{
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
  preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

  GraphicsPrim primitives;
  {
    primitives.header.prim_type = GRAPH_PRIM_LINE_SEGMENT_DIR;
    primitives.header.color = float3(0.f, 255.f, 255.f);

    const uint32_t pt_count = 5u;

    primitives.points.resize(2*pt_count);
    for (int i = 0; i < pt_count; ++i)
    {
      primitives.points[2*i] = float4(double(rand()) / (RAND_MAX >> 1) - 1.f,
                                      double(rand()) / (RAND_MAX >> 1) - 1.f,
                                      double(rand()) / (RAND_MAX >> 1) - 1.f,
                                      0.02f);
      primitives.points[2*i+1] = float4(double(rand()) / (RAND_MAX >> 1) - 1.f,
                                        double(rand()) / (RAND_MAX >> 1) - 1.f,
                                        double(rand()) / (RAND_MAX >> 1) - 1.f, 0.f);
      //printf("Arrow: [%f, %f, %f], [%f, %f, %f]\n", primitives.points[2*i].x, primitives.points[2*i].y, primitives.points[2*i].z,
      //                                                primitives.points[2*i+1].x, primitives.points[2*i+1].y, primitives.points[2*i+1].z);
    }
  }

  unsigned W = 1024, H = 1024;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);
  LiteImage::Image2D<uint32_t> sbs_image(W, H);

  printf("TEST 36. Primitive visualization\n");
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    GraphicsPrimView prim_view{primitives};
    pRender->SetScene(prim_view);

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_36_primitives.bmp", image); 
    ref_image = image;
  }

}

std::optional<std::vector<float3>> read_polygon_from_file(char const *file_name) {
    static size_t constexpr BUF_SIZE = (1'024 + 2) / sizeof(float3);

    auto const file = std::fopen(file_name, "rb");

    if (nullptr == file) {
        fprintf(
            stderr, "failed to open file '%s': %s\n", file_name, strerror(errno)
        );
        return std::nullopt;
    }

    auto result = std::vector<float3>{};
    auto buf = std::array<float3, BUF_SIZE>{};
    auto n_read = size_t{0};

    while (0 != (n_read = std::fread(buf.data(), sizeof(buf[0]), buf.size(), file))) {
        result.insert(result.begin(), buf.begin(), buf.begin() + n_read);
    }

    std::fclose(file);

    return result;
}

void litert_test_37_any_polygon() {
    namespace img = LiteImage;

    static uint constexpr WIDTH = 1'024;
    static uint constexpr HEIGHT = 1'024;
    static char constexpr POLYGON_BIN_FILE_NAME[] =
        "scenes/01_simple_scenes/data/polygon.bin";
    static float3 const CAMERA_POSITION = float3{0.0f, 0.0f, 2.5f};
    static float3 const CAMERA_TARGET = float3{0.0f};
    static float3 const CAMERA_UP = float3{0.0f, 1.0f, 0.0f};
    static char constexpr RENDERER_NAME_HOST[] = "CPU";
    static char constexpr RENDERER_NAME_DEVICE[] = "GPU";

    auto vertices = read_polygon_from_file(POLYGON_BIN_FILE_NAME);

    if (!vertices.has_value()) {
        fprintf(stderr, "failed to read polygon data, skipping test 37\n");
        return;
    }

    fprintf(stderr, "TEST 37. Rendering non-planar polygon\n");

    auto const polygon = AnyPolygon{std::move(vertices.value())};

    auto render_preset = getDefaultPreset();
    render_preset.render_mode = MULTI_RENDER_MODE_LAMBERT;

    auto render_custom = [&](char const *renderer_name) -> img::Image2D<uint32_t> {
        auto image = img::Image2D<uint32_t>{WIDTH, HEIGHT};

        auto renderer = CreateMultiRenderer(renderer_name);
        renderer->SetPreset(render_preset);
        renderer->SetViewport(0, 0, WIDTH, HEIGHT);
        renderer->SetScene(polygon);

        render(image, renderer, CAMERA_POSITION, CAMERA_TARGET, CAMERA_UP, render_preset);

        return image;
    };

    auto const host_image = render_custom(RENDERER_NAME_HOST);
    auto const device_image = render_custom(RENDERER_NAME_DEVICE);

    auto const psnr = image_metrics::PSNR(host_image, device_image);

    printf(" 31.1. %-64s", "CPU and GPU render image_metrics::PSNR > 45 ");
    if (psnr <= 45) {
        printf("passed    (%.2f)\n", psnr);
    } else {
        printf("FAILED, psnr = %f\n", psnr);
    }
}

void perform_tests_litert(const std::vector<int> &test_ids)
{
  std::vector<int> tests = test_ids;

  std::vector<std::function<void(void)>> test_functions = {
      litert_test_1_framed_octree, litert_test_2_SVS, litert_test_3_SBS_verify,
      litert_test_4_hydra_scene, litert_test_5_interval_tracing, litert_test_6_faster_bvh_build,
      litert_test_7_stub, litert_test_8_SDF_grid, litert_test_9_mesh, 
      litert_test_10_save_load, litert_test_11_stub, litert_test_12_stub,
      litert_test_13_stub, litert_test_14_octree_nodes_removal, litert_test_15_frame_octree_nodes_removal, 
      litert_test_16_SVS_nodes_removal, litert_test_17_all_types_sanity_check, litert_test_18_mesh_normalization,
      litert_test_19_marching_cubes, litert_test_20_radiance_fields, litert_test_21_rf_to_mesh,
      litert_test_22_sdf_grid_smoothing, litert_test_23_textured_sdf, litert_test_24_demo_meshes,
      litert_test_25_float_images, litert_test_26_sbs_shallow_bvh, litert_test_27_textured_colored_SBS,
      litert_test_28_sbs_reg, litert_test_29_smoothed_frame_octree, litert_test_30_verify_SBS_SBSAdapt,
      litert_test_31_fake_nurbs_render, litert_test_32_smooth_sbs_normals, litert_test_33_verify_SBS_SBSAdapt_split, 
      litert_test_34_tricubic_sbs, litert_test_35_SBSAdapt_greed_creating, litert_test_36_primitive_visualization,
      litert_test_37_any_polygon};

  if (tests.empty())
  {
    tests.resize(test_functions.size());
    for (int i = 0; i < test_functions.size(); i++)
      tests[i] = i + 1;
  }

  for (int i = 0; i < 80; i++)
    printf("#");
  printf("\nSDF SCENE TESTS\n");
  for (int i = 0; i < 80; i++)
    printf("#");
  printf("\n");

  for (int i : tests)
  {
    assert(i > 0 && i <= test_functions.size());
    test_functions[i - 1]();
  }
}
