#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "../utils/sparse_octree.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"
#include "../NeuralRT/NeuralRT.h"
#include "../utils/hp_octree.h"

#include <functional>
#include <cassert>
#include <chrono>

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

float PSNR(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2)
{
  assert(image_1.vector().size() == image_2.vector().size());
  unsigned sz = image_1.vector().size();
  double sum = 0.0;
  for (int i=0;i<sz;i++)
  {
    unsigned r1 = (image_1.vector()[i] & 0x000000FF);
    unsigned g1 = (image_1.vector()[i] & 0x0000FF00) >> 8;
    unsigned b1 = (image_1.vector()[i] & 0x00FF0000) >> 16;
    unsigned r2 = (image_2.vector()[i] & 0x000000FF);
    unsigned g2 = (image_2.vector()[i] & 0x0000FF00) >> 8;
    unsigned b2 = (image_2.vector()[i] & 0x00FF0000) >> 16;
    sum += ((r1-r2)*(r1-r2)+(g1-g2)*(g1-g2)+(b1-b2)*(b1-b2)) / (3.0f*255.0f*255.0f);
  }
  float mse = sum / sz;

  return -10*log10(std::max<double>(1e-10, mse));
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
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    SparseOctreeBuilder builder;
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 9);
    std::vector<SdfFrameOctreeNode> frame_nodes;

    builder.construct([&mesh_bvh](const float3 &p) { return mesh_bvh.get_signed_distance(p); }, settings);
    builder.convert_to_frame_octree(frame_nodes);

    unsigned W = 2048, H = 2048;
    LiteImage::Image2D<uint32_t> image(W, H);
    float timings[4] = {0,0,0,0};

    std::vector<unsigned> presets_ob = {SDF_OCTREE_BLAS_NO, SDF_OCTREE_BLAS_DEFAULT,
                                        SDF_OCTREE_BLAS_DEFAULT, SDF_OCTREE_BLAS_DEFAULT,
                                        SDF_OCTREE_BLAS_DEFAULT, SDF_OCTREE_BLAS_DEFAULT};

    std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_DEFAULT, SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                                        SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                                        SDF_OCTREE_NODE_INTERSECT_NEWTON, SDF_OCTREE_NODE_INTERSECT_BBOX};

    std::vector<std::string> names = {"no_bvh_traversal", "bvh_traversal", "bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

    for (int i=0; i<presets_ob.size(); i++)
    {
      MultiRenderPreset preset = getDefaultPreset();
      preset.mode = MULTI_RENDER_MODE_PHONG;
      preset.sdf_frame_octree_blas = presets_ob[i];
      preset.sdf_frame_octree_intersect = presets_oi[i];

      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene({(unsigned)frame_nodes.size(), 
                        frame_nodes.data()});

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
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    SparseOctreeBuilder builder;
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 9);
    std::vector<SdfSVSNode> frame_nodes;

    builder.construct([&mesh_bvh](const float3 &p) { return mesh_bvh.get_signed_distance(p); }, settings);
    builder.convert_to_sparse_voxel_set(frame_nodes);

    unsigned W = 2048, H = 2048;
    LiteImage::Image2D<uint32_t> image(W, H);
    float timings[4] = {0,0,0,0};

    std::vector<unsigned> presets_ob = {SDF_OCTREE_BLAS_NO, SDF_OCTREE_BLAS_DEFAULT,
                                        SDF_OCTREE_BLAS_DEFAULT, SDF_OCTREE_BLAS_DEFAULT,
                                        SDF_OCTREE_BLAS_DEFAULT, SDF_OCTREE_BLAS_DEFAULT};

    std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_DEFAULT, SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                                        SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                                        SDF_OCTREE_NODE_INTERSECT_NEWTON, SDF_OCTREE_NODE_INTERSECT_BBOX};

    std::vector<std::string> names = {"no_bvh_traversal", "bvh_traversal", "bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

    for (int i=0; i<presets_ob.size(); i++)
    {
      MultiRenderPreset preset = getDefaultPreset();
      preset.mode = MULTI_RENDER_MODE_PHONG;
      preset.sdf_frame_octree_blas = presets_ob[i];
      preset.sdf_frame_octree_intersect = presets_oi[i];

      auto pRender = CreateMultiRenderer("GPU");
      pRender->SetPreset(preset);
      pRender->SetScene({(unsigned)frame_nodes.size(), 
                        frame_nodes.data()});

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
  preset.mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
  preset.sdf_frame_octree_blas = SDF_OCTREE_BLAS_DEFAULT;
  preset.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+"scenes/01_simple_scenes/data/teapot.vsgf").c_str());

  float3 mb1, mb2, ma1, ma2;
  cmesh4::get_bbox(mesh, &mb1, &mb2);
  cmesh4::rescale_mesh(mesh, float3(-0.9, -0.9, -0.9), float3(0.9, 0.9, 0.9));
  cmesh4::get_bbox(mesh, &ma1, &ma2);
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  SparseOctreeBuilder builder;
  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);

  std::vector<SdfFrameOctreeNode> frame_nodes;
  std::vector<SdfSVSNode> svs_nodes;
  std::vector<SdfSBSNode> sbs_nodes_1_1;
  std::vector<SdfSBSNode> sbs_nodes_1_2;
  std::vector<SdfSBSNode> sbs_nodes_2_1;
  std::vector<SdfSBSNode> sbs_nodes_2_2;
  std::vector<uint32_t> sbs_data_1_1;
  std::vector<uint32_t> sbs_data_1_2;
  std::vector<uint32_t> sbs_data_2_1;
  std::vector<uint32_t> sbs_data_2_2;

  builder.construct([&mesh_bvh](const float3 &p)
                    { return mesh_bvh.get_signed_distance(p); },
                    settings);
  builder.convert_to_frame_octree(frame_nodes);
  builder.convert_to_sparse_voxel_set(svs_nodes);

  SdfSBSHeader header_1_1{1,0,1,2};
  builder.convert_to_sparse_brick_set(header_1_1, sbs_nodes_1_1, sbs_data_1_1);

  SdfSBSHeader header_1_2{1,0,2,2};
  builder.convert_to_sparse_brick_set(header_1_2, sbs_nodes_1_2, sbs_data_1_2);

  SdfSBSHeader header_2_1{2,0,1,3};
  builder.convert_to_sparse_brick_set(header_2_1, sbs_nodes_2_1, sbs_data_2_1);

  SdfSBSHeader header_2_2{2,0,2,3};
  builder.convert_to_sparse_brick_set(header_2_2, sbs_nodes_2_2, sbs_data_2_2);

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
    pRender->SetScene({(unsigned)svs_nodes.size(), svs_nodes.data()});

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SVS.bmp", image); 
    svs_image = image;

    float psnr = PSNR(ref_image, image);

    printf("  3.1. %-64s", "[CPU] SVS and mesh PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("CPU");
    pRender->SetPreset(preset);
    pRender->SetScene(SdfSBSView(header_1_1, sbs_nodes_1_1, sbs_data_1_1));

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_1_1.bmp", image); 

    float psnr = PSNR(ref_image, image);
    printf("  3.2. %-64s", "[CPU] 1-voxel,1-byte SBS and mesh PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);

    float svs_psnr = PSNR(svs_image, image);
    printf("  3.3. %-64s", "[CPU] 1-voxel,1-byte SBS matches SVS");
    if (svs_psnr >= 90)
      printf("passed\n");
    else
      printf("FAILED, psnr = %f\n", svs_psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(SdfSBSView(header_1_1, sbs_nodes_1_1, sbs_data_1_1));

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_1_1.bmp", image); 

    float psnr = PSNR(ref_image, image);
    printf("  3.4. %-64s", "1-voxel,1-byte SBS and mesh PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);

    float svs_psnr = PSNR(svs_image, image);
    printf("  3.5. %-64s", "1-voxel,1-byte SBS matches SVS");
    if (svs_psnr >= 90)
      printf("passed\n");
    else
      printf("FAILED, psnr = %f\n", svs_psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(SdfSBSView(header_1_2, sbs_nodes_1_2, sbs_data_1_2));

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_1_2.bmp", image); 

    float psnr = PSNR(ref_image, image);
    printf("  3.6. %-64s", "1-voxel,2-byte SBS and mesh PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(SdfSBSView(header_2_1, sbs_nodes_2_1, sbs_data_2_1));

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_2_1.bmp", image); 

    float psnr = PSNR(ref_image, image);
    printf("  3.7. %-64s", "8-voxel,1-byte SBS and mesh PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
  {
    auto pRender = CreateMultiRenderer("GPU");
    pRender->SetPreset(preset);
    pRender->SetScene(SdfSBSView(header_2_2, sbs_nodes_2_2, sbs_data_2_2));

    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
    LiteImage::SaveImage<uint32_t>("saves/test_3_SBS_2_2.bmp", image); 

    float psnr = PSNR(ref_image, image);
    printf("  3.8. %-64s", "8-voxel,2-byte SBS and mesh PSNR > 40 ");
    if (psnr >= 40)
      printf("passed    (%.2f)\n", psnr);
    else
      printf("FAILED, psnr = %f\n", psnr);
  }
}

void litert_test_4_hydra_scene()
{
  //create renderers for SDF scene and mesh scene
  const char *scene_name = "scenes/01_simple_scenes/instanced_objects.xml";
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  //const char *scene_name = "scenes/01_simple_scenes/bunny_cornell.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.mode = MULTI_RENDER_MODE_LAMBERT;
  preset.sdf_frame_octree_blas = SDF_OCTREE_BLAS_DEFAULT;
  preset.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_ANALYTIC;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("GPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->LoadSceneHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS, 
                          SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9));

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  //render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
  //render(ref_image, pRenderRef, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset);
  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset);

  LiteImage::SaveImage<uint32_t>("saves/test_4_res.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_4_ref.bmp", ref_image);

  float psnr = PSNR(ref_image, image);
  printf("TEST 4. Rendering Hydra scene\n");
  printf("  4.1. %-64s", "mesh and SDF PSNR > 30 ");
  if (psnr >= 30)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

void litert_test_5_interval_tracing()
{
  //create renderers for SDF scene and mesh scene
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  const char *scene_name = "scenes/01_simple_scenes/instanced_objects.xml";
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  //const char *scene_name = "scenes/01_simple_scenes/bunny_cornell.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset_ref = getDefaultPreset();
  MultiRenderPreset preset_1 = getDefaultPreset();
  preset_1.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_ST;
  MultiRenderPreset preset_2 = getDefaultPreset();
  preset_2.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_IT;
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
  pRender->LoadSceneHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS,
                          SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9));

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  pRender->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset_1);
  pRender->Render(image_2.data(), image_2.width(), image_2.height(), m1, m2, preset_2);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset_ref);

  LiteImage::SaveImage<uint32_t>("saves/test_5_ST.bmp", image_1); 
  LiteImage::SaveImage<uint32_t>("saves/test_5_IT.bmp", image_2); 
  LiteImage::SaveImage<uint32_t>("saves/test_5_ref.bmp", ref_image);

  float psnr_1 = PSNR(image_1, image_2);
  float psnr_2 = PSNR(ref_image, image_2);
  printf("TEST 5. Interval tracing\n");
  printf("  5.1. %-64s", "mesh and SDF PSNR > 30 ");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  printf("  5.2. %-64s", "Interval and Sphere tracing PSNR > 45 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);
}

void litert_test_6_faster_bvh_build()
{
  //create renderers for SDF scene and mesh scene
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  const char *scene_name = "scenes/01_simple_scenes/instanced_objects.xml";
  //const char *scene_name = "large_scenes/02_casual_effects/dragon/change_00000.xml";
  //const char *scene_name = "scenes/01_simple_scenes/bunny_cornell.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset_ref = getDefaultPreset();
  MultiRenderPreset preset_1 = getDefaultPreset();
  preset_1.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
  MultiRenderPreset preset_2 = getDefaultPreset();
  preset_2.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
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
  pRender_1->LoadSceneHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS,
                            SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9));
auto t2 = std::chrono::steady_clock::now();
  float time_1 = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

  auto pRender_2 = CreateMultiRenderer("GPU");
  pRender_2->SetPreset(preset_2);
  pRender_2->SetViewport(0,0,W,H);
auto t3 = std::chrono::steady_clock::now();
  pRender_2->LoadSceneHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_SVS,
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

  float psnr_1 = PSNR(image_1, image_2);
  float psnr_2 = PSNR(ref_image, image_2);
  printf("TEST 6. MESH_TLO SDF BVH build. default %.1f s, mesh TLO %.1f s\n", time_1/1000.0f, time_2/1000.0f);
  printf("  6.1. %-64s", "mesh and SDF PSNR > 30 ");
  if (psnr_2 >= 30)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);
  printf("  6.2. %-64s", "DEFAULT and MESH_TLO PSNR > 45 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);
}

void test_7_neural_SDF()
{
  const char *scene_name = "scenes/02_sdf_scenes/sdf_neural.xml"; 
  unsigned W = 1024, H = 1024;

  MultiRenderPreset preset_1 = getDefaultPreset();
  preset_1.mode = MULTI_RENDER_MODE_LINEAR_DEPTH;

  LiteImage::Image2D<uint32_t> image_1(W, H);
  LiteImage::Image2D<uint32_t> image_2(W, H);
  LiteImage::Image2D<uint32_t> image_3(W, H);
  LiteImage::Image2D<uint32_t> image_4(W, H);
  LiteImage::Image2D<uint32_t> image_5(W, H);
  LiteImage::Image2D<uint32_t> image_6(W, H);

  auto pRender_1 = CreateMultiRenderer("GPU");
  pRender_1->SetPreset(preset_1);
  pRender_1->SetViewport(0,0,W,H);
  pRender_1->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  ISdfSceneFunction *sdf_func = dynamic_cast<ISdfSceneFunction*>(pRender_1->GetAccelStruct().get());
  SparseOctreeBuilder builder;
  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
  std::vector<SdfSVSNode> frame_nodes;

  builder.construct([&sdf_func](const float3 &p) { return sdf_func->eval_distance(p); }, settings);
  builder.convert_to_sparse_voxel_set(frame_nodes);

  auto pRender_2 = CreateMultiRenderer("GPU");
  pRender_2->SetPreset(preset_1);
  pRender_2->SetScene({(unsigned)frame_nodes.size(), frame_nodes.data()});

  std::shared_ptr<NeuralRT> neuralRT1 = CreateNeuralRT("CPU");
  std::shared_ptr<NeuralRT> neuralRT2 = CreateNeuralRT("GPU");
  BVHRT *rt = static_cast<BVHRT*>(pRender_1->GetAccelStruct().get());
  neuralRT1->AddGeom_NeuralSdf(rt->m_SdfNeuralProperties[0], rt->m_SdfParameters.data());
  neuralRT2->AddGeom_NeuralSdf(rt->m_SdfNeuralProperties[0], rt->m_SdfParameters.data());

  auto m1 = pRender_1->getWorldView();
  auto m2 = pRender_1->getProj();

  float timings[5][4];

  pRender_1->Render(image_1.data(), image_1.width(), image_1.height(), m1, m2, preset_1);
  pRender_1->GetExecutionTime("CastRaySingleBlock", timings[0]);

  neuralRT1->Render(image_2.data(), image_2.width(), image_2.height(), m1, m2);

  neuralRT2->Render(image_3.data(), image_3.width(), image_3.height(), m1, m2, NEURALRT_RENDER_SIMPLE);
  neuralRT2->GetExecutionTime("Render_internal", timings[1]);
  neuralRT2->Render(image_4.data(), image_4.width(), image_4.height(), m1, m2, NEURALRT_RENDER_BLOCKED);
  neuralRT2->GetExecutionTime("Render_internal", timings[2]);

  neuralRT2->Render(image_5.data(), image_5.width(), image_5.height(), m1, m2, NEURALRT_RENDER_COOP_MATRICES);
  neuralRT2->GetExecutionTime("Render_internal", timings[3]);

  pRender_2->Render(image_6.data(), image_6.width(), image_6.height(), m1, m2, preset_1);
  pRender_2->GetExecutionTime("CastRaySingleBlock", timings[4]);

  LiteImage::SaveImage<uint32_t>("saves/test_7_default.bmp", image_1); 
  LiteImage::SaveImage<uint32_t>("saves/test_7_NeuralRT_CPU.bmp", image_2); 
  LiteImage::SaveImage<uint32_t>("saves/test_7_NeuralRT_GPU_default.bmp", image_3); 
  LiteImage::SaveImage<uint32_t>("saves/test_7_NeuralRT_GPU_blocked.bmp", image_4); 
  LiteImage::SaveImage<uint32_t>("saves/test_7_NeuralRT_GPU_coop_matrices.bmp", image_5); 
  LiteImage::SaveImage<uint32_t>("saves/test_7_octree.bmp", image_6); 

  float psnr_1 = PSNR(image_1, image_2);
  float psnr_2 = PSNR(image_1, image_3);
  float psnr_3 = PSNR(image_1, image_4);
  float psnr_4 = PSNR(image_1, image_5);
  float psnr_5 = PSNR(image_1, image_6);
  printf("TEST 7. NEURAL SDF rendering\n");
  printf("  7.1. %-64s", "default and NeuralRT CPU PSNR > 45 ");
  if (psnr_1 >= 45)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);

  printf("  7.2. %-64s", "default and NeuralRT GPU default PSNR > 45 ");
  if (psnr_2 >= 45)
    printf("passed    (%.2f)\n", psnr_2);
  else
    printf("FAILED, psnr = %f\n", psnr_2);

  printf("  7.3. %-64s", "default and NeuralRT GPU blocked PSNR > 45 ");
  if (psnr_3 >= 45)
    printf("passed    (%.2f)\n", psnr_3);
  else
    printf("FAILED, psnr = %f\n", psnr_3);

  printf("  7.4. %-64s", "default and NeuralRT GPU coop matrices PSNR > 45 ");
  if (psnr_4 >= 45)
    printf("passed    (%.2f)\n", psnr_4);
  else
    printf("FAILED, psnr = %f\n", psnr_4);

  printf("  7.5. %-64s", "default and SDF octree PSNR > 25 ");
  if (psnr_5 >= 25)
    printf("passed    (%.2f)\n", psnr_5);
  else
    printf("FAILED, psnr = %f\n", psnr_5);

  printf("timings: reference = %f; default = %f; blocked = %f; coop matrices = %f; octree = %f\n", 
         timings[0][0], timings[1][0], timings[2][0], timings[3][0], timings[4][0]);
}

void litert_test_8_SDF_grid()
{
  //create renderers for SDF scene and mesh scene
  const char *scene_name = "scenes/01_simple_scenes/teapot.xml";
  unsigned W = 2048, H = 2048;

  MultiRenderPreset preset = getDefaultPreset();
  preset.mode = MULTI_RENDER_MODE_LAMBERT;
  preset.sdf_frame_octree_blas = SDF_OCTREE_BLAS_DEFAULT;
  preset.sdf_frame_octree_intersect = SDF_OCTREE_NODE_INTERSECT_ANALYTIC;
  LiteImage::Image2D<uint32_t> image(W, H);
  LiteImage::Image2D<uint32_t> ref_image(W, H);

  auto pRenderRef = CreateMultiRenderer("GPU");
  pRenderRef->SetPreset(preset);
  pRenderRef->SetViewport(0,0,W,H);
  pRenderRef->LoadSceneHydra((scenes_folder_path+scene_name).c_str());

  auto pRender = CreateMultiRenderer("GPU");
  pRender->SetPreset(preset);
  pRender->SetViewport(0,0,W,H);
  pRender->LoadSceneHydra((scenes_folder_path+scene_name).c_str(), TYPE_SDF_GRID, 
                          SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 7));

  auto m1 = pRender->getWorldView();
  auto m2 = pRender->getProj();

  pRender->Render(image.data(), image.width(), image.height(), m1, m2, preset);
  pRenderRef->Render(ref_image.data(), ref_image.width(), ref_image.height(), m1, m2, preset);

  LiteImage::SaveImage<uint32_t>("saves/test_8_res.bmp", image); 
  LiteImage::SaveImage<uint32_t>("saves/test_8_ref.bmp", ref_image);

  float psnr = PSNR(ref_image, image);
  printf("TEST 8. Rendering Hydra scene\n");
  printf("  8.1. %-64s", "mesh and SDF grid PSNR > 30 ");
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

  float psnr = PSNR(ref_image, image);
  printf("TEST 9. Rendering simple mesh\n");
  printf("  9.1. %-64s", "CPU and GPU render PSNR > 45 ");
  if (psnr >= 45)
    printf("passed    (%.2f)\n", psnr);
  else
    printf("FAILED, psnr = %f\n", psnr);
}

// save and load octrees of all types
void litert_test_10_save_load()
{
  auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path + "scenes/01_simple_scenes/data/teapot.vsgf").c_str());

  float3 mb1, mb2, ma1, ma2;
  cmesh4::get_bbox(mesh, &mb1, &mb2);
  cmesh4::rescale_mesh(mesh, float3(-0.9, -0.9, -0.9), float3(0.9, 0.9, 0.9));
  cmesh4::get_bbox(mesh, &ma1, &ma2);

  printf("total triangles %d\n", (int)mesh.TrianglesNum());
  printf("bbox [(%f %f %f)-(%f %f %f)] to [(%f %f %f)-(%f %f %f)]\n",
         mb1.x, mb1.y, mb1.z, mb2.x, mb2.y, mb2.z, ma1.x, ma1.y, ma1.z, ma2.x, ma2.y, ma2.z);
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  {
    SparseOctreeBuilder builder;
    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 7);

    std::vector<SdfOctreeNode> octree_nodes;
    std::vector<SdfFrameOctreeNode> frame_nodes;
    std::vector<SdfSVSNode> svs_nodes;
    SdfSBS sbs;
    sbs.header = SdfSBSHeader{1, 0, 1, 2};

    builder.construct([&mesh_bvh](const float3 &p)
                      { return mesh_bvh.get_signed_distance(p); },
                      settings);
    octree_nodes = builder.get_nodes();
    builder.convert_to_frame_octree(frame_nodes);
    builder.convert_to_sparse_voxel_set(svs_nodes);
    builder.convert_to_sparse_brick_set(sbs.header, sbs.nodes, sbs.values);

    save_sdf_octree({(unsigned)octree_nodes.size(), octree_nodes.data()}, "saves/test_10_octree.bin");
    save_sdf_frame_octree({(unsigned)frame_nodes.size(), frame_nodes.data()}, "saves/test_10_frame_octree.bin");
    save_sdf_SVS({(unsigned)svs_nodes.size(), svs_nodes.data()}, "saves/test_10_svs.bin");
    save_sdf_SBS(sbs, "saves/test_10_sbs.bin");
  }

  std::vector<SdfOctreeNode> octree_nodes;
  std::vector<SdfFrameOctreeNode> frame_nodes;
  std::vector<SdfSVSNode> svs_nodes;
  SdfSBS sbs;

  load_sdf_octree(octree_nodes, "saves/test_10_octree.bin");
  load_sdf_frame_octree(frame_nodes, "saves/test_10_frame_octree.bin");
  load_sdf_SVS(svs_nodes, "saves/test_10_svs.bin");
  load_sdf_SBS(sbs, "saves/test_10_sbs.bin");

  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.mode = MULTI_RENDER_MODE_LAMBERT;
  preset.sdf_octree_sampler = SDF_OCTREE_SAMPLER_MIPSKIP_3X3;

  LiteImage::Image2D<uint32_t> image_ref(W, H);
  auto pRender_ref = CreateMultiRenderer("GPU");
  pRender_ref->SetPreset(preset);
  pRender_ref->LoadSceneHydra("scenes/01_simple_scenes/teapot.xml");
  render(image_ref, pRender_ref, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_ref.bmp", image_ref);

  LiteImage::Image2D<uint32_t> image_1(W, H);
  auto pRender_1 = CreateMultiRenderer("GPU");
  pRender_1->SetPreset(preset);
  pRender_1->SetScene({(unsigned)octree_nodes.size(), octree_nodes.data()});
  render(image_1, pRender_1, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_octree.bmp", image_1);

  LiteImage::Image2D<uint32_t> image_2(W, H);
  auto pRender_2 = CreateMultiRenderer("GPU");
  pRender_2->SetPreset(preset);
  pRender_2->SetScene({(unsigned)frame_nodes.size(), frame_nodes.data()});
  render(image_2, pRender_2, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_frame_octree.bmp", image_2);

  LiteImage::Image2D<uint32_t> image_3(W, H);
  auto pRender_3 = CreateMultiRenderer("GPU");
  pRender_3->SetPreset(preset);
  pRender_3->SetScene({(unsigned)svs_nodes.size(), svs_nodes.data()});
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
  pRender_5->LoadSceneHydra("scenes/02_sdf_scenes/teapot_svs.xml");
  render(image_5, pRender_5, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset);
  LiteImage::SaveImage<uint32_t>("saves/test_10_hydra_scene.bmp", image_5);

  float psnr_1 = PSNR(image_1, image_1);
  float psnr_2 = PSNR(image_1, image_2);
  float psnr_3 = PSNR(image_1, image_3);
  float psnr_4 = PSNR(image_1, image_4);
  float psnr_5 = PSNR(image_1, image_5);

  printf(" 10.1. %-64s", "SDF Octree ");
  if (psnr_1 >= 25)
    printf("passed    (%.2f)\n", psnr_1);
  else
    printf("FAILED, psnr = %f\n", psnr_1);

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
  
  //printf(" 10.5. %-64s", "SDF Scene loaded from Hydra scene");
  //if (psnr_5 >= 25)
  //  printf("passed    (%.2f)\n", psnr_5);
  //else
  //  printf("FAILED, psnr = %f\n", psnr_5);
}

static double urand(double from=0, double to=1)
{
  return ((double)rand() / RAND_MAX) * (to - from) + from;
}
void litert_test_11_hp_octree_legacy()
{
  HPOctreeBuilder builder;
  builder.readLegacy(scenes_folder_path+"scenes/02_sdf_scenes/sphere_hp_legacy.bin");

  double diff = 0.0;
  int cnt = 10000;
  for (int i = 0; i < cnt; i++)
  {
    float3 rnd_pos = float3(urand(-0.5f, 0.5f), urand(-0.5f, 0.5f), urand(-0.5f, 0.5f));
    float dist_real = length(rnd_pos) - 0.5f;
    float dist = builder.QueryLegacy(rnd_pos);
    diff += abs(dist_real - dist);
    //printf("%.4f - %.4f = %.4f\n", dist_real, dist, dist_real - dist);
  }
  diff /= cnt;
  printf("TEST 11. hp-Octree legacy\n");
  printf(" 11.1. %-64s", "reading from Legacy format ");
  if (diff < 1e-4f)
    printf("passed    (%f)\n", diff);
  else
    printf("FAILED, diff = %f\n", diff);
}

void perform_tests_litert(const std::vector<int> &test_ids)
{
  std::vector<int> tests = test_ids;

  std::vector<std::function<void(void)>> test_functions = {
      litert_test_1_framed_octree, litert_test_2_SVS, litert_test_3_SBS_verify,
      litert_test_4_hydra_scene, litert_test_5_interval_tracing, litert_test_6_faster_bvh_build,
      test_7_neural_SDF, litert_test_8_SDF_grid, litert_test_9_mesh, 
      litert_test_10_save_load, litert_test_11_hp_octree_legacy};

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