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

std::string scenes_folder_path = "./";

void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset)
{
  float fov_degrees = 60;
  float z_near = 0.1f;
  float z_far = 100.0f;
  float aspect   = 1.0f;
  auto proj      = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
  auto worldView = LiteMath::lookAt(pos, target, up);

  pRender->Render(image.data(), image.width(), image.height(), worldView, proj, preset);
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
    SparseOctreeSettings settings{9, 4, 0.0f};
    std::vector<SdfFrameOctreeNode> frame_nodes;

    builder.construct([&mesh_bvh](const float3 &p) { return mesh_bvh.get_signed_distance(p); }, settings);
    builder.convert_to_frame_octree(frame_nodes);

    unsigned W = 2048, H = 2048;
    LiteImage::Image2D<uint32_t> image(W, H);
    float timings[4] = {0,0,0,0};

    std::vector<unsigned> presets_ob = {SDF_FRAME_OCTREE_BLAS_NO, SDF_FRAME_OCTREE_BLAS_DEFAULT,
                                        SDF_FRAME_OCTREE_BLAS_DEFAULT, SDF_FRAME_OCTREE_BLAS_DEFAULT,
                                        SDF_FRAME_OCTREE_BLAS_DEFAULT, SDF_FRAME_OCTREE_BLAS_DEFAULT};

    std::vector<unsigned> presets_oi = {SDF_FRAME_OCTREE_INTERSECT_DEFAULT, SDF_FRAME_OCTREE_INTERSECT_DEFAULT, 
                                        SDF_FRAME_OCTREE_INTERSECT_ST, SDF_FRAME_OCTREE_INTERSECT_ANALYTIC, 
                                        SDF_FRAME_OCTREE_INTERSECT_NEWTON, SDF_FRAME_OCTREE_INTERSECT_BBOX};

    std::vector<std::string> names = {"no_bvh_traversal", "bvh_traversal", "bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

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
    SparseOctreeSettings settings{9, 4, 0.0f};
    std::vector<SdfSVSNode> frame_nodes;

    builder.construct([&mesh_bvh](const float3 &p) { return mesh_bvh.get_signed_distance(p); }, settings);
    builder.convert_to_sparse_voxel_set(frame_nodes);

    unsigned W = 2048, H = 2048;
    LiteImage::Image2D<uint32_t> image(W, H);
    float timings[4] = {0,0,0,0};

    std::vector<unsigned> presets_ob = {SDF_FRAME_OCTREE_BLAS_NO, SDF_FRAME_OCTREE_BLAS_DEFAULT,
                                        SDF_FRAME_OCTREE_BLAS_DEFAULT, SDF_FRAME_OCTREE_BLAS_DEFAULT,
                                        SDF_FRAME_OCTREE_BLAS_DEFAULT, SDF_FRAME_OCTREE_BLAS_DEFAULT};

    std::vector<unsigned> presets_oi = {SDF_FRAME_OCTREE_INTERSECT_DEFAULT, SDF_FRAME_OCTREE_INTERSECT_DEFAULT, 
                                        SDF_FRAME_OCTREE_INTERSECT_ST, SDF_FRAME_OCTREE_INTERSECT_ANALYTIC, 
                                        SDF_FRAME_OCTREE_INTERSECT_NEWTON, SDF_FRAME_OCTREE_INTERSECT_BBOX};

    std::vector<std::string> names = {"no_bvh_traversal", "bvh_traversal", "bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

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

void perform_tests_litert(const std::vector<int> &test_ids)
{
  std::vector<int> tests = test_ids;

  std::vector<std::function<void(void)>> test_functions = {
      litert_test_1_framed_octree, litert_test_2_SVS};

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