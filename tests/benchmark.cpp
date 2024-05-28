#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "../utils/sparse_octree.h"
#include "../utils/hp_octree.h"
#include "../utils/sdf_converter.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"

#include <functional>
#include <cassert>
#include <chrono>
#include <map>

void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum);
float PSNR(const LiteImage::Image2D<uint32_t> &image_1, const LiteImage::Image2D<uint32_t> &image_2);

struct BenchmarkResult
{
  unsigned iters;
  float4 render_average_time_ms;
  float4 render_min_time_ms;
  std::string scene_name;
  std::string render_name;
  std::string as_name;
  std::string preset_name;
};

void benchmark_framed_octree_intersection()
{
  constexpr unsigned iters = 3;
  constexpr unsigned pass_size = 50;
  SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
  SdfSBSHeader header;
  header.brick_size = 1;
  header.brick_pad = 0;
  header.bytes_per_value = 1;
  unsigned W = 1024, H = 1024;
  bool save_images = true;

  std::vector<BenchmarkResult> results;

  std::vector<std::string> scene_paths = {"scenes/01_simple_scenes/data/teapot.vsgf"}; 
  std::vector<std::string> scene_names = {"Teapot", "Bunny"};

  std::vector<unsigned> render_modes = {MULTI_RENDER_MODE_LAMBERT};
  std::vector<std::string> render_names = {"lambert"};

  std::vector<unsigned> AS_types = {TYPE_SDF_FRAME_OCTREE, TYPE_SDF_SVS, TYPE_SDF_HP, TYPE_SDF_SBS, TYPE_MESH_TRIANGLE};
  std::vector<std::string> AS_names = {"framed_octree", "sparse_voxel_set", "hp-adaptive_octree","sparse_brick_set", "mesh"};

  std::vector<std::vector<unsigned>> presets_ob(5);
  std::vector<std::vector<unsigned>> presets_oi(5);
  std::vector<std::vector<std::string>> preset_names(5);

  presets_ob[0] = {SDF_OCTREE_BLAS_NO, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT,
                   SDF_OCTREE_BLAS_DEFAULT,
                   SDF_OCTREE_BLAS_DEFAULT};

  presets_oi[0] = {SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                   SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                   SDF_OCTREE_NODE_INTERSECT_ST, 
                   SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                   SDF_OCTREE_NODE_INTERSECT_NEWTON,
                   SDF_OCTREE_NODE_INTERSECT_IT,
                   SDF_OCTREE_NODE_INTERSECT_BBOX};

  preset_names[0] = {"no_bvh_traversal",
                     "bvh_traversal",
                     "bvh_sphere_tracing",
                     "bvh_analytic",
                     "bvh_newton",
                     "bvh_interval_tracing",
                     "bvh_nodes"};

  presets_ob[1] = {SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT,
                   SDF_OCTREE_BLAS_DEFAULT,
                   SDF_OCTREE_BLAS_DEFAULT};

  presets_oi[1] = {SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                   SDF_OCTREE_NODE_INTERSECT_ST, 
                   SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                   SDF_OCTREE_NODE_INTERSECT_NEWTON,
                   SDF_OCTREE_NODE_INTERSECT_IT,
                   SDF_OCTREE_NODE_INTERSECT_BBOX};

  preset_names[1] = {"bvh_traversal",
                     "bvh_sphere_tracing",
                     "bvh_analytic",
                     "bvh_newton",
                     "bvh_interval_tracing",
                     "bvh_nodes"};

  presets_ob[2] = {SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT, 
                   SDF_OCTREE_BLAS_DEFAULT,
                   SDF_OCTREE_BLAS_DEFAULT,
                   SDF_OCTREE_BLAS_DEFAULT};

  presets_oi[2] = {SDF_OCTREE_NODE_INTERSECT_DEFAULT, 
                   SDF_OCTREE_NODE_INTERSECT_ST, 
                   SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                   SDF_OCTREE_NODE_INTERSECT_NEWTON,
                   SDF_OCTREE_NODE_INTERSECT_IT,
                   SDF_OCTREE_NODE_INTERSECT_BBOX};

  preset_names[2] = {"bvh_traversal",
                     "bvh_sphere_tracing",
                     "bvh_analytic",
                     "bvh_newton",
                     "bvh_interval_tracing",
                     "bvh_nodes"};

  presets_ob[3] = {SDF_OCTREE_BLAS_DEFAULT};
  presets_oi[3] = {SDF_OCTREE_NODE_INTERSECT_ST};
  preset_names[3] = {"bvh_sphere_tracing"};

  presets_ob[4] = {SDF_OCTREE_BLAS_DEFAULT};
  presets_oi[4] = {SDF_OCTREE_NODE_INTERSECT_DEFAULT};
  preset_names[4] = {"default"};

  assert(scene_names.size() >= scene_paths.size());
  assert(render_modes.size() >= render_names.size());

  assert(AS_types.size() >= AS_names.size());
  assert(AS_types.size() >= presets_ob.size());
  assert(AS_types.size() >= presets_oi.size());
  assert(AS_types.size() >= preset_names.size());

  for (int scene_n = 0; scene_n < scene_paths.size(); scene_n++)
  {
    auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+scene_paths[scene_n]).c_str());

    float3 mb1,mb2, ma1,ma2;
    cmesh4::get_bbox(mesh, &mb1, &mb2);
    cmesh4::rescale_mesh(mesh, float3(-0.9,-0.9,-0.9), float3(0.9,0.9,0.9));
    cmesh4::get_bbox(mesh, &ma1, &ma2);
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    SparseOctreeBuilder builder;

    std::vector<SdfFrameOctreeNode> frame_nodes;
    std::vector<SdfSVSNode> svs_nodes;
    std::vector<SdfSBSNode> sbs_nodes;
    std::vector<uint32_t>   sbs_data;

    builder.construct([&mesh_bvh](const float3 &p) { return mesh_bvh.get_signed_distance(p); }, settings);
    builder.convert_to_frame_octree(frame_nodes);
    builder.convert_to_sparse_voxel_set(svs_nodes);
    builder.convert_to_sparse_brick_set(header, sbs_nodes, sbs_data);
    
    HPOctreeBuilder hp_builder;
    hp_builder.construct(mesh);

    LiteImage::Image2D<uint32_t> image(W, H);

    for (int rm=0; rm<render_names.size(); rm++)
    {
      for (int as_n=0; as_n<AS_types.size(); as_n++)
      {
        for (int i=0; i<presets_ob[as_n].size(); i++)
        {
          MultiRenderPreset preset = getDefaultPreset();
          preset.mode = render_modes[rm];
          preset.sdf_frame_octree_blas = presets_ob[as_n][i];
          preset.sdf_frame_octree_intersect = presets_oi[as_n][i];

          auto pRender = CreateMultiRenderer("GPU");
          pRender->SetPreset(preset);
          if (AS_types[as_n] == TYPE_SDF_FRAME_OCTREE)
            pRender->SetScene({(unsigned)frame_nodes.size(), frame_nodes.data()});
          else if (AS_types[as_n] == TYPE_SDF_SVS) 
            pRender->SetScene({(unsigned)svs_nodes.size(), svs_nodes.data()});
          else if (AS_types[as_n] == TYPE_MESH_TRIANGLE) 
            pRender->SetScene(mesh);
          else if (AS_types[as_n] == TYPE_SDF_SBS)
            pRender->SetScene(SdfSBSView(header, sbs_nodes, sbs_data));
          else if (AS_types[as_n] == TYPE_SDF_HP)
            pRender->SetScene(SdfHPOctreeView(hp_builder.octree.nodes, hp_builder.octree.data));

          double sum_ms[4] = {0,0,0,0};
          double min_ms[4] = {1e6,1e6,1e6,1e6};
          double max_ms[4] = {0,0,0,0};
          render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset, pass_size);
          for (int iter = 0; iter<iters; iter++)
          {
            float timings[4] = {0,0,0,0};
            
            auto t1 = std::chrono::steady_clock::now();
            pRender->Render(image.data(), image.width(), image.height(), "color", pass_size); 
            pRender->GetExecutionTime("CastRaySingleBlock", timings);
            auto t2 = std::chrono::steady_clock::now();
            
            float time_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();
            //printf("%s rendered in %.1f ms. %d kRays/s\n", "SDF Framed Octree", time_ms, (int)((W * H) / time_ms));
            //printf("CastRaySingleBlock took %.1f ms\n", timings[0]);

            if (iter == 0 && save_images)
              LiteImage::SaveImage<uint32_t>(("saves/benchmark_"+scene_names[scene_n]+"_"+render_names[rm]+"_"+AS_names[as_n]+"_"+preset_names[as_n][i]+".bmp").c_str(), image); 
            for (int i=0;i<4;i++)
            {
              min_ms[i] = std::min(min_ms[i], (double)timings[i]);
              max_ms[i] = std::max(max_ms[i], (double)timings[i]);
              sum_ms[i] += timings[i];
            }
          }

          results.emplace_back();
          results.back().scene_name = scene_names[scene_n];
          results.back().render_name = render_names[rm];
          results.back().as_name = AS_names[as_n];
          results.back().preset_name = preset_names[as_n][i];
          results.back().iters = iters;
          results.back().render_average_time_ms = float4(sum_ms[0], sum_ms[1], sum_ms[2], sum_ms[3])/(iters*pass_size);
          results.back().render_min_time_ms = float4(min_ms[0], min_ms[1], min_ms[2], min_ms[3])/pass_size;
        }
      }
    }
  }

  for (auto &res : results)
  {
    printf("[%10s + %10s + %20s + %20s] min:%6.2f + %5.2f, av:%6.2f + %5.2f ms/frame \n", 
           res.scene_name.c_str(), res.render_name.c_str(), res.as_name.c_str(), res.preset_name.c_str(), 
           res.render_min_time_ms.x,
           res.render_min_time_ms.y + res.render_min_time_ms.z + res.render_min_time_ms.w,
           res.render_average_time_ms.x,
           res.render_average_time_ms.y + res.render_average_time_ms.z + res.render_average_time_ms.w);
  }
}

void quality_check(const char *path)
{
  auto mesh = cmesh4::LoadMeshFromVSGF(path);

  float3 mb1, mb2, ma1, ma2;
  cmesh4::get_bbox(mesh, &mb1, &mb2);
  cmesh4::rescale_mesh(mesh, float3(-0.99, -0.99, -0.99), float3(0.99, 0.99, 0.99));
  cmesh4::get_bbox(mesh, &ma1, &ma2);

  printf("total triangles %d\n", (int)mesh.TrianglesNum());
  printf("bbox [(%f %f %f)-(%f %f %f)] to [(%f %f %f)-(%f %f %f)]\n",
         mb1.x, mb1.y, mb1.z, mb2.x, mb2.y, mb2.z, ma1.x, ma1.y, ma1.z, ma2.x, ma2.y, ma2.z);
  MeshBVH mesh_bvh;
  mesh_bvh.init(mesh);

  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.mode = MULTI_RENDER_MODE_PHONG;
  preset.sdf_octree_sampler = SDF_OCTREE_SAMPLER_MIPSKIP_3X3;

  LiteImage::Image2D<uint32_t> image_ref(W, H);
  auto pRender_ref = CreateMultiRenderer("GPU");
  pRender_ref->SetPreset(preset);
  pRender_ref->SetScene(mesh);
  render(image_ref, pRender_ref, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset, 1);
  LiteImage::SaveImage<uint32_t>("saves/ref.bmp", image_ref);

  for (int depth = 6; depth <= 10; depth++)
  {
    std::vector<SdfSVSNode> svs_nodes;
    SparseOctreeBuilder builder;
    SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, depth);

    builder.construct([&mesh_bvh](const float3 &p)
                      { return mesh_bvh.get_signed_distance(p); },
                      settings);
    builder.convert_to_sparse_voxel_set(svs_nodes);

    save_sdf_SVS({(unsigned)svs_nodes.size(), svs_nodes.data()}, 
                 ("saves/svs_"+std::to_string(depth)+".bin").c_str());

    LiteImage::Image2D<uint32_t> image_1(W, H);
    auto pRender_1 = CreateMultiRenderer("GPU");
    pRender_1->SetPreset(preset);
    pRender_1->SetScene({(unsigned)svs_nodes.size(), svs_nodes.data()});
    render(image_1, pRender_1, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset, 1);
    LiteImage::SaveImage<uint32_t>(("saves/svs_"+std::to_string(depth)+".bmp").c_str(), 
                                   image_1);

    float psnr_1 = PSNR(image_ref, image_1);
    printf("depth = %d PSNR = %f\n", depth, psnr_1);
  }
}

void hydra_benchmark(const std::string &path)
{
  struct StructureInfo
  {
    unsigned nodes;
    unsigned memory;
    unsigned max_depth;
    float build_time_ms;
    bool valid = false;
  };

  const std::string mesh_name = "teapot";
  const std::string mesh_path = path + "/mesh.vsgf";
  auto mesh = cmesh4::LoadMeshFromVSGF(mesh_path.c_str());
  cmesh4::normalize_mesh(mesh);
  MeshBVH mesh_bvh;

  auto t1 = std::chrono::steady_clock::now();
  mesh_bvh.init(mesh);
  auto t2 = std::chrono::steady_clock::now();
  float mesh_bvh_build_time  = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

  std::map<std::string, StructureInfo> building_results;

  std::vector<std::string> structures =          {"sdf_grid", "sdf_octree", "sdf_frame_octree", "sdf_SVS", "sdf_SBS-2-1", "sdf_SBS-2-2", "sdf_hp_octree"};
  std::vector<unsigned> average_bytes_per_node = {         4,            8,                 36,        16,            44,            72,              71};
  
  std::vector<unsigned> max_depths = {    8,   8,     9,    10};
  std::vector<float> size_limit_Mb = {0.25f, 1.0f, 4.0f, 16.0f};
  
  for (int d_id = 0; d_id < max_depths.size(); d_id++)
  {
    unsigned max_depth = max_depths[d_id];

    for (int s_id = 0; s_id < structures.size(); s_id++)
    {
      unsigned nodes_limit = size_limit_Mb[d_id] * 1000 * 1000 / average_bytes_per_node[s_id];
      //printf("nodes_limit = %u\n", nodes_limit);
      std::string structure = structures[s_id];
      std::string full_name = mesh_name + "_" + structure + "_" + std::to_string(size_limit_Mb[d_id]);
      std::string filename = path + "/" + full_name + ".bin";
      StructureInfo res;
      res.max_depth = max_depth;

      if (structure == "sdf_grid")
      {
        unsigned size = cbrt(nodes_limit);
        t1 = std::chrono::steady_clock::now();
        auto grid = sdf_converter::create_sdf_grid(GridSettings(size), mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time + mesh_bvh_build_time;
        res.nodes = grid.data.size();
        res.memory = sizeof(float) * res.nodes;
        res.valid = true;

        save_sdf_grid(grid, filename);
      }
      else if (structure == "sdf_hp_octree")
      {
        HPOctreeBuilder::BuildSettings settings;
        settings.threads = 16;
        settings.target_error = 0.0f;
        settings.nodesLimit = 9*nodes_limit/8 + 2000;

        t1 = std::chrono::steady_clock::now();
        auto scene = sdf_converter::create_sdf_hp_octree(settings, mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time;
        res.nodes = scene.nodes.size();
        res.memory = sizeof(SdfHPOctreeNode) * scene.nodes.size() + sizeof(float)*scene.data.size();
        res.valid = true;

        save_sdf_hp_octree(scene, filename);
      }
      else if (structure == "sdf_octree")
      {
        unsigned all_nodes_limit = 8*nodes_limit/9;
        t1 = std::chrono::steady_clock::now();
        auto scene = sdf_converter::create_sdf_octree(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, max_depth, all_nodes_limit), mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time + mesh_bvh_build_time;
        res.nodes = scene.size();
        res.memory = sizeof(SdfOctreeNode) * res.nodes;
        res.valid = true;

        save_sdf_octree({(unsigned)scene.size(), scene.data()}, filename);
      }
      else if (structure == "sdf_frame_octree")
      {
        unsigned all_nodes_limit = 8*nodes_limit/9;
        t1 = std::chrono::steady_clock::now();
        auto scene = sdf_converter::create_sdf_frame_octree(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, max_depth, all_nodes_limit), mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time + mesh_bvh_build_time;
        res.nodes = scene.size();
        res.memory = sizeof(SdfFrameOctreeNode) * res.nodes;
        res.valid = true;

        save_sdf_frame_octree({(unsigned)scene.size(), scene.data()}, filename);
      }
      else if (structure == "sdf_SVS")
      {
        t1 = std::chrono::steady_clock::now();
        auto scene = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, max_depth, nodes_limit), mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time + mesh_bvh_build_time;
        res.nodes = scene.size();
        res.memory = sizeof(SdfSVSNode) * res.nodes;
        res.valid = true;

        save_sdf_SVS({(unsigned)scene.size(), scene.data()}, filename);
      }
      else if (structure == "sdf_SBS-2-1")
      {
        SdfSBSHeader header;
        header.brick_size = 2;
        header.brick_pad = 0;
        header.bytes_per_value = 1;

        t1 = std::chrono::steady_clock::now();
        auto scene = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, max_depth, nodes_limit), header, mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time + mesh_bvh_build_time;
        res.nodes = scene.nodes.size();
        res.memory = sizeof(SdfSBSNode) * res.nodes + sizeof(uint32_t) * scene.values.size();
        res.valid = true;

        save_sdf_SBS(scene, filename);
      }
      else if (structure == "sdf_SBS-2-2")
      {
        SdfSBSHeader header;
        header.brick_size = 2;
        header.brick_pad = 0;
        header.bytes_per_value = 2;

        t1 = std::chrono::steady_clock::now();
        auto scene = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, max_depth, nodes_limit), header, mesh);
        t2 = std::chrono::steady_clock::now();
        float build_time = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

        res.build_time_ms = build_time + mesh_bvh_build_time;
        res.nodes = scene.nodes.size();
        res.memory = sizeof(SdfSBSNode) * res.nodes + sizeof(uint32_t) * scene.values.size();
        res.valid = true;

        save_sdf_SBS(scene, filename);
      }

      building_results[full_name] = res;
      printf("  %32s: %6u nodes, %8u kb, %5.1f s\n", full_name.c_str(), res.nodes, res.memory / 1000, res.build_time_ms/1000.0f);
    }
    printf("\n");
  }
}