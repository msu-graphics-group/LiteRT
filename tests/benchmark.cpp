#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "../utils/sdf_converter.h"
#include "../utils/image_metrics.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"
#include <filesystem>

#include <functional>
#include <cassert>
#include <chrono>
#include <map>

void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum);

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

  std::vector<unsigned> render_modes = {MULTI_RENDER_MODE_LAMBERT_NO_TEX};
  std::vector<std::string> render_names = {"lambert"};

  std::vector<unsigned> AS_types = {TYPE_SDF_FRAME_OCTREE, TYPE_SDF_SVS, TYPE_SDF_SBS, TYPE_MESH_TRIANGLE};
  std::vector<std::string> AS_names = {"framed_octree", "sparse_voxel_set", "sparse_brick_set", "mesh"};

  std::vector<std::vector<unsigned>> presets_oi(5);
  std::vector<std::vector<std::string>> preset_names(5);

  presets_oi[0] = {SDF_OCTREE_NODE_INTERSECT_ST, 
                   SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                   SDF_OCTREE_NODE_INTERSECT_NEWTON,
                   SDF_OCTREE_NODE_INTERSECT_IT,
                   SDF_OCTREE_NODE_INTERSECT_BBOX};

  preset_names[0] = {"bvh_sphere_tracing",
                     "bvh_analytic",
                     "bvh_newton",
                     "bvh_interval_tracing",
                     "bvh_nodes"};

  presets_oi[1] = {SDF_OCTREE_NODE_INTERSECT_ST, 
                   SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                   SDF_OCTREE_NODE_INTERSECT_NEWTON,
                   SDF_OCTREE_NODE_INTERSECT_IT,
                   SDF_OCTREE_NODE_INTERSECT_BBOX};

  preset_names[1] = {"bvh_sphere_tracing",
                     "bvh_analytic",
                     "bvh_newton",
                     "bvh_interval_tracing",
                     "bvh_nodes"};

  presets_oi[2] = {SDF_OCTREE_NODE_INTERSECT_ST, 
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

  presets_oi[3] = {SDF_OCTREE_NODE_INTERSECT_ST};
  preset_names[3] = {"bvh_sphere_tracing"};

  presets_oi[4] = {SDF_OCTREE_NODE_INTERSECT_ST};
  preset_names[4] = {"default"};

  assert(scene_names.size() >= scene_paths.size());
  assert(render_modes.size() >= render_names.size());

  assert(AS_types.size() >= AS_names.size());
  assert(AS_types.size() >= presets_oi.size());
  assert(AS_types.size() >= preset_names.size());

  for (int scene_n = 0; scene_n < scene_paths.size(); scene_n++)
  {
    auto mesh = cmesh4::LoadMeshFromVSGF((scenes_folder_path+scene_paths[scene_n]).c_str());
    cmesh4::rescale_mesh(mesh, float3(-0.9,-0.9,-0.9), float3(0.9,0.9,0.9));

    float3 mb1,mb2, ma1,ma2;
    cmesh4::get_bbox(mesh, &mb1, &mb2);
    cmesh4::rescale_mesh(mesh, float3(-0.9,-0.9,-0.9), float3(0.9,0.9,0.9));
    cmesh4::get_bbox(mesh, &ma1, &ma2);
    MeshBVH mesh_bvh;
    mesh_bvh.init(mesh);

    std::vector<SdfFrameOctreeNode> frame_nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
    std::vector<SdfSVSNode> svs_nodes = sdf_converter::create_sdf_SVS(settings, mesh);
    SdfSBS sbs = sdf_converter::create_sdf_SBS(settings, header, mesh);

    LiteImage::Image2D<uint32_t> image(W, H);

    for (int rm=0; rm<render_names.size(); rm++)
    {
      for (int as_n=0; as_n<AS_types.size(); as_n++)
      {
        for (int i=0; i<presets_oi[as_n].size(); i++)
        {
          MultiRenderPreset preset = getDefaultPreset();
          preset.render_mode = render_modes[rm];
          preset.sdf_node_intersect = presets_oi[as_n][i];

          auto pRender = CreateMultiRenderer(DEVICE_GPU);
          pRender->SetPreset(preset);
          if (AS_types[as_n] == TYPE_SDF_FRAME_OCTREE)
            pRender->SetScene(frame_nodes);
          else if (AS_types[as_n] == TYPE_SDF_SVS) 
            pRender->SetScene(svs_nodes);
          else if (AS_types[as_n] == TYPE_MESH_TRIANGLE) 
            pRender->SetScene(mesh);
          else if (AS_types[as_n] == TYPE_SDF_SBS)
            pRender->SetScene(sbs);

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
  cmesh4::rescale_mesh(mesh, float3(-0.99, -0.99, -0.99), float3(0.99, 0.99, 0.99));

  unsigned W = 1024, H = 1024;
  MultiRenderPreset preset = getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_PHONG_NO_TEX;

  LiteImage::Image2D<uint32_t> image_ref(W, H);
  auto pRender_ref = CreateMultiRenderer(DEVICE_GPU);
  pRender_ref->SetPreset(preset);
  pRender_ref->SetScene(mesh);
  render(image_ref, pRender_ref, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset, 1);
  LiteImage::SaveImage<uint32_t>("saves/ref.bmp", image_ref);

  for (int depth = 6; depth <= 10; depth++)
  {
    SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, depth);
    std::vector<SdfSVSNode> svs_nodes = sdf_converter::create_sdf_SVS(settings, mesh);

    save_sdf_SVS(svs_nodes, ("saves/svs_"+std::to_string(depth)+".bin").c_str());

    LiteImage::Image2D<uint32_t> image_1(W, H);
    auto pRender_1 = CreateMultiRenderer(DEVICE_GPU);
    pRender_1->SetPreset(preset);
    pRender_1->SetScene(svs_nodes);
    render(image_1, pRender_1, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0), preset, 1);
    LiteImage::SaveImage<uint32_t>(("saves/svs_"+std::to_string(depth)+".bmp").c_str(), image_1);

    float psnr_1 = image_metrics::PSNR(image_ref, image_1);
    printf("depth = %d image_metrics::PSNR = %f\n", depth, psnr_1);
  }
}

void direct_test(std::string path, std::string type, MultiRenderPreset preset,
                 float rotation_angle, unsigned width, unsigned height, unsigned spp, 
                 float *out_timings, std::vector<uint32_t> &out_image);

void main_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags,
                     std::string image_prefix,
                     std::vector<std::string> use_structure,
                     std::vector<std::string> use_size,
                     std::vector<std::string> use_intersect,
                     unsigned base_pass_size = 25,
                     unsigned base_iters = 10,
                     unsigned render_device = DEVICE_GPU)
{
  struct StructureInfo
  {
    unsigned nodes = 0;
    unsigned memory = 0;
    unsigned max_depth = 0;
    float build_time_ms = 0.0f;
    bool valid = false;
  };

  unsigned W = 2048, H = 2048;
  unsigned hydra_spp = 256;
  const std::string mesh_path = path + "/mesh.vsgf";
  auto mesh = cmesh4::LoadMeshFromVSGF(mesh_path.c_str());
  cmesh4::normalize_mesh(mesh);
  MeshBVH mesh_bvh;
  auto t1 = std::chrono::steady_clock::now();
  mesh_bvh.init(mesh);
  auto t2 = std::chrono::steady_clock::now();
  float mesh_bvh_build_time  = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count();

  const std::string results_file_path = path + "/results.txt";
  FILE* log_fd = fopen(results_file_path.c_str(), "a+");

  //different types of structures
  std::vector<std::string> structures =          {           "mesh",        "mesh_lod", "sdf_grid", "sdf_octree", "sdf_frame_octree", "sdf_SVS", "sdf_SBS_tricubic", "sdf_SBS-2-1", "sdf_SBS-2-2", "sdf_SBS-3-1", "sdf_SBS-3-1_SN"};
  std::vector<std::string> structure_types =     {"normalized_mesh", "normalized_mesh", "sdf_grid", "sdf_octree", "sdf_frame_octree", "sdf_svs",          "sdf_sbs",     "sdf_sbs",     "sdf_sbs",     "sdf_sbs",        "sdf_sbs"};
  std::vector<unsigned> average_bytes_per_node = {                0,                 0,          4,            8,                 36,        16,                44,             44,            72,            80,               80};
  
  //different sizes
  std::vector<unsigned> max_depths =          {      7,      7,      7,     8,     8,     9,     9,     10,     10,     11};
  std::vector<float> size_limit_Mb =          { 0.125f,  0.25f,   0.5f,  1.0f,  2.0f,  4.0f,  8.0f,  16.0f,  32.0f,  64.0f};
  std::vector<std::string> size_limit_names = {"125Kb","250Kb","500Kb", "1Mb", "2Mb", "4Mb", "8Mb", "16Mb", "32Mb", "64Mb"};

  //different render settings
  std::vector<unsigned> intersect_modes = {SDF_OCTREE_NODE_INTERSECT_ST, 
                                           SDF_OCTREE_NODE_INTERSECT_ANALYTIC, 
                                           SDF_OCTREE_NODE_INTERSECT_NEWTON,
                                           SDF_OCTREE_NODE_INTERSECT_IT,
                                           SDF_OCTREE_NODE_INTERSECT_BBOX};

  std::vector<std::string> intersect_mode_names = {"bvh_sphere_tracing",
                                                   "bvh_analytic",
                                                   "bvh_newton",
                                                   "bvh_interval_tracing",
                                                   "bvh_nodes"};

  std::vector<unsigned> render_modes = {BENCHMARK_FLAG_RENDER_RT,
                                        BENCHMARK_FLAG_RENDER_DEPTH,
                                        BENCHMARK_FLAG_RENDER_HYDRA};

  std::vector<std::string> render_mode_names = {"lambert", "depth", "hydra"};

  if (flags & BENCHMARK_FLAG_BUILD)
  {
    fprintf(log_fd, "mesh %u vertices, %u triangles, %f kb\n", (unsigned)mesh.vPos4f.size(), (unsigned)mesh.TrianglesNum(), 
            (mesh.vPos4f.size() * 8 * sizeof(float) + mesh.TrianglesNum() * 3 * sizeof(unsigned)) / 1000.0f);
    fprintf(log_fd, "mesh_name, structure, size_limit, nodes, memory (Kb), build_time (s)\n");
    fflush(log_fd);

    for (int d_id = 0; d_id < max_depths.size(); d_id++)
    {
      unsigned max_depth = max_depths[d_id];
      std::string size_limit = size_limit_names[d_id];
      if (std::find(use_size.begin(), use_size.end(), size_limit) == use_size.end())
        continue;

      for (int s_id = 0; s_id < structures.size(); s_id++)
      {
        std::string structure = structures[s_id];
        if (structure == "mesh" || structure == "mesh_lod" ||
            std::find(use_structure.begin(), use_structure.end(), structure) == use_structure.end())
          continue;
        
        unsigned nodes_limit = size_limit_Mb[d_id] * 1000 * 1000 / average_bytes_per_node[s_id];
        std::string full_name = mesh_name + "_" + structure + "_" + size_limit;
        std::filesystem::create_directory(path + "/" + full_name);
        std::string filename = path + "/" + full_name + "/data.bin";
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

          save_sdf_frame_octree(scene, filename);
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

          save_sdf_SVS(scene, filename);
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
        else if (structure == "sdf_SBS_tricubic")
        {
          #ifdef USE_TRICUBIC
          SdfSBSHeader header;
          header.brick_size = 2;
          header.brick_pad = 1;
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
          #else
          printf("Picked tricubic interpolation, but did not build prj with it\n");
          #endif
        }
        else if (structure == "openVDB")
        {
          
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
        else if (structure == "sdf_SBS-3-1" || structure == "sdf_SBS-3-1_SN")
        {
          SdfSBSHeader header;
          header.brick_size = 3;
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

        fprintf(log_fd, "%20s, %20s, %6s, %8u, %8u, %5.1f\n", 
                mesh_name.c_str(), structure.c_str(), size_limit.c_str(), 
                res.nodes, res.memory / 1000, res.build_time_ms/1000.0f);
        fflush(log_fd);
      }
    }
  }

  fprintf(log_fd, "mesh_name, device, render_mode, structure, size_limit, intersect_mode, image_metrics::PSNR, average_time (ms), min_time (ms)\n");
  fflush(log_fd);

  for (int rm_id = 0; rm_id < render_modes.size(); rm_id++)
  {
    std::string render_mode = render_mode_names[rm_id];

    if ((flags & render_modes[rm_id]) == 0)
      continue;

    //render reference image
    std::vector<LiteImage::Image2D<uint32_t>> image_ref(base_iters, LiteImage::Image2D<uint32_t>(W, H));

    for (int d_id = 0; d_id < max_depths.size(); d_id++)
    {
      std::string size_limit = size_limit_names[d_id];
      if (std::find(use_size.begin(), use_size.end(), size_limit) == use_size.end())
        continue;

      for (int s_id = 0; s_id < structures.size(); s_id++)
      {
        std::string structure = structures[s_id];
        if (structure != "mesh" && std::find(use_structure.begin(), use_structure.end(), structure) == use_structure.end())
          continue;
        bool mesh_rendered = (size_limit != use_size[0]);

        for (int r_id = 0; r_id < intersect_mode_names.size(); r_id++)
        {
          std::string intersect_mode = intersect_mode_names[r_id];
          if (std::find(use_intersect.begin(), use_intersect.end(), intersect_mode) == use_intersect.end())
            continue;
          
          if (structure == "mesh")
          {
            if (mesh_rendered)
              continue;
            mesh_rendered = true;
          }
          
          std::string full_name = mesh_name + "_" + structure + "_" + size_limit;
          std::string filename = path + "/" + full_name + "/data.bin";
          std::string experiment_name = mesh_name + "_" + structure + "_" + size_limit + "_" + intersect_mode;
          unsigned pass_size = (structure == "sdf_octree" || render_modes[rm_id] == BENCHMARK_FLAG_RENDER_HYDRA) ? 1 : base_pass_size;
          unsigned iters = structure == "sdf_octree" ? 1 : base_iters;

          MultiRenderPreset preset = getDefaultPreset();
          preset.render_mode = render_modes[rm_id] == BENCHMARK_FLAG_RENDER_DEPTH ? MULTI_RENDER_MODE_LINEAR_DEPTH : MULTI_RENDER_MODE_LAMBERT_NO_TEX;
          preset.sdf_node_intersect = intersect_modes[r_id];

          LiteImage::Image2D<uint32_t> image(W, H);
            double sum_ms[4] = {0,0,0,0};
            double min_ms[4] = {1e6,1e6,1e6,1e6};
            double max_ms[4] = {0,0,0,0};
            float psnr = 0.0f;

            if (structure == "mesh")
              std::filesystem::create_directory(path + "/" + full_name);

            for (int iter = 0; iter<iters; iter++)
            {
              const float dist = 3;
              float angle = 2.0f*M_PI*iter/iters;
              const float3 pos = dist*float3(sin(angle), 0, cos(angle));
              float timings[4] = {0,0,0,0};  

              if (render_modes[rm_id] == BENCHMARK_FLAG_RENDER_HYDRA)
              {
                if (structure != "sdf_octree")
                {
                  std::vector<uint32_t> image_vector;
                  std::string model_path = structure == "mesh" ? mesh_path : filename;
                  direct_test(model_path, structure_types[s_id], preset, angle, W, H, hydra_spp, timings, image_vector);
                  image = LiteImage::Image2D<uint32_t>(W, H, image_vector.data());
                }
                //else sdf octree is too slow, no need to render with hydra
              }
              else
              {
                std::shared_ptr<MultiRenderer> pRender;
                if (structure == "mesh")
                {
                  std::filesystem::create_directory(path + "/" + full_name);

                  pRender = CreateMultiRenderer(render_device);
                  pRender->SetPreset(preset);
                  pRender->SetScene(mesh);
                }
                else if (structure == "mesh_lod")
                {
                  auto mesh_lod = cmesh4::LoadMeshFromVSGF(filename.c_str());
                  cmesh4::normalize_mesh(mesh_lod);

                  pRender = CreateMultiRenderer(render_device);
                  pRender->SetPreset(preset);
                  pRender->SetScene(mesh_lod);
                }
                else if (structure == "sdf_grid")
                {
                  SdfGrid grid;
                  load_sdf_grid(grid, filename);

                  pRender = CreateMultiRenderer(render_device);
                  pRender->SetPreset(preset);
                  pRender->SetScene(grid);
                }
                else if (structure == "sdf_frame_octree")
                {
                  std::vector<SdfFrameOctreeNode> frame_nodes;
                  load_sdf_frame_octree(frame_nodes, filename);

                  pRender = CreateMultiRenderer(render_device, frame_nodes.size() + 1);
                  pRender->SetPreset(preset);                  
                  pRender->SetScene(frame_nodes);
                }
                else if (structure == "sdf_SVS")
                {
                  std::vector<SdfSVSNode> svs_nodes;
                  load_sdf_SVS(svs_nodes, filename);

                  pRender = CreateMultiRenderer(render_device, svs_nodes.size() + 1);
                  pRender->SetPreset(preset);                  
                  pRender->SetScene(svs_nodes);
                }
                else if (structure == "openVDB")
                {
                    /*
                    /,    CPU,      lambert, OpenVDB, 4Mb,  bvh_sphere_tracing, 28.97, 2280, 2280
                    /,    CPU,      lambert, OpenVDB, 16Mb,  bvh_sphere_tracing, 37.82, 2316, 2316
                    /,    CPU,      lambert, OpenVDB, 64Mb, bvh_sphere_tracing, 38.16, 3098, 3098
                    */
                }
                else if (structure == "sdf_SBS_tricubic")
                {
                  #ifdef USE_TRICUBIC
                  SdfSBS sbs;
                  load_sdf_SBS(sbs, filename);

                  SdfSBSHeader header;
                  header.brick_size = 2;
                  header.brick_pad = 1;
                  header.bytes_per_value = 1;

                  SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 5);

                  pRender = CreateMultiRenderer(render_device, sbs.nodes.size()*std::pow(sbs.header.brick_size, 3) + 1);
                  auto indexed_SBS = sdf_converter::create_sdf_SBS_indexed_with_neighbors(settings, header, mesh, 0, pRender->getMaterials(), pRender->getTextures());
                  pRender->SetScene(indexed_SBS);
                  #else
                  printf("Picked tricubic interpolation, but did not build prj with it\n");
                  #endif
                }
                else if (structure == "sdf_SBS-2-1" || structure == "sdf_SBS-2-2"|| structure == "sdf_SBS-3-1")
                {
                  SdfSBS sbs;
                  load_sdf_SBS(sbs, filename);

                  pRender = CreateMultiRenderer(render_device, sbs.nodes.size()*std::pow(sbs.header.brick_size, 3) + 1);
                  pRender->SetPreset(preset);                  
                  pRender->SetScene(sbs);
                }
                else if (structure == "sdf_SBS-3-1_SN")
                {
                  SdfSBS sbs;
                  load_sdf_SBS(sbs, filename);

                  pRender = CreateMultiRenderer(render_device, sbs.nodes.size() + 1);
                  pRender->SetPreset(preset);                  
                  pRender->SetScene(sbs);                  
                }

                render(image, pRender, pos, float3(0,0,0), float3(0,1,0), preset, 1);

                pRender->Render(image.data(), image.width(), image.height(), "color", pass_size); 
                pRender->GetExecutionTime("CastRaySingleBlock", timings);
              }

              //if (iter == 0)
              LiteImage::SaveImage<uint32_t>((path + "/" + full_name + "/" + render_mode + "_" + intersect_mode + "_" + std::to_string(iter)+".bmp").c_str(), image); 

              for (int i=0;i<4;i++)
              {
                min_ms[i] = std::min(min_ms[i], (double)timings[i]);
                max_ms[i] = std::max(max_ms[i], (double)timings[i]);
                sum_ms[i] += timings[i];
              }

              if (structure == "mesh")
                image_ref[iter] = image;

              float psnr_one = image_metrics::PSNR(image_ref[iter], image);
              //printf("psnr one %f\n", psnr_one);
              psnr += psnr_one;
            }

            float4 render_average_time_ms = float4(sum_ms[0], sum_ms[1], sum_ms[2], sum_ms[3])/(iters*pass_size);
            float4 render_min_time_ms = float4(min_ms[0], min_ms[1], min_ms[2], min_ms[3])/pass_size;

            std::vector<std::string> render_device_names = {"CPU", "GPU", "RTX"};

            fprintf(log_fd, "%20s, %6s, %12s, %20s, %6s, %20s, %6.2f, %7.2f, %7.2f\n", 
                    mesh_name.c_str(), render_device_names[render_device].c_str(), render_mode.c_str(), structure.c_str(), size_limit.c_str(), intersect_mode.c_str(),
                    psnr/iters,
                    render_average_time_ms.x,
                    render_min_time_ms.x);
            fflush(log_fd);
        }
      }
    }
  }
}

void main_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags, const std::string &supported_type)
{
/*
  main_benchmark(path, mesh_name, BENCHMARK_FLAG_RENDER_RT, "image", 
  std::vector<std::string>{"sdf_SVS"},
  std::vector<std::string>{"125Kb","250Kb","500Kb", "1Mb", "2Mb", "4Mb", "8Mb", "16Mb", "32Mb", "64Mb"},
  std::vector<std::string>{"bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_interval_tracing"}, 25, 10);
  return;
*/
  std::vector<std::string> types = {"mesh", "mesh_lod", "sdf_grid", "sdf_octree", "sdf_frame_octree", "sdf_SVS", "sdf_SBS-2-1", "sdf_SBS-2-2"};
  if (supported_type != "")
    types = {supported_type};

  main_benchmark(path, mesh_name, flags, "image", 
  types,
  std::vector<std::string>{"250Kb","1Mb","4Mb","16Mb", "64Mb"},
  std::vector<std::string>{"bvh_newton"});

  if (supported_type == "sdf_SVS")
  {
    main_benchmark(path, mesh_name, flags & (~BENCHMARK_FLAG_BUILD), "image", 
    std::vector<std::string>{"sdf_SVS"},
    std::vector<std::string>{"250Kb","1Mb","4Mb","16Mb", "64Mb"},
    std::vector<std::string>{"bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_interval_tracing", "bvh_nodes"},
    50, 10);
  }
}

void SBS_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags)
{
  main_benchmark(path, mesh_name, flags, "image", 
                 std::vector<std::string>{"sdf_SVS", "sdf_SBS-3-1_SN", "sdf_SBS-3-1"},
                 std::vector<std::string>{"16Mb"},
                 std::vector<std::string>{"bvh_analytic"},
                 25, 10);
}

void full_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags)
{
  main_benchmark(path, mesh_name, flags, "image", 
                 std::vector<std::string>{"mesh", "sdf_grid", "sdf_SVS", "sdf_SBS_tricubic", "sdf_SBS-3-1_SN", "sdf_SBS-3-1"},
                 std::vector<std::string>{"4Mb","16Mb", "64Mb"},
                 std::vector<std::string>{"bvh_sphere_tracing"},
                 1, 10, DEVICE_CPU);
}

void rtx_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags, const std::string &supported_type, unsigned device)
{
  std::vector<std::string> types = {"mesh", "sdf_grid", "sdf_frame_octree", "sdf_SVS", "sdf_SBS-2-1"};
  if (supported_type != "")
    types = {supported_type};

  int pass_size = 25;
  if (device == DEVICE_CPU)
    pass_size = 1;

  main_benchmark(path, mesh_name, flags, "image", 
  types,
  std::vector<std::string>{"1Mb","4Mb","16Mb","64Mb"},
  std::vector<std::string>{"bvh_newton"}, pass_size, 8, device);
}