#include "benchmark_backend.h"
#include "utils/coctree/similarity_compression.h"
#ifdef USE_GPU
#include "vk_context.h"
#include "vk_utils.h"
#endif

namespace BenchmarkBackend
{
// Common

  std::string get_model_name(std::string model_path)
  {
    uint32_t last_dot = model_path.rfind('.');
    uint32_t last_slash = model_path.rfind('/');
    if (last_slash == model_path.npos && model_path.rfind('\\') != model_path.npos)
      last_slash = model_path.rfind('\\');

    uint32_t substr_beg = last_slash + (last_slash != model_path.npos);
    uint32_t substr_end = last_dot;

    std::string model_name = model_path.substr(substr_beg, substr_end - substr_beg);
    return model_name;
  }

  std::string generate_filename_model_no_ext(std::string model_path, std::string repr_type, std::string repr_config_name)
  {
    return "benchmark/saves/" + get_model_name(model_path) + "/models/" + repr_type + "/" + repr_config_name;
  }

  std::string generate_filename_image(std::string model_path, std::string renderer, std::string backend, std::string repr_type, std::string repr_config_name, uint32_t camera)
  {
    return "benchmark/saves/" + get_model_name(model_path) + "/" + renderer + "/" + backend + "/" + repr_type +
          "/" + repr_config_name + "_cam_" + std::to_string(camera) + ".png";
  }


 // Build

  void build_model(std::string render_config_str)
  {
    // Read config string

    Block render_config;
    load_block_from_string(render_config_str, render_config);

    std::string model_path = render_config.get_string("model");
    std::string repr_type = render_config.get_string("type");
    Block *repr_config = render_config.get_block("repr_config");
    if (repr_config == nullptr)
    {
      printf("Warning: no representation config passed to build_model\n");
      exit(-1);
    }
    std::string repr_config_name = render_config.get_string("repr_config_name");

    const int mat_id = render_config.get_int("mat_id", 6);
    DemoScene scene = DemoScene::SINGLE_OBJECT;

    if (render_config.get_string("hydra_scene") == "SINGLE_OBJECT")
      scene = DemoScene::SINGLE_OBJECT;
    else if (render_config.get_string("hydra_scene") == "CORNELL_BOX")
      scene = DemoScene::CORNELL_BOX;

    // Load mesh
    cmesh4::SimpleMesh mesh = cmesh4::LoadMesh(model_path.c_str());
    cmesh4::set_mat_id(mesh, mat_id);

    SparseOctreeSettings settings(repr_config->get_int("depth"));

    std::string fname_no_ext = generate_filename_model_no_ext(model_path, repr_type, repr_config_name);
    std::string tmp_fname = get_model_name(fname_no_ext);

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    if (repr_type == "MESH")
    {
      std::string fname_vsgf = fname_no_ext + ".vsgf";
      cmesh4::SaveMeshToVSGF(fname_vsgf.c_str(), mesh);
      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".vsgf", mesh, scene);
    }
    else if (repr_type == "MESH_LOD")
    {
      std::string fname_vsgf_before_lod = fname_no_ext + "_nolod.vsgf";
      std::string fname_vsgf = fname_no_ext + ".vsgf";
      cmesh4::SaveMeshToVSGF(fname_vsgf_before_lod.c_str(), mesh);

      float lod_param = repr_config->get_double("decimate_percentage");

      std::string lod_command = "dependencies/lod_maker/lod_maker '" + fname_vsgf_before_lod + "' '" + fname_vsgf + "' " + std::to_string(lod_param);

      t1 = std::chrono::steady_clock::now();
      std::system(lod_command.c_str());
      t2 = std::chrono::steady_clock::now();

      std::filesystem::remove(fname_vsgf_before_lod.c_str()); // the only reason to import filesystem, delete original mesh, it can already be found in MESH directory

      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".vsgf", mesh, scene);
    }
    else if (repr_type == "SDF_GRID")
    {
      GridSettings grid_settings{};
      grid_settings.size = repr_config->get_int("size");

      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_grid(grid_settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_grid(model_new);

      save_sdf_grid(model_new, fname_no_ext + ".bin");
      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".bin", info, mat_id, scene);
    }
    else if (repr_type == "SDF_SVS")
    {
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_SVS(settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_SVS(model_new);

      save_sdf_SVS(model_new, fname_no_ext + ".bin");
      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".bin", info, mat_id, scene);
    }
    else if (repr_type == "SDF_SBS")
    {
      SdfSBSHeader header{};
      header.brick_size = repr_config->get_int("brick_size", 4);
      header.brick_pad = repr_config->get_int("brick_pad", 0);
      header.bytes_per_value = repr_config->get_int("bytes_per_value", 2);
      header.aux_data = repr_config->get_int("aux_data", 1) << 24; // 1 << 24


      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_SBS(settings, header, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_SBS(model_new);

      save_sdf_SBS(model_new, fname_no_ext + ".bin");
      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".bin", info, mat_id, scene);
    }
    else if (repr_type == "SDF_FRAME_OCTREE")
    {
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_frame_octree(settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_frame_octree(model_new);

      save_sdf_frame_octree(model_new, fname_no_ext + ".bin");
      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".bin", info, mat_id, scene);
    }
    else if (repr_type == "SDF_FRAME_OCTREE_COMPACT")
    {
      COctreeV3Settings co_settings{};
      co_settings.brick_size = repr_config->get_int("brick_size", 4);
      co_settings.brick_pad = repr_config->get_int("brick_pad", 0);
      co_settings.bits_per_value = repr_config->get_int("bits_per_value", 8);
      co_settings.uv_size = repr_config->get_int("uv_size", 0);
      co_settings.sim_compression = repr_config->get_int("sim_compression", 0);

      scom::Settings scom_settings;
      if (co_settings.sim_compression == 1)
      {
        std::string clustering_algorithm_str = repr_config->get_string("clustering_algorithm", "HIERARCHICAL");
        if (clustering_algorithm_str == "REPLACEMENT")
          scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::REPLACEMENT;
        else if (clustering_algorithm_str == "COMPONENTS_RECURSIVE_FILL")
          scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::COMPONENTS_RECURSIVE_FILL;
        else if (clustering_algorithm_str == "HIERARCHICAL")
          scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::HIERARCHICAL;
        scom_settings.similarity_threshold = repr_config->get_double("similarity_threshold", scom_settings.similarity_threshold);
        scom_settings.target_leaf_count = repr_config->get_int("target_leaf_count", scom_settings.target_leaf_count);
      }


      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_COctree_v3(settings, co_settings, scom_settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_coctree_v3(model_new);

      save_coctree_v3(model_new, fname_no_ext + ".bin");
      save_scene_xml(fname_no_ext + ".xml", get_model_name(fname_no_ext) + ".bin", info, mat_id, scene);
    }

    //  Both models' size calculation

    float original_memory = 0.f, memory = 0.f;

    {
      auto mr_renderer = CreateMultiRenderer(DEVICE_CPU);
      mr_renderer->SetPreset(getDefaultPreset());
      mr_renderer->SetScene(mesh);

      original_memory = static_cast<BVHRT*>(mr_renderer->GetAccelStruct().get())->get_model_size() * (1.f / (1024 * 1024));
    }
    {
      std::string fname_str = fname_no_ext + ".xml";

      auto mr_renderer = CreateMultiRenderer(DEVICE_CPU);
      mr_renderer->SetPreset(getDefaultPreset());
      mr_renderer->LoadScene(fname_str.c_str());

      memory = static_cast<BVHRT*>(mr_renderer->GetAccelStruct().get())->get_model_size() * (1.f / (1024 * 1024));
    }

    //  Time calculation
    float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f; // ms

    //  Write down measurements
    std::fstream f;
    f.open("benchmark/results/build.csv", std::ios::app);
    f << get_model_name(model_path) << ", " << repr_type << ", " << repr_config_name << ", " << original_memory << ", " << memory << ", " << t << std::endl;
  }

// Render

  void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender,
              float3 pos, float3 target, float3 up,
              MultiRenderPreset preset, int a_passNum)
  {
    float fov_degrees = 60;
    float z_near = 0.1f;
    float z_far = 100.0f;
    float aspect = 1.0f;
    auto proj = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
    auto worldView = LiteMath::lookAt(pos, target, up);

    pRender->Render(image.data(), image.width(), image.height(), worldView, proj, preset, a_passNum);
  }

  void Render(LiteImage::Image2D<uint32_t> &image, IRenderer* pRender, uint32_t width, uint32_t height, const LiteMath::float3 &pos, float *t)
  {
    float fov_degrees = 60;
    float z_near = 0.1f;
    float z_far = 100.0f;
    float aspect = float(width) / height;
    auto proj = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
    auto worldView = LiteMath::lookAt(pos, float3(0,0,0), float3(0,1,0));

    pRender->SetViewport(0,0, width, height);
    pRender->UpdateCamera(worldView, proj);
    pRender->CommitDeviceData();
    pRender->Clear(width, height, "color");

    pRender->Render(image.data(), width, height, "color");

    pRender->GetExecutionTime("CastRaySingleBlock", t);
  }

#ifdef USE_GPU
void shutTheFUpCallback(vk_utils::LogLevel level, const char *msg, const char* file, int line)
{
  if (level == vk_utils::LogLevel::LOG_ERROR || level == vk_utils::LogLevel::LOG_FATAL)
  {
    #ifdef _DEBUG
      fprintf(stderr, "[VkUtils::%s] [%s:%d]: %s\n", logLevelToString(level), file, line, msg);
    #else
      fprintf(stderr, "[VkUtils::%s] %s\n", logLevelToString(level), msg);
    #endif
    fflush(stderr);
  }
}
#endif


  void
  getMetrics(const std::string &render_config_str)
  {
    //disable annoying VkUtils logs
    #ifdef USE_GPU
    vk_utils::setLogCallback(shutTheFUpCallback);
    #endif

    Block render_config;
    load_block_from_string(render_config_str, render_config);

    std::string model_path = render_config.get_string("model");
    std::string model_name = render_config.get_string("model_name");
    std::string repr_type = render_config.get_string("type");
    std::string repr_config_name = render_config.get_string("repr_config_name");
    std::string mesh_config_name = render_config.get_string("mesh_config_name");
    std::string backend = render_config.get_string("backend");
    std::string renderer_type = render_config.get_string("renderer");
    
    std::vector<std::string> render_modes;
    render_config.get_arr("render_modes", render_modes);

    int width = render_config.get_int("width");
    int height = render_config.get_int("height");
    int cameras = render_config.get_int("cameras");
    int iters = render_config.get_int("iters");
    int spp = render_config.get_int("spp");
    int hydra_spp = render_config.get_int("hydra_spp");

    //  Start measurements
    std::fstream f;
    f.open("benchmark/results/render.csv", std::ios::app);

    if (renderer_type == "Hydra" && !render_modes.empty())
    {
      render_modes.resize(1); // Note: no render modes for Hydra, render only once
      render_modes[0] = "Hydras_render_mode"; // TODO
    }

    // Create renderer

    int device = getDevice(backend);
    float memory = 0;

    std::shared_ptr<IRenderer> pRender;

    if (renderer_type == "MR")
    {
      pRender = CreateMultiRenderer(device);
    }
    else if (renderer_type == "Hydra")
    {
      pRender = std::make_shared<HydraRenderer>(device);
      HydraRenderPreset hydra_preset = getDefaultHydraRenderPreset();
      hydra_preset.spp = hydra_spp;
      static_cast<HydraRenderer*>(pRender.get())->SetPreset(width, height, hydra_preset);
    }
    else
    {
      // TODO: throw something at someone
    }

    pRender->LoadScene(model_path.c_str());
    memory = static_cast<BVHRT*>(pRender->GetAccelStruct().get())->get_model_size() * (1.f / (1024 * 1024));


    // Render modes loop

    for (const auto &render_mode: render_modes)
    {
      if (renderer_type == "MR")
      {
        // Set MultiRenderer render mode
        MultiRenderPreset mr_preset = createPreset(render_mode, spp);
        static_cast<MultiRenderer*>(pRender.get())->SetPreset(mr_preset);
      }

      float min_time =  1e8, max_time = -1, average_time = 0;
      float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
      float min_flip = 1000, max_flip = -1, average_flip = 0;
      float res_time = 1e8, res_psnr = 1000, res_flip = 1000;
      
      int render_num = 0;
      for (int iter = 0; iter < iters; ++iter)
      {
        float avg_per_iter_time = 0;
        float avg_per_iter_psnr = 0;
        float avg_per_iter_flip = 0;

        for (int camera = 0; camera < cameras; ++camera)
        {
          const float dist = 3;
          float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
          const float3 pos = dist * float3(sin(angle), 0, cos(angle));

          LiteImage::Image2D<uint32_t> image(width, height);

          float t = 0.f;
          float t_arr[4] = {0,0,0,0};
          Render(image, pRender.get(), width, height, pos, t_arr);
          
          //  Time calculation
          t = t_arr[0] / 1000.f;
          calcMetrics(min_time, max_time, average_time, t);

          //  Save image
          std::string save_name = generate_filename_image(model_name + ".workaround", renderer_type, backend, repr_type, repr_config_name, camera),
                      mesh_name = generate_filename_image(model_name + ".workaround", renderer_type, backend,    "MESH", mesh_config_name, camera);

          printf("\r[%d/%d] Rendering: ", ++render_num, iters*cameras);
          fflush(stdout);
          LiteImage::SaveImage<uint32_t>(save_name.c_str(), image);

          LiteImage::Image2D<uint32_t> ref_image;

          if (repr_type == "MESH")
          {
            ref_image = image;
          }
          else
          {
            ref_image = LiteImage::LoadImage<uint32_t>(mesh_name.c_str());
          }

          //  Calculate metrics
          float psnr = image_metrics::PSNR(image, ref_image);
          float flip = image_metrics::FLIP(image, ref_image);
          calcMetrics(min_psnr, max_psnr, average_psnr, psnr);
          calcMetrics(min_flip, max_flip, average_flip, flip);

          avg_per_iter_time += t;
          avg_per_iter_psnr += psnr;
          avg_per_iter_flip += flip;
        }

        avg_per_iter_time /= (float)cameras;
        avg_per_iter_psnr /= (float)cameras;
        avg_per_iter_flip /= (float)cameras;

        res_time = std::min(res_time, avg_per_iter_time); // TODO
      }
      printf("\n\n");

      average_time /= (float)cameras * iters;
      average_psnr /= (float)cameras * iters;
      average_flip /= (float)cameras * iters;

      std::string device_name = "CPU";
      if (backend != "CPU")
      {
#ifdef USE_GPU
        VkPhysicalDeviceProperties properties{};
        vkGetPhysicalDeviceProperties(vk_utils::globalContextGet().physicalDevice, &properties);

        device_name = properties.deviceName;
#endif
      }

      f << model_name << ", " << backend << ", " << device_name << ", " << renderer_type << ", " << repr_type << ", " << repr_config_name << ", " << render_mode << ", " << memory << ", " << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
    }

    f.close();
  }

  MultiRenderPreset
  createPreset(const std::string &render_mode, const int spp)
  {
    MultiRenderPreset preset = getDefaultPreset();

    if (render_mode == "MASK")
      preset.render_mode = MULTI_RENDER_MODE_MASK;
    else if (render_mode == "LAMBERT_NO_TEX")
      preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    else if (render_mode == "DEPTH")
      preset.render_mode = MULTI_RENDER_MODE_DEPTH;
    else if (render_mode == "LINEAR_DEPTH")
      preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
    else if (render_mode == "INVERSE_LINEAR_DEPTH")
      preset.render_mode = MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH;
    else if (render_mode == "PRIMITIVE")
      preset.render_mode = MULTI_RENDER_MODE_PRIMITIVE;
    else if (render_mode == "TYPE")
      preset.render_mode = MULTI_RENDER_MODE_TYPE;
    else if (render_mode == "GEOM")
      preset.render_mode = MULTI_RENDER_MODE_GEOM;
    else if (render_mode == "NORMAL")
      preset.render_mode = MULTI_RENDER_MODE_NORMAL;
    else if (render_mode == "BARYCENTRIC")
      preset.render_mode = MULTI_RENDER_MODE_BARYCENTRIC;
    else if (render_mode == "ST_ITERATIONS")
      preset.render_mode = MULTI_RENDER_MODE_ST_ITERATIONS;
    else if (render_mode == "RF")
      preset.render_mode = MULTI_RENDER_MODE_RF;
    else if (render_mode == "PHONG_NO_TEX")
      preset.render_mode = MULTI_RENDER_MODE_PHONG_NO_TEX;
    else if (render_mode == "GS")
      preset.render_mode = MULTI_RENDER_MODE_GS;
    else if (render_mode == "RF_DENSITY")
      preset.render_mode = MULTI_RENDER_MODE_RF_DENSITY;
    else if (render_mode == "TEX_COORDS")
      preset.render_mode = MULTI_RENDER_MODE_TEX_COORDS;
    else if (render_mode == "DIFFUSE")
      preset.render_mode = MULTI_RENDER_MODE_DIFFUSE;
    else if (render_mode == "LAMBERT")
      preset.render_mode = MULTI_RENDER_MODE_LAMBERT;
    else if (render_mode == "PHONG")
      preset.render_mode = MULTI_RENDER_MODE_PHONG;
    else if (render_mode == "HSV_DEPTH")
      preset.render_mode = MULTI_RENDER_MODE_HSV_DEPTH;
    else if (render_mode == "LOD")
      preset.render_mode = MULTI_RENDER_MODE_LOD;

    preset.spp = spp;

    return preset;
  }

  int getDevice(const std::string backend)
  {
    if (backend == "CPU")
    {
      return DEVICE_CPU;
    }
    else if (backend == "GPU" || backend == "GPU_RQ")
    {
      return DEVICE_GPU;
    }
    else if (backend == "RTX")
    {
      return DEVICE_GPU_RTX;
    }

    return -1;
  }

  void
  calcMetrics(float &min, float &max, float &average, const float &new_val)
  {
    if (new_val < min)
    {
      min = new_val;
    }
    if (new_val > max)
    {
      max = new_val;
    }

    average += new_val;
  }
};