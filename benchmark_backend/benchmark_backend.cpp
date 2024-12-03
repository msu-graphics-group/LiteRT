#include "benchmark_backend.hpp"

namespace BenchmarkBackend
{
  /*

  build_SVS(SimpleMesh mesh, std::string lod, std::string param_string)
  {
  if (lod == "low")
    depth = 4;
  if (lod == "high")
    depth = 6;
  }
  build_SBS(SimpleMesh mesh, std::string lod, std::string param_string)
  {

  }

  void render()
  {
   string xml_path = "benchmark/bunny/models/SBS/lod_high_brick4.xml";
   //it should be also "benchmark/bunny/models/SBS/lod_high_brick4.bin";
   //brick4 - param string

   IRenderer *renderer;
   if (...)
    renderer = MultiRenderer(device)
   else
    renderer = HydraRenderer(device)

    cameras = ...
    preset  = ...
    for (auto &camera : cameras)
    {
      Image2D<uint32_t> image_ref = LoadImage("benchmark/bunny/hydra/GPU/mesh/lod_high_default_cam_i.png");

      renderer->setCamera(camera);
      renderer->render();

      save_image(...)
      psnr = ...
      time = ...
    }

    log.AddLog(...)
  }

  */

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

  std::string generate_filename_model_no_ext(std::string model_path, std::string repr_type, std::string lod, std::string param_string)
  {
    // string xml_path = "benchmark/bunny/models/SBS/lod_high_brick4" + ".xml";
    return "benchmark/" + get_model_name(model_path) + "/models/" + repr_type + "/lod_" + lod + '_' + param_string;
  }

  std::string generate_filename_image(std::string model_path, std::string renderer, std::string backend, std::string repr_type, std::string lod, std::string param_string, uint32_t camera)
  {
    // LoadImage("benchmark/bunny/hydra/GPU/mesh/lod_high_default_cam_i.png");
    return "benchmark/" + get_model_name(model_path) + "/" + renderer + "/" + backend + "/" + repr_type +
            "/lod_" + lod + '_' + param_string + "_cam_" + std::to_string(camera) + ".png";
  }


  SparseOctreeSettings get_build_settings(std::string lod)
  {
    SparseOctreeSettings res{};

    res.build_type = SparseOctreeBuildType::MESH_TLO;

    if (lod == "low")
      res.depth = 4;
    if (lod == "mid")
      res.depth = 6;
    if (lod == "high")
      res.depth = 8;

    return res;
  }

  GridSettings get_build_settings_grid(std::string lod)
  {
    GridSettings res{};

    if (lod == "low")
      res.size = 32;
    if (lod == "mid")
      res.size = 48;
    if (lod == "high")
      res.size = 64;

    return res;
  }

  // TODO: Param string is used for SBS and COctreeV3, and "brick_size" and "brick_pad" are the same fields for now. Should they be?

  void parse_param_string(std::string param_string, SdfSBSHeader *sbs_header, COctreeV3Header *coctree_header)
  {
    // param string example: COctreeV3 - 'size_4_pad_1_bits_1_uv_0_sim_0'
    // param string example:       SBS - 'size_4_pad_0_bytes_1_aux_3'
    // aux then transformed (aux = aux << 24)

    std::vector<uint32_t> separators = {0};

    for (uint32_t i = 0u; i < param_string.size(); ++i)
      if (param_string[i] == '_')
        separators.push_back(i);
    separators.push_back(param_string.size());

    // Setting default values
    SdfSBSHeader res_header_sbs{};
    res_header_sbs.brick_size = 4;
    res_header_sbs.brick_pad = 0;
    res_header_sbs.bytes_per_value = 2;
    res_header_sbs.aux_data = SDF_SBS_NODE_LAYOUT_DX; // 1 << 24

    COctreeV3Header res_header_coctree{};
    res_header_coctree.brick_size = 4;
    res_header_coctree.brick_pad = 0;
    res_header_coctree.bits_per_value = 8;
    res_header_coctree.uv_size = 0;
    res_header_coctree.sim_compression = 0;

    // Process parameters
    if (!param_string.empty() || param_string != "default") // otherwise load default and skip
    {
      if ((separators.size() & 1u) == 0u)
      {
        printf("Error: incorrect param_string: '%s'\n", param_string.c_str());
        exit(-1);
      }
      else
      {
        for (uint32_t i = 2u; i < separators.size(); i += 2u)
        {
          uint32_t tag_beg = separators[i-2] + 1, tag_end = separators[i-1],
                  val_beg = separators[i-1] + 1, val_end = separators[i  ];

          std::string tag     = param_string.substr(tag_beg, tag_end - tag_beg);
          std::string val_str = param_string.substr(val_beg, val_end - val_beg);
          int val = std::atoi(val_str.c_str());

          if (tag == "size")
          {
            assert(val >= 1 && val <= 16);
            res_header_sbs.brick_size = val;
            res_header_coctree.brick_size = val;
          }
          else if (tag == "pad")
          {
            assert(val >= 0 && val <= 1);
            res_header_sbs.brick_pad = val;
            res_header_coctree.brick_pad = val;
          }
          else if (tag == "bits")
          {
            assert(val == 6 || val == 8 || val == 10 || val == 16 || val == 32);
            res_header_coctree.bits_per_value = val;
          }
          else if (tag == "bytes")
          {
            assert(val == 1 || val == 2 || val == 4);
            res_header_sbs.bytes_per_value = val;
          }
          else if (tag == "uv")
          {
            assert(val >= 0 && val <= 2);
            res_header_coctree.uv_size = val;
          }
          else if (tag == "sim")
          {
            assert(val >= 0 && val <= 1);
            res_header_coctree.sim_compression = val;
          }
          else if (tag == "aux")
          {
            assert(val >= 0 && val <= 5);
            res_header_sbs.aux_data = val << 24u;
          }
        }
      }
    }

    if (sbs_header != nullptr)
      *sbs_header = res_header_sbs;
    if (coctree_header != nullptr)
      *coctree_header = res_header_coctree;
  }


 // Build

  void build_model(std::string render_config_str)
  {
    // Read config string

    Block render_config;
    load_block_from_string(render_config_str, render_config);

    std::string model_path = render_config.get_string("model");
    std::string lod = render_config.get_string("lod");
    std::string repr_type = render_config.get_string("type");
    std::string param_string = render_config.get_string("param_string");

    // Not needed in build
    // std::vector<std::string> render_modes;
    // render_config.get_arr("render_modes", render_modes);

    // Load mesh

    const int mat_id = 6;
    cmesh4::SimpleMesh mesh = cmesh4::LoadMesh(model_path.c_str());
    cmesh4::set_mat_id(mesh, mat_id);

    SparseOctreeSettings settings = get_build_settings(lod);
    GridSettings grid_settings = get_build_settings_grid(lod);

    // TODO: build benchmark: time, mesh/result size, some 3D metrics

    std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();
    std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();


    if (repr_type == "SDF_GRID")
    {
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_grid(grid_settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_grid(model_new);

      std::string fname_no_ext = generate_filename_model_no_ext(model_path, repr_type, lod, param_string);

      save_sdf_grid(model_new, fname_no_ext + ".bin");
      save_xml_string(get_xml_string_model_demo_scene(fname_no_ext + ".bin", info, mat_id), fname_no_ext + ".xml");
    }
    if (repr_type == "SDF_SVS")
    {
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_SVS(settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_SVS(model_new);

      std::string fname_no_ext = generate_filename_model_no_ext(model_path, repr_type, lod, param_string);

      save_sdf_SVS(model_new, fname_no_ext + ".bin");
      save_xml_string(get_xml_string_model_demo_scene(fname_no_ext + ".bin", info, mat_id), fname_no_ext + ".xml");
    }
    if (repr_type == "SDF_SBS")
    {
      SdfSBSHeader header{};
      parse_param_string(param_string, &header);
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_SBS(settings, header, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_SBS(model_new);

      std::string fname_no_ext = generate_filename_model_no_ext(model_path, repr_type, lod, param_string);

      save_sdf_SBS(model_new, fname_no_ext + ".bin");
      save_xml_string(get_xml_string_model_demo_scene(fname_no_ext + ".bin", info, mat_id), fname_no_ext + ".xml");
    }
    if (repr_type == "SDF_FRAME_OCTREE")
    {
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_sdf_frame_octree(settings, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_sdf_frame_octree(model_new);

      std::string fname_no_ext = generate_filename_model_no_ext(model_path, repr_type, lod, param_string);

      save_sdf_frame_octree(model_new, fname_no_ext + ".bin");
      save_xml_string(get_xml_string_model_demo_scene(fname_no_ext + ".bin", info, mat_id), fname_no_ext + ".xml");
    }
    if (repr_type == "SDF_FRAME_OCTREE_COMPACT")
    {
      COctreeV3Header header{};
      parse_param_string(param_string, nullptr, &header);
      t1 = std::chrono::steady_clock::now();
      auto model_new = sdf_converter::create_COctree_v3(settings, header, mesh);
      t2 = std::chrono::steady_clock::now();
      ModelInfo info = get_info_coctree_v3(model_new);

      std::string fname_no_ext = generate_filename_model_no_ext(model_path, repr_type, lod, param_string);

      save_coctree_v3(model_new, fname_no_ext + ".bin");
      save_xml_string(get_xml_string_model_demo_scene(fname_no_ext + ".bin", info, mat_id), fname_no_ext + ".xml");
    }

    //  Time calculation
    float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f; // ms
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

  void
  getMetrics(const char **argv)
  {
    if (std::string(argv[5]) == "MESH")
    {
      BenchmarkBackend::getInfoMesh(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
    }
    else if (std::string(argv[5]) == "SDF_GRID")
    {
      BenchmarkBackend::getInfoGrid(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
    }
    else if (std::string(argv[5]) == "SDF_SVS")
    {
      BenchmarkBackend::getInfoSVS(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
    }
    else if (std::string(argv[5]) == "SDF_SBS")
    {
      BenchmarkBackend::getInfoSBS(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
    }
  }

  void
  getInfoMesh(const std::string &model, const std::string &backend, const std::string &renderer,
              const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
  {
    std::fstream f;
    f.open("benchmark/results/results.csv", std::ios::app);

    auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
    cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

    MultiRenderPreset preset = createPreset(renderer, spp);

    auto pRender = CreateMultiRenderer(getDevice(backend));
    pRender->SetPreset(preset);

    float min_time = 1e4, max_time = -1, average_time = 0;
    float memory = 0;
    float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
    float min_flip = 1000, max_flip = -1, average_flip = 0;

    memory = sizeof(float) * (float)(mesh.IndicesNum() + mesh.VerticesNum()) / 1024.f / 1024.f;
    pRender->SetScene(mesh);

    for (int camera = 0; camera < cameras; camera++)
    {
      const float dist = 2;
      float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
      const float3 pos = dist * float3(sin(angle), 0, cos(angle));

      LiteImage::Image2D<uint32_t> image(width, height);

      auto t1 = std::chrono::steady_clock::now();
      render(image, pRender, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);
      auto t2 = std::chrono::steady_clock::now();

      //  Time calculation
      float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
      calcMetrics(min_time, max_time, average_time, t);

            std::string img_name = "benchmark/saves/" + backend + "_" + renderer + "_" + type + "_" + std::to_string(camera) + ".bmp";
            LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);

      //  calculate metrics
      calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, image));
      calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, image));
    }

    average_time /= (float)cameras;
    average_psnr /= (float)cameras;
    average_flip /= (float)cameras;

    f << model << ", " << backend << ", " << renderer << ", " << type << ", " << lod << ", " << memory << ", " << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
    f.close();
  }

  void
  getInfoGrid(const std::string &model, const std::string &backend, const std::string &renderer,
              const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
  {
    float min_time = 1e4, max_time = -1, average_time = 0;
    float memory = 0;
    float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
    float min_flip = 1000, max_flip = -1, average_flip = 0;

    std::fstream f;
    f.open("benchmark/results/results.csv", std::ios::app);

    auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
    cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

    int size = 16;

    if (lod == "medium")
    {
      size = 32;
    }
    else
    {
      size = 64;
    }

    SdfGrid grid = sdf_converter::create_sdf_grid(GridSettings(size), mesh);
    memory = sizeof(float) * (float)grid.data.size() / 1024.f / 1024.f;

    MultiRenderPreset preset = createPreset(renderer, spp);

    auto pRender_mesh = CreateMultiRenderer(getDevice(backend));
    pRender_mesh->SetPreset(preset);
    pRender_mesh->SetScene(mesh);

    auto pRender_grid = CreateMultiRenderer(getDevice(backend));
    pRender_grid->SetPreset(preset);
    pRender_grid->SetScene(grid);

    for (int camera = 0; camera < cameras; camera++)
    {
      const float dist = 2;
      float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
      const float3 pos = dist * float3(sin(angle), 0, cos(angle));

      LiteImage::Image2D<uint32_t> image(width, height), ref_image(width, height);

      render(ref_image, pRender_mesh, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);

      auto t1 = std::chrono::steady_clock::now();
      render(image, pRender_grid, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);
      auto t2 = std::chrono::steady_clock::now();

      //  Time calculation
      float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
      calcMetrics(min_time, max_time, average_time, t);

            std::string img_name = "benchmark/saves/" + backend + "_" + renderer + "_" + type + "_" + std::to_string(camera) + ".bmp";
            std::string img_name2 = "benchmark/saves/" + backend + "_" + renderer + "_MESHREF_" + std::to_string(camera) + ".bmp";
            LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);
            LiteImage::SaveImage<uint32_t>(img_name2.c_str(), ref_image);

      //  calculate metrics
      calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, ref_image));
      calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, ref_image));
    }

    average_time /= (float)cameras;
    average_psnr /= (float)cameras;
    average_flip /= (float)cameras;

    f << model << ", " << backend << ", " << renderer << ", " << type << ", " << lod << ", " << memory << ", "
      << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr
      << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
  }

  void
  getInfoSVS(const std::string &model, const std::string &backend, const std::string &renderer,
             const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
  {
    float min_time = 1e4, max_time = -1, average_time = 0;
    float memory = 0;
    float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
    float min_flip = 1000, max_flip = -1, average_flip = 0;

    std::fstream f;
    f.open("benchmark/results/results.csv", std::ios::app);

    auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
    cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

    int depth = 4;

    if (lod == "medium")
    {
      depth = 6;
    }
    else
    {
      depth = 8;
    }

    SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, depth);
    std::vector<SdfSVSNode> frame_nodes = sdf_converter::create_sdf_SVS(settings, mesh);
    memory = sizeof(SdfSVSNode) * frame_nodes.size() / 1024.f / 1024.f;

    MultiRenderPreset preset = createPreset(renderer, spp);

    auto pRender_mesh = CreateMultiRenderer(getDevice(backend));
    pRender_mesh->SetPreset(preset);
    pRender_mesh->SetScene(mesh);

    auto pRender_svs = CreateMultiRenderer(getDevice(backend));
    pRender_svs->SetPreset(preset);
    pRender_svs->SetScene(frame_nodes);

    for (int camera = 0; camera < cameras; camera++)
    {
      const float dist = 2;
      float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
      const float3 pos = dist * float3(sin(angle), 0, cos(angle));

      LiteImage::Image2D<uint32_t> image(width, height), ref_image(width, height);

      render(ref_image, pRender_mesh, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);

      auto t1 = std::chrono::steady_clock::now();
      render(image, pRender_svs, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);
      auto t2 = std::chrono::steady_clock::now();

      //  Time calculation
      float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
      calcMetrics(min_time, max_time, average_time, t);

            std::string img_name = "benchmark/saves/" + backend + "_" + renderer + "_" + type + "_" + std::to_string(camera) + ".bmp";
            std::string img_name2 = "benchmark/saves/" + backend + "_" + renderer + "_MESHREF_" + std::to_string(camera) + ".bmp";
            LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);
            LiteImage::SaveImage<uint32_t>(img_name2.c_str(), ref_image);

      //  calculate metrics
      calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, ref_image));
      calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, ref_image));
    }

    average_time /= (float)cameras;
    average_psnr /= (float)cameras;
    average_flip /= (float)cameras;

    f << model << ", " << backend << ", " << renderer << ", " << type << ", " << lod << ", " << memory << ", "
      << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr
      << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
  }

  void
  getInfoSBS(const std::string &model, const std::string &backend, const std::string &renderer,
             const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
  {
    float min_time = 1e4, max_time = -1, average_time = 0;
    float memory = 0;
    float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
    float min_flip = 1000, max_flip = -1, average_flip = 0;

    std::fstream f;
    f.open("benchmark/results/results.csv", std::ios::app);

    auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
    cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

    SdfSBSHeader header;
    header.brick_size = 2;
    header.brick_pad = 0;
    header.bytes_per_value = 1;
    header.aux_data = SDF_SBS_NODE_LAYOUT_DX;

    if (lod == "medium")
    {
      header.brick_size = 4;
    }
    else
    {
      header.brick_size = 5;
    }

    auto sbs = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 5), header, mesh);

    MultiRenderPreset preset = createPreset(renderer, spp);

    auto pRender_mesh = CreateMultiRenderer(getDevice(backend));
    pRender_mesh->SetPreset(preset);
    pRender_mesh->SetScene(mesh);

    auto pRender_sbs = CreateMultiRenderer(getDevice(backend));
    pRender_sbs->SetPreset(preset);
    pRender_sbs->SetScene(sbs);

    BVHRT *bvh = dynamic_cast<BVHRT *>(pRender_sbs->GetAccelStruct().get());
    memory = bvh->m_allNodePairs.size() * sizeof(BVHNodePair) +
             bvh->m_primIdCount.size() * sizeof(uint32_t) +
             bvh->m_SdfSBSNodes.size() * sizeof(SdfSBSNode) +
             bvh->m_SdfSBSData.size() * sizeof(uint32_t) +
             bvh->m_SdfSBSDataF.size() * sizeof(float);
    memory /= 1024.0f * 1024.0f;

    for (int camera = 0; camera < cameras; camera++)
    {
      const float dist = 2;
      float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
      const float3 pos = dist * float3(sin(angle), 0, cos(angle));

      LiteImage::Image2D<uint32_t> image(width, height), ref_image(width, height);

      render(ref_image, pRender_mesh, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);

      auto t1 = std::chrono::steady_clock::now();
      render(image, pRender_sbs, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);
      auto t2 = std::chrono::steady_clock::now();

      //  Time calculation
      float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
      calcMetrics(min_time, max_time, average_time, t);

            std::string img_name = "benchmark/saves/" + backend + "_" + renderer + "_" + type + "_" + std::to_string(camera) + ".bmp";
            std::string img_name2 = "benchmark/saves/" + backend + "_" + renderer + "_MESHREF_" + std::to_string(camera) + ".bmp";
            LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);
            LiteImage::SaveImage<uint32_t>(img_name2.c_str(), ref_image);

      //  calculate metrics
      calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, ref_image));
      calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, ref_image));
    }

    average_time /= (float)cameras;
    average_psnr /= (float)cameras;
    average_flip /= (float)cameras;

    f << model << ", " << backend << ", " << renderer << ", " << type << ", " << lod << ", " << memory << ", "
      << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr
      << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
  }

  void
  getInfoAdaptSBS(const std::string &model, const std::string &backend, const std::string &renderer,
                  const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
  {
    float min_time = 1e4, max_time = -1, average_time = 0;
    float memory = 0;
    float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
    float min_flip = 1000, max_flip = -1, average_flip = 0;

    std::fstream f;
    f.open("benchmark/results/results.csv", std::ios::app);

    auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
    cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

    std::vector<MeshBVH> bvh(1);
    for (unsigned i = 0; i < 1; i++)
      bvh[i].init(mesh);
    auto real_sdf = [&](const float3 &p, unsigned idx) -> float
    { return bvh[idx].get_signed_distance(p); };

    int depth = 1;

    if (lod == "medium")
    {
      depth = 3;
    }
    else
    {
      depth = 5;
    }

    SdfSBSAdapt sbs_adapt = sdf_converter::greed_sbs_adapt(real_sdf, depth);

    MultiRenderPreset preset = createPreset(renderer, spp);

    auto pRender_mesh = CreateMultiRenderer(getDevice(backend));
    pRender_mesh->SetPreset(preset);
    pRender_mesh->SetScene(mesh);

    auto pRender_sbs_adapt = CreateMultiRenderer(getDevice(backend));
    pRender_sbs_adapt->SetPreset(preset);
    pRender_sbs_adapt->SetScene(sbs_adapt);

    //  dont know how to get structure memory size
    memory = 0;
    // memory /= 1024.0f * 1024.0f;

    for (int camera = 0; camera < cameras; camera++)
    {
      const float dist = 2;
      float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
      const float3 pos = dist * float3(sin(angle), 0, cos(angle));

      LiteImage::Image2D<uint32_t> image(width, height), ref_image(width, height);

      render(ref_image, pRender_mesh, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);

      auto t1 = std::chrono::steady_clock::now();
      render(image, pRender_sbs_adapt, pos, float3(0, 0, 0), float3(0, 1, 0), preset, 1);
      auto t2 = std::chrono::steady_clock::now();

      //  Time calculation
      float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
      calcMetrics(min_time, max_time, average_time, t);

      std::string img_name = "benchmark/saves/" + backend + "_" + type + "_" + std::to_string(camera) + ".bmp";
      LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);

      //  calculate metrics
      calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, ref_image));
      calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, ref_image));
    }

    average_time /= (float)cameras;
    average_psnr /= (float)cameras;
    average_flip /= (float)cameras;

    f << model << ", " << backend << ", " << renderer << ", " << type << ", " << lod << ", " << memory << ", "
      << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr
      << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
  }

  MultiRenderPreset
  createPreset(const std::string &render_mode, const int spp)
  {
    MultiRenderPreset preset = getDefaultPreset();

    if (render_mode == "LAMBERT")
    {
      preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    }

    preset.spp = spp;

    return preset;
  }

  int getDevice(const std::string backend)
  {
    if (backend == "CPU")
    {
      return DEVICE_CPU;
    }
    else if (backend == "GPU")
    {
      return DEVICE_GPU;
    }
    else if (backend == "RTX")
    {
      return DEVICE_GPU_RTX;
    }

    //  if GPU_RQ -> skip
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