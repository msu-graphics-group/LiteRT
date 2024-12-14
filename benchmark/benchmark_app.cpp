#include "benchmark_app.h"

#include <fstream>

//// Parameters:
//
// -s, --slicer        = str str // slicer parameters
//     --delete_models           // deletes 'benchmark/saves/*/models' dirs
//
///  Config files
// -e, --enum_conf     = str // path to a config file containing enums
//     --conf          = str // path to a config file (input arguments overwrite it)
// -v, --verbose             // show processing info
//
///  Benchmark_app-level
// -b, --backends      = {CPU|GPU|RTX|GPU_RQ}
// -r, --renderers     = {MR|Hydra}
// -m, --models        = str str str...
// -f, --force_build                           // rebuild existing models
//     --clear                                 // clear csv files with build and render results
//
///  Render_app-level
// -rm, --render_modes = {LAMBERT_NO_TEX|...}  // see "MULTI_RENDER_MODE" enum
// -w, --width         = uint
// -h, --height        = uint
//     --spp           = uint
// -c, --cameras       = uint
// -i, --iters         = uint
// -p, --param_strings = str str str...


/// Some notes:
//
// {...|...} - enums, config file with defaults also stores all supported elements
//
// Parameter loading scheme:
// defaults config < "--conf" config < input arguments



BenchmarkAppEnums read_enums_config(const char *enums_config_fpath)
{
  BenchmarkAppEnums res_defaults{};
  Block block_defaults{};

  if (!load_block_from_file(enums_config_fpath, block_defaults))
  {
    printf("Error: Could not read defaults file: %s\n", enums_config_fpath);
    exit(-1);
  }

  block_defaults.get_arr("backends_enum", res_defaults.backends);
  block_defaults.get_arr("renderers_enum", res_defaults.renderers);
  block_defaults.get_arr("render_modes_enum", res_defaults.render_modes);
  block_defaults.get_arr("types_enum", res_defaults.repr_types);

  return res_defaults;
}

void write_enums_config(const char *enums_config_fpath, const BenchmarkAppEnums &in_enums)
{
  Block block_defaults{};

  block_defaults.set_arr("backends_enum", in_enums.backends);
  block_defaults.set_arr("renderers_enum", in_enums.renderers);
  block_defaults.set_arr("render_modes_enum", in_enums.render_modes);
  block_defaults.set_arr("types_enum", in_enums.repr_types);

  save_block_to_file(enums_config_fpath, block_defaults);
}

BenchmarkAppConfig read_benchmark_config(const char *benchmark_config_fpath)
{
  BenchmarkAppConfig res_benchmark{};
  Block block_benchmark{};

  if (!load_block_from_file(benchmark_config_fpath, block_benchmark))
  {
    printf("Error: Could not read config file: %s\n", benchmark_config_fpath);
    exit(-1);
  }

  block_benchmark.get_arr("backends", res_benchmark.backends);
  block_benchmark.get_arr("renderers", res_benchmark.renderers);
  block_benchmark.get_arr("render_modes", res_benchmark.render_modes);
  block_benchmark.get_arr("models", res_benchmark.models);

  res_benchmark.width = block_benchmark.get_int("width");
  res_benchmark.height = block_benchmark.get_int("height");
  res_benchmark.cameras = block_benchmark.get_int("cameras");
  res_benchmark.iters = block_benchmark.get_int("iters");
  res_benchmark.spp = block_benchmark.get_int("spp");
  res_benchmark.hydra_material_id = block_benchmark.get_int("hydra_material_id");
  res_benchmark.hydra_spp = block_benchmark.get_int("hydra_spp");
  res_benchmark.hydra_scene = block_benchmark.get_string("hydra_scene");

  // Get representation configs
  Block *repr_configs = block_benchmark.get_block("repr_configs");

  if (repr_configs == nullptr)
  {
    printf("Error: config file has no representations to render\n");
    exit(-1);
  }

  for (uint32_t i = 0u; i < repr_configs->size(); ++i) // { repr_type : configs }
  {
    std::string curr_repr_type = repr_configs->get_name(i); // repr_type
    Block *curr_repr_type_configs = repr_configs->get_block(i); // configs


    if (res_benchmark.repr_configs.find(curr_repr_type) != res_benchmark.repr_configs.end())
    {
      printf("Error: config file has multiple blocks for representation: %s\n", curr_repr_type.c_str());
      exit(-1);
    }

    if (curr_repr_type_configs->size() == 0)
    {
      printf("No custom configs for %s, default is used", curr_repr_type.c_str());
    }

    std::string repr_config_string;
    res_benchmark.repr_configs[curr_repr_type] = {}; // create empty vector of configs
    for (uint32_t i = 0u; i < curr_repr_type_configs->size(); ++i)
    {
      std::string curr_repr_type_single = curr_repr_type_configs->get_name(i);
      Block *curr_repr_type_single_config = curr_repr_type_configs->get_block(i);

      save_block_to_string(curr_repr_type_single, *curr_repr_type_single_config); // "blockname{Block}"

      res_benchmark.repr_configs[curr_repr_type].push_back(curr_repr_type_single);
    }
  }

  return res_benchmark;
}

std::string write_render_config_s(const RenderAppConfig &in_render)
{
  std::string res_block_str;
  Block block_render{}, block_repr_config;

  load_block_from_string(in_render.repr_config, block_repr_config);

  block_render.set_string("type", in_render.type);
  block_render.set_string("model", in_render.model);
  block_render.set_string("backend", in_render.backend);
  block_render.set_string("renderer", in_render.renderer);
  block_render.set_string("model_name", in_render.model_name);
  block_render.set_string("repr_config_name", in_render.repr_config_name);
  block_render.set_string("mesh_config_name", in_render.mesh_config_name);
  block_render.set_arr("render_modes", in_render.render_modes);
  block_render.add_block("repr_config", &block_repr_config);

  block_render.set_int("width", in_render.width);
  block_render.set_int("height", in_render.height);
  block_render.set_int("cameras", in_render.cameras);
  block_render.set_int("iters", in_render.iters);
  block_render.set_int("spp", in_render.spp);

  block_render.set_int("hydra_material_id", in_render.hydra_material_id);
  block_render.set_int("hydra_spp", in_render.hydra_spp);
  block_render.set_string("hydra_scene", in_render.hydra_scene);

  save_block_to_string(res_block_str, block_render);
  return res_block_str;
}



// SAME AS IN BENCHMARK_BACKEND.CPP

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

// COPY END


std::vector<std::string> filter_enum_params(const std::vector<std::string> &tags, const std::vector<std::string> &supported_tags)
{
  std::vector<std::string> res;
  std::vector<bool> enum_flags;
  enum_flags.resize(supported_tags.size(), false);

  for (const auto &tag : tags)
  {
    bool found = false;
    for (uint32_t j = 0u; j < supported_tags.size(); ++j)
    {
      if (supported_tags[j] == tag && !enum_flags[j])
      {
        enum_flags[j] = true;
        res.push_back(supported_tags[j]);
        found = true;
        break;
      }
    }

    if (!found)
    {
      printf("Error: Could not match token: '%s'\n", tag.c_str());
      exit(-1);
    }
  }

  return res;
}


void call_kernel_slicer(const BenchmarkAppEnums &enums, const std::string &repr_type, const std::string &renderer,
                        const std::string &backend, const std::string &slicer_dir, const std::string &slicer_exec)
{
// Slicer preprocess

  std::string disable_flags;

  for (const auto &enum_repr_type : enums.repr_types)
  {
    // MESH_LOD is supported, but there's no DISABLE_MESH_LOD, it is still just mesh

    if (enum_repr_type == "MESH_LOD") // don't add -DDISABLE_MESH_LOD, even though it's not used anywhere
      continue;
    if (repr_type == "MESH_LOD" && enum_repr_type == "MESH") // don't add -DDISABLE_MESH when render MESH_LOD
      continue;


    if (enum_repr_type != repr_type)
      disable_flags += " -DDISABLE_" + enum_repr_type;
  }

  // current ans slicer directories
  std::filesystem::path curr_p = std::filesystem::current_path(), slicer_p;

  std::filesystem::current_path(slicer_dir);
  slicer_p = std::filesystem::current_path();
  std::filesystem::current_path(curr_p);


  // backend- and renderer-specific parameters

  std::string main_class = " MultiRenderer";
  std::string renderer_src = " " + curr_p.native() + "/Renderer/eye_ray.cpp";
  std::string parameters = "";

  if (backend == "GPU_RQ")
  {
    parameters += "\
    -options " + curr_p.native() + "/scripts/options.json \
    -intersectionShader AbstractObject::Intersect \
    -intersectionTriangle GeomDataTriangle \
    -intersectionBlackList GeomDataRF \
    -intersectionBlackList GeomDataGS";
  }
  else if (backend == "RTX")
  {
    parameters += "\
    -options " + curr_p.native() + "/scripts/options.json \
    -intersectionShader AbstractObject::Intersect";
  }
  parameters += " -enable_ray_tracing_pipeline " + std::to_string(backend == "RTX");

  if (renderer == "MR")
  {
    std::string multirenderer_suffix = " -suffix _gpu_rq";
    if (backend != "GPU_RQ")
      multirenderer_suffix =  " -suffix _" + backend;

    std::string i_params = "\
    -I" + slicer_p.native() + "/TINYSTL                     ignore  \
    -I" + curr_p.native() + "/dependencies          ignore  \
    -I" + curr_p.native() + "/dependencies/HydraCore3/external          ignore  \
    -I" + curr_p.native() + "/dependencies/HydraCore3/external/LiteMath ignore  \
    -I" + curr_p.native() + "/                      process \
    -I" + curr_p.native() + "/Renderer              process \
    -I" + curr_p.native() + "/BVH                   process \
    -I" + curr_p.native() + "/sdfScene              ignore";

    parameters += i_params + multirenderer_suffix + " -DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v ";
  }
  else if (renderer == "Hydra")
  {
    main_class = " Integrator";
    renderer_src = "\
    " + curr_p.native() + "/dependencies/HydraCore3/integrator_pt.cpp\
    " + curr_p.native() + "/dependencies/HydraCore3/integrator_pt_lgt.cpp\
    " + curr_p.native() + "/dependencies/HydraCore3/integrator_pt_mat.cpp\
    " + curr_p.native() + "/dependencies/HydraCore3/integrator_rt.cpp\
    " + curr_p.native() + "/dependencies/HydraCore3/integrator_spectrum.cpp";

    std::string i_params = "\
    -I" + slicer_p.native() + "/TINYSTL                     ignore  \
    -I" + slicer_p.native() + "/apps/LiteMathAux            ignore  \
    -I" + curr_p.native() + "/dependencies/HydraCore3/external/LiteScene ignore  \
    -I" + curr_p.native() + "/dependencies/HydraCore3/external/LiteMath  ignore  \
    -I" + curr_p.native() + "/dependencies/HydraCore3/external           ignore  \
    -I" + curr_p.native() + "/dependencies/HydraCore3/cam_plugin         process \
    -I" + curr_p.native() + "/                      process \
    -I" + curr_p.native() + "/BVH                   process \
    -I" + curr_p.native() + "/sdfScene              ignore  ";

    parameters += i_params + "\
    -enable_motion_blur 0 \
    -gen_gpu_api 0 \
    -DLITERT_RENDERER \
    -DKERNEL_SLICER -v";
  }


  std::string slicer_command = slicer_exec + renderer_src + " " + curr_p.native() + "/BVH/BVH2Common.cpp" +
  " -mainClass " + main_class +
  " -composInterface ISceneObject \
  -composImplementation BVHRT \
  -stdlibfolder $PWD/TINYSTL \
  -megakernel 1 \
  -timestamps 1 \
  -pattern rtv \
  -shaderCC glsl " + parameters + disable_flags;

  // std::ofstream out_f{std::string("config/out_slicer_") + backend + '_' + renderer + ".txt"};
  // out_f << slicer_command;
  // out_f.close();

  slicer_command += " >> " + curr_p.native() + "/benchmark/saves/slicer_out.txt" + " 2>&1";

  std::filesystem::current_path(slicer_p);
  std::system(slicer_command.c_str());
  std::filesystem::current_path(curr_p);


// Slicer build shaders

  if (renderer == "MR")
  {
    std::string shaders_dir = "shaders";
    if (backend == "GPU")
      shaders_dir += "_gpu";
    else if (backend == "RTX")
      shaders_dir += "_rtx";
    else if (backend == "GPU_RQ")
      shaders_dir += "_gpu_rq";

    std::filesystem::remove_all(shaders_dir);
    std::filesystem::create_directory(shaders_dir);

    auto tmp_p = curr_p;
    tmp_p.append("Renderer");
    tmp_p.append(shaders_dir);
    tmp_p = tmp_p.lexically_normal();
    std::filesystem::current_path(tmp_p);

    std::system((std::string("bash build.sh") + " >> " + curr_p.native() + "/benchmark/saves/slicer_out.txt" + " 2>&1").c_str());
    std::string commd = "find -name \"*.spv\" | xargs cp --parents -t ../../" + shaders_dir;
    std::system(commd.c_str());
    std::filesystem::current_path(curr_p);
  }
  else if (renderer == "Hydra")
  {
    auto tmp_p = curr_p;
    tmp_p.append("dependencies/HydraCore3/shaders_generated");
    tmp_p = tmp_p.lexically_normal();
    std::filesystem::current_path(tmp_p);

    std::system((std::string("bash build.sh") + " >> " + curr_p.native() + "/benchmark/saves/slicer_out.txt" + " 2>&1").c_str());
    std::filesystem::current_path(curr_p);
  }
}



int main(int argc, const char **argv)
{
  BenchmarkAppEnums enums{};
  BenchmarkAppConfig config{};

  bool verbose = false, force_build = false, delete_models = false;
  bool clear_results = false;
  const char *enums_config_fpath = "benchmark/enums.blk", *base_config_fpath = "benchmark/config.blk";
  const char *slicer_dir = "~/kernel_slicer/", *slicer_exec = "~/kernel_slicer/cmake-build-release/kslicer";

  config = read_benchmark_config(base_config_fpath); // reads defaults

// Find flags

  std::vector<uint32_t> param_indices;
  long param_ind_defaults = -1, param_ind_base = -1;

  for (uint32_t i = 1u; i < argc; ++i)
  {
    if (argv[i][0] == '-')
    {
      if (std::string("-e") == argv[i] || std::string("--enum_conf") == argv[i])
        param_ind_defaults = param_indices.size();
      else if (std::string("--conf") == argv[i])
        param_ind_base = param_indices.size();

      // printf("Found flag %s\n", argv[i]);
      param_indices.push_back(i);
    }
  }
  param_indices.push_back(argc);


// Process config flags
  if (param_ind_defaults >= 0u) {
    // --enum_conf
    uint32_t start = param_indices[param_ind_defaults] + 1, fin = param_indices[param_ind_defaults + 1];
    if ((fin - start) != 1)
    {
      printf("Error: --enum_conf expects 1 parameter, got %d\n", fin - start);
      exit(-1);
    }
    enums_config_fpath = argv[start];
  }

  if (param_ind_base >= 0u) {
    // --conf
    uint32_t start = param_indices[param_ind_base] + 1, fin = param_indices[param_ind_base + 1];
    if (long(fin - start) != 1u)
    {
      printf("Error: --conf expects 1 parameter, got %d\n", fin - start);
      exit(-1);
    }
    base_config_fpath = argv[start];
  }

  enums = read_enums_config(enums_config_fpath); // reads enums
  config = read_benchmark_config(base_config_fpath); // reads defaults



// Process benchmark flags

  for (uint32_t i = 1u; i < param_indices.size(); ++i)
  {
    uint32_t start = param_indices[i-1], fin = param_indices[i];
    std::string flag = argv[start++];


    if (flag == "-v" || flag == "--verbose")
      verbose = true;
    else if (flag == "-f" || flag == "--force_build")
      force_build = true;
    else if (flag == "--clear")
      clear_results = true;
    else if (flag == "--delete_models")
      delete_models = true;
    else if ("-e" == flag || "--enum_conf" == flag || "--conf" == flag)
    {} // skip
    else if (flag == "-b" || flag == "--backends")
    {
      config.backends.clear();
      config.backends.reserve(fin - start);
      for (uint32_t i = start; i < fin; ++i)
        config.backends.push_back(std::string(argv[i]));
    }
    else if (flag == "-r" || flag == "--renderers")
    {
      config.renderers.clear();
      config.renderers.reserve(fin - start);
      for (uint32_t i = start; i < fin; ++i)
        config.renderers.push_back(std::string(argv[i]));
    }
    else if (flag == "-rm" || flag == "--render_modes")
    {
      config.render_modes.clear();
      config.render_modes.reserve(fin - start);
      for (uint32_t i = start; i < fin; ++i)
        config.render_modes.push_back(std::string(argv[i]));
    }
    else if (flag == "-m" || flag == "--models")
    {
      config.models.clear();
      config.models.reserve(fin - start);
      for (uint32_t i = start; i < fin; ++i)
        config.models.push_back(std::string(argv[i]));
    }
    else if (flag == "-s" || flag == "--slicer")
    {
      if (fin - start != 2)
      {
        printf("Error: --slicer expects 2 parameters, got %d\n", fin - start);
        exit(-1);
      }
      slicer_dir  = argv[start];
      slicer_exec = argv[start + 1];
    }
    else // uint parameter
    {
      if (fin - start != 1)
      {
        printf("Error: parameter %s must be a uint\n", flag.c_str());
        exit(-1);
      }
      uint32_t val = std::stoul(argv[start]);

      if (flag == "-w" || flag == "--width")
        config.width = val;
      else if (flag == "-h" || flag == "--height")
        config.height = val;
      else if (flag == "-c" || flag == "--cameras")
        config.cameras = val;
      else if (flag == "-i" || flag == "--iters")
        config.iters = val;
      else if (flag == "--spp")
        config.spp = val;
    }
  }

// Filter enums

  config.backends = filter_enum_params(config.backends, enums.backends);
  config.renderers = filter_enum_params(config.renderers, enums.renderers);
  config.render_modes = filter_enum_params(config.render_modes, enums.render_modes);

  {
    std::vector <std::string> repr_types_vec = { "MESH" };
    std::map <std::string, std::vector<std::string>> repr_types_map;

    for (const auto &repr_config : config.repr_configs)
    {
      if (repr_config.first != "MESH")
        repr_types_vec.push_back(repr_config.first);
    }

    if (config.repr_configs.find("MESH") == config.repr_configs.end() || config.repr_configs["MESH"].size() != 1)
    {
      printf("Error: only one config for mesh is supported\n");
      exit(-1);
    }

    repr_types_vec = filter_enum_params(repr_types_vec, enums.repr_types);

    for (const auto &repr_type : repr_types_vec)
    {
      repr_types_map[repr_type] = (config.repr_configs.find(repr_type) != config.repr_configs.end()) ? config.repr_configs[repr_type] : std::vector<std::string>{};
    }

    config.repr_configs = repr_types_map; // filtered repr_configs
  }

// Process slicer_dir path

  std::string slicer_adir = slicer_dir;

  // replace '~' with a home dir path on Linux, filesystem lib isn't able to do this
  if (slicer_adir.find("~") != slicer_adir.npos)
  {
    std::string home_dir = std::getenv("HOME");
    if (home_dir.empty())
    {
      printf("Error: Could not replace '~' with a home dir in slicer path.\n");
      exit(-1);
    }

    while (slicer_adir.find("~") != slicer_adir.npos)
      slicer_adir.replace(slicer_adir.find("~"), 1, home_dir);
  }


// Print config

  if (verbose)
  {
    printf("Verbose: %d\n", verbose);
    printf("Def conf: %s\n", enums_config_fpath);
    printf("Base conf: %s\n", base_config_fpath);
    printf("Slicer dir: %s\n", slicer_adir.c_str());
    printf("Slicer exec: %s\n", slicer_exec);

    printf("\nCONFIG.");
    printf("\nModels:\n");
    for (auto elem : config.models)
      printf("%s\n", elem.c_str());
    printf("\nBackends:\n");
    for (auto elem : config.backends)
      printf("%s, ", elem.c_str());
    printf("\nRenderers:\n");
    for (auto elem : config.renderers)
      printf("%s, ", elem.c_str());
    printf("\nRender modes:\n");
    for (auto elem : config.render_modes)
      printf("%s, ", elem.c_str());
    printf("\nTypes:\n");
    for (auto elem : config.repr_configs)
      printf("%s x %ld, ", elem.first.c_str(), std::max(elem.second.size(), 1ul));

    printf("\nWidth: %d\n", config.width);
    printf("Height: %d\n", config.height);
    printf("Cameras: %d\n", config.cameras);
    printf("Iters: %d\n", config.iters);
    printf("SPP: %d\n", config.spp);
  }

  // Create dirs and .csv for results

  std::fstream f;
  std::filesystem::create_directories("benchmark/results");
  std::filesystem::create_directories("benchmark/saves");

  std::string build_results_dir = "benchmark/results/build.csv";
  std::string render_results_dir = "benchmark/results/render.csv";

  if (!std::filesystem::exists(build_results_dir) || clear_results)
  {
    f.open(build_results_dir, std::ios::out);
    f << "model_name,type,config_name,original_model_size(Mb),model_size(Mb),build_time(s)\n";
    f.close();
  }

  if (!std::filesystem::exists(render_results_dir) || clear_results)
  {
    f.open(render_results_dir, std::ios::out);
    f << "model_name,backend,device,renderer,type,";
    f << "config_name,render_mode,model_size(Mb),time(ms),alt time(ms),";
    f << "psnr_average,psnr_min,psnr_max,";
    f << "flip_average,flip_min,flip_max,";
    f << "ssim_average,ssim_min,ssim_max,";
    f << "time min(ms),time max(ms),time average(ms),";
    f << "alt time min(ms),alt time max(ms),alt time average(ms)\n";
    f.close();
  }

  f.open("benchmark/saves/cmake_out.txt", std::ios::in | std::ios::out | std::ios::trunc);
  f.close();
  f.open("benchmark/saves/slicer_out.txt", std::ios::in | std::ios::out | std::ios::trunc);
  f.close();

  // Benchmark loop

  uint32_t configs_overall = 0u;
  for (const auto &repr_config : config.repr_configs)
    configs_overall += repr_config.second.size();

  // Build

  RenderAppConfig build_config;
  build_config.spp = config.spp;
  build_config.iters = config.iters;
  build_config.width = config.width;
  build_config.height = config.height;
  build_config.cameras = config.cameras;
  build_config.render_modes = config.render_modes;
  build_config.mesh_config_name = config.repr_configs["MESH"][0];
  build_config.mesh_config_name = build_config.mesh_config_name.substr(0, build_config.mesh_config_name.find('{'));

  build_config.hydra_material_id = config.hydra_material_id;
  build_config.hydra_spp = config.hydra_spp;
  build_config.hydra_scene = config.hydra_scene;

  auto benchmark_t1 = std::chrono::steady_clock::now();

  printf("\n\nBENCHMARK uses config %s\n", base_config_fpath);
  printf("BENCHMARK - BUILD\n");

  uint32_t builds_overall = config.models.size() * configs_overall;

  int counter_models = -1;
  for (const auto &model : config.models)
  {
    ++counter_models;
    printf("\nMODEL [%d/%d]: %s\n", counter_models+1, (int)config.models.size(), model.c_str());

    build_config.model_name = get_model_name(model);

    int counter_configs = -1; // for 2 cycles
    for (const auto &repr_config : config.repr_configs)
    {
      build_config.type = repr_config.first;

      for (const auto &repr_config_single_n: repr_config.second)
      {
        ++counter_configs;
        uint32_t bracket_ind = repr_config_single_n.find('{');
        std::string repr_config_name = repr_config_single_n.substr(0, bracket_ind);
        std::string repr_config_single = repr_config_single_n.substr(bracket_ind);

        // Print current model build information

        uint32_t current_test_id = counter_configs;
        current_test_id += counter_models * configs_overall;

        printf("\nBENCHMARK BUILD [%d/%d]: %s, %s, %s.\n", current_test_id + 1, builds_overall, build_config.model_name.c_str(),
                                                           build_config.type.c_str(), repr_config_name.c_str());


        // Create directories for model and tests

        build_config.model = model;
        build_config.repr_config = repr_config_single;
        build_config.repr_config_name = repr_config_name;
        std::string xml_path = generate_filename_model_no_ext(build_config.model, build_config.type, repr_config_name) + ".xml";
        std::string experiment_path = generate_filename_image(build_config.model, build_config.renderer, build_config.backend,
                                                              build_config.type,  repr_config_name, build_config.cameras);
        std::string dirs_to_create = xml_path.substr(0, (xml_path.rfind('/') != xml_path.npos) ? xml_path.rfind('/') : xml_path.rfind('\\'));
        printf("Model dir: %s\n", dirs_to_create.c_str());
        std::filesystem::create_directories(dirs_to_create);


        // Build

        if (!std::filesystem::exists(xml_path) || force_build)
        {
          build_config.model = model;
          std::string config_str = write_render_config_s(build_config);

          std::string cmd = "DRI_PRIME=1 ./render_app -backend_benchmark build ";
          cmd += "'" + config_str + "'";

          std::system(cmd.c_str());
          // printf("\n\nCommand:\n%s\n\n", cmd.c_str());
        }
        else
          printf("Model already exists.\n");
      }
    }
  }


  // Render

  RenderAppConfig render_config{};
  render_config.spp = config.spp;
  render_config.iters = config.iters;
  render_config.width = config.width;
  render_config.height = config.height;
  render_config.cameras = config.cameras;
  render_config.render_modes = config.render_modes;
  render_config.mesh_config_name = config.repr_configs["MESH"][0];
  render_config.mesh_config_name = render_config.mesh_config_name.substr(0, render_config.mesh_config_name.find('{'));
  
  render_config.hydra_material_id = config.hydra_material_id;
  render_config.hydra_spp = config.hydra_spp;
  render_config.hydra_scene = config.hydra_scene;

  uint32_t tests_overall = config.models.size() * config.renderers.size() * config.backends.size() * configs_overall;

  printf("\n\nBENCHMARK - RENDER\n\n");

  counter_models = -1;
  for (const auto &model : config.models)
  {
    ++counter_models;
    printf("MODEL [%d/%d]: %s\n", counter_models+1, (int)config.models.size(), model.c_str());

    render_config.model_name = get_model_name(model);

    int counter_renderers = -1;
    for (const auto &renderer : config.renderers)
    {
      ++counter_renderers;
      render_config.renderer = renderer;

      int counter_backends = -1;
      for (const auto &backend : config.backends)
      {
        ++counter_backends;
        render_config.backend = backend;

        bool do_slicing = backend != "CPU";
        bool recompile = true;

        int counter_configs = -1; // for 2 cycles
        for (const auto &repr_config : config.repr_configs)
        {
          render_config.type = repr_config.first;

          // Slice
          if (do_slicing)
          {
            printf("Rebuild shaders\n");
            call_kernel_slicer(enums, render_config.type, render_config.renderer, render_config.backend, slicer_adir, slicer_exec);
            recompile = true;
          }

          // Recompile
          if (recompile)
          {
            printf("Recompile code\n");

            std::string use_gpu = backend != "CPU" ? "ON" : "OFF";
            std::string use_rtx = (backend == "RTX") ? "ON" : "OFF";
            std::string use_gpu_rq = (backend == "GPU_RQ") ? "ON" : "OFF";
            std::string reconfigure_cmd = "cmake -S . -B build -DUSE_VULKAN=" + use_gpu + 
                                          " -DUSE_RTX=" + use_rtx + " -DUSE_GPU_RQ=" + use_gpu_rq +
                                          " -DCMAKE_BUILD_TYPE=Release -DUSE_STB_IMAGE=ON >> benchmark/saves/cmake_out.txt 2>&1";

            std::system(reconfigure_cmd.c_str());
            std::system("cmake --build build --target render_app -j8 >> benchmark/saves/cmake_out.txt 2>&1");
            recompile = false;
          }


          for (const auto &repr_config_single_n: repr_config.second)
          {
            ++counter_configs;
            uint32_t bracket_ind = repr_config_single_n.find('{');
            std::string repr_config_name = repr_config_single_n.substr(0, bracket_ind);
            std::string repr_config_single = repr_config_single_n.substr(bracket_ind);

            // Print current test information

            uint32_t current_test_id = counter_configs;
            current_test_id += counter_backends * configs_overall;
            current_test_id += counter_renderers * config.backends.size() * configs_overall;
            current_test_id += counter_models * config.renderers.size() * config.backends.size() * configs_overall;
            // config.models.size() * config.renderers.size() * config.backends.size() * configs_overall

            printf("BENCHMARK TEST [%d/%d]: %s, %s, %s, %s, %s.\n", current_test_id + 1, tests_overall, render_config.model_name.c_str(), renderer.c_str(),
                                                                    backend.c_str(), render_config.type.c_str(), repr_config_name.c_str());


            // Create directories for tests

            render_config.model = model;
            render_config.repr_config = repr_config_single;
            render_config.repr_config_name = repr_config_name;
            std::string xml_path = generate_filename_model_no_ext(render_config.model, render_config.type, repr_config_name) + ".xml";
            std::string experiment_path = generate_filename_image(render_config.model, render_config.renderer, render_config.backend,
                                                                  render_config.type,  repr_config_name, render_config.cameras);
            std::string dirs_to_create = experiment_path.substr(0, (experiment_path.rfind('/') != experiment_path.npos) ? experiment_path.rfind('/') : experiment_path.rfind('\\'));
            printf("Creating tests dir: %s\n", dirs_to_create.c_str());
            std::filesystem::create_directories(dirs_to_create);


            // Render

            {
              render_config.model = xml_path;
              std::string config_str = write_render_config_s(render_config);

              //do this if shit happens
              //std::string cmd = "gdb -ex=run --args ./render_app -backend_benchmark render ";
              std::string cmd = "DRI_PRIME=1 ./render_app -backend_benchmark render ";
              cmd += "'" + config_str + "'";

              std::system(cmd.c_str());
              // printf("\n\nCommand:\n%s\n\n", cmd.c_str());
            }
          }
        }
      }
    }
  }

  auto benchmark_t2 = std::chrono::steady_clock::now();
  float t = std::chrono::duration_cast<std::chrono::milliseconds>(benchmark_t2 - benchmark_t1).count() / 1000.f; // ms

  printf("Benchmark finished in %f s\n", t);

  if (delete_models)
  {
    printf("Deleting generated models...\n");
    auto saves_entries = std::filesystem::directory_iterator("./benchmark/saves");

    for (auto entry : saves_entries)
    {
      if (std::filesystem::is_directory(entry))
      {
        std::filesystem::path models_dir = entry;
        models_dir += "/models";
        if (std::filesystem::exists(models_dir))
        {
          printf("Deleting directory: %s\n", models_dir.c_str());
          std::filesystem::remove_all(models_dir);
        }
      }
    }
  }

  if (config.backends.size() > 1 || (config.backends.size() == 1 && config.backends[0] != "CPU")) {
    // Run "slicer_execute.sh" with all types On
    printf("Running slicer_execute...\n");
    std::string slicer_execute_command = "bash slicer_execute.sh " + slicer_adir + " " + slicer_exec + " >> benchmark/saves/slicer_out.txt" + " 2>&1";
    std::system(slicer_execute_command.c_str());
  }

  printf("Finished\n");
  return 0;
}