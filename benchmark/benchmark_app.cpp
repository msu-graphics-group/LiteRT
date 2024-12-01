#include "benchmark_app.h"

#include <fstream>


//// Parameters:
//
// -s, --slicer        = str str // slicer parameters
//
///  Config files
// -d, --def_conf      = str // path to a config file containing defaults and enums
//     --conf          = str // path to a config file (input arguments overwrite it)
// -v, --verbose             // show processing info
//
///  Benchmark_app-level
// -b, --backends      = {CPU|GPU|RTX|GPU_RQ}
// -r, --renderers     = {MR|Hydra}
// -t, --types         = {MESH|SDF_GRID|SDF_SBS|...}
// -m, --models        = str str str...
//
///  Render_app-level
// -rm, --render_modes = {lambert|...}  // see "MULTI_RENDER_MODE" enum, NOTE: in render_app config this parameter is "uint", matching enum
//     --lods          = {low|mid|high}
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



BenchmarkAppConfig read_enums_config(const char *defaults_config_fpath)
{
  BenchmarkAppConfig res_defaults{};
  Block block_defaults{};

  if (!load_block_from_file(defaults_config_fpath, block_defaults))
  {
    printf("Error: Could not read defaults file: %s\n", defaults_config_fpath);
    exit(-1);
  }

  block_defaults.get_arr("backends_enum", res_defaults.backends);
  block_defaults.get_arr("renderers_enum", res_defaults.renderers);
  block_defaults.get_arr("render_modes_enum", res_defaults.render_modes);
  block_defaults.get_arr("types_enum", res_defaults.types);
  block_defaults.get_arr("lods_enum", res_defaults.lods);

  return res_defaults;
}

void write_defaults_config(const char *defaults_config_fpath, const BenchmarkAppConfig &in_enums, const BenchmarkAppConfig &in_defaults)
{
  Block block_defaults{};

  block_defaults.set_arr("backends_enum", in_enums.backends);
  block_defaults.set_arr("renderers_enum", in_enums.renderers);
  block_defaults.set_arr("render_modes_enum", in_enums.render_modes);
  block_defaults.set_arr("types_enum", in_enums.types);
  block_defaults.set_arr("lods_enum", in_enums.lods);


  block_defaults.set_arr("backends", in_defaults.backends);
  block_defaults.set_arr("renderers", in_defaults.renderers);
  block_defaults.set_arr("render_modes", in_defaults.render_modes);
  block_defaults.set_arr("types", in_defaults.types);
  block_defaults.set_arr("lods", in_defaults.lods);
  block_defaults.set_arr("models", in_defaults.models);
  block_defaults.set_arr("param_strings", in_defaults.param_strings);

  block_defaults.set_int("width", in_defaults.width);
  block_defaults.set_int("height", in_defaults.height);
  block_defaults.set_int("cameras", in_defaults.cameras);
  block_defaults.set_int("iters", in_defaults.iters);
  block_defaults.set_int("spp", in_defaults.spp);

  save_block_to_file(defaults_config_fpath, block_defaults);
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
  block_benchmark.get_arr("types", res_benchmark.types);
  block_benchmark.get_arr("lods", res_benchmark.lods);
  block_benchmark.get_arr("models", res_benchmark.models);
  block_benchmark.get_arr("param_strings", res_benchmark.param_strings);

  res_benchmark.width = block_benchmark.get_int("width");
  res_benchmark.height = block_benchmark.get_int("height");
  res_benchmark.cameras = block_benchmark.get_int("cameras");
  res_benchmark.iters = block_benchmark.get_int("iters");
  res_benchmark.spp = block_benchmark.get_int("spp");

  return res_benchmark;
}

void write_benchmark_config(const char *defaults_config_fpath, const BenchmarkAppConfig &in_benchmark)
{
  Block block_benchmark{};

  block_benchmark.set_arr("backends", in_benchmark.backends);
  block_benchmark.set_arr("renderers", in_benchmark.renderers);
  block_benchmark.set_arr("render_modes", in_benchmark.render_modes);
  block_benchmark.set_arr("types", in_benchmark.types);
  block_benchmark.set_arr("lods", in_benchmark.lods);
  block_benchmark.set_arr("models", in_benchmark.models);
  block_benchmark.set_arr("param_strings", in_benchmark.param_strings);

  block_benchmark.set_int("width", in_benchmark.width);
  block_benchmark.set_int("height", in_benchmark.height);
  block_benchmark.set_int("cameras", in_benchmark.cameras);
  block_benchmark.set_int("iters", in_benchmark.iters);
  block_benchmark.set_int("spp", in_benchmark.spp);

  save_block_to_file(defaults_config_fpath, block_benchmark);
}

std::string write_render_config_s(const RenderAppConfig &in_render)
{
  std::string res_block_str;
  Block block_render{};

  block_render.set_string("model", in_render.model);
  block_render.set_string("render_mode", in_render.render_mode);
  block_render.set_arr("param_strings", in_render.param_strings);
  block_render.set_arr("lods", in_render.lods);

  block_render.set_int("width", in_render.width);
  block_render.set_int("height", in_render.height);
  block_render.set_int("cameras", in_render.cameras);
  block_render.set_int("iters", in_render.iters);
  block_render.set_int("spp", in_render.spp);

  save_block_to_string(res_block_str, block_render);
  return res_block_str;
}



std::vector<std::string> filter_enum_params(const char **argv, uint32_t len, const std::vector<std::string> supported_tags)
{
  std::vector<std::string> res;
  std::vector<bool> enum_flags;
  enum_flags.resize(supported_tags.size(), false);

  for (uint32_t i = 0u; i < len; ++i)
  {
    bool found = false;
    for (uint32_t j = 0u; j < supported_tags.size(); ++j)
    {
      if (supported_tags[j] == argv[i] && !enum_flags[j])
      {
        enum_flags[j] = true;
        res.push_back(supported_tags[j]);
        found = true;
        break;
      }
    }

    if (!found)
    {
      printf("Error: Could not match token: %s\n", argv[i]);
      exit(-1);
    }
  }

  return res;
}


void call_kernel_slicer(const BenchmarkAppConfig &defaults, const std::string &repr_type, const std::string &renderer,
                        const std::string &backend, const std::string &slicer_dir, const std::string &slicer_exec)
{
// Slicer preprocess

  std::string disable_flags;
  disable_flags.reserve(defaults.types.size() - 1);

  for (uint32_t i = 0u; i < defaults.types.size(); ++i)
  {
    if (defaults.types[i] != repr_type)
      disable_flags += " -DDISABLE_" + defaults.types[i];
  }

  // current ans slicer directories
  std::filesystem::path curr_p = std::filesystem::current_path(), slicer_p;

  std::filesystem::current_path(slicer_dir);
  slicer_p = std::filesystem::current_path();
  // printf("curr path: %s\n", curr_p.c_str());
  // printf("slicer path: %s\n", slicer_p.c_str());
  std::filesystem::current_path(curr_p);


  // backend- and renderer-specific parameters

  std::string main_class = " MultiRenderer";
  std::string renderer_src = " " + curr_p.native() + "/Renderer/eye_ray.cpp";
  std::string parameters = "";

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

    if (backend == "GPU_RQ")
      i_params += "\
      -options " + curr_p.native() + "/options.json \
      -intersectionShader AbstractObject::Intersect \
      -intersectionTriangle GeomDataTriangle \
      -intersectionBlackList GeomDataRF \
      -intersectionBlackList GeomDataGS";
    else if (backend == "RTX")
      i_params += "\
      -options " + curr_p.native() + "/options.json \
      -intersectionShader AbstractObject::Intersect";

    parameters = i_params + multirenderer_suffix + " -DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v "
    + "-enable_ray_tracing_pipeline " + std::to_string(backend == "RTX");
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

    parameters = i_params + "\
    -enable_ray_tracing_pipeline 0 \
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

    std::system("bash build.sh");
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

    std::system("bash build.sh");
    std::filesystem::current_path(curr_p);
  }
}



int main(int argc, const char **argv)
{
  BenchmarkAppConfig defaults{}, config{};
  RenderAppConfig render_config{};

  bool verbose = false;
  const char *defaults_config_fpath = "benchmark/defaults.blk", *base_config_fpath = "benchmark/defaults.blk";
  const char *slicer_dir = "~/kernel_slicer/", *slicer_exec = "~/kernel_slicer/cmake-build-release/kslicer";


// Find flags

  std::vector<uint32_t> param_indices;
  long param_ind_defaults = -1, param_ind_base = -1;

  for (uint32_t i = 1u; i < argc; ++i)
  {
    if (argv[i][0] == '-')
    {
      if (std::string("-d") == argv[i] || std::string("--def_conf") == argv[i])
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
    // --def_conf
    uint32_t start = param_indices[param_ind_defaults] + 1, fin = param_indices[param_ind_defaults + 1];
    if ((fin - start) != 1)
    {
      printf("Error: --def_conf expects 1 parameter, got %d\n", fin - start);
      exit(-1);
    }
    defaults_config_fpath = argv[start];
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

  defaults = read_enums_config(defaults_config_fpath); // reads enums
  config = read_benchmark_config(base_config_fpath); // reads defaults



// Process benchmark flags

  for (uint32_t i = 1u; i < param_indices.size(); ++i)
  {
    uint32_t start = param_indices[i-1], fin = param_indices[i];
    std::string flag = argv[start++];


    if (flag == "-v" || flag == "--verbose")
      verbose = true;
    else if ("-d" == flag || "--def_conf" == flag || "--conf" == flag)
    {} // skip
    else if (flag == "-b" || flag == "--backends")
    {
      config.backends = filter_enum_params(argv + start, fin - start, defaults.backends);
    }
    else if (flag == "-r" || flag == "--renderers")
    {
      config.renderers = filter_enum_params(argv + start, fin - start, defaults.renderers);
    }
    else if (flag == "-t" || flag == "--types")
    {
      config.types = filter_enum_params(argv + start, fin - start, defaults.types);
    }
    else if (flag == "-rm" || flag == "--render_modes")
    {
      config.render_modes = filter_enum_params(argv + start, fin - start, defaults.render_modes);
    }
    else if (flag == "--lods")
    {
      config.lods = filter_enum_params(argv + start, fin - start, defaults.lods);
    }
    else if (flag == "-m" || flag == "--models")
    {
      config.models.reserve(fin - start);
      for (uint32_t i = start; i < fin; ++i)
        config.models.push_back(std::string(argv[i]));
    }
    else if (flag == "-p" || flag == "--param_strings")
    {
      config.param_strings.reserve(fin - start);
      for (uint32_t i = start; i < fin; ++i)
        config.param_strings.push_back(argv[i]);
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
    printf("Verbose %d\n", verbose);
    printf("Def conf: %s\n", defaults_config_fpath);
    printf("Base conf: %s\n", base_config_fpath);
    printf("Slicer dir: %s\n", slicer_adir.c_str());
    printf("Slicer exec: %s\n", slicer_exec);

    printf("\nCONFIG.");
    printf("\nModels:\n");
    for (auto elem : config.models)
      printf("%s\n", elem.c_str());
    printf("\nParam strings:\n");
    for (auto elem : config.param_strings)
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
    printf("\nLODs:\n");
    for (auto elem : config.lods)
      printf("%s, ", elem.c_str());
    printf("\nTypes:\n");
    for (auto elem : config.types)
      printf("%s, ", elem.c_str());

    printf("\nWidth: %d\n", config.width);
    printf("Height: %d\n", config.height);
    printf("Cameras: %d\n", config.cameras);
    printf("Iters: %d\n", config.iters);
    printf("SPP: %d\n", config.spp);
  }

  std::fstream f;
  f.open("benchmark/results/results.csv", std::ios::out);
  f << "model_name, backend, renderer, type, lod, memory(Mb), time_min, time_max, time_average, psnr_min, psnr_max, psnr_average, flip_min, flip_max, flip_average\n";
  f.close();

  // Benchmark loop
  for (const auto &model : config.models)
  {
    // TODO: Build SDFs, benchmark_build

    for (const auto &renderer : config.renderers)
    {
      for (const auto &backend : config.backends)
      {
        bool do_slicing = backend != "CPU";
        if (renderer == "Hydra" && backend != "GPU")
          do_slicing = false;

        bool recompile = true;

        for (const auto &repr_type : config.types)
        {
          // Slice
          if (do_slicing)
          {
            call_kernel_slicer(defaults, repr_type, renderer, backend, slicer_adir, slicer_exec);
            recompile = true;
          }

          // Recompile
          if (recompile)
          {
            std::string use_gpu = backend != "CPU" ? "ON" : "OFF";
            std::string use_rtx = (backend == "RTX") ? "ON" : "OFF";
            std::string reconfigure_cmd = "cmake -S . -B build -DUSE_VULKAN=" + use_gpu + " -DUSE_RTX=" + use_rtx + " -DCMAKE_BUILD_TYPE=Debug -DUSE_STB_IMAGE=ON";

            std::system(reconfigure_cmd.c_str());
            std::system("cmake --build build --target render_app -j8");
            recompile = false;
          }

          // std::system("DRI_PRIME=1 ./render_app -tests_litert 9");

          for (const auto &lod: config.lods)
          {
            std::string cmd = "DRI_PRIME=1 ./render_app -backend_benchmark ";
            cmd += model + " " +  backend + " " + renderer + " " + repr_type + " " + 
                lod + " " + std::to_string(config.width) + " " + std::to_string(config.height) + 
                " " + std::to_string(config.spp) + " " + std::to_string(config.cameras);
            
            std::system(cmd.c_str());
          }
        }
      }
    }
  }

  printf("Benchmark finished\n");
  return 0;
}