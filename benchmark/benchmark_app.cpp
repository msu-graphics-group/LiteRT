#include "benchmark_app.h"


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
  std::string disable_flags;
  disable_flags.reserve(defaults.types.size() - 1);

  for (uint32_t i = 0u; i < defaults.types.size(); ++i)
  {
    if (defaults.types[i] != repr_type)
      disable_flags += " -DDISABLE_" + defaults.types[i];
  }


  // backend- and renderer-specific parameters

  std::string main_class = " MultiRenderer";
  std::string renderer_src = " ./Renderer/eye_ray.cpp";
  std::string parameters = "";

  if (renderer == "MR")
  {
    std::string multirenderer_suffix = " -suffix _gpu_rq";
    if (backend != "GPU_RQ")
      multirenderer_suffix =  " -suffix _" + backend;

    std::string i_params = " -I$PWD/TINYSTL                     ignore  \
  -I./dependencies          ignore  \
  -I./dependencies/HydraCore3/external          ignore  \
  -I./dependencies/HydraCore3/external/LiteMath ignore  \
  -I./                      process \
  -I./Renderer              process \
  -I./BVH                   process \
  -I./sdfScene              ignore";

    parameters = i_params + multirenderer_suffix + "-DPUGIXML_NO_EXCEPTIONS -DKERNEL_SLICER -v "
    + "-enable_ray_tracing_pipeline " + std::to_string(backend == "RTX");
  }
  else if (renderer == "Hydra")
  {
    main_class = " Integrator";
    renderer_src = " ./dependencies/HydraCore3/integrator_pt.cpp\
  ./dependencies/HydraCore3/integrator_pt_lgt.cpp\
  ./dependencies/HydraCore3/integrator_pt_mat.cpp\
  ./dependencies/HydraCore3/integrator_rt.cpp\
  ./dependencies/HydraCore3/integrator_spectrum.cpp";

    std::string i_params = " -I$PWD/TINYSTL                     ignore  \
  -I$PWD/apps/LiteMathAux            ignore  \
  -I./dependencies/HydraCore3/external/LiteScene ignore  \
  -I./dependencies/HydraCore3/external/LiteMath  ignore  \
  -I./dependencies/HydraCore3/external           ignore  \
  -I./dependencies/HydraCore3/cam_plugin         process \
  -I./                      process \
  -I./BVH                   process \
  -I./sdfScene              ignore  ";

    parameters = i_params + "-enable_ray_tracing_pipeline 0 \
-enable_motion_blur 0 \
-gen_gpu_api 0 \
-DLITERT_RENDERER \
-DKERNEL_SLICER -v";
  }


  std::string slicer_command = slicer_exec + renderer_src + " ./BVH/BVH2Common.cpp" +
    " -mainClass " + main_class +
    " -composInterface ISceneObject \
  -composImplementation BVHRT \
  -stdlibfolder $PWD/TINYSTL \
  -pattern rtv \
  -shaderCC glsl " + parameters;
}



int main(int argc, const char **argv)
{
  // BenchmarkAppConfig Defaults{};
  // Defaults.render_modes = {
  // "MASK",
  // "LAMBERT_NO_TEX",
  // "DEPTH",
  // "LINEAR_DEPTH",
  // "INVERSE_LINEAR_DEPTH",
  // "PRIMITIVE",
  // "TYPE",
  // "GEOM",
  // "NORMAL",
  // "BARYCENTRIC",
  // "ST_ITERATIONS",
  // "RF",
  // "PHONG_NO_TEX",
  // "GS",
  // "RF_DENSITY",
  // "TEX_COORDS",
  // "DIFFUSE",
  // "LAMBERT",
  // "PHONG",
  // "HSV_DEPTH",
  // };

  // Defaults.backends = {
  // "CPU",
  // "GPU",
  // "RTX",
  // "GPU_RQ",
  // };

  // Defaults.renderers = {
  // "MR",
  // "HYDRA",
  // };

  // Defaults.types = {
  // "MESH",
  // "SDF_GRID",
  // "SDF_SVS",
  // "SDF_SBS",
  // "SDF_SBS_ADAPT",
  // "SDF_FRAME_OCTREE",
  // "SDF_FRAME_OCTREE_TEX",
  // "SDF_FRAME_OCTREE_COMPACT",
  // "SDF_HP",
  // "RF_GRID",
  // "GS_PRIMITIVE",
  // "NURBS",
  // "RIBBON",
  // "CATMUL_CLARK",
  // "GRAPHICS_PRIM",
  // "OPENVDB",
  // };

  // Defaults.lods = {
  // "low",
  // "mid",
  // "high"
  // };

  // write_defaults_config("benchmark/defaults.blk", Defaults, config);

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

  // printf("Indices:\n");
  // for (auto ind : param_indices)
  //   printf("%d, ", ind);
  // printf("\n");


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

  if (verbose)
  {
    printf("Verbose %d\n", verbose);
    printf("Def conf: %s\n", defaults_config_fpath);
    printf("Base conf: %s\n", base_config_fpath);
    printf("Slicer dir: %s\n", slicer_dir);
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


// Benchmark loop

  for (const auto &model : config.models)
  {
    // TODO: Build SDFs, benchmark_build

    for (const auto &renderer : config.renderers)
    {
      for (const auto &backend : config.backends)
      {
        for (const auto &repr_type : config.types)
        {
          // TODO: call slicer
          call_kernel_slicer(defaults, repr_type, renderer, backend, slicer_dir, slicer_exec);
          // TODO: recompile

          for (const auto &param_string : config.param_strings)
          {
            for (const auto &lod : config.lods)
            {
              void* model_obj;
              auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
              cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

              MultiRenderPreset preset = getDefaultPreset();
              // preset.

              if (lod == "1")
              {

              }
              else if (lod == "2")
              {

              }
              else if (lod == "3")
              {
                //  Load model into chosen structure
                if (repr_type == "MESH")
                {
                  //  skip cause 
                }
                else if (repr_type == "SDF_GRID")
                {

                }
              }
              else
              {
                //  skip
              }
            }
          }
        }
      }
    }
  }

  return 0;
}