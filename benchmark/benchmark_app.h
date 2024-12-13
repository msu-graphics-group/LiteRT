#include <string>
#include <vector>
#include <cstdlib>
#include <filesystem>
#include <map>

#include "common/blk.h"

// This config contains allowed enum elements
struct BenchmarkAppEnums
{
  std::vector<std::string> backends;
  std::vector<std::string> renderers;
  std::vector<std::string> repr_types;
  std::vector<std::string> render_modes;
};

// This config can be saved/loaded to run the same tests
struct BenchmarkAppConfig
{
  std::vector<std::string> models;
  std::vector<std::string> backends;
  std::vector<std::string> renderers;
  std::map<std::string, std::vector<std::string>> repr_configs; // { Representation type : parameter blocks from config (representation configs) }

  std::vector<std::string> render_modes;
  int width, height;
  int spp;
  int cameras;
  int iters;

  int hydra_spp;
  int hydra_material_id;
  std::string hydra_scene;
};

// This config is passed to render_app
struct RenderAppConfig
{
  std::string model; // path to a generated model
  std::string model_name; // original name
  std::string renderer;
  std::string backend;
  std::string repr_config;
  std::string repr_config_name;
  std::string mesh_config_name;
  std::string type;
  std::vector<std::string> render_modes;
  int width, height;
  int spp;
  int cameras;
  int iters;

  int hydra_spp;
  int hydra_material_id;
  std::string hydra_scene;
};