#include <string>
#include <vector>
#include <cstdlib>
#include <filesystem>

#include "common/blk.h"


// This config can be saved to run the same tests
struct BenchmarkAppConfig
{
  std::vector<std::string> models;
  std::vector<std::string> backends;
  std::vector<std::string> renderers;
  std::vector<std::string> types;

  std::vector<std::string> render_modes;
  std::vector<std::string> lods;
  std::vector<std::string> param_strings;
  int width, height;
  int spp;
  int cameras;
  int iters;
};

// This config is passed to render_app
struct RenderAppConfig
{
  std::string model;
  std::string model_name;
  std::string renderer;
  std::string backend;
  std::string param_string;
  std::string lod;
  std::string type;
  std::vector<std::string> render_modes;
  int width, height;
  int spp;
  int cameras;
  int iters;
};