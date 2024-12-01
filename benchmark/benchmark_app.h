#include <string>
#include <vector>
#include <cstdlib>
#include <chrono>
#include <fstream>

#include "../utils/mesh.h"
#include "../utils/sdf_converter.h"
#include "../render_settings.h"
#include "../Renderer/eye_ray.h"
#include "../utils/image_metrics.h"
#include "../utils/mesh_bvh.h"
#include "blk.h"


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
  std::string render_mode;
  std::vector<std::string> lods;
  std::vector<std::string> param_strings;
  int width, height;
  int spp;
  int cameras;
  int iters;
};

MultiRenderPreset createPreset(const std::string& render_mode, const int spp);
int getDevice(const std::string backend);
int getFileSize(std::string file_name);
void calcMetrics(float& min, float& max, float& average, const float& new_val);

void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum = 1);