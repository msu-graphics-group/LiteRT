#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>
#include <filesystem>

#include "utils/mesh/mesh.h"
#include "utils/sdf/sdf_converter.h"
#include "render_settings.h"
#include "Renderer/eye_ray.h"
#include "utils/image_metrics.h"
#include "utils/mesh/mesh_bvh.h"
#include "utils/mesh/ply_reader.h"
#include "utils/common/blk.h"
#include "tests/hydra_integration.h"

namespace BenchmarkBackend
{
// Common

  std::string get_model_name(std::string model_path);
  std::string generate_filename_model_no_ext(std::string model_path, std::string repr_type, std::string repr_config_name);
  std::string generate_filename_image(std::string model_path, std::string renderer, std::string backend, std::string repr_type, std::string repr_config_name, uint32_t camera);

  void parse_param_string(std::string param_string, SdfSBSHeader *sbs_header = nullptr, COctreeV3Settings *coctree_header = nullptr);

// Build

  void build_model(std::string render_config_str);
  SparseOctreeSettings get_build_settings(std::string lod);
  GridSettings get_build_settings_grid(std::string lod);

// Render

  MultiRenderPreset createPreset(const std::string &render_mode, const int spp);
  int getDevice(const std::string backend);
  int getFileSize(std::string file_name);
  void calcMetrics(float &min, float &max, float &average, const float &new_val);

  void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender,
              float3 pos, float3 target, float3 up,
              MultiRenderPreset preset, int a_passNum = 1);
  void Render(LiteImage::Image2D<uint32_t> &image, IRenderer* pRender, uint32_t width, uint32_t height, const LiteMath::float3 &pos, std::chrono::steady_clock::time_point &t1, std::chrono::steady_clock::time_point &t2);

  void getMetrics(const std::string &render_config_str);

  // void getInfoMesh(const std::string &model, const std::string &backend, const std::string &renderer,
  //                  const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

  // void getInfoGrid(const std::string &model, const std::string &backend, const std::string &renderer,
  //                  const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

  // void getInfoSVS(const std::string &model, const std::string &backend, const std::string &renderer,
  //                 const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

  // void getInfoSBS(const std::string &model, const std::string &backend, const std::string &renderer,
  //                 const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

  // //  very slooooow
  // void getInfoAdaptSBS(const std::string &model, const std::string &backend, const std::string &renderer,
  //                      const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);
};