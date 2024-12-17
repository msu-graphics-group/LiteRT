#pragma once

#include "gltf_structure.h"
#include "gltf_structure_writer.h"
#include "utils/scene.h"

namespace gltf
{
  class GeneralGltfWriter
  {
  public:
    struct Settings
    {
      unsigned max_binary_file_size = 64 * (2 << 20); // 64 Mb
      unsigned max_models = 8192;
      unsigned max_cameras = 16;
      float max_bound = 25000;
      bool debug = true;
      bool calc_exact_bbox = true;
    } settings;

    void convert_to_gltf(const HydraScene &scene, std::string name);

  private:
    bool model_to_gltf(const cmesh4::SimpleMesh &m, FullData &full_data, int id);
    bool camera_to_gltf(const ::Camera &c, FullData &full_data, int id);
    bool write_to_binary_file(const char *data, int size, std::string file_name);
    bool add_to_binary_file(const char *data, int size, BinaryFile &b_file);
  };
}