#ifndef CHIMERA_IMAGE_LOADER_H
#define CHIMERA_IMAGE_LOADER_H

#include <string>
#include <cstdint>
#include <vector>
enum TEX_FORMAT
{
  IMG_IMAGE4UB,
  IMG_IMAGE4F,
  IMG_COMMON_LDR,
  IMG_COMMON_HDR
};

struct ImageFileInfo
{
  bool is_ok = false;
  bool is_normal_map = false;
  int width = 0u;
  int height = 0u;
  int channels = 0u;
  int bytesPerChannel = 0u;
  std::string path;
};

ImageFileInfo getImageInfo(const std::string& a_filename);
std::vector<unsigned char> loadImageLDR(const ImageFileInfo& info);
std::vector<float> loadImageHDR(const ImageFileInfo& info);

#endif// CHIMERA_IMAGE_LOADER_H
