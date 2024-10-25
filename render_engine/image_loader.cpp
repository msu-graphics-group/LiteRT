#include "image_loader.h"
#include <string>
#include <fstream>
#include <iostream>
#include <vector>
#include <cstring>

#include "stb_image.h"
#include "stb_image_write.h"

TEX_FORMAT guessFormatFromExtension(const std::string &filename)
{
  auto found = filename.find_last_of('.');
  if(found == std::string::npos)
  {
    std::cerr << "Image file with no extension: " << filename << "Assuming LDR format\n";
    return IMG_COMMON_LDR;
  }

  std::string ext = filename.substr(found, filename.size());

  if(ext == ".image4ub")
    return IMG_IMAGE4UB;
  else if(ext == ".image4f")
    return IMG_IMAGE4F;
  else if(ext == ".exr" || ext == ".hdr")
    return IMG_COMMON_HDR;
  else
    return IMG_COMMON_LDR;
}

ImageFileInfo getImageInfo(const std::string& a_filename)
{
  ImageFileInfo res = {};
  res.path = a_filename;
  res.is_ok = false;

  std::ifstream file(a_filename, std::ios::binary);
  if(!file.good())
  {
    file.close();
    return res;
  }

  auto tex_format = guessFormatFromExtension(a_filename);

  if (tex_format == IMG_IMAGE4UB || tex_format == IMG_IMAGE4F)
  {
    file.read((char*)&res.width,  sizeof(uint32_t));
    file.read((char*)&res.height, sizeof(uint32_t));
    res.channels = 4;

    if(tex_format == IMG_IMAGE4UB)
      res.bytesPerChannel = sizeof(unsigned char);
    else
      res.bytesPerChannel = sizeof(float);

    if(res.width >= 0 && res.height >= 0)
      res.is_ok = true;
  }
  else if (tex_format == IMG_COMMON_LDR)
  {
    stbi_info(a_filename.c_str(), &res.width, &res.height, &res.channels);
    res.bytesPerChannel = sizeof(unsigned char);

    if(res.width > 0 && res.height > 0 && res.channels > 0)
      res.is_ok = true;
  }
  else if (tex_format == IMG_COMMON_HDR && stbi_is_hdr(a_filename.c_str()))
  {
    //@TODO: test this
    stbi_info(a_filename.c_str(), &res.width, &res.height, &res.channels);
    res.bytesPerChannel = sizeof(float);

    if(res.width > 0 && res.height > 0 && res.channels > 0)
      res.is_ok = true;
  }
  file.close();

  return res;
}

std::vector<unsigned char> loadImage4ub(const std::string &filename)
{
  std::ifstream infile(filename, std::ios::binary);
  std::vector<unsigned char> result;
  if (infile.good())
  {
    int32_t w, h;
    infile.read((char*)&w, sizeof(int));
    infile.read((char*)&h, sizeof(int));

    result.resize(w * h * 4);
    infile.read((char*)result.data(), w * h * 4);
  }
  return result;
}

std::vector<unsigned char> loadImageLDR(const ImageFileInfo& info)
{
  auto tex_format = guessFormatFromExtension(info.path);
  if (tex_format == IMG_IMAGE4UB)
  {
    return loadImage4ub(info.path);
  }
  else
  {
    int w, h, channels, req_channels;
    if(info.channels == 3)
      req_channels = 4;
    else
      req_channels = info.channels;

    unsigned char *pixels = stbi_load(info.path.c_str(), &w, &h, &channels, req_channels);

    std::vector<unsigned char> result(w * h * req_channels);
    memcpy(result.data(), pixels, result.size());

    stbi_image_free(pixels);

    return result;
  }
}

std::vector<float> loadImageHDR(const ImageFileInfo& info)
{
  int w, h, channels, req_channels;
  if(info.channels == 3)
    req_channels = 4;
  else
    req_channels = info.channels;

  float* pixels = stbi_loadf(info.path.c_str(), &w, &h, &channels, req_channels);

  std::vector<float> result(w * h * req_channels);
  memcpy(result.data(), pixels, result.size());

  stbi_image_free(pixels);

  return result;
}
