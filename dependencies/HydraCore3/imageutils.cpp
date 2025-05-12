#include "imageutils.h"
#include "include/cglobals.h"
#include "Image2d.h"

#define TINYEXR_IMPLEMENTATION
#include "tinyexr.h"


bool SaveImage4fToEXR(const float* rgb, int width, int height, const char* outfilename, float a_normConst = 1.0f, bool a_invertY = false) 
{
  EXRHeader header;
  InitEXRHeader(&header);

  EXRImage image;
  InitEXRImage(&image);
  image.num_channels = 3;

  std::vector<float> images[3];
  images[0].resize(width * height);
  images[1].resize(width * height);
  images[2].resize(width * height);

  // Split RGBARGBARGBA... into R, G and B layer
  if(a_invertY) {
    for(int y=0;y<height;y++) {
      const int offsetY1 = y*width*4;
      const int offsetY2 = (height-y-1)*width*4;
      for(int x=0;x<width;x++) {
        images[0][(offsetY1 >> 2) + x] = rgb[offsetY2 + x*4 + 0]*a_normConst;
        images[1][(offsetY1 >> 2) + x] = rgb[offsetY2 + x*4 + 1]*a_normConst;
        images[2][(offsetY1 >> 2) + x] = rgb[offsetY2 + x*4 + 2]*a_normConst; 
      }
    }   
  }
  else {
    for (size_t i = 0; i < size_t(width * height); i++) {
      images[0][i] = rgb[4*i+0]*a_normConst;
      images[1][i] = rgb[4*i+1]*a_normConst;
      images[2][i] = rgb[4*i+2]*a_normConst;
    }
  }

  float* image_ptr[3];
  image_ptr[0] = images[2].data(); // B
  image_ptr[1] = images[1].data(); // G
  image_ptr[2] = images[0].data(); // R

  image.images = (unsigned char**)image_ptr;
  image.width  = width;
  image.height = height;
  header.num_channels = 3;
  header.channels     = (EXRChannelInfo *)malloc(sizeof(EXRChannelInfo) * header.num_channels);
  // Must be (A)BGR order, since most of EXR viewers expect this channel order.
  strncpy(header.channels[0].name, "B", 255); header.channels[0].name[strlen("B")] = '\0';
  strncpy(header.channels[1].name, "G", 255); header.channels[1].name[strlen("G")] = '\0';
  strncpy(header.channels[2].name, "R", 255); header.channels[2].name[strlen("R")] = '\0';

  header.pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
  header.requested_pixel_types = (int *)malloc(sizeof(int) * header.num_channels);
  for (int i = 0; i < header.num_channels; i++) {
    header.pixel_types[i]           = TINYEXR_PIXELTYPE_FLOAT; // pixel type of input image
    header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of output image to be stored in .EXR
  }
 
  const char* err = nullptr; 
  int ret = SaveEXRImageToFile(&image, &header, outfilename, &err);
  if (ret != TINYEXR_SUCCESS) {
    fprintf(stderr, "Save EXR err: %s\n", err);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    return false;
  }
  //printf("Saved exr file. [%s] \n", outfilename);

  free(header.channels);
  free(header.pixel_types);
  free(header.requested_pixel_types);

  return true;
}


bool SaveImage3DToEXR(const float* data, int width, int height, int channels, const char* outfilename) 
{
  EXRHeader header;
  InitEXRHeader(&header);

  EXRImage image;
  InitEXRImage(&image);

  std::vector<const float*> image_ptr(channels);
  for(int c=0;c<channels;c++)
    image_ptr[c] = data + c*width*height;

  std::vector<EXRChannelInfo> channelsVec(channels);
  std::vector<int>            auxIntData(2*channels);

  image.images       = (unsigned char**)image_ptr.data();
  image.width        = width;
  image.height       = height;
  image.num_channels = channels;

  
  header.num_channels          = channels;
  header.channels              = channelsVec.data();
  header.pixel_types           = auxIntData.data();
  header.requested_pixel_types = auxIntData.data() + channels;

  constexpr float IMG_LAMBDA_MIN = 360.0f;
  constexpr float IMG_LAMBDA_MAX = 830.0f;

  if(channels == 1)
  {
    std::string name = "Y";
    memset (header.channels[0].name, 0, 256);
    strncpy(header.channels[0].name, name.c_str(), 255);
    header.pixel_types[0]           = TINYEXR_PIXELTYPE_FLOAT; 
    header.requested_pixel_types[0] = TINYEXR_PIXELTYPE_FLOAT; 
  }
  else
  {
    for (int i = 0; i < channels; i++) 
    {
      const float t0     = float(i)/float(channels);
      const float t1     = float(i+1)/float(channels);
      const float lamdba0 = IMG_LAMBDA_MIN + t0*(IMG_LAMBDA_MAX - IMG_LAMBDA_MIN);
      const float lamdba1 = IMG_LAMBDA_MIN + t1*(IMG_LAMBDA_MAX - IMG_LAMBDA_MIN);
      std::stringstream strout;
      strout << int(lamdba0) << "-" << int(lamdba1)-1 << ".Y";
      std::string tmp = strout.str();
      //std::cout << tmp.c_str() << std::endl;
      memset (header.channels[i].name, 0, 256);
      strncpy(header.channels[i].name, tmp.c_str(), 255);
      header.pixel_types[i]           = TINYEXR_PIXELTYPE_FLOAT; // pixel type of input image
      header.requested_pixel_types[i] = TINYEXR_PIXELTYPE_FLOAT; // pixel type of output image to be stored in .EXR
    }
  }
 
  const char* err = nullptr; 
  int ret = SaveEXRImageToFile(&image, &header, outfilename, &err);
  if (ret != TINYEXR_SUCCESS) {
    fprintf(stderr, "Save EXR err: %s\n", err);
    FreeEXRErrorMessage(err); // free's buffer for an error message
    return false;
  }
  //printf("Saved exr file. [%s] \n", outfilename);

  return true;
}


void SaveImage3DToImage3D1f(const float* data, int width, int height, int channels, const char* outfilename) 
{ 
  std::ofstream fout(outfilename, std::ios::binary);
  int xyz[3] = {width, height, channels};
  fout.write((const char*)xyz, sizeof(int)*3);
  fout.write((const char*)data, sizeof(float)*size_t(width*height*channels));
  fout.close();
}


void FlipYAndNormalizeImage2D1f(float* data, int width, int height, float a_normConst = 1.0f)
{  
  const int halfHeight = height / 2;
  for (int y = 0; y < halfHeight; ++y) {
    const int offsetY1 = y * width;
    const int offsetY2 = (height - y - 1) * width;
    for (int x = 0; x < width; ++x) {
      const float tmp    = a_normConst*data[offsetY1 + x];
      data[offsetY1 + x] = a_normConst*data[offsetY2 + x];
      data[offsetY2 + x] = tmp;
    }
  }
}


void SaveFrameBufferToEXR(float* data, int width, int height, int channels, const char* outfilename, float a_normConst)
{
  if(channels == 4)
    SaveImage4fToEXR(data, width, height, outfilename, a_normConst, true);
  else
  {
    for(int c=0;c<channels;c++)
      FlipYAndNormalizeImage2D1f(data + width*height*c, width, height, a_normConst*float(channels));  

    if(std::string(outfilename).find(".image3d1f") != std::string::npos) 
      SaveImage3DToImage3D1f(data, width, height, channels, outfilename);
    else 
      SaveImage3DToEXR(data, width, height, channels, outfilename);
  }
}


inline float linearToSRGB(float l)
{
  if(l <= 0.00313066844250063f)
    return l * 12.92f;
  else
    return 1.055f * std::pow(l, 1.0f/2.4f) - 0.055f;
}

void FrameBufferColorToLDRImage(const float* rgb, int width, int height, float a_normConst, float a_gamma, std::vector<uint32_t>& pixelData, bool a_flip)
{
  if(std::abs(a_gamma - 2.4f) < 1e-5f) 
  {
    #pragma omp parallel for if (width*height > 512*512)
    for(int y=0;y<height;y++) // flip image and extract pixel data
    {
      int offset1 = y*width;
      int offset2 = a_flip ? (height-y-1)*width : offset1;
      for (int x = 0; x < width; x++)
      {
        float color[4];
        color[0]     = linearToSRGB(clamp(rgb[4*(offset1+x) + 0]*a_normConst, 0.0f, 1.0f));
        color[1]     = linearToSRGB(clamp(rgb[4*(offset1+x) + 1]*a_normConst, 0.0f, 1.0f));
        color[2]     = linearToSRGB(clamp(rgb[4*(offset1+x) + 2]*a_normConst, 0.0f, 1.0f));
        color[3]     = 1.0f;
        pixelData[offset2 + x] = RealColorToUint32(color);
      }
    }
  }
  else 
  {
    const float invGamma  = 1.0f/a_gamma;
    
    #pragma omp parallel for if (width*height > 512*512)
    for(int y=0;y<height;y++) // flip image and extract pixel data
    {
      int offset1 = y*width;
      int offset2 = a_flip ? (height-y-1)*width : offset1;
      for (int x = 0; x < width; x++)
      {
        float color[4];
        color[0]     = clamp(std::pow(rgb[4*(offset1+x) + 0]*a_normConst, invGamma), 0.0f, 1.0f);
        color[1]     = clamp(std::pow(rgb[4*(offset1+x) + 1]*a_normConst, invGamma), 0.0f, 1.0f);
        color[2]     = clamp(std::pow(rgb[4*(offset1+x) + 2]*a_normConst, invGamma), 0.0f, 1.0f);
        color[3]     = 1.0f;
        pixelData[offset2 + x] = RealColorToUint32(color);
      }
    }
  }
}

void MonoFrameBufferToLDRImage(const float* mono, int width, int height, float a_normConst, float a_gamma, std::vector<uint32_t>& pixelData, bool a_flip)
{
  if(std::abs(a_gamma - 2.4f) < 1e-5f) 
  {
    #pragma omp parallel for if (width*height > 512*512)
    for(int y=0;y<height;y++) // flip image and extract pixel data
    {
      int offset1 = y*width;
      int offset2 = a_flip ? (height-y-1)*width : offset1;
      for (int x = 0; x < width; x++)
      {
        float color[4];
        color[0]     = linearToSRGB(clamp(mono[(offset1+x)]*a_normConst, 0.0f, 1.0f));
        color[1]     = linearToSRGB(clamp(mono[(offset1+x)]*a_normConst, 0.0f, 1.0f));
        color[2]     = linearToSRGB(clamp(mono[(offset1+x)]*a_normConst, 0.0f, 1.0f));
        color[3]     = 1.0f;
        pixelData[offset2 + x] = RealColorToUint32(color);
      }
    }
  }
  else 
  {
    const float invGamma  = 1.0f/a_gamma;
    
    #pragma omp parallel for if (width*height > 512*512)
    for(int y=0;y<height;y++) // flip image and extract pixel data
    {
      int offset1 = y*width;
      int offset2 = a_flip ? (height-y-1)*width : offset1;
      for (int x = 0; x < width; x++)
      {
        float color[4];
        color[0]     = clamp(std::pow(mono[(offset1+x)]*a_normConst, invGamma), 0.0f, 1.0f);
        color[1]     = clamp(std::pow(mono[(offset1+x)]*a_normConst, invGamma), 0.0f, 1.0f);
        color[2]     = clamp(std::pow(mono[(offset1+x)]*a_normConst, invGamma), 0.0f, 1.0f);
        color[3]     = 1.0f;
        pixelData[offset2 + x] = RealColorToUint32(color);
      }
    }
  }
}

std::vector<uint32_t> FrameBufferColorToLDRImage(const float* rgb, int width, int height, float a_normConst, float a_gamma, bool a_flip)
{
  std::vector<uint32_t> pixelData(width*height);
  FrameBufferColorToLDRImage(rgb, width, height, a_normConst, a_gamma, pixelData, a_flip);
  return pixelData;
}

bool SaveImage4fToBMP(const float* rgb, int width, int height, int channels, const char* outfilename, float a_normConst, float a_gamma) 
{
  auto pixelData = FrameBufferColorToLDRImage(rgb, width, height, a_normConst, a_gamma, false);
  LiteImage::SaveBMP(outfilename, pixelData.data(), width, height);
  return true;
}

bool SaveImage4fByExtension(const float* data, int width, int height, int channels, const char* outfilename, float a_normConst, float a_gamma) 
{
  LiteImage::Image2D<uint32_t> tmp(width, height);
  if(channels == 1)
  {
    MonoFrameBufferToLDRImage(data, width, height, a_normConst, a_gamma, const_cast< std::vector<uint32_t>& >(tmp.vector()), true); 
  }
  else
  {
    FrameBufferColorToLDRImage(data, width, height, a_normConst, a_gamma, const_cast< std::vector<uint32_t>& >(tmp.vector()), true); 
  }
  return LiteImage::SaveImage(outfilename, tmp);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

std::vector<float> LoadImage1fFromEXR(const char* infilename, int* pW, int* pH)
{
  float* out; // width * height * RGBA
  int width       = 0;
  int height      = 0;
  const char* err = nullptr;

  int ret = LoadEXR(&out, &width, &height, infilename, &err);
  if (ret != TINYEXR_SUCCESS) {
    if (err) {
      fprintf(stderr, "[LoadImage1fFromEXR] : %s\n", err);
      std::cerr << "[LoadImage1fFromEXR] : " << err;
      std::cerr << " from path : " << infilename << std::endl;
      
      delete err;
    }
    return std::vector<float>();
  }

  const int imgSize = width * height;
  std::vector<float> result(imgSize);
  *pW = uint32_t(width);
  *pH = uint32_t(height);
  
  #pragma omp parallel for
  for(int y = 0; y < height; ++y)
  {
    for (int x = 0; x < width; ++x)
    {
      size_t idx = (x + (height - y - 1) * width) * 4;
      size_t out_idx = x + y * width;
      if (std::isinf(out[idx]))
        result[out_idx] = 65504.0f;                       // max half float according to ieee
      else
        result[out_idx] = clamp(out[idx], 0.0f, 65504.0f); // max half float according to ieee
      
    }
  }

  free(out);
  return result;
}


std::vector<float> LoadImage4fFromEXR(const char* infilename, int* pW, int* pH) 
{
  float* out; // width * height * RGBA
  int width       = 0;
  int height      = 0;
  const char* err = nullptr; 

  int ret = LoadEXR(&out, &width, &height, infilename, &err);
  if (ret != TINYEXR_SUCCESS) {
    if (err) {
      fprintf(stderr, "[LoadImage4fFromEXR] : %s\n", err);
      std::cerr << "[LoadImage4fFromEXR] : " << err << std::endl;
      delete err;
    }
    return std::vector<float>();
  }

  std::vector<float> result(width * height * 4);
  *pW = uint32_t(width);
  *pH = uint32_t(height);

  #pragma omp parallel for
  for(int y = 0; y < height; y++)
  {
    const int offset1 = (height - y - 1) * width * 4;
    const int offset2 = y * width * 4;
    memcpy((void*)(result.data() + offset1), (void*)(out + offset2), width * sizeof(float) * 4);
  }
  free(out);  
  
  return result;
}


float* LoadImage4fFromEXRUnsafe(const char* infilename, int* pW, int* pH)
{
  float* out; // width * height * RGBA
  int width  = 0;
  int height = 0;
  const char* err = nullptr; 

  int ret = LoadEXR(&out, &width, &height, infilename, &err);
  if (ret != TINYEXR_SUCCESS) {
    if (err) {
      fprintf(stderr, "[LoadImage4fFromEXR] : %s\n", err);
      std::cerr << "[LoadImage4fFromEXR] : " << err << std::endl;
      delete err;
    }
    return nullptr;
  }

  *pW = uint32_t(width);
  *pH = uint32_t(height);
  return out;
}

