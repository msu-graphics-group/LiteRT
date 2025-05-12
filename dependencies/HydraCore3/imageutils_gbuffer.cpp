#include "imageutils.h"
#include "include/cglobals.h"
#include "Image2d.h"

#include "integrator_pt.h"

#ifdef USE_STB_IMAGE
  #define SaveLDRImageM SaveImage4fByExtension
#else
  #define SaveLDRImageM SaveImage4fToBMP
#endif

void SaveGBufferImages(const std::string& imageOutClean, const std::string& imageOutFiExt, 
                       const std::vector<Integrator::GBufferPixel>& gbuffer, std::vector<float>& tmp, uint width, uint height)
{  
  // normal
  //
  float dmin = 1e38f;
  float dmax = 0.0f;

  std::fill(tmp.begin(), tmp.end(), 0.0f);
  for(size_t i=0;i<gbuffer.size();i++)
  {
    tmp[i*4+0] = std::abs(gbuffer[i].norm[0]);
    tmp[i*4+1] = std::abs(gbuffer[i].norm[1]);
    tmp[i*4+2] = std::abs(gbuffer[i].norm[2]);
    tmp[i*4+3] = 0.0f; 

    dmin = std::min(dmin, gbuffer[i].depth);
    dmax = std::max(dmax, gbuffer[i].depth);
  }

  auto nameColorNorm = imageOutClean + "2" + "." + imageOutFiExt; 
  SaveLDRImageM(tmp.data(), width, height, 4, nameColorNorm.c_str(), 1.0f, 1.0f);

  // diffuse color
  //
  std::fill(tmp.begin(), tmp.end(), 0.0f);
  for(size_t i=0;i<gbuffer.size();i++)
  {
    tmp[i*4+0] = gbuffer[i].rgba[0];
    tmp[i*4+1] = gbuffer[i].rgba[1];
    tmp[i*4+2] = gbuffer[i].rgba[2];
    tmp[i*4+3] = gbuffer[i].rgba[3]; 
  }
  
  auto nameColorTex = imageOutClean + "3" + "." + imageOutFiExt; 
  SaveLDRImageM(tmp.data(), width, height, 4, nameColorTex.c_str(), 1.0f, 2.4f);

  // depth buffer
  std::fill(tmp.begin(), tmp.end(), 0.0f);
  for(size_t i=0;i<gbuffer.size();i++)
  {
    const float d = (gbuffer[i].depth - dmin) / (dmax - dmin);

    //int r = (int)((1.0f-d)*255.0f);
    //if(r > 255)
    //  r = 255;
    //else if (r < 1)
    //  r = 1;
    //if (d > 1e5f || gbuffer[i].matId < 0)
    //  r = 0;

   tmp[i*4+0] = float(d);
   tmp[i*4+1] = float(d);
   tmp[i*4+2] = float(d);
   tmp[i*4+3] = float(d); 
  }

  auto nameColorDepth = imageOutClean + "4" + "." + imageOutFiExt; 
  SaveLDRImageM(tmp.data(), width, height, 4, nameColorDepth.c_str(), 1.0f, 1.0f);

  //Next: matId, onjId, instId

  LiteImage::Image2D<uint32_t> tempImage(width, height);
  uint32_t* tempDataRef = tempImage.data();

  const unsigned int palette[20] = { 0xffe6194b, 0xff3cb44b, 0xffffe119, 0xff0082c8,
                                     0xfff58231, 0xff911eb4, 0xff46f0f0, 0xfff032e6,
                                     0xffd2f53c, 0xfffabebe, 0xff008080, 0xffe6beff,
                                     0xffaa6e28, 0xfffffac8, 0xff800000, 0xffaaffc3,
                                     0xff808000, 0xffffd8b1, 0xff000080, 0xff808080 };

  const unsigned paletteSize = 20;
  
  // mat id
  //
  for(int y=0;y<int(height);y++)
  for(int x=0;x<int(width); x++)
  //for(size_t i=0;i<gbuffer.size();i++)
  {
    int i1 = y*width+x;
    int i2 = (height-y-1)*width+x;
    int index = gbuffer[i2].matId;
    if (index < 0)
      index = 0xff000000;
    else
      index = palette[index % paletteSize];
    tempDataRef[i1] = uint32_t(index);
  }

  auto nameColorMatId = imageOutClean + "5" + "." + imageOutFiExt; 
  LiteImage::SaveImage(nameColorMatId.c_str(), tempImage, 1.0f);

  // obj id
  //
  for(int y=0;y<int(height);y++)
  for(int x=0;x<int(width); x++)
  //for(size_t i=0;i<gbuffer.size();i++)
  {
    int i1 = y*width+x;
    int i2 = (height-y-1)*width+x;
    int index = gbuffer[i2].objId;
    if (index < 0)
      index = 0xff000000;
    else
      index = palette[index % paletteSize];
    tempDataRef[i1] = uint32_t(index);
  }

  auto nameColorObjId = imageOutClean + "7" + "." + imageOutFiExt; 
  LiteImage::SaveImage(nameColorObjId.c_str(), tempImage, 1.0f);

  // obj id
  //
  for(int y=0;y<int(height);y++)
  for(int x=0;x<int(width); x++)
  //for(size_t i=0;i<gbuffer.size();i++)
  {
    int i1 = y*width+x;
    int i2 = (height-y-1)*width+x;
    int index = gbuffer[i2].instId;
    if (index < 0)
      index = 0xff000000;
    else
      index = palette[index % paletteSize];
    tempDataRef[i1] = uint32_t(index);
  }

  auto nameColorInstId = imageOutClean + "6" + "." + imageOutFiExt; 
  LiteImage::SaveImage(nameColorInstId.c_str(), tempImage, 1.0f);

}