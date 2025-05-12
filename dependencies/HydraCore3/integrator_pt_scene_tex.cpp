#include "imageutils.h"
#include "integrator_pt_scene.h"


//bool LoadHDRImageFromFile(const wchar_t* a_fileName, int* pW, int* pH, std::vector<float>& a_data);

std::shared_ptr<ICombinedImageSampler> MakeWhiteDummy()
{
  constexpr uint32_t WHITE = 0x00FFFFFF;
  std::shared_ptr< Image2D<uint32_t> > pTexture1 = std::make_shared< Image2D<uint32_t> >(1, 1, &WHITE);
  Sampler sampler;
  sampler.filter   = Sampler::Filter::NEAREST; 
  sampler.addressU = Sampler::AddressMode::CLAMP;
  sampler.addressV = Sampler::AddressMode::CLAMP;
  return MakeCombinedTexture2D(pTexture1, sampler);
}


std::shared_ptr<ICombinedImageSampler> LoadTextureAndMakeCombined(const TextureInfo& a_texInfo, const Sampler& a_sampler, bool a_disableGamma)
{
  std::shared_ptr<ICombinedImageSampler> pResult = nullptr;

  if(a_texInfo.path.find(L".bmp") != std::string::npos || a_texInfo.path.find(L".ppm") != std::string::npos || 
     a_texInfo.path.find(L".jpg") != std::string::npos || a_texInfo.path.find(L".jpeg") != std::string::npos ||
     a_texInfo.path.find(L".png") != std::string::npos)
  {
    const std::string fileName = hydra_xml::ws2s(a_texInfo.path);    
    Image2D<uint32_t> image    = LiteImage::LoadImage<uint32_t>(fileName.c_str());
    auto pTexture              = std::make_shared< Image2D<uint32_t> >(std::move(image));
    pTexture->setSRGB(!a_disableGamma);
    pResult                    = MakeCombinedTexture2D(pTexture, a_sampler);
  }
  else if (a_texInfo.path.find(L".exr") != std::string::npos)
  {
    int        wh[2]        = { 0, 0 };
    const auto fileName     = hydra_xml::ws2s(a_texInfo.path);

    if (a_texInfo.bpp == 16)
    {
      const auto image_vect = LoadImage4fFromEXR(fileName.c_str(), &wh[0], &wh[1]);
      auto       pTexture   = std::make_shared<Image2D<float4>>(wh[0], wh[1], (const float4*)image_vect.data());
      pTexture->setSRGB(false);
      pResult               = MakeCombinedTexture2D(pTexture, a_sampler);
    }
    else
    {
      const auto image_vect = LoadImage1fFromEXR(fileName.c_str(), &wh[0], &wh[1]);
      auto       pTexture   = std::make_shared<Image2D<float>>(wh[0], wh[1], (const float*)image_vect.data());
      pTexture->setSRGB(false);
      pResult               = MakeCombinedTexture2D(pTexture, a_sampler);
    }
  }
  else if(a_texInfo.path.find(L".image") != std::string::npos) // hydra image formats: image4f, image4ub
  {
    int wh[2] = {0,0};
    std::string fnameA(a_texInfo.path.begin(), a_texInfo.path.end());

    #ifdef WIN32
    std::ifstream fin(a_texInfo.path.c_str(), std::ios::binary);
    #else
    std::ifstream fin(fnameA.c_str(), std::ios::binary);
    if(!fin.is_open())
      std::cout << "[LoadTextureAndMakeCombined]: can't open '" << fnameA << "'" << std::endl;
    #endif
    
    fin.read((char*)wh, sizeof(int)*2);
    if(wh[0] == 0 || wh[1] == 0)
    {
      //std::cout << "[LoadTextureAndMakeCombined]: can't read texture from file '" << fnameA.c_str() << "'; use white dummy;" << std::endl;
      float4 data[1] = {float4(1.0f, 1.0f, 1.0f, 1.0f)};
      auto pTexture  = std::make_shared< Image2D<float4> >(1, 1, data);
      pTexture->setSRGB(false);
      pResult        = MakeCombinedTexture2D(pTexture, a_sampler);
    }
    else if(a_texInfo.bpp == 16) // image4f
    {
      std::vector<float> data(wh[0]*wh[1]*4);
      fin.read((char*)data.data(), sizeof(float)*4*data.size());
      fin.close();
  
      auto pTexture = std::make_shared< Image2D<float4> >(wh[0], wh[1], (const float4*)data.data());
      pResult       = MakeCombinedTexture2D(pTexture, a_sampler);
    }
    else                        // image4ub
    {
      std::vector<uint32_t> data(wh[0]*wh[1]);
      fin.read((char*)data.data(), sizeof(uint32_t)*data.size());
      fin.close();
  
      auto pTexture = std::make_shared< Image2D<uint32_t> >(wh[0], wh[1], data.data());
      pTexture->setSRGB(!a_disableGamma);
      pResult       = MakeCombinedTexture2D(pTexture, a_sampler);
    }
  }

  //if(pResult->width() <= 1 || pResult->height() <= 1) 
  //{
  //  const std::string fileName = hydra_xml::ws2s(a_texInfo.path);  
  //  std::cout << "[LoadTextureAndMakeCombined]: ALERT! texture at path '" << fileName.c_str() << "' is not found, file does not exists!" << std::endl;
  //}
 
  return pResult;
}

std::pair<HydraSampler, uint32_t> LoadTextureFromNode(const pugi::xml_node& node, const std::vector<TextureInfo> &texturesInfo,
                                                      std::unordered_map<HydraSampler, uint32_t, HydraSamplerHash> &texCache, 
                                                      std::vector< std::shared_ptr<ICombinedImageSampler> > &textures)
{
  HydraSampler sampler = ReadSamplerFromColorNode(node);
  auto p = texCache.find(sampler);
  uint32_t texId = 0;
  if(p == texCache.end())
  {
    texCache[sampler] = uint(textures.size());
    auto texNode = node.child(L"texture");
    texId  = texNode.attribute(L"id").as_uint();

    bool disableGamma = false;
    if(texNode.attribute(L"input_gamma").as_int() == 1)
      disableGamma = true;

    textures.push_back(LoadTextureAndMakeCombined(texturesInfo[texId], sampler.sampler, disableGamma));
    p = texCache.find(sampler);
  }

  return {sampler, p->second};
}

std::pair<HydraSampler, uint32_t> LoadTextureById(uint32_t texId, const std::vector<TextureInfo> &texturesInfo, const HydraSampler& sampler,
                                                  std::unordered_map<HydraSampler, uint32_t, HydraSamplerHash> &texCache, 
                                                  std::vector< std::shared_ptr<ICombinedImageSampler> > &textures)
{
  auto p = texCache.find(sampler);
  if(p == texCache.end())
  {
    texCache[sampler] = uint(textures.size());
    bool disableGamma = true;

    textures.push_back(LoadTextureAndMakeCombined(texturesInfo[texId], sampler.sampler, disableGamma));
    p = texCache.find(sampler);
  }

  return {sampler, p->second};
}
