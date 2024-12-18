#include "scene.h"
#include "LiteScene/hydraxml.h"
#include "HydraCore3/integrator_pt_scene.h"

static const std::wstring hydraOldMatTypeStr       {L"hydra_material"};
static const std::wstring hydraGLTFTypeStr         {L"gltf"};
static const std::wstring roughConductorMatTypeStr {L"rough_conductor"};
static const std::wstring thinFilmMatTypeStr       {L"thin_film"};
static const std::wstring simpleDiffuseMatTypeStr  {L"diffuse"};
static const std::wstring blendMatTypeStr          {L"blend"};
static const std::wstring plasticMatTypeStr        {L"plastic"};
static const std::wstring dielectricMatTypeStr     {L"dielectric"};

void save_hydra_scene_xml(const std::string &path, const HydraScene &hydra_scene)
{

}

void load_hydra_scene_xml(const std::string &path, HydraScene &hydra_scene)
{ 
  int m_spectral_mode = 0;

  std::vector<int>              m_remapInst;
  std::vector<int>              m_allRemapLists;
  std::vector<int>              m_allRemapListsOffsets;
  std::vector<uint32_t>         m_instIdToLightInstId;
  std::vector<float>            m_pdfLightData;

  std::string scenePathStr = path;
  std::string sceneDirStr = "";  
  hydra_xml::HydraScene scene;

  auto loadRes = scene.LoadState(scenePathStr, sceneDirStr);
  if (loadRes != 0)
  {
    printf("load_hydra_scene_xml: loading %s failed\n", scenePathStr.c_str());
    return;
  }

  std::vector<TextureInfo> texturesInfo;
  texturesInfo.resize(0);
  texturesInfo.reserve(100);

  #ifdef WIN32
  size_t endPos = scenePathStr.find_last_of("\\");
  if(endPos == std::string::npos)
    endPos = scenePathStr.find_last_of("/");
  #else
  size_t endPos = scenePathStr.find_last_of('/');
  #endif

  const std::string sceneFolder = (sceneDirStr == "") ? scenePathStr.substr(0, endPos) : sceneDirStr;

  //// (0) load textures info
  //
  for(auto texNode : scene.TextureNodes())
  {
    TextureInfo tex;

    if (texNode.attribute(L"loc").empty())
      tex.path = std::wstring(texNode.attribute(L"path").as_string());
    else
      tex.path = std::wstring(sceneFolder.begin(), sceneFolder.end()) + L"/" + texNode.attribute(L"loc").as_string();
    
    tex.width  = texNode.attribute(L"width").as_uint();
    tex.height = texNode.attribute(L"height").as_uint();
    if(tex.width != 0 && tex.height != 0)
    {
      const size_t byteSize = texNode.attribute(L"bytesize").as_ullong();
      tex.bpp = uint32_t(byteSize / size_t(tex.width*tex.height));
      texturesInfo.push_back(tex);
    }
    
  }

  std::unordered_map<HydraSampler, uint32_t, HydraSamplerHash> texCache;
  texCache[HydraSampler()] = 0; // zero white texture
  
  hydra_scene.textures.resize(0);
  hydra_scene.textures.reserve(256);
  hydra_scene.textures.push_back(MakeWhiteDummy());

  // (1) load lights
  //
  std::vector<uint32_t> oldLightIdToNewLightId(scene.GetInstancesNum(), uint32_t(-1));

  m_instIdToLightInstId.resize(scene.GetInstancesNum(), -1);
  
  uint32_t oldLightId = 0;
  for(auto lightInst : scene.InstancesLights())
  {
    auto lightSource = LoadLightSourceFromNode(lightInst, sceneFolder, m_spectral_mode, texturesInfo, texCache, hydra_scene.textures);                                

    bool addToLightSources = true;             // don't sample LDR, perez or simple colored env lights
    if(lightSource.geomType == LIGHT_GEOM_ENV) // just account for them in implicit strategy
    {
      float4x4 transformTexCoord;   
      transformTexCoord.set_row(0, lightSource.samplerRow0);
      transformTexCoord.set_row(1, lightSource.samplerRow1);
      
      float4x4 transformTexCoordInv = inverse4x4(transformTexCoord);
      lightSource.samplerRow0Inv = transformTexCoordInv.get_row(0);
      lightSource.samplerRow1Inv = transformTexCoordInv.get_row(1);
      
      if(lightSource.texId != uint(-1))
      {
        auto info         = texturesInfo[lightSource.texId];
        addToLightSources = (info.path.find(L".exr") != std::wstring::npos) || (info.bpp > 4);
  
        if(addToLightSources) // add appropriate pdf table to table data
        {
          const auto pTex = hydra_scene.textures[lightSource.texId];
          int tableW = 0, tableH = 0;
          std::vector<float> pdfImage = PdfTableFromImage(pTex, &tableW, &tableH);
          lightSource.pdfTableOffset  = uint32_t(m_pdfLightData.size());
          lightSource.pdfTableSize    = uint32_t(pdfImage.size());
          lightSource.pdfTableSizeX   = tableW;
          lightSource.pdfTableSizeY   = tableH;
          m_pdfLightData.insert(m_pdfLightData.end(), pdfImage.begin(), pdfImage.end());
        }
      }
      else
        addToLightSources = false;
    }
    
    if(addToLightSources)
      oldLightIdToNewLightId[oldLightId] = uint32_t(hydra_scene.lights.size());
    oldLightId++;

    if(addToLightSources)
      hydra_scene.lights.push_back(lightSource);
  }

  //// (2) load materials
  //
  hydra_scene.materials.resize(0);
  hydra_scene.materials.reserve(100);

  std::set<uint32_t> loadedSpectralTextures = {};
  for(auto materialNode : scene.MaterialNodes())
  {
    Material mat = {};
    auto mat_type = materialNode.attribute(L"type").as_string();
    
    mat.data[EMISSION_MULT] = 1.0f;

    if(mat_type == hydraOldMatTypeStr)
    {
      mat = ConvertOldHydraMaterial(materialNode, texturesInfo, texCache, hydra_scene.textures, m_spectral_mode);
    }
    else if(mat_type == hydraGLTFTypeStr)
    {
      mat = ConvertGLTFMaterial(materialNode, texturesInfo, texCache, hydra_scene.textures, m_spectral_mode);
    }
    else if(mat_type == roughConductorMatTypeStr)
    {
      mat = LoadRoughConductorMaterial(materialNode, texturesInfo, texCache, hydra_scene.textures, m_spectral_mode);
    }
    else if (mat_type == thinFilmMatTypeStr)
    {
      printf("HydraScene: thin film material not supported\n");  
    }
    else if(mat_type == simpleDiffuseMatTypeStr)
    {
      printf("HydraScene: spectral diffuse material not supported\n"); 
    }
    else if(mat_type == blendMatTypeStr)
    {
      printf("HydraScene: spectral blend material not supported\n"); 
    }
    else if(mat_type == plasticMatTypeStr)
    {
      printf("HydraScene: spectral plastic material not supported\n"); 
    }
    else if(mat_type == dielectricMatTypeStr)
    {
      mat = LoadDielectricMaterial(materialNode, texturesInfo, texCache, hydra_scene.textures, m_spectral_mode);
    }

    if(materialNode.attribute(L"light_id") != nullptr)
    {
      int lightId = materialNode.attribute(L"light_id").as_int();
      if(lightId >= 0 && lightId < static_cast<int>(hydra_scene.lights.size()))
      {
        auto tmp = mat.colors[EMISSION_COLOR] != hydra_scene.lights[lightId].intensity;
        if(tmp.x == 0xFFFFFFFF && tmp.y == 0xFFFFFFFF && tmp.z == 0xFFFFFFFF && tmp.w == 0xFFFFFFFF)
          std::cout << "Color in material for light geom and color in light intensity node are different! " 
                    << "Using values from light intensity node. lightId = " << lightId << std::endl;

        mat.colors[EMISSION_COLOR] = hydra_scene.lights[lightId].intensity;

        if(mat.data[EMISSION_MULT] != hydra_scene.lights[lightId].mult)
          std::cout << "Color multiplier in material for light geom and in light intensity node are different! " 
                    << "Using values from light intensity node. lightId = " << lightId << std::endl;

        mat.data[EMISSION_MULT] = hydra_scene.lights[lightId].mult;

        if(mat.spdid[0] != hydra_scene.lights[lightId].specId)
          std::cout << "Spectrum in material for light geom and in light intensity node are different! " 
                    << "Using values from light intensity node. lightId = " << lightId << std::endl;
        
        mat.spdid[0] = hydra_scene.lights[lightId].specId;
        hydra_scene.lights[lightId].matId = uint(hydra_scene.materials.size());
      }
    }

    // setup normal map
    //
    mat.texid[1] = 0xFFFFFFFF;

    if(materialNode.child(L"displacement") != nullptr)
    {
      auto dispNode = materialNode.child(L"displacement");
      if(dispNode.attribute(L"type").as_string() != std::wstring(L"normal_bump"))
      {
        std::string bumpType = hydra_xml::ws2s(dispNode.attribute(L"type").as_string());
        std::cout << "[Integrator::LoadScene]: bump type '" << bumpType.c_str() << "' is not supported! only 'normal_bump' is allowed."  << std::endl;
      }
      else
      {
        auto normalNode  = dispNode.child(L"normal_map");
        auto invertNode  = normalNode.child(L"invert");   // todo read swap flag also
        auto textureNode = normalNode.child(L"texture");

        const auto& [sampler, texID] = LoadTextureFromNode(normalNode, texturesInfo, texCache, hydra_scene.textures);

        mat.row0 [1] = sampler.row0;
        mat.row1 [1] = sampler.row1;
        mat.texid[1] = texID;

        const bool invertX = (invertNode.attribute(L"x").as_int() == 1);
        const bool invertY = (invertNode.attribute(L"y").as_int() == 1);
        const bool swapXY  = (invertNode.attribute(L"swap_xy").as_int() == 1);

        if(invertX)
          mat.cflags |= uint(FLAG_NMAP_INVERT_X);
        if(invertY)
          mat.cflags |= uint(FLAG_NMAP_INVERT_Y);
        if(swapXY)
          mat.cflags |= uint(FLAG_NMAP_SWAP_XY);
      }
    }

    hydra_scene.materials.push_back(mat);
  }

  //TODO: do something with cameras
  for(auto cam : scene.Cameras())
  {

  }

  //// (2) load meshes
  //
  auto mIter = scene.GeomNodes().begin();
  while (mIter != scene.GeomNodes().end())
  {
    std::string dir = sceneFolder + "/" + hydra_xml::ws2s(std::wstring(mIter->attribute(L"loc").as_string()));
    std::string name = hydra_xml::ws2s(std::wstring(mIter->name()));

    if (name == "mesh")
    {
      #ifdef _DEBUG
      std::cout << "[LoadScene]: mesh = " << dir.c_str() << std::endl;
      #endif
      auto currMesh = cmesh4::LoadMeshFromVSGF(dir.c_str());
      hydra_scene.meshes.emplace_back();
      hydra_scene.meshes.back().mesh = currMesh;
    }
    else
    {
      printf("HydraScene: geometry type \"%s\" is not supported!\n", name.c_str());
    }
    mIter++;
  }

  //// (3) make instances of created meshes
  //
  m_remapInst.clear();    m_remapInst.reserve(1000);
  uint32_t realInstId = 0;
  for(auto inst : scene.InstancesGeom())
  {
    hydra_scene.meshes[inst.geomId].transforms.push_back(inst.matrix);
  }
}
