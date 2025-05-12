#include "hydraxml.h"

#include <iostream>
#include <sstream>
#include <fstream>
#include <locale>
#include <codecvt>

#if defined(__ANDROID__)
#define LOGE(...) \
  ((void)__android_log_print(ANDROID_LOG_ERROR, "HydraXML", __VA_ARGS__))
#endif

namespace hydra_xml
{
  std::wstring s2ws(const std::string& str)
  {
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;
    return converterX.from_bytes(str);
  }

  std::string ws2s(const std::wstring& wstr)
  {
    using convert_typeX = std::codecvt_utf8<wchar_t>;
    std::wstring_convert<convert_typeX, wchar_t> converterX;
    return converterX.to_bytes(wstr);
  }

  void HydraScene::LogError(const std::string &msg)
  {
    std::cout << "HydraScene ERROR: " << msg << std::endl;
  }

#if defined(__ANDROID__)
  int HydraScene::LoadState(AAssetManager* mgr, const std::string& path, const std::string& scnDir)
  {
    AAsset* asset = AAssetManager_open(mgr, path.c_str(), AASSET_MODE_STREAMING);
    if (!asset)
    {
      LOGE("Could not load scene from \"%s\"!", path.c_str());
    }
    assert(asset);

    size_t asset_size = AAsset_getLength(asset);

    assert(asset_size > 0);

    void* data = malloc(asset_size);
    AAsset_read(asset, data, asset_size);
    AAsset_close(asset);

    pugi::xml_document xmlDoc;

    auto loaded = xmlDoc.load_buffer(data, asset_size);

    if(!loaded)
    {
      std::string str(loaded.description());
      LOGE("pugixml error loading scene: %s", str.c_str());

      return -1;
    }

    auto pos = path.find_last_of(L'/');
    m_libraryRootDir = path.substr(0, pos);
    if(scnDir != "")
      m_libraryRootDir = scnDir;
    
    pugi::xml_node root = xmlDoc;
    if(xmlDoc.child(L"root") != nullptr)
      root = xmlDoc.child(L"root");
    
    auto texturesLib  = root.child(L"textures_lib");
    auto materialsLib = root.child(L"materials_lib");
    auto geometryLib  = root.child(L"geometry_lib");
    auto lightsLib    = root.child(L"lights_lib");

    auto cameraLib    = root.child(L"cam_lib");
    auto settingsNode = root.child(L"render_lib");
    auto sceneNode    = root.child(L"scenes");

    if (texturesLib == nullptr || materialsLib == nullptr || lightsLib == nullptr || cameraLib == nullptr ||
        geometryLib == nullptr || settingsNode == nullptr || sceneNode == nullptr)
    {
      std::string errMsg = "Loaded state (" +  path + ") doesn't have one of (textures_lib, materials_lib, lights_lib, cam_lib, geometry_lib, render_lib, scenes";
      LogError(errMsg);
      return -1;
    }

    parseInstancedMeshes(sceneNode, geometryLib);

    return 0;
  }
#else
  int HydraScene::LoadState(const std::string& path, const std::string& scnDir)
  {
    auto loaded = m_xmlDoc.load_file(path.c_str());

    if(!loaded)
    {
      std::string  str(loaded.description());
      std::wstring errorMsg(str.begin(), str.end());

      LogError("Error loading scene from: " + path);
      LogError(ws2s(errorMsg));

      return -1;
    }

    #ifdef WIN32
    size_t pos = path.find_last_of("\\");
    if(pos == std::string::npos)
      pos = path.find_last_of("/");
    #else
    size_t pos = path.find_last_of('/');
    #endif

    m_libraryRootDir = path.substr(0, pos);
    if(scnDir != "")
      m_libraryRootDir = scnDir;

    pugi::xml_node root = m_xmlDoc;
    if(m_xmlDoc.child(L"root") != nullptr)
      root = m_xmlDoc.child(L"root");

    m_texturesLib  = root.child(L"textures_lib");
    m_materialsLib = root.child(L"materials_lib");
    m_geometryLib  = root.child(L"geometry_lib");
    m_lightsLib    = root.child(L"lights_lib");
    m_spectraLib   = root.child(L"spectra_lib");

    m_cameraLib    = root.child(L"cam_lib");
    m_settingsNode = root.child(L"render_lib");
    m_scenesNode   = root.child(L"scenes");

    if (m_texturesLib == nullptr || m_materialsLib == nullptr || m_lightsLib == nullptr || m_cameraLib == nullptr || m_geometryLib == nullptr || m_settingsNode == nullptr || m_scenesNode == nullptr)
    {
      std::string errMsg = "Loaded state (" +  path + ") doesn't have one of (textures_lib, materials_lib, lights_lib, cam_lib, geometry_lib, render_lib, scenes";
      LogError(errMsg);
      return -1;
    }

    parseInstancedMeshes(m_scenesNode, m_geometryLib);

    return 0;
  }
#endif

  void HydraScene::parseInstancedMeshes(pugi::xml_node a_scenelib, pugi::xml_node a_geomlib)
  {
    auto scene = a_scenelib.first_child();
    for (pugi::xml_node inst = scene.first_child(); inst != nullptr; inst = inst.next_sibling())
    {
      if (std::wstring(inst.name()) == L"instance_light")
        continue;

      m_numInstances += 1;

      auto mesh_id = inst.attribute(L"mesh_id").as_string();
      auto matrix = std::wstring(inst.attribute(L"matrix").as_string());

      auto meshNode = a_geomlib.find_child_by_attribute(L"id", mesh_id);

      if(meshNode != nullptr)
      {
        auto meshLoc = ws2s(std::wstring(meshNode.attribute(L"loc").as_string()));
        
        if(meshLoc == std::string("unknown"))
        {
          meshLoc = ws2s(std::wstring(meshNode.attribute(L"path").as_string()));
        }
        else
        {
          meshLoc = m_libraryRootDir + "/" + meshLoc;
        }

#if not defined(__ANDROID__)
        std::ifstream checkMesh(meshLoc);
        if(!checkMesh.good())
        {
          LogError("Mesh not found at: " + meshLoc + ". Loader will skip it.");
          continue;
        }
        else
        {
          checkMesh.close();
        }
#endif

        if(unique_meshes.find(meshLoc) == unique_meshes.end())
        {
          unique_meshes.emplace(meshLoc);
          //m_meshloc.push_back(meshLoc);
        }


        if(m_instancesPerMeshLoc.find(meshLoc) != m_instancesPerMeshLoc.end())
        {
          m_instancesPerMeshLoc[meshLoc].push_back(float4x4FromString(matrix));
        }
        else
        {
          std::vector<LiteMath::float4x4> tmp = { float4x4FromString(matrix) };
          m_instancesPerMeshLoc[meshLoc] = tmp;
        }
      }
    }

    if(scene.attribute(L"bbox"))
    {
      auto bbox_str = scene.attribute(L"bbox").as_string();
      std::wstringstream inputStream(bbox_str);
      
      inputStream >> m_scene_bbox.boxMin.x >> m_scene_bbox.boxMax.x;
      inputStream >> m_scene_bbox.boxMin.y >> m_scene_bbox.boxMax.y;
      inputStream >> m_scene_bbox.boxMin.z >> m_scene_bbox.boxMax.z;
    }
  }

  LiteMath::float4x4 float4x4FromString(const std::wstring &matrix_str)
  {
    LiteMath::float4x4 result;
    std::wstringstream inputStream(matrix_str);
    
    float data[16];
    for(int i=0;i<16;i++)
      inputStream >> data[i];
    
    result.set_row(0, LiteMath::float4(data[0],data[1], data[2], data[3]));
    result.set_row(1, LiteMath::float4(data[4],data[5], data[6], data[7]));
    result.set_row(2, LiteMath::float4(data[8],data[9], data[10], data[11]));
    result.set_row(3, LiteMath::float4(data[12],data[13], data[14], data[15])); 

    return result;
  }

  LiteMath::float3 read3f(pugi::xml_attribute a_attr)
  {
    LiteMath::float3 res(0, 0, 0);
    const wchar_t* camPosStr = a_attr.as_string();
    if (camPosStr != nullptr)
    {
      std::wstringstream inputStream(camPosStr);
      inputStream >> res.x >> res.y >> res.z;
    }
    return res;
  }

  LiteMath::float3 read3f(pugi::xml_node a_node)
  {
    LiteMath::float3 res(0,0,0);
    const wchar_t* camPosStr = a_node.text().as_string();
    if (camPosStr != nullptr)
    {
      std::wstringstream inputStream(camPosStr);
      inputStream >> res.x >> res.y >> res.z;
    }
    return res;
  }

  void readValuesFromStr(const std::wstring &a_str, std::vector<float> &a_vals)
  {
    float val = 0.0f;
    std::wstringstream ss(a_str);
    while (ss >> val) 
    {
      a_vals.push_back(val);
    }
  }

  std::vector<float> readNf(const pugi::xml_node &a_node)
  {
    std::vector<float> res;
    const wchar_t* pStr = a_node.text().as_string();
    if (pStr != nullptr)
    {
      std::wstring str{pStr};
      readValuesFromStr(str, res);
    }
    return res;
  }

  std::vector<float> readNf(const pugi::xml_attribute &a_attr)
  {
    std::vector<float> res;
    const wchar_t* pStr = a_attr.as_string();
    if (pStr != nullptr)
    {
      std::wstring str{pStr};
      readValuesFromStr(str, res);
    }
    return res;
  }

  std::variant<float, float3, float4> readvalVariant(const pugi::xml_node &a_node)
  {
    std::vector<float> values;
    if(a_node.attribute(L"val") != nullptr)
      values = hydra_xml::readNf(a_node.attribute(L"val"));
    else
      values = hydra_xml::readNf(a_node);

    std::variant<float, float3, float4> res;
    if(values.size() == 1)
    {
      res = values[0];
    }
    else if(values.size() == 3)
    {
      res = LiteMath::float3 {values[0], values[1], values[2]};
    }
    else if(values.size() == 4)
    {
      res = LiteMath::float4 {values[0], values[1], values[2], values[3]};
    }
    else
    {
      std::wstring nodeName = a_node.name();
      std::cout << "Node " << ws2s(nodeName) << " contains unexpected number of values: " << values.size();
      return LiteMath::float4 {0.0f};
    }
    
    return res;
  }

  std::vector<uint32_t> readvalVectorU(const pugi::xml_attribute &a_attr)
  {
    std::vector<uint32_t> res;

    std::wstringstream ws(a_attr.as_string());
    uint32_t val = 0xFFFFFFFF;
    while (ws >> val) 
    {
      res.push_back(val);
    }
    return res;
  }

  LiteMath::float3 readval3f(const pugi::xml_node a_node)
  {
    float3 color;
    if(a_node.attribute(L"val") != nullptr)
      color = hydra_xml::read3f(a_node.attribute(L"val"));
    else
      color = hydra_xml::read3f(a_node);
    return color;
  }

  float readval1f(const pugi::xml_node a_color, float default_val) 
  {
    float color = default_val;
    if(!a_color)
    {
      return color;
    }
    if (a_color.attribute(L"val") != nullptr)
      color = a_color.attribute(L"val").as_float();
    else
      color = a_color.text().as_float();        // deprecated
    return color;
  }

  int readval1i(const pugi::xml_node a_color, int default_val)
  {
    int color = default_val;
    if(!a_color)
    {
      return color;
    }
    if (a_color.attribute(L"val") != nullptr)
      color = a_color.attribute(L"val").as_int();
    else
      color = a_color.text().as_int();          // deprecated
    return color;
  }

  unsigned int readval1u(const pugi::xml_node a_color, uint32_t default_val)
  {
    unsigned int color = default_val;
    if(!a_color)
    {
      return color;
    }
    if (a_color.attribute(L"val") != nullptr)
      color = a_color.attribute(L"val").as_uint();
    else
      color = a_color.text().as_uint();          // deprecated
    return color;
  }

  std::vector<LightInstance> HydraScene::InstancesLights(uint32_t a_sceneId) 
  {
    auto sceneNode = m_scenesNode.child(L"scene");
    if(a_sceneId != 0)
    {
      std::wstringstream temp;
      temp << a_sceneId;
      std::wstring tempStr = temp.str();
      sceneNode = m_scenesNode.find_child_by_attribute(L"id", tempStr.c_str());
    }

    std::vector<pugi::xml_node> lights; 
    lights.reserve(256);
    for(auto lightNode : m_lightsLib.children())
      lights.push_back(lightNode);

    std::vector<LightInstance> result;
    result.reserve(256);

    LightInstance inst;
    for(auto instNode = sceneNode.child(L"instance_light"); instNode != nullptr; instNode = instNode.next_sibling())
    {
      std::wstring nameStr = instNode.name();
      if(nameStr != L"instance_light")
        continue;
      inst.instNode  = instNode;
      inst.instId    = instNode.attribute(L"id").as_uint();
      inst.lightId   = instNode.attribute(L"light_id").as_uint(); 
      inst.lightNode = lights[inst.lightId];
      inst.node      = instNode;
      inst.matrix    = float4x4FromString(instNode.attribute(L"matrix").as_string());
      result.push_back(inst);
    }
    return result;
  }

}

