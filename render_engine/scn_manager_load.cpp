#include "scene_mgr.h"
#include "vk_utils.h"


bool SceneManager::LoadSceneXML(const std::string &scenePath, bool transpose)
{
  auto hscene_main = std::make_shared<hydra_xml::HydraScene>();
  auto res         = hscene_main->LoadState(scenePath);

  if(res < 0)
  {
    RUN_TIME_ERROR("LoadSceneXML error");
    return false;
  }

  m_pMeshData = std::make_shared<Mesh8F>();

  uint32_t maxVertexCountPerMesh    = 0u;
  uint32_t maxPrimitiveCountPerMesh = 0u;
  uint32_t totalPrimitiveCount      = 0u;
  uint32_t totalVerticesCount       = 0u;
  uint32_t totalMeshes              = 0u;
  if(m_config.load_geometry)
  {
    for(auto mesh_node : hscene_main->GeomNodes())
    {
      uint32_t vertNum = mesh_node.attribute(L"vertNum").as_int();
      uint32_t primNum = mesh_node.attribute(L"triNum").as_int();
      maxVertexCountPerMesh    = std::max(vertNum, maxVertexCountPerMesh);
      maxPrimitiveCountPerMesh = std::max(primNum, maxPrimitiveCountPerMesh);
      totalVerticesCount      += vertNum;
      totalPrimitiveCount     += primNum;
      totalMeshes++;
    }

    InitGeoBuffersGPU(totalMeshes, totalVerticesCount, totalPrimitiveCount * 3, maxPrimitiveCountPerMesh);
    if(m_config.build_acc_structs)
    {
      m_pBuilderV2->Init(maxVertexCountPerMesh, maxPrimitiveCountPerMesh, totalPrimitiveCount, m_pMeshData->SingleVertexSize(),
        m_config.build_acc_structs_while_loading_scene);
    }

    for(auto loc : hscene_main->MeshFiles())
    {
      auto meshId = AddMeshFromFile(loc);

      if(m_config.debug_output)
        std::cout << "Loading mesh # " << meshId << std::endl;

      LoadOneMeshOnGPU(meshId);
      if(m_config.build_acc_structs)
      {
        VkDeviceOrHostAddressConstKHR vertexBufferDeviceAddress{};
        VkDeviceOrHostAddressConstKHR indexBufferDeviceAddress{};

        vertexBufferDeviceAddress.deviceAddress = vk_rt_utils::getBufferDeviceAddress(m_device, m_geoVertBuf);
        indexBufferDeviceAddress.deviceAddress  = vk_rt_utils::getBufferDeviceAddress(m_device, m_geoIdxBuf);

        m_meshInfos[meshId].blasId = m_pBuilderV2->AddBLAS(m_meshInfos[meshId], m_pMeshData->SingleVertexSize(), vertexBufferDeviceAddress, indexBufferDeviceAddress);
      }

      auto instances = hscene_main->GetAllInstancesOfMeshLoc(loc);
      for(size_t j = 0; j < instances.size(); ++j)
      {
        if(transpose)
          InstanceMesh(meshId, LiteMath::transpose(instances[j]));
        else
          InstanceMesh(meshId, instances[j]);
      }
    }
  }

  for(auto cam : hscene_main->Cameras())
  {
    m_sceneCameras.push_back(cam);
  }

  if(m_config.load_geometry)
  {
    LoadCommonGeoDataOnGPU();
  }

  if(m_config.instance_matrix_as_vertex_attribute)
  {
    LoadInstanceDataOnGPU();
  }

  hscene_main = nullptr;

  return true;
}