#include "VulkanRTX.h"
#include "simple_render.h"

// ***************************************************************************************************************************
// setup full screen quad to display ray traced image
void SimpleRender::SetupQuadRenderer()
{
  vk_utils::RenderTargetInfo2D rtargetInfo = {};
  rtargetInfo.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  rtargetInfo.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED;
  rtargetInfo.format = m_swapchain.GetFormat();
  rtargetInfo.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR;
  rtargetInfo.size   = m_swapchain.GetExtent();
  m_pFSQuad.reset();
  m_pFSQuad = std::make_shared<vk_utils::QuadRenderer>(0,0, m_width, m_height);
  m_pFSQuad->Create(m_device, "./shaders/quad3_vert.vert.spv", "./shaders/my_quad.frag.spv", rtargetInfo);
}

void SimpleRender::SetupQuadDescriptors()
{
  m_pBindings->BindBegin(VK_SHADER_STAGE_FRAGMENT_BIT);
  m_pBindings->BindImage(0, m_rtImage.view, m_rtImageSampler, VK_DESCRIPTOR_TYPE_COMBINED_IMAGE_SAMPLER);
  m_pBindings->BindEnd(&m_quadDS, &m_quadDSLayout);
}

void SimpleRender::SetupRTImage()
{
  vk_utils::deleteImg(m_device, &m_rtImage);

  // change format and usage according to your implementation of RT
  m_rtImage.aspectMask = VK_IMAGE_ASPECT_COLOR_BIT;
  createImgAllocAndBind(m_device, m_physicalDevice, m_width, m_height, VK_FORMAT_R8G8B8A8_UNORM,
    VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT | VK_IMAGE_USAGE_SAMPLED_BIT | VK_IMAGE_USAGE_TRANSFER_DST_BIT, &m_rtImage);

  if(m_rtImageSampler == VK_NULL_HANDLE)
  {
    m_rtImageSampler = vk_utils::createSampler(m_device, VK_FILTER_LINEAR, VK_SAMPLER_ADDRESS_MODE_REPEAT, VK_BORDER_COLOR_FLOAT_TRANSPARENT_BLACK);
  }
}
// ***************************************************************************************************************************

// convert geometry data and pass it to acceleration structure builder
void SimpleRender::SetupRTScene()
{
  m_pRayTracerCPU = CreateMultiRenderer(DEVICE_CPU);

  //m_pAccelStruct = std::shared_ptr<ISceneObject>(CreateSceneRT("BVH2Common", "cbvh_embree2", "SuperTreeletAlignedMerged4"));
  m_pRayTracerCPU->GetAccelStruct()->ClearGeom();

  auto meshesData = m_pScnMgr->GetMeshData();
  std::unordered_map<uint32_t, uint32_t> meshMap;
  for(size_t i = 0; i < m_pScnMgr->MeshesNum(); ++i)
  {
    const auto& info = m_pScnMgr->GetMeshInfo(i);
    auto vertices = reinterpret_cast<float*>((char*)meshesData->VertexData() + info.m_vertexOffset * meshesData->SingleVertexSize());
    auto indices = meshesData->IndexData() + info.m_indexOffset;

    auto stride = meshesData->SingleVertexSize() / sizeof(float);
    std::vector<float3> m_vPos3f(info.m_vertNum);
    std::vector<uint32_t> m_indicesReordered(info.m_indNum);
    for(size_t v = 0; v < info.m_vertNum; ++v)
    {
      m_vPos3f[v] = float3(vertices[v * stride + 0], vertices[v * stride + 1], vertices[v * stride + 2]);
    }
    memcpy(m_indicesReordered.data(), indices, info.m_indNum * sizeof(m_indicesReordered[0]));

    auto geomId = m_pRayTracerCPU->GetAccelStruct()->AddGeom_Triangles3f((float*)(m_vPos3f.data()), m_vPos3f.size(),
                                                      m_indicesReordered.data(), m_indicesReordered.size());
    cmesh4::SimpleMesh mesh;
    mesh.vPos4f.resize(m_vPos3f.size());
    mesh.vNorm4f.resize(m_vPos3f.size());
    mesh.vTexCoord2f.resize(m_vPos3f.size());
    mesh.indices.resize(m_indicesReordered.size());
    for(size_t v = 0; v < m_vPos3f.size(); ++v)
    {
      mesh.vPos4f[v] = to_float4(m_vPos3f[v], 1.0f);
      mesh.vNorm4f[v] = float4(1.0, 0.0, 0.0, 0.0f);
      mesh.vTexCoord2f[v] = float2(0.0f, 0.0f);
    }
    for(size_t v = 0; v < m_indicesReordered.size(); ++v)
    {
      mesh.indices[v] = m_indicesReordered[v];
    }

    m_pRayTracerCPU->add_mesh_internal(mesh, geomId);
    meshMap[i] = geomId;
  }

  m_pRayTracerCPU->GetAccelStruct()->ClearScene();
  for(size_t i = 0; i < m_pScnMgr->InstancesNum(); ++i)
  {
    const auto& info = m_pScnMgr->GetInstanceInfo(i);
    if(meshMap.count(info.mesh_id))
    {
      //m_pAccelStruct->AddInstance(meshMap[info.mesh_id], m_pScnMgr->GetInstanceMatrix(info.inst_id));
      m_pRayTracerCPU->AddInstance(meshMap[info.mesh_id], m_pScnMgr->GetInstanceMatrix(info.inst_id));
    }
  }
  m_pRayTracerCPU->GetAccelStruct()->CommitScene();
  auto preset  =getDefaultPreset();
  preset.render_mode = MULTI_RENDER_MODE_PRIMITIVE;
  m_pRayTracerCPU->SetPreset(preset);
  m_pRayTracerCPU->SetViewport(0,0, m_width, m_height);
  m_pRayTracerCPU->CommitDeviceData();
  m_pRayTracerCPU->Clear(m_width, m_height, "color");
}

// perform ray tracing on the CPU and upload resulting image on the GPU
void SimpleRender::RayTraceCPU()
{
  m_pRayTracerCPU->UpdateCamera(m_worldViewMatrix, OpenglToVulkanProjectionMatrixFix() * m_projectionMatrix);
  m_pRayTracerCPU->Render(m_raytracedImageData.data(), m_width, m_height, "color");

  m_pCopyHelper->UpdateImage(m_rtImage.image, m_raytracedImageData.data(), m_width, m_height, 4, VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL);
}

void SimpleRender::RayTraceGPU()
{
//   if(!m_pRayTracerGPU)
//   {
//     m_pRayTracerGPU = std::make_unique<RayTracer_GPU>(m_width, m_height);
//     m_pRayTracerGPU->InitVulkanObjects(m_device, m_physicalDevice, m_width * m_height);
//     m_pRayTracerGPU->InitMemberBuffers();

//     const size_t bufferSize1 = m_width * m_height * sizeof(uint32_t);

//     m_genColorBuffer = vk_utils::createBuffer(m_device, bufferSize1,  VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_DST_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT);
//     m_colorMem       = vk_utils::allocateAndBindWithPadding(m_device, m_physicalDevice, {m_genColorBuffer});

//     auto tmp = std::make_shared<VulkanRTX>(m_pScnMgr);
//     tmp->CommitScene();

//     m_pRayTracerGPU->SetScene(tmp);
//     m_pRayTracerGPU->SetVulkanInOutFor_CastSingleRay(m_genColorBuffer, 0);
//     m_pRayTracerGPU->UpdateAll(m_pCopyHelper);
//   }

//   m_pRayTracerGPU->UpdateView(m_cam.pos, m_inverseProjViewMatrix);
//   m_pRayTracerGPU->UpdatePlainMembers(m_pCopyHelper);
  
//   // do ray tracing
//   //
//   {
//     VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(m_device, m_commandPool);

//     VkCommandBufferBeginInfo beginCommandBufferInfo = {};
//     beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
//     beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_ONE_TIME_SUBMIT_BIT;

//     vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
//     m_pRayTracerGPU->CastSingleRayCmd(commandBuffer, m_width, m_height, nullptr);
    
//     // prepare buffer and image for copy command
//     {
//       VkBufferMemoryBarrier transferBuff = {};
      
//       transferBuff.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
//       transferBuff.pNext               = nullptr;
//       transferBuff.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
//       transferBuff.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
//       transferBuff.size                = VK_WHOLE_SIZE;
//       transferBuff.offset              = 0;
//       transferBuff.buffer              = m_genColorBuffer;
//       transferBuff.srcAccessMask       = VK_ACCESS_SHADER_WRITE_BIT;
//       transferBuff.dstAccessMask       = VK_ACCESS_TRANSFER_READ_BIT;

//       VkImageMemoryBarrier transferImage;
//       transferImage.sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
//       transferImage.pNext               = nullptr;
//       transferImage.srcAccessMask       = 0;
//       transferImage.dstAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT;
//       transferImage.oldLayout           = VK_IMAGE_LAYOUT_UNDEFINED;
//       transferImage.newLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL; 
//       transferImage.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
//       transferImage.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
//       transferImage.image               = m_rtImage.image;

//       transferImage.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
//       transferImage.subresourceRange.baseMipLevel   = 0;
//       transferImage.subresourceRange.baseArrayLayer = 0;
//       transferImage.subresourceRange.layerCount     = 1;
//       transferImage.subresourceRange.levelCount     = 1;
    
//       vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_TRANSFER_BIT, 0, 0, nullptr, 1, &transferBuff, 1, &transferImage);
//     }

//     // execute copy
//     //
//     {
//       VkImageSubresourceLayers subresourceLayers = {};
//       subresourceLayers.aspectMask               = VK_IMAGE_ASPECT_COLOR_BIT;
//       subresourceLayers.mipLevel                 = 0;
//       subresourceLayers.baseArrayLayer           = 0;
//       subresourceLayers.layerCount               = 1;

//       VkBufferImageCopy copyRegion = {};
//       copyRegion.bufferOffset      = 0;
//       copyRegion.bufferRowLength   = uint32_t(m_width);
//       copyRegion.bufferImageHeight = uint32_t(m_height);
//       copyRegion.imageExtent       = VkExtent3D{ uint32_t(m_width), uint32_t(m_height), 1 };
//       copyRegion.imageOffset       = VkOffset3D{ 0, 0, 0 };
//       copyRegion.imageSubresource  = subresourceLayers;
  
//       vkCmdCopyBufferToImage(commandBuffer, m_genColorBuffer, m_rtImage.image, VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL, 1, &copyRegion);
//     }
    
//     // get back normal image layout
//     {
//       VkImageMemoryBarrier transferImage;
//       transferImage.sType               = VK_STRUCTURE_TYPE_IMAGE_MEMORY_BARRIER;
//       transferImage.pNext               = nullptr;
//       transferImage.srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT;
//       transferImage.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
//       transferImage.oldLayout           = VK_IMAGE_LAYOUT_TRANSFER_DST_OPTIMAL;
//       transferImage.newLayout           = VK_IMAGE_LAYOUT_SHADER_READ_ONLY_OPTIMAL; 
//       transferImage.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
//       transferImage.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
//       transferImage.image               = m_rtImage.image;

//       transferImage.subresourceRange.aspectMask     = VK_IMAGE_ASPECT_COLOR_BIT;
//       transferImage.subresourceRange.baseMipLevel   = 0;
//       transferImage.subresourceRange.baseArrayLayer = 0;
//       transferImage.subresourceRange.layerCount     = 1;
//       transferImage.subresourceRange.levelCount     = 1;
    
//       vkCmdPipelineBarrier(commandBuffer, VK_PIPELINE_STAGE_TRANSFER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 0, nullptr, 0, nullptr, 1, &transferImage);
//     }


//     vkEndCommandBuffer(commandBuffer);

//     vk_utils::executeCommandBufferNow(commandBuffer, m_graphicsQueue, m_device);
//   }

}