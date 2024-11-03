#ifndef SIMPLE_RENDER_H
#define SIMPLE_RENDER_H

#define VK_NO_PROTOTYPES

#include "LiteScene/scene_mgr.h"
#include "render_common.h"
#include "render_gui.h"
#include "../shaders/common.h"
#include "../Renderer/eye_ray.h"

#include <geom/vk_mesh.h>
#include <vk_descriptor_sets.h>
#include <vk_fbuf_attachment.h>
#include <vk_quad.h>
#include <vk_images.h>
#include <vk_swapchain.h>
#include <string>
#include <iostream>

#include "CrossRT.h"

#if defined(USE_RTX)
  #include "../Renderer/eye_ray_rtx.h"
  typedef MultiRenderer_RTX MultiRendererGPUImpl;
  std::shared_ptr<MultiRenderer> CreateMultiRenderer_RTX(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated);

  static std::shared_ptr<MultiRenderer> Create_MultiRendererGPUImpl(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, uint32_t max_width, uint32_t max_height)
  {
    return CreateMultiRenderer_RTX(maxPrimitives, a_ctx, max_width * max_height);
  }
#else
  #include "../Renderer/eye_ray_gpu.h"
  typedef MultiRenderer_GPU MultiRendererGPUImpl;
  std::shared_ptr<MultiRenderer> CreateMultiRenderer_GPU(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, size_t a_maxThreadsGenerated);

  static std::shared_ptr<MultiRenderer> Create_MultiRendererGPUImpl(uint32_t maxPrimitives, vk_utils::VulkanContext a_ctx, uint32_t max_width, uint32_t max_height)
  {
    return CreateMultiRenderer_GPU(maxPrimitives, a_ctx, max_width * max_height);
  }
#endif

enum class RenderMode
{
  RASTERIZATION,
  RAYTRACING,
};

class SimpleRender : public IRender
{
public:
  const std::string VERTEX_SHADER_PATH   = "./shaders/simple.vert";
  const std::string FRAGMENT_SHADER_PATH = "./shaders/simple.frag";

  static constexpr uint64_t STAGING_MEM_SIZE = 16 * 16 * 1024u;

  SimpleRender(uint32_t a_width, uint32_t a_height);
  ~SimpleRender()  { Cleanup(); };

  inline uint32_t     GetWidth()      const override { return m_width; }
  inline uint32_t     GetHeight()     const override { return m_height; }
  inline VkInstance   GetVkInstance() const override { return m_instance; }
  void InitVulkan(const char** a_instanceExtensions, uint32_t a_instanceExtensionsCount, uint32_t a_deviceId) override;

  void InitPresentation(VkSurfaceKHR& a_surface) override;

  void ProcessInput(const AppInput& input) override;
  void UpdateCamera(const Camera* cams, uint32_t a_camsCount) override;
  Camera GetCurrentCamera() override {return m_cam;}
  void UpdateView();

  void LoadScene(const char *path) override;
  void DrawFrame(float a_time, DrawMode a_mode) override;

  //////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

  // debugging utils
  //
  static VKAPI_ATTR VkBool32 VKAPI_CALL debugReportCallbackFn(
    VkDebugReportFlagsEXT                       flags,
    VkDebugReportObjectTypeEXT                  objectType,
    uint64_t                                    object,
    size_t                                      location,
    int32_t                                     messageCode,
    const char* pLayerPrefix,
    const char* pMessage,
    void* pUserData)
  {
    (void)flags;
    (void)objectType;
    (void)object;
    (void)location;
    (void)messageCode;
    (void)pUserData;
    std::cout << pLayerPrefix << ": " << pMessage << std::endl;
    return VK_FALSE;
  }

  VkDebugReportCallbackEXT m_debugReportCallback = nullptr;
protected:

  VkInstance       m_instance       = VK_NULL_HANDLE;
  VkCommandPool    m_commandPool    = VK_NULL_HANDLE;
  VkPhysicalDevice m_physicalDevice = VK_NULL_HANDLE;
  VkDevice         m_device         = VK_NULL_HANDLE;
  VkQueue          m_graphicsQueue  = VK_NULL_HANDLE;
  VkQueue          m_transferQueue  = VK_NULL_HANDLE;
  VkQueue          m_computeQueue   = VK_NULL_HANDLE;

  std::shared_ptr<vk_utils::ICopyEngine> m_pCopyHelper;
  std::shared_ptr<vk_utils::IMemoryAlloc> m_pAllocatorSpecial;

  vk_utils::QueueFID_T m_queueFamilyIDXs {UINT32_MAX, UINT32_MAX, UINT32_MAX};

  RenderMode m_currentRenderMode = RenderMode::RAYTRACING;
  bool m_RasterSceneSetUp = false;
  bool m_RTSceneSetUp     = false;
  std::string m_scenePath;

  struct
  {
    uint32_t    currentFrame      = 0u;
    VkQueue     queue             = VK_NULL_HANDLE;
    VkSemaphore imageAvailable    = VK_NULL_HANDLE;
    VkSemaphore renderingFinished = VK_NULL_HANDLE;
  } m_presentationResources;

  std::vector<VkFence> m_frameFences;
  std::vector<VkCommandBuffer> m_cmdBuffersDrawMain;

  struct
  {
    LiteMath::float4x4 projView;
    LiteMath::float4x4 model;
  } pushConst2M;

  UniformParams m_uniforms {};
  VkBuffer m_ubo = VK_NULL_HANDLE;
  VkDeviceMemory m_uboAlloc = VK_NULL_HANDLE;
  void* m_uboMappedMem = nullptr;

  std::shared_ptr<vk_utils::DescriptorMaker> m_pBindings = nullptr;

  pipeline_data_t m_basicForwardPipeline {};

  VkDescriptorSet m_dSet = VK_NULL_HANDLE;
  VkDescriptorSetLayout m_dSetLayout = VK_NULL_HANDLE;
  VkRenderPass m_screenRenderPass = VK_NULL_HANDLE; // rasterization renderpass

  LiteMath::float4x4 m_projectionMatrix;
  LiteMath::float4x4 m_worldViewMatrix;
  LiteMath::float4x4 m_inverseProjViewMatrix;

  // *** ray tracing
  // full screen quad resources to display ray traced image
  std::vector<uint32_t> m_raytracedImageData;
  std::shared_ptr<vk_utils::IQuad> m_pFSQuad;
  VkDescriptorSet m_quadDS = VK_NULL_HANDLE;
  VkDescriptorSetLayout m_quadDSLayout = VK_NULL_HANDLE;
  vk_utils::VulkanImageMem m_rtImage;
  VkSampler                m_rtImageSampler = VK_NULL_HANDLE;

  bool m_force_rayrace_cpu = false;
  std::shared_ptr<MultiRenderer> m_pRayTracer;
  MultiRendererGPUImpl *m_pRayTracerGPU; //it is the same object as m_pRayTracer, but only it's GPU handle
  void RayTraceCPU();
  void RayTraceGPU();

  VkBuffer m_genColorBuffer = VK_NULL_HANDLE;
  VkDeviceMemory m_colorMem = VK_NULL_HANDLE;
  //

  // *** presentation
  VkSurfaceKHR m_surface = VK_NULL_HANDLE;
  VulkanSwapChain m_swapchain;
  std::vector<VkFramebuffer> m_frameBuffers;
  vk_utils::VulkanImageMem m_depthBuffer{};
  // ***

  // *** GUI
  std::shared_ptr<IRenderGUI> m_pGUIRender;
  void SetupGUIElements();
  void DrawFrameWithGUI();
  //

  Camera   m_cam;
  uint32_t m_width  = 1024u;
  uint32_t m_height = 1024u;
  uint32_t m_framesInFlight  = 2u;
  bool m_vsync = false;

  VkPhysicalDeviceFeatures m_enabledDeviceFeatures = {};
  std::vector<const char*> m_instanceExtensions    = {};

  bool m_enableValidation;
  std::vector<const char*> m_validationLayers;

  std::shared_ptr<SceneManager> m_pScnMgr = nullptr;

  void DrawFrameSimple();

  void CreateInstance();
  void CreateDevice(uint32_t a_deviceId);

  void BuildCommandBufferSimple(VkCommandBuffer cmdBuff, VkFramebuffer frameBuff,
                                VkImageView a_targetImageView, VkPipeline a_pipeline);

  // *** Ray tracing related stuff
  void BuildCommandBufferQuad(VkCommandBuffer a_cmdBuff, VkImageView a_targetImageView);
  void SetupQuadRenderer();
  void SetupQuadDescriptors();
  void SetupRTImage();
  void SetupRTScene(const char *path);
  void OnScreenResolutionChangeRT();
  // ***************************

  void SetupSimplePipeline();
  void CleanupPipelineAndSwapchain();
  void RecreateSwapChain();

  void CreateUniformBuffer();
  void UpdateUniformBuffer(float a_time);

  void Cleanup();

  VkPhysicalDeviceFeatures2 SetupDeviceFeatures();
  std::vector<const char*> SetupDeviceExtensions();
  void SetupValidationLayers();
};


#endif //SIMPLE_RENDER_H
