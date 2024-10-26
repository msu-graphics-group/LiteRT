#include "simple_render.h"
#include "glfw_window.h"
#include "../Renderer/eye_ray_gpu.h"

void initVulkanGLFW(std::shared_ptr<IRender> &app, GLFWwindow* window, int deviceID)
{
  uint32_t glfwExtensionCount = 0;
  const char** glfwExtensions;
  glfwExtensions  = glfwGetRequiredInstanceExtensions(&glfwExtensionCount);

  if(glfwExtensions == nullptr)
  {
    std::cout << "WARNING. Can't connect Vulkan to GLFW window (glfwGetRequiredInstanceExtensions returns NULL)" << std::endl;
  }

  app->InitVulkan(glfwExtensions, glfwExtensionCount, deviceID);

  if(glfwExtensions != nullptr)
  {
    VkSurfaceKHR surface;
    VK_CHECK_RESULT(glfwCreateWindowSurface(app->GetVkInstance(), window, nullptr, &surface));
    setupImGuiContext(window);
    app->InitPresentation(surface);
  }
}

int main()
{
  constexpr int WIDTH = 1024;
  constexpr int HEIGHT = 1024;
  constexpr int VULKAN_DEVICE_ID = 0;

  std::shared_ptr<IRender> app = std::make_shared<SimpleRender>(WIDTH, HEIGHT);
  std::shared_ptr<SimpleRender> app_sr = std::dynamic_pointer_cast<SimpleRender>(app);

  if(app == nullptr)
  {
    std::cout << "Can't create render of specified type" << std::endl;
    return 1;
  }

  auto* window = initWindow(WIDTH, HEIGHT);

  initVulkanGLFW(app, window, VULKAN_DEVICE_ID);

  app->LoadScene("./scenes/043_cornell_normals/statex_00001.xml");
  // app->LoadScene("../resources/scenes/buggy/Buggy.gltf");

  auto mr = std::make_shared<MultiRenderer_GPU>(1000000);
  //mr->SetVulkanContext(a_ctx);
  mr->InitVulkanObjects(app_sr->m_device, app_sr->m_physicalDevice, 2048*2048);

  bool showGUI = true;
  mainLoop(app, window, showGUI);

  return 0;
}
