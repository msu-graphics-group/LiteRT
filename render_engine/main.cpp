#include "simple_render.h"
#include "glfw_window.h"
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

int main(int argc, const char ** argv)
{
  constexpr int WIDTH = 1024;
  constexpr int HEIGHT = 1024;
  constexpr int VULKAN_DEVICE_ID = 0;

  std::string scene_fpath = "./scenes/01_simple_scenes/bunny.xml";
  if (argc > 1)
  {
    for (int i=1; i<argc; i++)
    {
      if (std::string(argv[i]) == "-scene")
      {
        if (i+1 < argc)
          scene_fpath = argv[++i];
        else
          std::cout << "No scene file path provided" << std::endl;
      }
    }
  }

  std::shared_ptr<IRender> app = std::make_shared<SimpleRender>(WIDTH, HEIGHT);

  if(app == nullptr)
  {
    std::cout << "Can't create render of specified type" << std::endl;
    return 1;
  }

  auto* window = initWindow(WIDTH, HEIGHT);

  initVulkanGLFW(app, window, VULKAN_DEVICE_ID);

  app->LoadScene(scene_fpath.c_str());
  bool showGUI = true;
  mainLoop(app, window, showGUI);

  return 0;
}
