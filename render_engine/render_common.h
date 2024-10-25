#ifndef CHIMERA_RENDER_COMMON_H
#define CHIMERA_RENDER_COMMON_H

#include "volk.h"
#include "vk_utils.h"
#include "Camera.h"
#include <cstring>

struct AppInput
{
  AppInput(){
    cams[1].pos    = float3(4.0f, 4.0f, 4.0f);
    cams[1].lookAt = float3(0, 0, 0);
    cams[1].up     = float3(0, 1, 0);
  }

  enum {MAXKEYS = 384};
  Camera cams[2];
  bool   keyPressed[MAXKEYS]{};
  bool   keyReleased[MAXKEYS]{};
  void clearKeys() { memset(keyPressed, 0, MAXKEYS*sizeof(bool)); memset(keyReleased, 0, MAXKEYS*sizeof(bool)); }
};

struct pipeline_data_t
{
  VkPipelineLayout layout;
  VkPipeline pipeline;
};

enum class DrawMode
{
  WITH_GUI,
  NO_GUI
};

class IRender
{
public:
  virtual uint32_t     GetWidth() const = 0;
  virtual uint32_t     GetHeight() const = 0;
  virtual VkInstance   GetVkInstance() const = 0;

  virtual void InitVulkan(const char** a_instanceExtensions, uint32_t a_instanceExtensionsCount, uint32_t a_deviceId) = 0;
  virtual void InitPresentation(VkSurfaceKHR& a_surface) = 0;
  virtual void ProcessInput(const AppInput& input) = 0;
  virtual void UpdateCamera(const Camera* cams, uint32_t a_camsCount) = 0;
  virtual Camera GetCurrentCamera() { return { };};
  virtual void LoadScene(const char* path) = 0;
  virtual void DrawFrame(float a_time, DrawMode a_mode) = 0;

  virtual ~IRender() = default;

};

#endif//CHIMERA_RENDER_COMMON_H
