#pragma once
#include <string>
#include "render_settings.h"
#include "IRenderer.h"

class Integrator;

struct HydraRenderPreset
{
  unsigned integratorType; //only default one (Integrator::INTEGRATOR_MIS_PT) is supported
  unsigned fbLayer;        //only default one (Integrator::FB_COLOR) is supported
  unsigned spp;
};

HydraRenderPreset getDefaultHydraRenderPreset();

class HydraRenderer : public IRenderer
{
public:
  constexpr static unsigned MAX_WIDTH  = 2048;
  constexpr static unsigned MAX_HEIGHT = 2048;
  constexpr static float GAMMA = 2.4f;

  HydraRenderer(unsigned device = DEVICE_CPU);
  void SetPreset(uint32_t a_width, uint32_t a_height, HydraRenderPreset a_preset);
  virtual const char* Name() const  override { return "HydraRenderer"; }
  virtual bool LoadScene(const char* a_scenePath) override;

  virtual void Clear (uint32_t a_width, uint32_t a_height, const char* a_what) override;
  virtual void Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum = 1) override;

  virtual void SetViewport(int a_xStart, int a_yStart, int a_width, int a_height) override;
  virtual void SetAccelStruct(std::shared_ptr<ISceneObject> a_customAccelStruct) override;
  virtual std::shared_ptr<ISceneObject> GetAccelStruct() override;
  
  virtual void GetExecutionTime(const char* a_funcName, float a_out[4]) override;
  virtual void CommitDeviceData() override;
private:
  uint32_t m_width;
  uint32_t m_height;
  std::vector<float> realColor;
  std::shared_ptr<Integrator> m_pImpl = nullptr;
  HydraRenderPreset m_preset;
  unsigned m_device;
};

void hydra_integration_example(unsigned device = DEVICE_CPU, std::string scene_filename = "dependencies/HydraCore3/scenes/test_035/statex_00001.xml");