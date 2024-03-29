#pragma once

#include <cstdint>
#include <memory>
#include <array>
#include <string>
#include <vector>
#include <iostream>
#include <unordered_map>

#include "LiteMath.h"
#include "../LiteRT.h"
#include "../IRenderer.h"
#include "../BVH/BVH2Common.h"

using LiteMath::uint;

class EyeRayCaster : public IRenderer
{
public:
  EyeRayCaster(); 
  const char* Name() const override;
  
  virtual void SceneRestrictions(uint32_t a_restrictions[4]) const
  {
    uint32_t maxMeshes            = 1024;
    uint32_t maxTotalVertices     = 8'000'000;
    uint32_t maxTotalPrimitives   = 8'000'000;
    uint32_t maxPrimitivesPerMesh = 4'000'000;

    a_restrictions[0] = maxMeshes;
    a_restrictions[1] = maxTotalVertices;
    a_restrictions[2] = maxTotalPrimitives;
    a_restrictions[3] = maxPrimitivesPerMesh;
  }

  virtual bool LoadScene(const char* a_scenePath) override;

  void Clear (uint32_t a_width, uint32_t a_height, const char* a_what) override;
  void Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum) override;
  void SetViewport(int a_xStart, int a_yStart, int a_width, int a_height) override;

  void SetAccelStruct(std::shared_ptr<ISceneObject> a_customAccelStruct) override { m_pAccelStruct = a_customAccelStruct;}
  std::shared_ptr<ISceneObject> GetAccelStruct() override { return m_pAccelStruct; }

  void GetExecutionTime(const char* a_funcName, float a_out[4]) override;

  void CommitDeviceData() override {}
  
protected:
  bool LoadSceneHydra(const std::string& a_path);

  virtual void PackXYBlock(uint tidX, uint tidY, uint a_passNum);
  virtual void PackXY(uint tidX, uint tidY);
  virtual void kernel_PackXY(uint tidX, uint tidY, uint* out_pakedXY);
  
  void SetPresets(const RenderPreset& a_presets) override;
  virtual void CastRaySingle(uint32_t tidX, uint32_t* out_color __attribute__((size("tidX"))));

  virtual void CastRaySingleBlock(uint32_t tidX, uint32_t* out_color, uint32_t a_numPasses = 1);

  void kernel_InitEyeRay(uint32_t tidX, LiteMath::float4* rayPosAndNear, LiteMath::float4* rayDirAndFar);
  void kernel_RayTrace(uint32_t tidX, const LiteMath::float4* rayPosAndNear, const LiteMath::float4* rayDirAndFar, uint32_t* out_color);

  uint32_t m_width;
  uint32_t m_height;
  uint32_t m_measureOverhead = 0;

  int m_gltfCamId = -1;
  LiteMath::float4x4 m_projInv;
  LiteMath::float4x4 m_worldViewInv;

  std::shared_ptr<ISceneObject> m_pAccelStruct;
  std::vector<uint32_t>         m_packedXY;

  // color palette to select color for objects based on mesh/instance id
  static constexpr uint32_t palette_size = 20;
  static constexpr uint32_t m_palette[palette_size] = {
    0xffe6194b, 0xff3cb44b, 0xffffe119, 0xff0082c8,
    0xfff58231, 0xff911eb4, 0xff46f0f0, 0xfff032e6,
    0xffd2f53c, 0xfffabebe, 0xff008080, 0xffe6beff,
    0xffaa6e28, 0xfffffac8, 0xff800000, 0xffaaffc3,
    0xff808000, 0xffffd8b1, 0xff000080, 0xff808080
  };

  static constexpr uint32_t BSIZE =8;

  std::unordered_map<std::string, float> timeDataByName;
  mutable std::string m_tempName;
  double m_avgLCV;

  uint64_t m_totalTris         = 0;
  uint64_t m_totalTrisVisiable = 0;

  uint32_t GetGeomNum() const override { return m_pAccelStruct->GetGeomNum(); };
  uint32_t GetInstNum() const override { return m_pAccelStruct->GetInstNum(); };
  const LiteMath::float4* GetGeomBoxes() const  override { return m_pAccelStruct->GetGeomBoxes(); };
};
