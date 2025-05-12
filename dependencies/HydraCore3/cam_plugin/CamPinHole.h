#pragma once
#include "CamPluginAPI.h"

#include "LiteMath.h"
#include "../include/cglobals.h"
#include "../include/crandom.h"
#include "../spectrum.h"

#include <vector>

using LiteMath::float4x4;
using LiteMath::float4;
using LiteMath::float3;

class CamPinHole : public ICamRaysAPI2
{
public:
  CamPinHole();
  virtual ~CamPinHole();

  void SetParameters(int a_width, int a_height, const CamParameters& a_params) override;
  void SetBatchSize(int a_tileSize) override { Init(a_tileSize); };
  
  void MakeRaysBlock(RayPosAndW* out_rayPosAndNear4f [[size("in_blockSize")]], 
                     RayDirAndT* out_rayDirAndFar4f  [[size("in_blockSize")]], uint32_t in_blockSize, int subPassId)  override;
                     
  void AddSamplesContributionBlock(float* out_color4f [[size("a_width*a_height*4")]], const float* colors4f [[size("in_blockSize*4")]], uint32_t in_blockSize, 
                                   uint32_t a_width, uint32_t a_height, int subPassId) override;

  virtual void CommitDeviceData(){}
  virtual void GetExecutionTime(const char* a_funcName, float a_out[4]){}

protected:

  void kernel1D_MakeEyeRay   (int in_blockSize, RayPosAndW* out_rayPosAndNear4f, RayDirAndT* out_rayDirAndFar4f, int subPassId);
  void kernel1D_ContribSample(int in_blockSize, const float* in_color, float* out_color, int subPassId);

  float4x4 m_proj;
  float4x4 m_projInv;

  int m_width;
  int m_height;
  int m_spectral_mode;

  std::vector<RandomGen>  m_randomGens;
  std::vector<float>      m_storedWaves;
  void Init(int a_maxThreads);

  std::vector<float> m_cie_x;
  std::vector<float> m_cie_y;
  std::vector<float> m_cie_z;

  static constexpr float CAM_LAMBDA_MIN = 360.0f;
  static constexpr float CAM_LAMBDA_MAX = 830.0f;
};
