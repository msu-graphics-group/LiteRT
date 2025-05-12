#pragma once

#include "integrator_pt.h"
#include "include/crandom.h"

#include "include/cmaterial.h"
#include "include/cmat_gltf.h"
#include "include/cmat_conductor.h"

#include <chrono>
#include <string>

#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;
using namespace LiteMath;

#include "utils.h"

void Integrator::PackXYBlock(uint tidX, uint tidY, uint a_passNum)
{
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int y = 0; y < tidY; ++y)
    for(int x = 0; x < tidX; ++x)
      PackXY(uint(x), (uint)(y));
}

void Integrator::CastSingleRayBlock(uint tid, float* out_color, uint a_passNum)
{ 
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int i = 0; i < tid; ++i)
    CastSingleRay(uint(i), out_color);
}

void Integrator::NaivePathTraceBlock(uint tid, uint channels, float* out_color, uint a_passNum)
{
  ConsoleProgressBar progress(tid);
  progress.Start();

  auto start = std::chrono::high_resolution_clock::now();
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int i = 0; i < tid; ++i) {
    for(int j = 0; j < a_passNum; ++j) {
      NaivePathTrace(uint(i), channels, out_color);
    }
    progress.Update();
  }
  progress.Done();
  naivePtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;
}

void Integrator::PathTraceBlock(uint tid, uint channels, float* out_color, uint a_passNum)
{
  ConsoleProgressBar progress(tid);
  progress.Start();
  auto start = std::chrono::high_resolution_clock::now();
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for (int i = 0; i < tid; ++i) {
    for (int j = 0; j < a_passNum; ++j) {
      PathTrace(uint(i), channels, out_color);
    }
    progress.Update();
  }
  progress.Done();
  shadowPtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;
}

void Integrator::RayTraceBlock(uint tid, uint channels, float* out_color, uint a_passNum)
{
  ConsoleProgressBar progress(tid);
  progress.Start();
  auto start = std::chrono::high_resolution_clock::now();
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for(int i = 0; i < tid; ++i)
  {
    RayTrace(uint(i), channels, out_color);
    progress.Update();
  }
  progress.Done();
  raytraceTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;
}

void Integrator::PathTraceFromInputRaysBlock(uint tid, uint channels, const RayPosAndW* in_rayPosAndNear, const RayDirAndT* in_rayDirAndFar, float* out_color, uint a_passNum)
{
  auto start = std::chrono::high_resolution_clock::now();
  #ifndef _DEBUG
  #pragma omp parallel for default(shared)
  #endif
  for (int i = 0; i < tid; ++i) {
    for (int j = 0; j < a_passNum; ++j)
      PathTraceFromInputRays(uint(i), channels, in_rayPosAndNear, in_rayDirAndFar, out_color);
  }
  fromRaysPtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;
}

//ConsoleProgressBar g_progressbarForGPU(1000);
//
//void Integrator::_ProgressBarStart()                  { g_progressbarForGPU.Start(); }
//void Integrator::_ProgressBarAccum(float a_progress)  { g_progressbarForGPU.Update(int64_t(a_progress*1000.0f)); }
//void Integrator::_ProgressBarDone()                   { g_progressbarForGPU.Done(); }

void Integrator::_ProgressBarStart() 
{ 
  m_currProgress = 0.0f; 
  std::cout << "rendering (gpu), progress = " << std::fixed << std::setprecision(2) << 0 << " %   \r";
  std::cout.flush();
}
void Integrator::_ProgressBarAccum(float a_progress)  
{ 
  m_currProgress += a_progress; 
  if(std::abs(m_currProgress - m_currProgressOld) > 0.05f)
  {
    std::cout << "rendering (gpu), progress = " << std::fixed << std::setprecision(2) << 100.0f*m_currProgress << " %   \r";
    std::cout.flush();
    m_currProgressOld = m_currProgress;
  }
}
void Integrator::_ProgressBarDone() 
{ 
  std::cout << "rendering (gpu), progress = " << std::fixed << std::setprecision(2) << 100.0f*m_currProgress << " %   \r";
  std::cout << std::endl; std::cout.flush(); 
}