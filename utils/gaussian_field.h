#pragma once

#include <array>
#include <string>
#include <vector>

#include <LiteMath.h>

#ifndef KERNEL_SLICER

struct GSScene {
  // x         y         z         f_dc_0
  // f_dc_1    f_dc_2    opacity   scale_0
  // scale_1   scale_2   rot_0     rot_1
  // rot_2     rot_3     [empty]   [empty]
  std::vector<LiteMath::float4x4> data_0{};
};

void load_gs_scene(GSScene& scene, const std::string& path);

#endif
