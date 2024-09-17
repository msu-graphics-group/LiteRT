#pragma once

#include <array>
#include <string>
#include <vector>

#include <LiteMath.h>

#ifndef KERNEL_SLICER

struct GSScene {
  // x           y           z           opacity
  // rot_0       rot_1       rot_2       rot_3
  // scale_0     scale_1     scale_2     [empty]
  // [empty]     [empty]     [empty]     [empty]
  std::vector<LiteMath::float4x4> data{};

  // f_dc_0      f_rest_0    f_rest_1    f_rest_2
  // f_rest_3    f_rest_4    f_rest_5    f_rest_6
  // f_rest_7    f_rest_8    f_rest_9    f_rest_10
  // f_rest_11   f_rest_12   f_rest_13   f_rest_14
  std::vector<LiteMath::float4x4> data_r{};

  // f_dc_1      f_rest_15   f_rest_16   f_rest_17
  // f_rest_18   f_rest_19   f_rest_20   f_rest_21
  // f_rest_22   f_rest_23   f_rest_24   f_rest_25
  // f_rest_26   f_rest_27   f_rest_28   f_rest_29
  std::vector<LiteMath::float4x4> data_g{};

  // f_dc_2      f_rest_30   f_rest_31   f_rest_32
  // f_rest_33   f_rest_34   f_rest_35   f_rest_36
  // f_rest_37   f_rest_38   f_rest_39   f_rest_40
  // f_rest_41   f_rest_42   f_rest_43   f_rest_44
  std::vector<LiteMath::float4x4> data_b{};
};

void load_gs_scene(
  GSScene& scene,
  const std::string& points_path);

#endif
