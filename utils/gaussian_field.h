#pragma once

#include <array>
#include <string>
#include <vector>

#ifndef KERNEL_SLICER

struct GSScene {
  std::size_t count{};

  std::vector<float> x{};
  std::vector<float> y{};
  std::vector<float> z{};

  std::vector<float> nx{};
  std::vector<float> ny{};
  std::vector<float> nz{};

  std::vector<float> f_dc_0{};
  std::vector<float> f_dc_1{};
  std::vector<float> f_dc_2{};

  std::vector<float> f_rest_0{};
  std::vector<float> f_rest_1{};
  std::vector<float> f_rest_2{};
  std::vector<float> f_rest_3{};
  std::vector<float> f_rest_4{};
  std::vector<float> f_rest_5{};
  std::vector<float> f_rest_6{};
  std::vector<float> f_rest_7{};
  std::vector<float> f_rest_8{};
  std::vector<float> f_rest_9{};
  std::vector<float> f_rest_10{};
  std::vector<float> f_rest_11{};
  std::vector<float> f_rest_12{};
  std::vector<float> f_rest_13{};
  std::vector<float> f_rest_14{};
  std::vector<float> f_rest_15{};
  std::vector<float> f_rest_16{};
  std::vector<float> f_rest_17{};
  std::vector<float> f_rest_18{};
  std::vector<float> f_rest_19{};
  std::vector<float> f_rest_20{};
  std::vector<float> f_rest_21{};
  std::vector<float> f_rest_22{};
  std::vector<float> f_rest_23{};
  std::vector<float> f_rest_24{};
  std::vector<float> f_rest_25{};
  std::vector<float> f_rest_26{};
  std::vector<float> f_rest_27{};
  std::vector<float> f_rest_28{};
  std::vector<float> f_rest_29{};
  std::vector<float> f_rest_30{};
  std::vector<float> f_rest_31{};
  std::vector<float> f_rest_32{};
  std::vector<float> f_rest_33{};
  std::vector<float> f_rest_34{};
  std::vector<float> f_rest_35{};
  std::vector<float> f_rest_36{};
  std::vector<float> f_rest_37{};
  std::vector<float> f_rest_38{};
  std::vector<float> f_rest_39{};
  std::vector<float> f_rest_40{};
  std::vector<float> f_rest_41{};
  std::vector<float> f_rest_42{};
  std::vector<float> f_rest_43{};
  std::vector<float> f_rest_44{};

  std::vector<float> opacity{};

  std::vector<float> scale_0{};
  std::vector<float> scale_1{};
  std::vector<float> scale_2{};

  std::vector<float> rot_0{};
  std::vector<float> rot_1{};
  std::vector<float> rot_2{};
  std::vector<float> rot_3{};

  std::vector<float> base_color_0{};
  std::vector<float> base_color_1{};
  std::vector<float> base_color_2{};

  std::vector<float> roughness{};

  std::vector<float> metallic{};

  std::vector<float> incidents_dc_0{};
  std::vector<float> incidents_dc_1{};
  std::vector<float> incidents_dc_2{};

  std::vector<float> incidents_rest_0{};
  std::vector<float> incidents_rest_1{};
  std::vector<float> incidents_rest_2{};
  std::vector<float> incidents_rest_3{};
  std::vector<float> incidents_rest_4{};
  std::vector<float> incidents_rest_5{};
  std::vector<float> incidents_rest_6{};
  std::vector<float> incidents_rest_7{};
  std::vector<float> incidents_rest_8{};
  std::vector<float> incidents_rest_9{};
  std::vector<float> incidents_rest_10{};
  std::vector<float> incidents_rest_11{};
  std::vector<float> incidents_rest_12{};
  std::vector<float> incidents_rest_13{};
  std::vector<float> incidents_rest_14{};
  std::vector<float> incidents_rest_15{};
  std::vector<float> incidents_rest_16{};
  std::vector<float> incidents_rest_17{};
  std::vector<float> incidents_rest_18{};
  std::vector<float> incidents_rest_19{};
  std::vector<float> incidents_rest_20{};
  std::vector<float> incidents_rest_21{};
  std::vector<float> incidents_rest_22{};
  std::vector<float> incidents_rest_23{};
  std::vector<float> incidents_rest_24{};
  std::vector<float> incidents_rest_25{};
  std::vector<float> incidents_rest_26{};
  std::vector<float> incidents_rest_27{};
  std::vector<float> incidents_rest_28{};
  std::vector<float> incidents_rest_29{};
  std::vector<float> incidents_rest_30{};
  std::vector<float> incidents_rest_31{};
  std::vector<float> incidents_rest_32{};
  std::vector<float> incidents_rest_33{};
  std::vector<float> incidents_rest_34{};
  std::vector<float> incidents_rest_35{};
  std::vector<float> incidents_rest_36{};
  std::vector<float> incidents_rest_37{};
  std::vector<float> incidents_rest_38{};
  std::vector<float> incidents_rest_39{};
  std::vector<float> incidents_rest_40{};
  std::vector<float> incidents_rest_41{};
  std::vector<float> incidents_rest_42{};
  std::vector<float> incidents_rest_43{};
  std::vector<float> incidents_rest_44{};

  std::vector<float> visibility_dc_0{};

  std::vector<float> visibility_rest_0{};
  std::vector<float> visibility_rest_1{};
  std::vector<float> visibility_rest_2{};
  std::vector<float> visibility_rest_3{};
  std::vector<float> visibility_rest_4{};
  std::vector<float> visibility_rest_5{};
  std::vector<float> visibility_rest_6{};
  std::vector<float> visibility_rest_7{};
  std::vector<float> visibility_rest_8{};
  std::vector<float> visibility_rest_9{};
  std::vector<float> visibility_rest_10{};
  std::vector<float> visibility_rest_11{};
  std::vector<float> visibility_rest_12{};
  std::vector<float> visibility_rest_13{};
  std::vector<float> visibility_rest_14{};
};

void load_gs_scene(GSScene& scene, const std::string& path);

#endif
