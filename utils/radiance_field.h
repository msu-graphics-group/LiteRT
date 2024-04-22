#pragma once
#include "LiteMath/LiteMath.h"
#include <vector>
#include <string>
#include <memory>

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::float4x4;
using LiteMath::float3x3;
using LiteMath::cross;
using LiteMath::dot;
using LiteMath::length;
using LiteMath::normalize;
using LiteMath::to_float4;
using LiteMath::to_float3;
using LiteMath::max;
using LiteMath::min;

#ifndef KERNEL_SLICER
const size_t CellSize = 28;

// structure to actually store SdfScene data
struct RFScene
{
  int size;
  float scale;
  std::vector<float> data; //size.x*size.y*size.z*CellSize values
};

void load_rf_scene(RFScene &scene, const std::string &path); // loads scene from raw SIREN weights file
#endif