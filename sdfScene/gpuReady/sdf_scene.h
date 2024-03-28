
#pragma once
#include "LiteMath/LiteMath.h"
#include <vector>

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

// enum SdfPrimitiveType
static constexpr unsigned SDF_PRIM_SPHERE = 0;
static constexpr unsigned SDF_PRIM_BOX = 1;
static constexpr unsigned SDF_PRIM_CYLINDER = 2;
static constexpr unsigned SDF_PRIM_SIREN = 3;

struct SdfObject
{
  unsigned type;          // from enum SdfPrimitiveType
  unsigned params_offset; // in parameters vector
  unsigned params_count;
  unsigned neural_id; // index in neural_properties if type is neural

  float distance_mult;
  float distance_add;
  unsigned complement; // 0 or 1
  unsigned _pad; 

  float4 max_pos;    //float4 to prevent padding issues
  float4 min_pos;    //float4 to prevent padding issues

  float4x4 transform;
};
struct SdfConjunction
{
  float4 max_pos;
  float4 min_pos;
  unsigned offset; // in objects vector
  unsigned size;
  unsigned _pad[2];
};

constexpr int NEURAL_SDF_MAX_LAYERS = 8;
constexpr int NEURAL_SDF_MAX_LAYER_SIZE = 1024;
constexpr float SIREN_W0 = 30;

struct NeuralDenseLayer
{
  unsigned offset;
  unsigned in_size;
  unsigned out_size;
};
struct NeuralProperties
{
  unsigned layer_count;
  NeuralDenseLayer layers[NEURAL_SDF_MAX_LAYERS];
};

struct SdfHit
{
  float4 hit_pos;  // hit_pos.w < 0 if no hit, hit_pos.w > 0 otherwise
  float4 hit_norm; // hit_norm.w is not used
};

static float2 box_intersects(const float3 &min_pos, const float3 &max_pos, const float3 &origin, const float3 &dir)
{
  float3 safe_dir = sign(dir) * max(float3(1e-9f), abs(dir));
  float3 tMin = (min_pos - origin) / safe_dir;
  float3 tMax = (max_pos - origin) / safe_dir;
  float3 t1 = min(tMin, tMax);
  float3 t2 = max(tMin, tMax);
  float tNear = std::max(t1.x, std::max(t1.y, t1.z));
  float tFar = std::min(t2.x, std::min(t2.y, t2.z));

  return float2(tNear, tFar);
}

static float eval_dist_prim(const float *parameters, const SdfObject *objects, const SdfConjunction *conjunctions, const NeuralProperties *neural_properties,
                     unsigned parameters_count, unsigned objects_count, unsigned conjunctions_count, unsigned neural_properties_count,
                     unsigned prim_id, float3 p)
{
  const SdfObject &prim = objects[prim_id];
  float3 pos = prim.transform * p;
  // printf("%f %f %f -- %f %f %f\n", p.x, p.y, p.z, pos.x, pos.y, pos.z);
  // printf("%f %f %f %f\n %f %f %f %f\n %f %f %f %f\n %f %f %f %f\n",
  //        prim.transform(0, 0), prim.transform(0, 1), prim.transform(0, 2), prim.transform(0, 3),
  //        prim.transform(1, 0), prim.transform(1, 1), prim.transform(1, 2), prim.transform(1, 3),
  //        prim.transform(2, 0), prim.transform(2, 1), prim.transform(2, 2), prim.transform(2, 3),
  //        prim.transform(3, 0), prim.transform(3, 1), prim.transform(3, 2), prim.transform(3, 3));

  switch (prim.type)
  {
  case SDF_PRIM_SPHERE:
  {
    float r = parameters[prim.params_offset + 0];
    // fprintf(stderr, "sphere %f %f %f - %f",pos.x, pos.y, pos.z, r);
    return length(pos) - r;
  }
  case SDF_PRIM_BOX:
  {
    float3 size(parameters[prim.params_offset + 0],
                parameters[prim.params_offset + 1],
                parameters[prim.params_offset + 2]);
    // fprintf(stderr, "box %f %f %f - %f %f %f - %f %f %f",p.x, p.y, p.z, pos.x, pos.y, pos.z, size.x, size.y, size.z);
    float3 q = abs(pos) - size;
    return length(max(q, float3(0.0f))) + min(max(q.x, max(q.y, q.z)), 0.0f);
  }
  case SDF_PRIM_CYLINDER:
  {
    float h = parameters[prim.params_offset + 0];
    float r = parameters[prim.params_offset + 1];
    float2 d = abs(float2(sqrt(pos.x * pos.x + pos.z * pos.z), pos.y)) - float2(r, h);
    return min(max(d.x, d.y), 0.0f) + length(max(d, float2(0.0f)));
  }
  case SDF_PRIM_SIREN:
  {
    float tmp_mem[2 * NEURAL_SDF_MAX_LAYER_SIZE];

    auto &prop = neural_properties[prim.neural_id];
    unsigned t_ofs1 = 0;
    unsigned t_ofs2 = NEURAL_SDF_MAX_LAYER_SIZE;

    tmp_mem[t_ofs1 + 0] = p.x;
    tmp_mem[t_ofs1 + 1] = p.y;
    tmp_mem[t_ofs1 + 2] = p.z;

    for (int l = 0; l < prop.layer_count; l++)
    {
      unsigned m_ofs = prop.layers[l].offset;
      unsigned b_ofs = prop.layers[l].offset + prop.layers[l].in_size * prop.layers[l].out_size;
      for (int i = 0; i < prop.layers[l].out_size; i++)
      {
        tmp_mem[t_ofs2 + i] = parameters[b_ofs + i];
        for (int j = 0; j < prop.layers[l].in_size; j++)
          tmp_mem[t_ofs2 + i] += tmp_mem[t_ofs1 + j] * parameters[m_ofs + i * prop.layers[l].in_size + j];
        if (l < prop.layer_count - 1)
          tmp_mem[t_ofs2 + i] = std::sin(SIREN_W0 * tmp_mem[t_ofs2 + i]);
      }

      t_ofs2 = t_ofs1;
      t_ofs1 = (t_ofs1 + NEURAL_SDF_MAX_LAYER_SIZE) % (2 * NEURAL_SDF_MAX_LAYER_SIZE);
    }

    return tmp_mem[t_ofs1];
  }
  default:
    //fprintf(stderr, "unknown type %u", prim.type);
    //assert(false);
    break;
  }
  return -1000;
}

static float eval_dist_conjunction(const float *parameters, const SdfObject *objects, const SdfConjunction *conjunctions, const NeuralProperties *neural_properties,
                            unsigned parameters_count, unsigned objects_count, unsigned conjunctions_count, unsigned neural_properties_count,
                            unsigned conj_id, float3 p)
{
  const SdfConjunction &conj = conjunctions[conj_id];
  float conj_d = -1e6;
  for (unsigned pid = conj.offset; pid < conj.offset + conj.size; pid++)
  {
    float prim_d = objects[pid].distance_mult * eval_dist_prim(parameters, objects, conjunctions, neural_properties,
                                                               parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                                               pid, p) +
                   objects[pid].distance_add;
    conj_d = max(conj_d, objects[pid].complement ? -prim_d : prim_d);
  }
  return conj_d;
}

// evaluate distance to a whole scene (minimum of distances to all conjunctions)
float eval_dist_scene(const float *parameters, const SdfObject *objects, const SdfConjunction *conjunctions, const NeuralProperties *neural_properties,
                      unsigned parameters_count, unsigned objects_count, unsigned conjunctions_count, unsigned neural_properties_count,
                      float3 p);

// perform sphere tracing to find ray intersection with a specific conjunction and inside given bbox
// (it can be smaller than real bbox of conjunction). Use with acceleration structure on conjunction bboxes
// dir vector MUST be normalized
SdfHit sdf_conjunction_sphere_tracing(const float *parameters, const SdfObject *objects, const SdfConjunction *conjunctions, const NeuralProperties *neural_properties,
                                      unsigned parameters_count, unsigned objects_count, unsigned conjunctions_count, unsigned neural_properties_count,
                                      unsigned conj_id, const float3 &min_pos, const float3 &max_pos,
                                      const float3 &pos, const float3 &dir, bool need_norm = false);