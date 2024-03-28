#include "sdf_scene.h"

using namespace LiteMath;

SdfHit sdf_conjunction_sphere_tracing(const float *parameters, const SdfObject *objects, const SdfConjunction *conjunctions, const NeuralProperties *neural_properties,
                                    unsigned parameters_count, unsigned objects_count, unsigned conjunctions_count, unsigned neural_properties_count,
                                    unsigned conj_id, const float3 &min_pos, const float3 &max_pos,
                                    const float3 &pos, const float3 &dir, bool need_norm)
{
  const float EPS = 1e-5;

  SdfHit hit;
  float2 tNear_tFar = box_intersects(min_pos, max_pos, pos, dir);
  float t = tNear_tFar.x;
  float tFar = tNear_tFar.y;
  if (t > tFar)
    return hit;
  
  int iter = 0;
  float d = eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                  parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                  conj_id, pos + t * dir);
  while (iter < 1000 && d > EPS && t < tFar)
  {
    t += d + EPS;
    d = eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                              parameters_count, objects_count, conjunctions_count, neural_properties_count,
                              conj_id, pos + t * dir);
    iter++;
  }

  float3 p0 = pos + t * dir;
  float3 norm = float3(1,0,0);
  if (need_norm)
  {
    constexpr float h = 0.001;
    float ddx = (eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                       parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                       conj_id, p0 + float3(h, 0, 0)) -
                 eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                       parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                       conj_id, p0 + float3(-h, 0, 0))) /
                (2 * h);
    float ddy = (eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                       parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                       conj_id, p0 + float3(0, h, 0)) -
                 eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                       parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                       conj_id, p0 + float3(0, -h, 0))) /
                (2 * h);
    float ddz = (eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                       parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                       conj_id, p0 + float3(0, 0, h)) -
                 eval_dist_conjunction(parameters, objects, conjunctions, neural_properties,
                                       parameters_count, objects_count, conjunctions_count, neural_properties_count,
                                       conj_id, p0 + float3(0, 0, -h))) /
                (2 * h);

    norm = normalize(float3(ddx, ddy, ddz));
    // fprintf(stderr, "st %d (%f %f %f)\n", iter, surface_normal->x, surface_normal->y, surface_normal->z);
  }
  // fprintf(stderr, "st %d (%f %f %f)", iter, p0.x, p0.y, p0.z);
  hit.hit_pos = to_float4(p0, (d <= EPS) ? 1 : -1);
  hit.hit_norm = to_float4(norm, 1.0f);
  return hit;
}