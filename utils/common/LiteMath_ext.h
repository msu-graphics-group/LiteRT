#pragma once
#include "LiteMath/LiteMath.h"

namespace LiteMath
{
  struct AABB
  {
    float3 max_pos = float3(0, 0, 0);
    float3 min_pos = float3(0, 0, 0);

    AABB()
    {
    }
    AABB(const AABB &aabb)
    {
      min_pos = aabb.min_pos;
      max_pos = aabb.max_pos;
    }
    AABB(AABB &&aabb)
    {
      min_pos = aabb.min_pos;
      max_pos = aabb.max_pos;
    }
    AABB &operator=(const AABB &aabb)
    {
      min_pos = aabb.min_pos;
      max_pos = aabb.max_pos;
      return *this;
    }
    AABB &operator=(AABB &&aabb)
    {
      min_pos = aabb.min_pos;
      max_pos = aabb.max_pos;
      return *this;
    }
    AABB(float3 _min_pos, float3 _max_pos)
    {
      min_pos = _min_pos;
      max_pos = _max_pos;
    }
    inline float3 size() const
    {
      return max_pos - min_pos;
    }
    inline float3 center() const
    {
      return 0.5f * (min_pos + max_pos);
    }
    inline float volume() const
    {
      return (max_pos.x - min_pos.x) * (max_pos.y - min_pos.y) * (max_pos.z - min_pos.z);
    }
    inline AABB expand(float ratio) const
    {
      return AABB(center() - 0.5f * ratio * size(), center() + 0.5f * ratio * size());
    }

    inline bool contains(const float3 &p) const
    {
      return (p.x >= min_pos.x) && (p.x < max_pos.x) &&
             (p.y >= min_pos.y) && (p.y < max_pos.y) &&
             (p.z >= min_pos.z) && (p.z < max_pos.z);
    }
    inline bool intersects(const AABB &aabb) const
    {
      return (aabb.min_pos.x <= max_pos.x) &&
             (aabb.max_pos.x >= min_pos.x) &&
             (aabb.min_pos.y <= max_pos.y) &&
             (aabb.max_pos.y >= min_pos.y) &&
             (aabb.min_pos.z <= max_pos.z) &&
             (aabb.max_pos.z >= min_pos.z);
    }
    inline bool intersects(const float3 &origin, const float3 &dir, float *t_near = nullptr, float *t_far = nullptr) const
    {
      float3 safe_dir = sign(dir) * max(float3(1e-9f), abs(dir));
      float3 tMin = (min_pos - origin) / safe_dir;
      float3 tMax = (max_pos - origin) / safe_dir;
      float3 t1 = min(tMin, tMax);
      float3 t2 = max(tMin, tMax);
      float tNear = std::max(t1.x, std::max(t1.y, t1.z));
      float tFar = std::min(t2.x, std::min(t2.y, t2.z));

      if (t_near)
        *t_near = tNear;
      if (t_far)
        *t_far = tFar;

      return tNear <= tFar;
    }
    inline bool empty() const
    {
      return min_pos.x >= max_pos.x ||
             min_pos.y >= max_pos.y ||
             min_pos.z >= max_pos.z;
    }
    inline AABB intersect_bbox(const AABB &bbox) const
    {
      return AABB(max(min_pos, bbox.min_pos), min(max_pos, bbox.max_pos));
    }
  };
}