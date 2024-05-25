#pragma once
#include "LiteMath/LiteMath.h"
namespace cmesh4
{
  using LiteMath::int2;
  using LiteMath::int3;
  using LiteMath::int4;
  using LiteMath::uint2;
  using LiteMath::uint3;
  using LiteMath::uint4;

  struct cmpUint2 {
    bool operator()(const uint2& a, const uint2& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      return a.y < b.y;
      
    }
};

struct cmpUint3 {
    bool operator()(const uint3& a, const uint3& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      if (a.y < b.y)
        return true;
      else if (a.y > b.y)
        return false;
      return a.z < b.z;
      
    }
};

struct cmpUint4 {
    bool operator()(const uint4& a, const uint4& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      if (a.y < b.y)
        return true;
      else if (a.y > b.y)
        return false;
      if (a.z < b.z)
        return true;
      else if (a.z > b.z)
        return false;
      return a.w < b.w;
      
    }
  };

    struct cmpInt2 {
    bool operator()(const int2& a, const int2& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      return a.y < b.y;
      
    }
};

struct cmpInt3 {
    bool operator()(const int3& a, const int3& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      if (a.y < b.y)
        return true;
      else if (a.y > b.y)
        return false;
      return a.z < b.z;
      
    }
};

struct cmpInt4 {
    bool operator()(const int4& a, const int4& b) const 
    {
      if (a.x < b.x)
        return true;
      else if (a.x > b.x)
        return false;
      if (a.y < b.y)
        return true;
      else if (a.y > b.y)
        return false;
      if (a.z < b.z)
        return true;
      else if (a.z > b.z)
        return false;
      return a.w < b.w;
      
    }
  };
}