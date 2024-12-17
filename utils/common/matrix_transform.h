#pragma once
#include "LiteMath_ext.h"
#include <cassert>

namespace LiteMath
{
  static inline float4x4 eulerAngleXYZ(float x_angle, float y_angle, float z_angle)
  {
    float c1 = cos(-x_angle);
    float c2 = cos(-y_angle);
    float c3 = cos(-z_angle);
    float s1 = sin(-x_angle);
    float s2 = sin(-y_angle);
    float s3 = sin(-z_angle);

    float4x4 v1;
    v1(0,0) = c2 * c3;
    v1(1,0) = -c1 * s3 + s1 * s2 * c3;
    v1(2,0) = s1 * s3 + c1 * s2 * c3;
    v1(3,0) = 0;
    v1(0,1) = c2 * s3;
    v1(1,1) = c1 * c3 + s1 * s2 * s3;
    v1(2,1) = -s1 * c3 + c1 * s2 * s3;
    v1(3,1) = 0;
    v1(0,2) = -s2;
    v1(1,2) = s1 * c2;
    v1(2,2) = c1 * c2;
    v1(3,2) = 0;
    v1(0,3) = 0;
    v1(1,3) = 0;
    v1(2,3) = 0;
    v1(3,3) = 1;

    return v1;
  }

  static inline float4x4 lookAtRH(const float3 &eye, const float3 &center, const float3 &up)
	{
		float3 f(normalize(center - eye));
		float3 s(normalize(cross(f, up)));
		float3 u(cross(s, f));

		float4x4 Result;
		Result(0,0) = s.x;
		Result(0,1) = s.y;
		Result(0,2) = s.z;
		Result(1,0) = u.x;
		Result(1,1) = u.y;
		Result(1,2) = u.z;
		Result(2,0) =-f.x;
		Result(2,1) =-f.y;
		Result(2,2) =-f.z;

		Result(0,3) =-dot(s, eye);
		Result(1,3) =-dot(u, eye);
		Result(2,3) = dot(f, eye);

    return Result;
	}

  //perspective projection with right-handed coordinate system to unit cube [-1,1]^3
  //fov in radians
  static inline float4x4 perspectiveRH_NO(float fovy, float aspect, float zNear, float zFar)
	{
    assert(aspect > 0.001f);
		assert(fovy > 0.001f);
    assert(zNear > 0);
    assert(zFar > zNear);

		float tanHalfFovy = tan(fovy / 2.0f);

		float4x4 Result(0,0,0,0, 0,0,0,0, 0,0,0,0, 0,0,0,0);
		Result(0,0) = 1.0f / (aspect * tanHalfFovy);
		Result(1,1) = 1.0f / (tanHalfFovy);
		Result(2,2) = - (zFar + zNear) / (zFar - zNear);
		Result(3,2) = - 1.0f;
		Result(2,3) = - (2.0f * zFar * zNear) / (zFar - zNear);
		return Result;
	}

  //perspective projection
  static inline float4x4 perspective(float fovy, float aspect, float zNear, float zFar)
  {
    return perspectiveRH_NO(fovy, aspect, zNear, zFar);
  }

  //orthographic projection with right-handed coordinate system to unit cube [-1,1]^3
  static inline float4x4 orthoRH_NO(float left, float right, float bottom, float top, float zNear, float zFar)
	{
		float4x4 Result;
		Result(0,0) =  2.0f / (right - left);
		Result(1,1) =  2.0f / (top - bottom);
		Result(2,2) = -2.0f / (zFar - zNear);
		Result(0,3) = - (right + left) / (right - left);
		Result(1,3) = - (top + bottom) / (top - bottom);
		Result(2,3) = - (zFar + zNear) / (zFar - zNear);
		return Result;
	}

  //orthographic projection
  static inline float4x4 ortho(float left, float right, float bottom, float top, float zNear, float zFar)
	{
    return orthoRH_NO(left, right, bottom, top, zNear, zFar);
  }

  static inline float4x4 rotate(const float4x4 &m, float angle, const float3 &v)
	{
		float a = angle;
		float c = cos(a);
		float s = sin(a);

		float3 axis(normalize(v));
		float3 temp((1.0f - c) * axis);

		float4x4 Rotate;
		Rotate(0,0) = c + temp[0] * axis[0];
		Rotate(1,0) = temp[0] * axis[1] + s * axis[2];
		Rotate(2,0) = temp[0] * axis[2] - s * axis[1];

		Rotate(0,1) = temp[1] * axis[0] - s * axis[2];
		Rotate(1,1) = c + temp[1] * axis[1];
		Rotate(2,1) = temp[1] * axis[2] + s * axis[0];

		Rotate(0,2) = temp[2] * axis[0] + s * axis[1];
		Rotate(1,2) = temp[2] * axis[1] - s * axis[0];
		Rotate(2,2) = c + temp[2] * axis[2];

		float4x4 Result;
		Result.set_col(0, m.get_col(0) * Rotate(0,0) + m.get_col(1) * Rotate(1,0) + m.get_col(2) * Rotate(2,0));
		Result.set_col(1, m.get_col(0) * Rotate(0,1) + m.get_col(1) * Rotate(1,1) + m.get_col(2) * Rotate(2,1));
		Result.set_col(2, m.get_col(0) * Rotate(0,2) + m.get_col(1) * Rotate(1,2) + m.get_col(2) * Rotate(2,2));
		Result.set_col(3, m.get_col(3));
		
    return Result;
	}

  static inline float4x4 translate(float4x4 const& m, float3 const& v)
	{
		float4x4 Result(m);
		Result.set_col(3, m.get_col(0) * v[0] + m.get_col(1) * v[1] + m.get_col(2) * v[2] + m.get_col(3));
    return Result;
	}

  static inline float4x4 scale(float4x4 const& m, float3 const& v)
	{
		float4x4 Result;
		Result.set_col(0, m.get_col(0) * v[0]);
		Result.set_col(1, m.get_col(1) * v[1]);
		Result.set_col(2, m.get_col(2) * v[2]);
		Result.set_col(3, m.get_col(3));
    return Result;
	}
}