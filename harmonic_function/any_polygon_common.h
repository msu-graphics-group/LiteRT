#pragma once

#include <vector>

#include "CrossRT.h"
#include "LiteMath.h"
#include "render_settings.h"

namespace lm = LiteMath;
using LiteMath::float3;
using LiteMath::float2;
using LiteMath::uint;

static float constexpr SIGNED_SOLID_ANGLE_MIN_VALUE = -4.0 * lm::M_PI;

#ifndef KERNEL_SLICER

struct AnyPolygon {
    std::vector<float3> points;
};

#endif  // !defined(KERNEL_SLICER)

struct AnyPolygonDataHeader {
    uint offset;
    uint size;
};

/**
 * \brief Calculates a signed solid angle of a triangle
 * \param p1 first triangle point
 * \param p2 seconst triangle point
 * \param p3 third triangle point
 * \param point_of_view point of view to calculate solid angle from
 * \return signed solid angle value
 */
inline float triangle_solid_angle(
    float3 p1, float3 p2, float3 p3, float3 point_of_view
) {
    auto const a = p1 - point_of_view;
    auto const b = p2 - point_of_view;
    auto const c = p3 - point_of_view;
    auto const a_len = lm::length(a);
    auto const b_len = lm::length(b);
    auto const c_len = lm::length(c);

    return 2.0 * std::atan2(
                     lm::dot(a, lm::cross(b, c)),
                     a_len * b_len * c_len + c_len * lm::dot(a, b) +
                         a_len * lm::dot(b, c) + b_len * lm::dot(a, c)
                 );
}

inline float3 any_polygon_solid_angle_gradient(
    std::vector<float3> const& points, AnyPolygonDataHeader header,
    float3 point_of_view
) {
    auto result = float3(0.0);

    for (uint i = 0; i < header.size + 1; ++i) {
        auto const left = points[header.offset + i] - point_of_view;
        auto const right =
            points[header.offset + (i + 1) % header.size] - point_of_view;
        auto const cross = lm::cross(left, right);
        auto const cross_len = lm::length(cross);

        result +=
            lm::dot(left - right, lm::normalize(left) - lm::normalize(right)) *
            cross / (cross_len * cross_len);
    }

    return result;
}

inline float any_polygon_solid_angle(
    std::vector<float3> const& triangles, AnyPolygonDataHeader header,
    float3 point_of_view
) {
    auto result = 0.0f;

    for (uint i = 0; i < header.size; ++i) {
        auto const p1 = triangles[header.offset + 3 * i + 0];
        auto const p2 = triangles[header.offset + 3 * i + 1];
        auto const p3 = triangles[header.offset + 3 * i + 2];

        result += triangle_solid_angle(p1, p2, p3, point_of_view);
    }

    return result;
}

/**
 * \brief Calculates a distance between segment and point in 3D
 * \param segment_start first segment point
 * \param segment_end second segment point
 * \param point given point
 * \return the distance
 */
inline float point_segment_distance(
    float3 segment_start, float3 segment_end, float3 point
) {
    static float constexpr EPS = 1.1920929e-7f;

    auto const length_squared =
        lm::dot(segment_end - segment_start, segment_end - segment_start);

    if (length_squared < EPS) {
        return lm::length(point - segment_start);
    }

    auto const t = lm::clamp(
        lm::dot(point - segment_start, segment_end - segment_start) /
            length_squared,
        0.0, 1.0
    );

    auto const projection = segment_start + t * (segment_end - segment_start);

    return lm::length(projection - point);
}

inline float any_polygon_boundary_distance(
    std::vector<float3> const& points, AnyPolygonDataHeader header,
    float3 point_of_view
) {
    auto result = lm::INF_POSITIVE;

    for (lm::uint i = 0; i < header.size + 1; ++i) {
        auto const start = points[header.offset + i];
        auto const end = points[header.offset + (i + 1) % header.size];

        result =
            lm::min(result, point_segment_distance(start, end, point_of_view));
    }

    return result;
}

inline void any_polygon_fill_crt_hit(
    CRT_Hit* hit_ptr, float distance, float2 encoded_normal, uint32_t prim_id,
    uint32_t inst_id, uint32_t geom_id
) {
    hit_ptr->t = distance;
    hit_ptr->primId = prim_id;
    hit_ptr->instId = inst_id;
    hit_ptr->geomId = geom_id | (TYPE_ANY_POLYGON << SH_TYPE);
    hit_ptr->coords[0] = 0.0f;
    hit_ptr->coords[1] = 0.0f;
    hit_ptr->coords[2] = encoded_normal.x;
    hit_ptr->coords[3] = encoded_normal.y;
}
