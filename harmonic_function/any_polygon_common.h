#pragma once

#include <vector>

#include "CrossRT.h"
#include "LiteMath.h"
#include "render_settings.h"

namespace lm = LiteMath;
using LiteMath::float2;
using LiteMath::float3;
using LiteMath::uint;

#ifndef KERNEL_SLICER

class AnyPolygon {
public:
    AnyPolygon() = default;
    ~AnyPolygon() = default;
    AnyPolygon(AnyPolygon const&) = default;
    AnyPolygon(AnyPolygon&&) noexcept = default;
    AnyPolygon& operator=(AnyPolygon const&) = default;
    AnyPolygon& operator=(AnyPolygon&&) noexcept = default;

    inline explicit AnyPolygon(std::vector<float3> points)
        : m_vertices{std::move(points)}
        , m_middle_point{} {
        for (auto const point : m_vertices) {
            m_middle_point += point;
        }

        m_middle_point /= (float) m_vertices.size();
    }

    bool is_empty() const noexcept { return m_vertices.empty(); }

    std::vector<float3> const& vertices() const noexcept { return m_vertices; }

    float3 middle() const noexcept { return m_middle_point; }

private:
    std::vector<float3> m_vertices{};
    float3 m_middle_point{};
};

#endif  // !defined(KERNEL_SLICER)

struct AnyPolygonDataHeader {
    uint offset;
    uint size;
};

inline float2 atan2_sum_args(float2 lhs, float2 rhs) {
    return float2{lhs.x * rhs.x - lhs.y * rhs.y, lhs.x * rhs.y + lhs.y * rhs.x};
}

inline float2 triangle_solid_angle_atan2_args(
    float3 p1, float3 p2, float3 p3, float3 point_of_view
) {
    auto const a = p1 - point_of_view;
    auto const b = p2 - point_of_view;
    auto const c = p3 - point_of_view;
    auto const a_len = lm::length(a);
    auto const b_len = lm::length(b);
    auto const c_len = lm::length(c);

    return float2{
        a_len * b_len * c_len + c_len * lm::dot(a, b) + a_len * lm::dot(b, c) +
            b_len * lm::dot(a, c),
        lm::dot(a, lm::cross(b, c))
    };
}

inline float3 any_polygon_solid_angle_gradient(
    std::vector<float3> const& points, AnyPolygonDataHeader header,
    float3 point_of_view
) {
    auto result = float3{0.0};

    for (uint i = 0; i < header.size; ++i) {
        auto const left = points[header.offset + i] - point_of_view;
        auto const right =
            points[header.offset + (i + 1) % header.size] - point_of_view;
        auto const cross = lm::cross(right, left);
        auto const cross_len_sqr = lm::dot(cross, cross);

        result +=
            cross / cross_len_sqr *
            ((-lm::dot(left, right) + lm::dot(left, left)) / lm::length(left) +
             (-lm::dot(left, right) + lm::dot(right, right)) / lm::length(right)
            );
    }

    return result;
}

inline float any_polygon_solid_angle(
    std::vector<float3> const& triangles, AnyPolygonDataHeader header,
    float3 point_of_view
) {
    auto running_angle = float2{1.0f, 0.0f};

    for (uint i = 0; i < header.size; ++i) {
        auto const p1 = triangles[header.offset + 3 * i + 0];
        auto const p2 = triangles[header.offset + 3 * i + 1];
        auto const p3 = triangles[header.offset + 3 * i + 2];

        auto const solid_angle_args =
            triangle_solid_angle_atan2_args(p1, p2, p3, point_of_view);
        running_angle = atan2_sum_args(running_angle, solid_angle_args);
    }

    return 2.0 * std::atan2(running_angle.y, running_angle.x);
}

/**
 * \brief Calculates the height vector from point to segment
 * \param segment_start first segment point
 * \param segment_end second segment point
 * \param point given point
 * \return height vector
 */
inline float3 point_segment_height(
    float3 segment_start, float3 segment_end, float3 point
) {
    static float constexpr EPS = 1.1920929e-7f;

    auto const length_squared =
        lm::dot(segment_end - segment_start, segment_end - segment_start);

    if (length_squared < EPS) {
        return point - segment_start;
    }

    auto const t = lm::clamp(
        lm::dot(point - segment_start, segment_end - segment_start) /
            length_squared,
        0.0, 1.0
    );

    auto const projection = segment_start + t * (segment_end - segment_start);

    return projection - point;
}

inline float any_polygon_boundary_distance(
    std::vector<float3> const& points, AnyPolygonDataHeader header,
    float3 point_of_view
) {
    auto result_squared = lm::INF_POSITIVE;

    for (uint i = 0; i < header.size; ++i) {
        auto const start = points[header.offset + i];
        auto const end = points[header.offset + (i + 1) % header.size];

        auto const height = point_segment_height(start, end, point_of_view);

        result_squared = lm::min(result_squared, lm::dot(height, height));
    }

    return std::sqrt(result_squared);
}

inline void any_polygon_fill_crt_hit(
    CRT_Hit* hit_ptr, float distance, float2 encoded_normal, uint32_t prim_id,
    uint32_t inst_id, uint32_t geom_id
) {
    if (nullptr == hit_ptr) {
        return;
    }

    hit_ptr->t = distance;
    hit_ptr->primId = prim_id;
    hit_ptr->instId = inst_id;
    hit_ptr->geomId = geom_id | (TYPE_ANY_POLYGON << SH_TYPE);
    hit_ptr->coords[0] = 0.0f;
    hit_ptr->coords[1] = 0.0f;
    hit_ptr->coords[2] = encoded_normal.x;
    hit_ptr->coords[3] = encoded_normal.y;
}
