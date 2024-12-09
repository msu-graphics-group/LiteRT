#pragma once
#include "LiteMath/LiteMath.h"
#include <vector>

// enum GraphicsPrimType
// Note: all primitives without its own color should be added before GRAPH_PRIM_POINT_COLOR. Their color is set in header.
static constexpr unsigned GRAPH_PRIM_POINT = 0u;            // point.                                      1 float4 per primitive
static constexpr unsigned GRAPH_PRIM_LINE = 1u;             // line.                                       2 float4 per primitive
static constexpr unsigned GRAPH_PRIM_LINE_SEGMENT = 2u;     // line segment.                               2 float4 per primitive
static constexpr unsigned GRAPH_PRIM_LINE_SEGMENT_DIR = 3u; // directed line segment==vector==arrow.       2 float4 per primitive
static constexpr unsigned GRAPH_PRIM_BOX = 4u;              // box.                                        2 float4 per primitive
static constexpr unsigned GRAPH_PRIM_POINT_COLOR = 5u;            // point.                                2 float4 per primitive
static constexpr unsigned GRAPH_PRIM_LINE_COLOR = 6u;             // line.                                 3 float4 per primitive
static constexpr unsigned GRAPH_PRIM_LINE_SEGMENT_COLOR = 7u;     // line segment.                         3 float4 per primitive
static constexpr unsigned GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR = 8u; // directed line segment==vector==arrow. 3 float4 per primitive
static constexpr unsigned GRAPH_PRIM_BOX_COLOR = 9u;              // box.                                  3 float4 per primitive


struct GraphicsPrimHeader
{
    unsigned prim_type; // enum GraphicsPrimType
    float3 color; // used when prim_type isn't GRAPH_PRIM_*_COLOR
};

#ifndef KERNEL_SLICER
struct GraphicsPrim
{
    GraphicsPrimHeader header;
    std::vector<LiteMath::float4> points; // A vector of points. The way to interpret them is defined by header.prim_type
};

struct GraphicsPrimView
{
    GraphicsPrimView() = default;
    GraphicsPrimView(const GraphicsPrim &graph_prim)
    {
        header = graph_prim.header;
        points = graph_prim.points.data();
        size   = graph_prim.points.size();
    }

    GraphicsPrimHeader header;
    unsigned size;
    const LiteMath::float4 *points;
};
#endif