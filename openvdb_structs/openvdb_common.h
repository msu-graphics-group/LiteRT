#pragma once

#include <iostream>
#include <string>

#ifndef KERNEL_SLICER
#ifndef DISABLE_OPENVDB

#include <LiteMath.h>
using LiteMath::float3;

struct OpenVDB_Grid
{
public:
    void* grid_ptr;
public:
    OpenVDB_Grid();
    ~OpenVDB_Grid();
    float get_distance(LiteMath::float3 point);
    void mesh2sdf(void* mesh_ptr, const float voxel_size, const float w);
    float mem_usage() const;
    uint32_t get_voxels_count() const; 
};

#endif
#else
struct OpenVDB_Grid
{
    bool foo;
};
#endif