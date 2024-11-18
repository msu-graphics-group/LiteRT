#include "openvdb_common.h"

#ifndef KERNEL_SLICER
#ifndef DISABLE_OPENVDB

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/tools/ChangeBackground.h>
#include <openvdb/tools/FastSweeping.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include "../utils/mesh.h"

void
OpenVDB_Grid::mesh2sdf(void* mesh_ptr, const float voxel_size, const float w)
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> indices;

    auto mesh = (*reinterpret_cast<std::shared_ptr<cmesh4::SimpleMesh> *>(mesh_ptr)).get(); 

    for (auto point: mesh->vPos4f)
    {
        points.push_back(openvdb::Vec3s(point.x, point.y, point.z));
    }

    for (int i = 0; i < mesh->IndicesNum(); i += 3)
    {
        indices.push_back(openvdb::Vec3I(mesh->indices[i], mesh->indices[i + 1], mesh->indices[i + 2]));
    }

    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxel_size);
    auto grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
        *transform,
        points,
        indices,
        w
    );

    grid_ptr = reinterpret_cast<void*>(&grid);
}


OpenVDB_Grid::OpenVDB_Grid() 
{ 
    openvdb::initialize(); 
}

OpenVDB_Grid::~OpenVDB_Grid()
{
    // if (grid_ptr != nullptr)
    // {
    //     delete grid_ptr;
    // }
}

float 
OpenVDB_Grid::get_distance(LiteMath::float3 point)
{
    auto grid = *reinterpret_cast<std::shared_ptr<openvdb::FloatGrid> *>(grid_ptr);
    openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
    openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler> sampler(accessor, grid->transform());

    return sampler.wsSample(openvdb::Vec3f(point.x, point.y, point.z));
}

float 
OpenVDB_Grid::mem_usage() const
{
    auto grid = *reinterpret_cast<std::shared_ptr<openvdb::FloatGrid> *>(grid_ptr);
    return grid->memUsage();
}

uint32_t 
OpenVDB_Grid::get_voxels_count() const
{
    auto grid = *reinterpret_cast<std::shared_ptr<openvdb::FloatGrid> *>(grid_ptr);
    return grid->tree().activeVoxelCount();
}

#endif
#endif