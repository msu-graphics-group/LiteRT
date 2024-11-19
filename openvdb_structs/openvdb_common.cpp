#include "openvdb_common.h"

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/tools/ChangeBackground.h>
#include <openvdb/tools/FastSweeping.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

void
OpenVDB_Grid::mesh2sdf(const cmesh4::SimpleMesh& mesh, const float voxel_size, const float w)
{
    std::vector<openvdb::Vec3s> points;
    std::vector<openvdb::Vec3I> indices;

    for (auto point: mesh.vPos4f)
    {
        points.push_back(openvdb::Vec3s(point.x, point.y, point.z));
    }

    for (int i = 0; i < mesh.IndicesNum(); i += 3)
    {
        indices.push_back(openvdb::Vec3I(mesh.indices[i], mesh.indices[i + 1], mesh.indices[i + 2]));
    }

    openvdb::math::Transform::Ptr transform = openvdb::math::Transform::createLinearTransform(voxel_size);
    auto grid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
        *transform,
        points,
        indices,
        w
    );

    openvdb::FloatGrid::Ptr* ptr = new decltype(grid)(grid);
    grid_ptr = (void*)(ptr);
}


OpenVDB_Grid::OpenVDB_Grid() 
{ 
    openvdb::initialize(); 
}

OpenVDB_Grid::~OpenVDB_Grid()
{
    // auto *ptr = (openvdb::FloatGrid::Ptr*)(grid_ptr);

    // if (ptr != nullptr)
    // {
    //     delete ptr;
    // }
}

void* 
OpenVDB_Grid::create_samler()
{
    auto grid = *(openvdb::FloatGrid::Ptr*)(grid_ptr);
    openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
    openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler> sampler(accessor, grid->transform());

    openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler>* ptr = new decltype(sampler)(sampler);
    return (void*)(ptr);
}

float 
OpenVDB_Grid::get_distance(LiteMath::float3 point, void* sampler_ptr)
{
    auto grid = *(openvdb::FloatGrid::Ptr*)(grid_ptr);
    // auto sampler = *(openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler>*)(sampler_ptr);
    openvdb::FloatGrid::ConstAccessor accessor = grid->getConstAccessor();
    openvdb::tools::GridSampler<openvdb::FloatGrid::ConstAccessor, openvdb::tools::BoxSampler> sampler(accessor, grid->transform());

    return sampler.wsSample(openvdb::Vec3f(point.x, point.y, point.z));
}

float 
OpenVDB_Grid::mem_usage() const
{
    auto grid = *(openvdb::FloatGrid::Ptr*)(grid_ptr);
    return grid->memUsage();
}

uint32_t 
OpenVDB_Grid::get_voxels_count() const
{
    auto grid = *(openvdb::FloatGrid::Ptr*)(grid_ptr);
    return grid->tree().activeVoxelCount();
}