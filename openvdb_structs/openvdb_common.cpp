#include "openvdb_common.h"

void
OpenVDB_Grid::mesh2sdf(const cmesh4::SimpleMesh& mesh, const float& voxel_size, const float& w)
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
    this->sdfGrid = openvdb::tools::meshToLevelSet<openvdb::FloatGrid>(
        *transform,
        points,
        indices,
        w
    );
}