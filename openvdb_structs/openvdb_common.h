#pragma once

#include <iostream>
#include <string>

#include <openvdb/openvdb.h>
#include <openvdb/Grid.h>
#include <openvdb/Types.h>
#include <openvdb/tools/ChangeBackground.h>
#include <openvdb/tools/FastSweeping.h>
#include <openvdb/tools/MeshToVolume.h>
#include <openvdb/tools/VolumeToMesh.h>
#include <openvdb/tools/Interpolation.h>

#include "../utils/mesh.h"

struct OpenVDB_Grid
{
public:
    openvdb::FloatGrid::Ptr sdfGrid;
public:
    OpenVDB_Grid() { openvdb::initialize(); }
    void mesh2sdf(const cmesh4::SimpleMesh& mesh, const float& voxel_size, const float& w);
};