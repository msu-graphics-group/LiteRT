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


void load_mesh(const std::string& path);
void create_sdf4Mesh();

struct OpenVDB_GRID
{

};