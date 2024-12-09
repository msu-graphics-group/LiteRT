#pragma once
#include <string>
#include <testing_framework/helpers/source_location.h>
#include "utils/mesh/mesh.h"

namespace testing
{

    using Mesh = cmesh4::SimpleMesh;

    /*
        Loads mesh from file
        If there is no file, skips test
    */
    Mesh load_vsgf_mesh_by_path(
        const std::string&path,
        source_location loc = source_location::current()
    );

    /*
        Fits mesh inside bounding box
    */
    void fit_mesh(Mesh&mesh, LiteMath::float3 min, LiteMath::float3 max);

    /*
        Loads mesh from <scenes-dir>/<name>
        Fits mesh into bounding box [-box, box]^3
        If there is no file, skips test
    */
    Mesh load_vsgf_mesh(
        const std::string&name,
        float box = 0.9,
        source_location loc = source_location::current()
    );

}