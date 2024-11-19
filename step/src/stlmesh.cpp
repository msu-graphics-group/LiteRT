#include "stlmesh.h"
#include "LiteMath.h"
#include "stl.h"

namespace cmesh4 {

    SimpleMesh LoadMeshFromSTL(const char* filepath, bool& success) {
        stl_reader::StlMesh<float, unsigned int> stlmesh;
        try {
            stlmesh = stl_reader::StlMesh(filepath);
        } catch (std::exception& e) {
            success = false;
            return SimpleMesh(0, 0);
        }
        success = true;

        cmesh4::SimpleMesh mesh(stlmesh.num_vrts(), SimpleMesh::POINTS_IN_TRIANGLE * stlmesh.num_tris());

        auto rawV = stlmesh.raw_coords();
        for (size_t i = 0; i < mesh.VerticesNum(); i++) {
            float x        = rawV[SimpleMesh::POINTS_IN_TRIANGLE * i + 0];
            float y        = rawV[SimpleMesh::POINTS_IN_TRIANGLE * i + 1];
            float z        = rawV[SimpleMesh::POINTS_IN_TRIANGLE * i + 2];
            mesh.vPos4f[i] = LiteMath::float4(x, y, z, 1);
        }

        /*
        auto rawN = stlmesh.raw_normals();
        for (size_t i = 0; i < mesh.TrianglesNum(); i++) {
            float x = rawN[SimpleMesh::POINTS_IN_TRIANGLE * i + 0];
            float y = rawN[SimpleMesh::POINTS_IN_TRIANGLE * i + 1];
            float z = rawN[SimpleMesh::POINTS_IN_TRIANGLE * i + 2];
            mesh.vNorm4f[i] = LiteMath::float4(x, y, z, 1);
        }
        */

        auto rawI = stlmesh.raw_tris();
        for (size_t i = 0; i < mesh.TrianglesNum(); i++) {
            size_t index        = SimpleMesh::POINTS_IN_TRIANGLE * i + 0;
            mesh.indices[index] = rawI[index];

            index               = SimpleMesh::POINTS_IN_TRIANGLE * i + 1;
            mesh.indices[index] = rawI[index];

            index               = SimpleMesh::POINTS_IN_TRIANGLE * i + 2;
            mesh.indices[index] = rawI[index];
        }

        return mesh;
    }

} // namespace cmesh4
