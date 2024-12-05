#include "marching_cubes.h"
#include "marching_cubes_lookup_table.h"
#include "omp.h"

namespace cmesh4
{
  float3 interp(float3 edgeVertex1, float valueAtVertex1, float3 edgeVertex2, float valueAtVertex2, float isolevel)
  {
    return (edgeVertex1 + (isolevel - valueAtVertex1) * (edgeVertex2 - edgeVertex1)  / (valueAtVertex2 - valueAtVertex1));
  }

  float3 VertexInterp(float isolevel, float3 p1, float3 p2, float valp1, float valp2)
  {
    float mu;
    float3 p;

    if (std::abs(isolevel - valp1) < 0.00001)
      return (p1);
    if (std::abs(isolevel - valp2) < 0.00001)
      return (p2);
    if (std::abs(valp1 - valp2) < 0.00001)
      return (p1);
    mu = (isolevel - valp1) / (valp2 - valp1);
    p.x = p1.x + mu * (p2.x - p1.x);
    p.y = p1.y + mu * (p2.y - p1.y);
    p.z = p1.z + mu * (p2.z - p1.z);

    return (p);
  }

  cmesh4::SimpleMesh create_mesh_marching_cubes(MarchingCubesSettings settings, MultithreadedDensityFunction density, unsigned max_threads)
  {
    float3 size = settings.max_pos - settings.min_pos;
    std::vector<std::vector<float3>> vertices(max_threads);
    std::vector<std::vector<float3>> normals(max_threads);

    #pragma omp parallel for num_threads(max_threads) 
    for (int thread_id = 0; thread_id < max_threads; thread_id++)
    {
    unsigned steps = (settings.size.x + max_threads - 1) / max_threads;
    unsigned start = thread_id * steps;
    unsigned end = std::min((thread_id + 1) * steps, settings.size.x);
    for (int xi = start; xi < end; xi++)
    {
      float cubeValues[8];
      float3 cubePositions[8];
      float3 vertlist[12];
      for (int yi = 0; yi < settings.size.y; yi++)
      {
        for (int zi = 0; zi < settings.size.z; zi++)
        {
          unsigned cubeIndex = 0;
          for (int l = 0; l < 8; l++)
          {
            float3 p = settings.min_pos + size*((float3(xi, yi, zi) + pOffsets[l]) / float3(settings.size));
            cubePositions[l] = p;
            cubeValues[l] = density(p, thread_id);
          }

          for (int l = 0; l < 8; l++)
          {
            if (cubeValues[l] < settings.iso_level)
              cubeIndex |= (1 << l);
          }

          if (edgeTable[cubeIndex] == 0)
            continue;
          
          /* Find the vertices where the surface intersects the cube */
          if (edgeTable[cubeIndex] & 1)
            vertlist[0] = VertexInterp(settings.iso_level, cubePositions[0], cubePositions[1], cubeValues[0], cubeValues[1]);
          if (edgeTable[cubeIndex] & 2)
            vertlist[1] = VertexInterp(settings.iso_level, cubePositions[1], cubePositions[2], cubeValues[1], cubeValues[2]);
          if (edgeTable[cubeIndex] & 4)
            vertlist[2] = VertexInterp(settings.iso_level, cubePositions[2], cubePositions[3], cubeValues[2], cubeValues[3]);
          if (edgeTable[cubeIndex] & 8)
            vertlist[3] = VertexInterp(settings.iso_level, cubePositions[3], cubePositions[0], cubeValues[3], cubeValues[0]);
          if (edgeTable[cubeIndex] & 16)
            vertlist[4] = VertexInterp(settings.iso_level, cubePositions[4], cubePositions[5], cubeValues[4], cubeValues[5]);
          if (edgeTable[cubeIndex] & 32)
            vertlist[5] = VertexInterp(settings.iso_level, cubePositions[5], cubePositions[6], cubeValues[5], cubeValues[6]);
          if (edgeTable[cubeIndex] & 64)
            vertlist[6] = VertexInterp(settings.iso_level, cubePositions[6], cubePositions[7], cubeValues[6], cubeValues[7]);
          if (edgeTable[cubeIndex] & 128)
            vertlist[7] = VertexInterp(settings.iso_level, cubePositions[7], cubePositions[4], cubeValues[7], cubeValues[4]);
          if (edgeTable[cubeIndex] & 256)
            vertlist[8] = VertexInterp(settings.iso_level, cubePositions[0], cubePositions[4], cubeValues[0], cubeValues[4]);
          if (edgeTable[cubeIndex] & 512)
            vertlist[9] = VertexInterp(settings.iso_level, cubePositions[1], cubePositions[5], cubeValues[1], cubeValues[5]);
          if (edgeTable[cubeIndex] & 1024)
            vertlist[10] = VertexInterp(settings.iso_level, cubePositions[2], cubePositions[6], cubeValues[2], cubeValues[6]);
          if (edgeTable[cubeIndex] & 2048)
            vertlist[11] = VertexInterp(settings.iso_level, cubePositions[3], cubePositions[7], cubeValues[3], cubeValues[7]);

          float3 worldPos = settings.min_pos + size*(float3(xi, yi, zi) / float3(settings.size));
          const int *edges = triTable[cubeIndex];
          
          for (int i = 0; edges[i] != -1; i++)
          {
            float3 p = vertlist[edges[i]];

            const float h = 0.0001f;
            float dx = (density(p + float3(h, 0, 0), thread_id) - density(p - float3(h, 0, 0), thread_id)) / (2*h);
            float dy = (density(p + float3(0, h, 0), thread_id) - density(p - float3(0, h, 0), thread_id)) / (2*h);
            float dz = (density(p + float3(0, 0, h), thread_id) - density(p - float3(0, 0, h), thread_id)) / (2*h);
            float3 n = -normalize(float3(dx, dy, dz) + float3(1e-8f, 1e-8f, 1e-8f));

            vertices[thread_id].push_back(p);
            normals[thread_id].push_back(n);
          }
        }
      }
    }
    }

    cmesh4::SimpleMesh mesh;
    unsigned mesh_size = 0;
    for (auto &vvec : vertices)
      mesh_size += vvec.size();

    mesh.vPos4f.reserve(mesh_size);
    mesh.vNorm4f.reserve(mesh_size);
    mesh.vTexCoord2f.reserve(mesh_size);
    mesh.vTang4f.reserve(mesh_size);

    mesh.matIndices.reserve(mesh_size/3);
    mesh.indices.reserve(mesh_size);

    for (int vvec=0; vvec<vertices.size(); vvec++)
    {
      for (int i=0;i<vertices[vvec].size();i+=3)
      {
        float3 a = vertices[vvec][i];
        float3 b = vertices[vvec][i+1];
        float3 c = vertices[vvec][i+2];
        float3 n1 = normals[vvec][i];
        float3 n2 = normals[vvec][i+1];
        float3 n3 = normals[vvec][i+2];

        float3 tang1 = LiteMath::normalize(LiteMath::cross(n1, a - b));
        float3 tang2 = LiteMath::normalize(LiteMath::cross(n2, b - c));
        float3 tang3 = LiteMath::normalize(LiteMath::cross(n3, c - a));

        float2 tc = float2(0, 0);

        mesh.vPos4f.push_back(LiteMath::to_float4(a, 1));
        mesh.vPos4f.push_back(LiteMath::to_float4(b, 1));
        mesh.vPos4f.push_back(LiteMath::to_float4(c, 1));

        mesh.vNorm4f.push_back(LiteMath::to_float4(n1, 0));
        mesh.vNorm4f.push_back(LiteMath::to_float4(n2, 0));
        mesh.vNorm4f.push_back(LiteMath::to_float4(n3, 0));

        mesh.vTang4f.push_back(LiteMath::to_float4(tang1, 0));
        mesh.vTang4f.push_back(LiteMath::to_float4(tang2, 0));
        mesh.vTang4f.push_back(LiteMath::to_float4(tang3, 0));

        mesh.vTexCoord2f.push_back(tc);
        mesh.vTexCoord2f.push_back(tc);
        mesh.vTexCoord2f.push_back(tc);

        mesh.matIndices.push_back(0);

        mesh.indices.push_back(mesh.indices.size());
        mesh.indices.push_back(mesh.indices.size());
        mesh.indices.push_back(mesh.indices.size());
      }
    }

    printf("Marching Cubes: %u vertices, %u triangles\n", (unsigned)mesh.vPos4f.size(), (unsigned)mesh.indices.size()/3);

    return mesh;
  }
}