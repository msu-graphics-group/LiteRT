#include "demo_meshes.h"

namespace cmesh4_demo
{
  using LiteMath::float4;
  using LiteMath::float3;
  using LiteMath::float2;
  using LiteMath::uint3;

  struct RawVertex
  {
    RawVertex() = default;
    RawVertex(float3 a_pos, float2 a_tc)
    {
      pos = a_pos;
      tc = a_tc;
    }
    float3 pos;
    float2 tc;
  };

  static cmesh4::SimpleMesh create_mesh(const std::vector<RawVertex>& a_vertices, std::vector<uint3>& a_triangles,
                                        const std::vector<unsigned>& a_materials,
                                        VerticesType a_verticesType, NormalsType a_normalsType, bool fix_normals, float3 center)  
  {
    std::vector<unsigned> materials = a_materials;
    assert(a_triangles.size() == materials.size() || materials.size() == 0);
    
    if (materials.size() == 0)
      materials.resize(a_triangles.size(), 0);

    //#1 swap indices in triangle to make geometry normals point outward
    if (fix_normals)
    {
      for (uint32_t i = 0; i < a_triangles.size(); i++)
      {
        uint3 tri = a_triangles[i];
        float3 a = a_vertices[tri.x].pos;
        float3 b = a_vertices[tri.y].pos;
        float3 c = a_vertices[tri.z].pos;
        float3 normal = LiteMath::cross(b - a, c - a);
        if (LiteMath::dot(normal, a - center) < 0)
        {
          unsigned t = tri.z;
          tri.z = tri.y;
          tri.y = t;
        }
        a_triangles[i] = tri;
      }
    }

    //#2 define proper vertex normals and tangents
    std::vector<float3> normals(a_vertices.size(), float3(0,0,0));
    std::vector<float3> tangents(a_vertices.size(), float3(0,0,0));
    std::vector<float> counts(a_vertices.size(), 0);
    for (uint32_t i = 0; i < a_triangles.size(); i++)
    {
      uint3 tri = a_triangles[i];
      float3 a = a_vertices[tri.x].pos;
      float3 b = a_vertices[tri.y].pos;
      float3 c = a_vertices[tri.z].pos;
      float3 normal = LiteMath::normalize(LiteMath::cross(b - a, c - a));
      //printf("triangle %u: a: %f %f %f b: %f %f %f c: %f %f %f normal: %f %f %f\n", i, a.x, a.y, a.z, b.x, b.y, b.z, c.x, c.y, c.z, normal.x, normal.y, normal.z);

      normals[tri.x] += normal;
      normals[tri.y] += normal;
      normals[tri.z] += normal;
      tangents[tri.x] += LiteMath::normalize(b - a);
      tangents[tri.y] += LiteMath::normalize(b - a);
      tangents[tri.z] += LiteMath::normalize(b - a);
      counts[tri.x]++;
      counts[tri.y]++;
      counts[tri.z]++;
    }
    for (uint32_t i = 0; i < normals.size(); i++)
    {
      if (counts[i] > 0)
      {
        normals[i] /= counts[i];
        tangents[i] /= counts[i];
      }
      else
      {
        normals[i] = float3(0,0,1);
        tangents[i] = float3(1,0,0);
        printf("WARNING: vertex %d is not used by any triangle\n", i);
      }
    }

    cmesh4::SimpleMesh mesh;
    if (a_verticesType == VerticesType::SHARED)
    {
      mesh.vPos4f.resize(a_vertices.size());
      mesh.vNorm4f.resize(a_vertices.size());
      mesh.vTang4f.resize(a_vertices.size());
      mesh.vTexCoord2f.resize(a_vertices.size());

      mesh.indices.resize(a_triangles.size() * 3);
      mesh.matIndices.resize(a_triangles.size());

      for (uint32_t i = 0; i < a_vertices.size(); i++)
      {
        mesh.vPos4f[i] = LiteMath::to_float4(a_vertices[i].pos, 1);
        mesh.vNorm4f[i] = LiteMath::to_float4(normals[i], 0);
        mesh.vTang4f[i] = LiteMath::to_float4(tangents[i], 0);
        mesh.vTexCoord2f[i] = a_vertices[i].tc;
      }

      for (uint32_t i = 0; i < a_triangles.size(); i++)
      {
        mesh.indices[i * 3 + 0] = a_triangles[i].x;
        mesh.indices[i * 3 + 1] = a_triangles[i].y;
        mesh.indices[i * 3 + 2] = a_triangles[i].z;
        mesh.matIndices[i] = materials[i];
      }
    }
    else
    {
      mesh.vPos4f.resize(a_triangles.size() * 3);
      mesh.vNorm4f.resize(a_triangles.size() * 3);
      mesh.vTang4f.resize(a_triangles.size() * 3);
      mesh.vTexCoord2f.resize(a_triangles.size() * 3);

      mesh.indices.resize(a_triangles.size() * 3);
      mesh.matIndices.resize(a_triangles.size());

      for (uint32_t i = 0; i < a_triangles.size(); i++)
      {
        uint3 tri = a_triangles[i];
        float3 a = a_vertices[tri.x].pos;
        float3 b = a_vertices[tri.y].pos;
        float3 c = a_vertices[tri.z].pos;
        float3 geom_normal = LiteMath::normalize(LiteMath::cross(b - a, c - a));
        float3 geom_tangent = LiteMath::normalize(b - a);

        mesh.vPos4f[i * 3 + 0] = LiteMath::to_float4(a, 1);
        mesh.vPos4f[i * 3 + 1] = LiteMath::to_float4(b, 1);
        mesh.vPos4f[i * 3 + 2] = LiteMath::to_float4(c, 1);

        mesh.vNorm4f[i * 3 + 0] = LiteMath::to_float4(a_normalsType == NormalsType::VERTEX ? normals[tri.x] : geom_normal, 0);
        mesh.vNorm4f[i * 3 + 1] = LiteMath::to_float4(a_normalsType == NormalsType::VERTEX ? normals[tri.y] : geom_normal, 0);
        mesh.vNorm4f[i * 3 + 2] = LiteMath::to_float4(a_normalsType == NormalsType::VERTEX ? normals[tri.z] : geom_normal, 0);

        mesh.vTang4f[i * 3 + 0] = LiteMath::to_float4(a_normalsType == NormalsType::VERTEX ? tangents[tri.x] : geom_tangent, 0);
        mesh.vTang4f[i * 3 + 1] = LiteMath::to_float4(a_normalsType == NormalsType::VERTEX ? tangents[tri.y] : geom_tangent, 0);
        mesh.vTang4f[i * 3 + 2] = LiteMath::to_float4(a_normalsType == NormalsType::VERTEX ? tangents[tri.z] : geom_tangent, 0);

        mesh.vTexCoord2f[i * 3 + 0] = a_vertices[tri.x].tc;
        mesh.vTexCoord2f[i * 3 + 1] = a_vertices[tri.y].tc;
        mesh.vTexCoord2f[i * 3 + 2] = a_vertices[tri.z].tc;

        mesh.indices[i * 3 + 0] = i * 3 + 0;
        mesh.indices[i * 3 + 1] = i * 3 + 1;
        mesh.indices[i * 3 + 2] = i * 3 + 2;

        mesh.matIndices[i] = materials[i];
      }
    }
    return mesh;
  }

  cmesh4::SimpleMesh create_cube(float3 scale, bool six_materials, VerticesType a_verticesType, NormalsType a_normalsType)
  {
    float3 c(0,0,0);

    std::vector<RawVertex> vertices = {
      RawVertex(c + float3(-1,-1,-1) * scale, float2(0,0)),
      RawVertex(c + float3(1,-1,-1) * scale, float2(1,0)),
      RawVertex(c + float3(1,1,-1) * scale, float2(1,1)),
      RawVertex(c + float3(-1,1,-1) * scale, float2(0,1)),
      RawVertex(c + float3(-1,-1,1) * scale, float2(0,0)),
      RawVertex(c + float3(1,-1,1) * scale, float2(1,0)),
      RawVertex(c + float3(1,1,1) * scale, float2(1,1)),
      RawVertex(c + float3(-1,1,1) * scale, float2(0,1))
    };

    std::vector<uint3> triangles = {
      uint3(0,1,2),
      uint3(0,2,3),
      uint3(1,5,6),
      uint3(1,6,2),

      uint3(5,4,7),
      uint3(5,7,6),
      uint3(4,0,3),
      uint3(4,3,7),

      uint3(0,4,5),
      uint3(0,5,1),
      uint3(3,2,6),
      uint3(3,6,7),
    };

    std::vector<uint32_t> materials;
    if (six_materials)
    {
      materials = {0,0, 1,1, 2,2, 3,3, 4,4, 5,5};
    }
    else
    {
      materials = {0,0,0,0,0,0,0,0,0,0,0,0};
    }

    return create_mesh(vertices, triangles, materials, a_verticesType, a_normalsType, true, c);
  }

  cmesh4::SimpleMesh create_tetrahedron(float3 scale, VerticesType a_verticesType, NormalsType a_normalsType)
  {
    return cmesh4::SimpleMesh();
  }

  cmesh4::SimpleMesh create_cylinder(float3 scale, unsigned c_segments, unsigned h_segments, 
                                     VerticesType a_verticesType, NormalsType a_normalsType)
  {
    float3 c(0,0,0);
    float d = scale.y;
    float r = std::min(scale.x, scale.z);
    float delta = 0.0;

    std::vector<RawVertex> vertices;
    std::vector<uint3> triangles;
    unsigned prev_start = 0;
    for (int i=0;i<=h_segments;i++)
    {
      unsigned start = vertices.size();

      for (int j=0;j<c_segments;j++)
      {
        float angle = float(j) / float(c_segments) * 2.0f * LiteMath::M_PI;
        float x = r*std::cos(angle);
        float y = d*(-1 + 2*float(i) / float(h_segments));
        float z = r*std::sin(angle);

        vertices.push_back(RawVertex(c + float3(x,y,z), float2(angle/(2*LiteMath::M_PI),0.25 + 0.5*float(i) / float(h_segments))));
      }

      if (i == 0 || i == h_segments)
      {
        //smooth border from side to cap
        for (int j=0;j<c_segments;j++)
          vertices.push_back(RawVertex(vertices[start + j].pos*float3(1-delta,1+delta,1-delta), vertices[start + j].tc + float2(0,delta*(i-0.5))));
        for (int j=1;j<=c_segments;j++)
        {
          //triangles.push_back(uint3(start + j - 1, start + j%c_segments, start+c_segments + j - 1));
          //triangles.push_back(uint3(start + j%c_segments, start+c_segments + j%c_segments, start+c_segments + j - 1));
        }

        if (i == 0)
          vertices.push_back(RawVertex(c + float3(0,-d*(1+delta),0), float2(0.5,0)));
        else
          vertices.push_back(RawVertex(c + float3(0,d*(1+delta),0), float2(0.5,1.0)));
      
        for (int j=1;j<=c_segments;j++)
        {
          triangles.push_back(uint3(start + c_segments + j - 1, start + c_segments + j%c_segments, vertices.size()-1));
        }
      }

      if (i > 0)
      {
        for (int j=1;j<=c_segments;j++)
        {
          triangles.push_back(uint3(prev_start + j - 1, prev_start + j%c_segments, start + j - 1));
          triangles.push_back(uint3(prev_start + j%c_segments, start + j%c_segments, start + j - 1));
        }
      }
      prev_start = start;
    }


    std::vector<uint32_t> materials(triangles.size(), 0);
    return create_mesh(vertices, triangles, materials, a_verticesType, a_normalsType, true, c);
  }
}