#include "gltf_utils.h"
#include "vk_utils.h"

LiteMath::float4x4 transformMatrixFromGLTFNode(const tinygltf::Node &node)
{
  LiteMath::float4x4 nodeMatrix;

  if(node.matrix.size() == 16)
  {
    nodeMatrix.set_col(0, float4(node.matrix[0], node.matrix[1], node.matrix[2], node.matrix[3]));
    nodeMatrix.set_col(1, float4(node.matrix[4], node.matrix[5], node.matrix[6], node.matrix[7]));
    nodeMatrix.set_col(2, float4(node.matrix[8], node.matrix[9], node.matrix[10], node.matrix[11]));
    nodeMatrix.set_col(3, float4(node.matrix[12], node.matrix[13], node.matrix[14], node.matrix[15]));
  }
  else
  {
    if(node.scale.size() == 3)
    {
      LiteMath::float3 s = LiteMath::float3(node.scale[0], node.scale[1], node.scale[2]);
      nodeMatrix         = LiteMath::scale4x4(s) * nodeMatrix;
    }
    if(node.rotation.size() == 4)// @TODO: add support for quaternion rotations
    {
//      LiteMath::float4 rot = LiteMath::float4(node.rotation[0], node.rotation[1], node.rotation[2], node.rotation[3]);
      //        nodeMatrix *= LiteMath::quternionRot(rot);
    }
    if(node.translation.size() == 3)
    {
      LiteMath::float3 t = LiteMath::float3(node.translation[0], node.translation[1], node.translation[2]);
      nodeMatrix         = LiteMath::translate4x4(t) * nodeMatrix;
    }
  }

  return nodeMatrix;
}

MaterialData_pbrMR materialDataFromGLTF(const tinygltf::Material &gltfMat)
{
  MaterialData_pbrMR mat = {};
  auto& baseColor = gltfMat.pbrMetallicRoughness.baseColorFactor;
  mat.baseColor   = LiteMath::float4(baseColor[0], baseColor[1], baseColor[2], baseColor[3]);
  mat.alphaCutoff = gltfMat.alphaCutoff;

  if(gltfMat.alphaMode == "OPAQUE")
    mat.alphaMode = 0;
  else if(gltfMat.alphaMode == "MASK")
    mat.alphaMode = 1;
  else if(gltfMat.alphaMode == "BLEND")
    mat.alphaMode = 2;

  mat.metallic    = gltfMat.pbrMetallicRoughness.metallicFactor;
  mat.roughness   = gltfMat.pbrMetallicRoughness.roughnessFactor;
  auto& emission = gltfMat.emissiveFactor;
  mat.emissionColor  = LiteMath::float3(emission[0], emission[1], emission[2]);

  // @TODO: textures
  mat.baseColorTexId = gltfMat.pbrMetallicRoughness.baseColorTexture.index;
  mat.emissionTexId  = gltfMat.emissiveTexture.index;
  mat.normalTexId    = gltfMat.normalTexture.index;
  mat.occlusionTexId = gltfMat.occlusionTexture.index;
  mat.metallicRoughnessTexId = gltfMat.pbrMetallicRoughness.metallicRoughnessTexture.index;

  return mat;
}

void getNumVerticesAndIndicesFromGLTFMesh(const tinygltf::Model &a_model, const tinygltf::Mesh &a_mesh, uint32_t& numVertices, uint32_t& numIndices)
{
  auto numPrimitives   = a_mesh.primitives.size();
  for(size_t j = 0; j < numPrimitives; ++j)
  {
    const tinygltf::Primitive &glTFPrimitive = a_mesh.primitives[j];
    if(glTFPrimitive.attributes.find("POSITION") != glTFPrimitive.attributes.end())
    {
      numVertices += a_model.accessors[glTFPrimitive.attributes.find("POSITION")->second].count;
    }

    numIndices += static_cast<uint32_t>(a_model.accessors[glTFPrimitive.indices].count);
  }
}

cmesh::SimpleMesh simpleMeshFromGLTFMesh(const tinygltf::Model &a_model, const tinygltf::Mesh &a_mesh)
{
  uint32_t numVertices = 0;
  uint32_t numIndices  = 0;
  auto numPrimitives   = a_mesh.primitives.size();

  getNumVerticesAndIndicesFromGLTFMesh(a_model, a_mesh, numVertices, numIndices);

  auto simpleMesh = cmesh::SimpleMesh(numVertices, numIndices);

  uint32_t firstIndex  = 0;
  uint32_t vertexStart = 0;
  for(size_t j = 0; j < numPrimitives; ++j)
  {
    const tinygltf::Primitive &glTFPrimitive = a_mesh.primitives[j];

    // Vertices
    size_t vertexCount = 0;
    {
      const float *positionBuffer  = nullptr;
      const float *normalsBuffer   = nullptr;
      const float *texCoordsBuffer = nullptr;
      const float *tangentsBuffer  = nullptr;

      if(glTFPrimitive.attributes.find("POSITION") != glTFPrimitive.attributes.end())
      {
        const tinygltf::Accessor &accessor = a_model.accessors[glTFPrimitive.attributes.find("POSITION")->second];
        const tinygltf::BufferView &view   = a_model.bufferViews[accessor.bufferView];
        positionBuffer = reinterpret_cast<const float *>(&(a_model.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
        vertexCount                        = accessor.count;
      }

      if(glTFPrimitive.attributes.find("NORMAL") != glTFPrimitive.attributes.end())
      {
        const tinygltf::Accessor &accessor = a_model.accessors[glTFPrimitive.attributes.find("NORMAL")->second];
        const tinygltf::BufferView &view   = a_model.bufferViews[accessor.bufferView];
        normalsBuffer = reinterpret_cast<const float *>(&(a_model.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
      }

      if(glTFPrimitive.attributes.find("TEXCOORD_0") != glTFPrimitive.attributes.end())
      {
        const tinygltf::Accessor &accessor = a_model.accessors[glTFPrimitive.attributes.find("TEXCOORD_0")->second];
        const tinygltf::BufferView &view   = a_model.bufferViews[accessor.bufferView];
        texCoordsBuffer = reinterpret_cast<const float *>(&(a_model.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
      }

      if(glTFPrimitive.attributes.find("TANGENT") != glTFPrimitive.attributes.end())
      {
        const tinygltf::Accessor &accessor = a_model.accessors[glTFPrimitive.attributes.find("TANGENT")->second];
        const tinygltf::BufferView &view   = a_model.bufferViews[accessor.bufferView];
        tangentsBuffer = reinterpret_cast<const float *>(&(a_model.buffers[view.buffer].data[accessor.byteOffset + view.byteOffset]));
      }

      for(size_t v = 0; v < vertexCount; v++)
      {
        simpleMesh.vPos4f[(vertexStart + v) * 4 + 0] = positionBuffer[v * 3 + 0];
        simpleMesh.vPos4f[(vertexStart + v) * 4 + 1] = positionBuffer[v * 3 + 1];
        simpleMesh.vPos4f[(vertexStart + v) * 4 + 2] = positionBuffer[v * 3 + 2];
        simpleMesh.vPos4f[(vertexStart + v) * 4 + 3] = 1.0f;

        simpleMesh.vNorm4f[(vertexStart + v) * 4 + 0] = normalsBuffer ? normalsBuffer[v * 3 + 0] : 0.0f;
        simpleMesh.vNorm4f[(vertexStart + v) * 4 + 1] = normalsBuffer ? normalsBuffer[v * 3 + 1] : 0.0f;
        simpleMesh.vNorm4f[(vertexStart + v) * 4 + 2] = normalsBuffer ? normalsBuffer[v * 3 + 2] : 0.0f;
        simpleMesh.vNorm4f[(vertexStart + v) * 4 + 3] = normalsBuffer ? 1.0f : 0.0f;

        simpleMesh.vTexCoord2f[(vertexStart + v) * 2 + 0] = texCoordsBuffer ? texCoordsBuffer[v * 2 + 0] : 0.0f;
        simpleMesh.vTexCoord2f[(vertexStart + v) * 2 + 1] = texCoordsBuffer ? texCoordsBuffer[v * 2 + 1] : 0.0f;

        simpleMesh.vTang4f[(vertexStart + v) * 4 + 0] = tangentsBuffer ? tangentsBuffer[v * 4 + 0] : 0.0f;
        simpleMesh.vTang4f[(vertexStart + v) * 4 + 1] = tangentsBuffer ? tangentsBuffer[v * 4 + 1] : 0.0f;
        simpleMesh.vTang4f[(vertexStart + v) * 4 + 2] = tangentsBuffer ? tangentsBuffer[v * 4 + 2] : 0.0f;
        simpleMesh.vTang4f[(vertexStart + v) * 4 + 3] = tangentsBuffer ? tangentsBuffer[v * 4 + 3] : 0.0f;
      }
    }

    // Indices
    {
      const tinygltf::Accessor &accessor     = a_model.accessors[glTFPrimitive.indices];
      const tinygltf::BufferView &bufferView = a_model.bufferViews[accessor.bufferView];
      const tinygltf::Buffer &buffer         = a_model.buffers[bufferView.buffer];

      auto indexCount = static_cast<uint32_t>(accessor.count);

      std::fill(simpleMesh.matIndices.begin() + firstIndex / 3,
            simpleMesh.matIndices.begin() + (firstIndex + indexCount) / 3, glTFPrimitive.material);

      switch(accessor.componentType)
      {
      case TINYGLTF_PARAMETER_TYPE_UNSIGNED_INT:
      {
        auto *buf = new uint32_t[accessor.count];
        memcpy(buf, &buffer.data[accessor.byteOffset + bufferView.byteOffset], accessor.count * sizeof(uint32_t));
        for(size_t index = 0; index < accessor.count; index++)
        {
          simpleMesh.indices[firstIndex + index] = buf[index] + vertexStart;
        }
        break;
      }
      case TINYGLTF_PARAMETER_TYPE_UNSIGNED_SHORT:
      {
        auto *buf = new uint16_t[accessor.count];
        memcpy(buf, &buffer.data[accessor.byteOffset + bufferView.byteOffset], accessor.count * sizeof(uint16_t));
        for(size_t index = 0; index < accessor.count; index++)
        {
          simpleMesh.indices[firstIndex + index] = buf[index] + vertexStart;
        }
        break;
      }
      case TINYGLTF_PARAMETER_TYPE_UNSIGNED_BYTE:
      {
        auto *buf = new uint8_t[accessor.count];
        memcpy(buf, &buffer.data[accessor.byteOffset + bufferView.byteOffset], accessor.count * sizeof(uint8_t));
        for(size_t index = 0; index < accessor.count; index++)
        {
          simpleMesh.indices[firstIndex + index] = buf[index] + vertexStart;
        }
        break;
      }
      default:
        vk_utils::logWarning("[LoadSceneGLTF]: Unsupported index component type");
        return { };
      }

      firstIndex  += indexCount;
      vertexStart += vertexCount;
    }
  }

  return simpleMesh;
}