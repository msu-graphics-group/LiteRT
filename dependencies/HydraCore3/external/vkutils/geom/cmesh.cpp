#include "cmesh.h"

#include <cmath>
#include <cstring>
#include <fstream>

#include "vk_utils.h"

std::vector<unsigned int> CreateQuadTriIndices(const uint32_t a_sizeX, const uint32_t a_sizeY)
{
  std::vector<unsigned int> indicesData(a_sizeY*a_sizeX * 6);
  unsigned int* indexBuf = indicesData.data();

  for (uint32_t i = 0; i < a_sizeY; i++)
  {
    for (uint32_t j = 0; j < a_sizeX; j++)
    {
      *indexBuf++ = (unsigned int)( (i + 0) * (a_sizeX + 1) + (j + 0) );
      *indexBuf++ = (unsigned int)( (i + 1) * (a_sizeX + 1) + (j + 0) );
      *indexBuf++ = (unsigned int)( (i + 1) * (a_sizeX + 1) + (j + 1) );
      *indexBuf++ = (unsigned int)( (i + 0) * (a_sizeX + 1) + (j + 0) );
      *indexBuf++ = (unsigned int)( (i + 1) * (a_sizeX + 1) + (j + 1) );
      *indexBuf++ = (unsigned int)( (i + 0) * (a_sizeX + 1) + (j + 1) );
    }
  }

  return indicesData;
}

cmesh::SimpleMesh cmesh::CreateQuad(const uint32_t a_sizeX, const uint32_t a_sizeY, const float a_size)
{
  const uint32_t vertNumX = a_sizeX + 1;
  const uint32_t vertNumY = a_sizeY + 1;

  const uint32_t quadsNum = a_sizeX * a_sizeY;
  const uint32_t vertNum  = vertNumX * vertNumY;

  cmesh::SimpleMesh res(vertNum, quadsNum*2*3);

  const float edgeLength  = a_size / float(a_sizeX);
//  const float edgeLength2 = sqrtf(2.0f)*edgeLength;

  // put vertices
  //
  const float startX = -0.5f*a_size;
  const float startY = -0.5f*a_size;

  float* vPos4f_f = res.vPos4f.data();
  float* vNorm4f  = res.vNorm4f.data();

  for (uint32_t y = 0; y < vertNumY; y++)
  {
    const float ypos = startY + float(y)*edgeLength;
    const uint32_t offset = y*vertNumX;

    for (uint32_t x = 0; x < vertNumX; x++)
    { 
      const float xpos = startX + float(x)*edgeLength;
      const uint32_t i = offset + x;

      vPos4f_f[i * 4 + 0] = xpos;
      vPos4f_f[i * 4 + 1] = ypos;
      vPos4f_f[i * 4 + 2] = 0.0f;
      vPos4f_f[i * 4 + 3] = 1.0f;

      vNorm4f[i * 4 + 0] = 0.0f;
      vNorm4f[i * 4 + 1] = 0.0f;
      vNorm4f[i * 4 + 2] = 1.0f;
      vNorm4f[i * 4 + 3] = 0.0f;

      res.vTexCoord2f[i * 2 + 0] = (xpos - startX) / a_size;
      res.vTexCoord2f[i * 2 + 1] = (ypos - startY) / a_size;
    }
  }
 
  res.indices = CreateQuadTriIndices(a_sizeX, a_sizeY);

  return res;
}

enum GEOM_FLAGS{ HAS_TANGENT    = 1,
    UNUSED2        = 2,
    UNUSED4        = 4,
    HAS_NO_NORMALS = 8};

struct Header
{
    uint64_t fileSizeInBytes;
    uint32_t verticesNum;
    uint32_t indicesNum;
    uint32_t materialsNum;
    uint32_t flags;
};

#if defined(__ANDROID__)
cmesh::SimpleMesh cmesh::LoadMeshFromVSGF(const char* a_fileName)
{
  AAsset* asset = AAssetManager_open(vk_utils::getAssetManager(), a_fileName, AASSET_MODE_STREAMING);
  assert(asset);

  size_t asset_size = AAsset_getLength(asset);
  assert(asset_size > 0);

  Header header = {};

  AAsset_read(asset, &header, sizeof(Header));
  SimpleMesh res(header.verticesNum, header.indicesNum);

  AAsset_read(asset, res.vPos4f.data(),  res.vPos4f.size() * sizeof(float));

  if(!(header.flags & HAS_NO_NORMALS))
    AAsset_read(asset, res.vNorm4f.data(), res.vNorm4f.size()*sizeof(float));
  else
    memset(res.vNorm4f.data(), 0, res.vNorm4f.size()*sizeof(float)); // #TODO: calc at flat normals in this case

  if(header.flags & HAS_TANGENT)
    AAsset_read(asset, res.vTang4f.data(), res.vTang4f.size()*sizeof(float));
  else
    memset(res.vTang4f.data(), 0, res.vTang4f.size()*sizeof(float));

  AAsset_read(asset, res.vTexCoord2f.data(), res.vTexCoord2f.size()*sizeof(float));
  AAsset_read(asset, res.indices.data(),    res.indices.size()*sizeof(unsigned int));
  AAsset_read(asset, res.matIndices.data(), res.matIndices.size()*sizeof(unsigned int));
  AAsset_close(asset);

  return res;
}
#else
cmesh::SimpleMesh cmesh::LoadMeshFromVSGF(const char* a_fileName)
{
  std::ifstream input(a_fileName, std::ios::binary);
  if(!input.is_open())
    return {};

  Header header = {};

  input.read((char*)&header, sizeof(Header));
  SimpleMesh res(header.verticesNum, header.indicesNum);

  input.read((char*)res.vPos4f.data(),  res.vPos4f.size() * sizeof(float));
  
  if(!(header.flags & HAS_NO_NORMALS))
    input.read((char*)res.vNorm4f.data(), res.vNorm4f.size()*sizeof(float));
  else
    memset(res.vNorm4f.data(), 0, res.vNorm4f.size()*sizeof(float)); // #TODO: calc at flat normals in this case

  if(header.flags & HAS_TANGENT)
    input.read((char*)res.vTang4f.data(), res.vTang4f.size()*sizeof(float));
  else
    memset(res.vTang4f.data(), 0, res.vTang4f.size()*sizeof(float));

  input.read((char*)res.vTexCoord2f.data(), res.vTexCoord2f.size()*sizeof(float));
  input.read((char*)res.indices.data(),    res.indices.size()*sizeof(unsigned int));
  input.read((char*)res.matIndices.data(), res.matIndices.size()*sizeof(unsigned int));
  input.close();

  return res; 
}
#endif

void cmesh::SaveMeshToVSGF(const char* a_fileName, const SimpleMesh& a_mesh)
{
  std::ofstream output(a_fileName, std::ios::binary);

  Header header = {};
  header.fileSizeInBytes = sizeof(header) + a_mesh.SizeInBytes();
  header.verticesNum     = (uint32_t)a_mesh.VerticesNum();
  header.indicesNum      = (uint32_t)a_mesh.IndicesNum();
  header.materialsNum    = (uint32_t)a_mesh.matIndices.size();
  header.flags           = 0;

  if(!a_mesh.vNorm4f.empty())
    header.flags |= HAS_NO_NORMALS;

  if(!a_mesh.vTang4f.empty())
    header.flags |= HAS_TANGENT;

  output.write((char*)&header, sizeof(Header));
  output.write((char*)a_mesh.vPos4f.data(), a_mesh.vPos4f.size() * sizeof(float));

  if(!a_mesh.vNorm4f.empty())
    output.write((char*)a_mesh.vNorm4f.data(), a_mesh.vNorm4f.size() * sizeof(float));

  if(!a_mesh.vTang4f.empty())
    output.write((char*)a_mesh.vTang4f.data(), a_mesh.vTang4f.size() * sizeof(float));

  output.write((char*)a_mesh.vTexCoord2f.data(), a_mesh.vTexCoord2f.size() * sizeof(float));
  output.write((char*)a_mesh.indices.data(),     a_mesh.indices.size() * sizeof(unsigned int));
  output.write((char*)a_mesh.matIndices.data(),  a_mesh.matIndices.size() * sizeof(unsigned int));

  output.close();
}


