#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <memory>
#include <array>
#include <map>

#include "BVH2Common.h"
#include "nurbs/nurbs_common_host.h"
#include "utils/sdf/sparse_octree_builder.h"
#include "catmul_clark/catmul_clark_host.h"
#include "ribbon/ribbon_host.h"
#include "utils/coctree/similarity_compression.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr std::size_t reserveSize = 1000;

uint32_t type_to_tag(uint32_t type)
{
  switch (type)
  {
  case TYPE_MESH_TRIANGLE:
    return AbstractObject::TAG_TRIANGLE;

  case TYPE_SDF_GRID:
    return AbstractObject::TAG_SDF_GRID;
  
  case TYPE_RF_GRID:
    return AbstractObject::TAG_RF;
  
  case TYPE_GS_PRIMITIVE:
    return AbstractObject::TAG_GS;
  
  case TYPE_SDF_SVS:
  case TYPE_SDF_FRAME_OCTREE:
  case TYPE_SDF_FRAME_OCTREE_TEX:
    return AbstractObject::TAG_SDF_NODE;
  
  case TYPE_SDF_SBS:
  case TYPE_SDF_SBS_TEX:
  case TYPE_SDF_SBS_COL:
    return AbstractObject::TAG_SDF_BRICK;

  case TYPE_SDF_SBS_ADAPT:
  case TYPE_SDF_SBS_ADAPT_TEX:
  case TYPE_SDF_SBS_ADAPT_COL:
    return AbstractObject::TAG_SDF_ADAPT_BRICK;
  
  case TYPE_NURBS:
    return AbstractObject::TAG_NURBS;
  
  case TYPE_CATMUL_CLARK:
    return AbstractObject::TAG_CATMUL_CLARK;
  
  case TYPE_RIBBON:
    return AbstractObject::TAG_RIBBON;

  case TYPE_GRAPHICS_PRIM:
    return AbstractObject::TAG_GRAPHICS_PRIM;

  case TYPE_COCTREE_V1:
  case TYPE_COCTREE_V2:
    return AbstractObject::TAG_COCTREE_SIMPLE;
  
  case TYPE_COCTREE_V3:
    return AbstractObject::TAG_COCTREE_BRICKED;

  case TYPE_OPENVDB_GRID:
    return AbstractObject::TAG_OPENVDB_GRID;

  default:
    return AbstractObject::TAG_NONE;
  }
}

void BVHRT::ClearGeom()
{
  m_geomData.reserve(std::max<std::size_t>(reserveSize, m_geomData.capacity()));
  m_geomData.resize(0);
  m_abstractObjectPtrs.reserve(m_geomData.capacity());
  m_abstractObjectPtrs.resize(0);

  startEnd.reserve(m_geomData.capacity());
  startEnd.resize(0);

  m_indices.reserve(std::max<std::size_t>(100000 * 3, m_indices.capacity()));
  m_indices.resize(0);

  m_vertPos.reserve(std::max<std::size_t>(100000, m_vertPos.capacity()));
  m_vertPos.resize(0);

  m_primIndices.reserve(std::max<std::size_t>(100000, m_primIndices.capacity()));
  m_primIndices.resize(0);

  m_allNodePairs.reserve(std::max<std::size_t>(100000, m_allNodePairs.capacity()));
  m_allNodePairs.resize(0);

  m_abstractObjects.reserve(reserveSize);
  m_abstractObjects.resize(0);

  ClearScene();
}

uint32_t BVHRT::AddCustomGeom_FromFile(const char *geom_type_name, const char *filename, ISceneObject *fake_this)
{
  std::string name = geom_type_name;
  if (name == "sdf")
  {
    std::cout << "[LoadScene]: sdf primitives scene was removed from LiteRT. Search for legacy version to load it. " << std::endl;
  }
  else if (name == "sdf_grid")
  {
    std::cout << "[LoadScene]: sdf grid = " << filename << std::endl;
    SdfGrid scene;
    load_sdf_grid(scene, filename);
    return AddGeom_SdfGrid(scene, fake_this);
  }
  else if (name == "sdf_octree")
  {
    std::cout << "[LoadScene]: sdf octree was removed from LiteRT. Search for legacy version to load it. " << std::endl;
  }
  else if (name == "sdf_frame_octree")
  {
    std::cout << "[LoadScene]: sdf frame octree = " << filename << std::endl;
    std::vector<SdfFrameOctreeNode> scene;
    load_sdf_frame_octree(scene, filename);
    return AddGeom_SdfFrameOctree(scene, fake_this);
  }
  else if (name == "sdf_svs")
  {
    std::cout << "[LoadScene]: sdf svs = " << filename << std::endl;
    std::vector<SdfSVSNode> scene;
    load_sdf_SVS(scene, filename);
    return AddGeom_SdfSVS(scene, fake_this);
  }
  else if (name == "sdf_sbs")
  {
    std::cout << "[LoadScene]: sdf sbs = " << filename << std::endl;
    SdfSBS scene;
    load_sdf_SBS(scene, filename);
    return AddGeom_SdfSBS(scene, fake_this);
  }
  else if (name == "sdf_hp")
  {
    std::cout << "[LoadScene]: sdf hp was removed from LiteRT. Search for legacy version to load it. " << std::endl;
  }
  else if (name == "nsdf")
  {
    std::cout << "[LoadScene]: neural sdf scene was removed from LiteRT. Search for legacy version to load it. " << std::endl;
  }
  else if (name == "rf")
  {
    std::cout << "[LoadScene]: radiance fields = " << filename << std::endl;
    RFScene scene;
    load_rf_scene(scene, filename);
    return AddGeom_RFScene(scene, fake_this);
  }
  else if (name == "gs")
  {
    std::cout << "[LoadScene]: gaussian splatting = " << filename << std::endl;
    GSScene scene;
    load_gs_scene(scene, filename);
    return AddGeom_GSScene(scene, fake_this);
  }
  else if (name == "sdf_coctree_v3")
  {
    std::cout << "[LoadScene]: SDF compact octree = " << filename << std::endl;
    COctreeV3 scene;
    load_coctree_v3(scene, filename);
    return AddGeom_COctreeV3(scene, 0, fake_this);
  }
  else
  {
    std::cout << "[LoadScene]: unknown geometry node type: " << name.c_str() << std::endl;
  }
  return 0;
}

void BVHRT::AppendTreeData(const std::vector<BVHNodePair>& a_nodes, const std::vector<uint32_t>& a_indices, const uint32_t *a_triIndices, size_t a_indNumber)
{
  m_allNodePairs.insert(m_allNodePairs.end(), a_nodes.begin(), a_nodes.end());
  m_primIndices.insert(m_primIndices.end(), a_indices.begin(), a_indices.end());
  
  const size_t oldIndexSize  = m_indices.size();
  m_indices.resize(oldIndexSize + a_indices.size()*3);
  for(size_t i=0;i<a_indices.size();i++)
  {
    const uint32_t triId = a_indices[i];
    m_indices[oldIndexSize + 3*i+0] = a_triIndices[triId*3+0];
    m_indices[oldIndexSize + 3*i+1] = a_triIndices[triId*3+1];
    m_indices[oldIndexSize + 3*i+2] = a_triIndices[triId*3+2];
  }
}

uint32_t BVHRT::AddGeom_Triangles3f(const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, uint32_t a_qualityLevel, size_t vByteStride)
{
  return AddGeom_Triangles3f(a_vpos3f, nullptr, a_vertNumber, a_triIndices, a_indNumber, BuildOptions(a_qualityLevel), vByteStride);
}
uint32_t BVHRT::AddGeom_Triangles3f(const float* a_vpos3f, const float* a_vnorm3f, size_t a_vertNumber, const uint32_t* a_triIndices, 
                                    size_t a_indNumber, BuildOptions a_qualityLevel, size_t vByteStride)
{
  const size_t vStride = vByteStride / 4;
  assert(vByteStride % 4 == 0);

  const uint32_t currGeomId = uint32_t(m_geomData.size());
  const size_t oldSizeVert  = m_vertPos.size();
  const size_t oldSizeInd   = m_indices.size();

  m_vertPos.resize(oldSizeVert + a_vertNumber);
  m_vertNorm.resize(oldSizeVert + a_vertNumber);

  Box4f bbox;
  for (size_t i = 0; i < a_vertNumber; i++)
  {
    const float4 v = float4(a_vpos3f[i * vStride + 0], a_vpos3f[i * vStride + 1], a_vpos3f[i * vStride + 2], 1.0f);
    m_vertPos[oldSizeVert + i] = v;
    m_vertNorm[oldSizeVert + i] = a_vnorm3f ? float4(a_vnorm3f[i * vStride + 0], a_vnorm3f[i * vStride + 1], a_vnorm3f[i * vStride + 2], 1.0f) : float4(1.0f, 0.0f, 0.0f, 1.0f);
    bbox.include(v);
  }

  // Build BVH for each geom and append it to big buffer;
  // append data to global arrays and fix offsets
  //
  const size_t oldBvhSize = m_allNodePairs.size();

  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataTriangle();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_MESH_TRIANGLE);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = bbox.boxMin;
  m_geomData.back().boxMax = bbox.boxMax;
  m_geomData.back().offset = uint2(oldSizeInd, oldSizeVert);
  m_geomData.back().bvhOffset = oldBvhSize;
  m_geomData.back().type = TYPE_MESH_TRIANGLE;

  std::vector<unsigned> startCount;
  
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFat((const float*)(m_vertPos.data() + oldSizeVert), a_vertNumber, 16, a_triIndices, a_indNumber, startCount, presets, layout);

  AppendTreeData(bvhData.nodes, bvhData.indices, a_triIndices, a_indNumber);

  const size_t oldSize = m_primIdCount.size();
  m_primIdCount.insert(m_primIdCount.end(), startCount.begin(), startCount.end());
  startEnd.push_back(uint2(uint32_t(oldSize), uint32_t(m_primIdCount.size())));

  return uint32_t(startEnd.size() - 1);
}

void BVHRT::UpdateGeom_Triangles3f(uint32_t a_geomId, const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, uint32_t a_qualityLevel, size_t vByteStride)
{
  std::cout << "[BVHRT::UpdateGeom_Triangles3f]: " << "not implemeted!" << std::endl; // not planned for this implementation (possible in general)
}

uint32_t BVHRT::AddGeom_AABB(uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount)
{
  // append data to global arrays and fix offsets
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFatCustom((const BVHNode*)boxMinMaxF8, a_boxNumber, presets, layout);
  
  m_allNodePairs.insert(m_allNodePairs.end(), bvhData.nodes.begin(), bvhData.nodes.end());

  const size_t oldSize = m_primIdCount.size();
  m_primIdCount.resize(oldSize + a_boxNumber);
  for (int i=0; i<a_boxNumber; i++)
    m_primIdCount[oldSize+i] = i;
  startEnd.push_back(uint2(uint32_t(oldSize), uint32_t(m_primIdCount.size())));

  return uint32_t(startEnd.size() - 1);
}

void BVHRT::UpdateGeom_AABB(uint32_t a_geomId, uint32_t a_typeId, const CRT_AABB* boxMinMaxF8, size_t a_boxNumber, void** a_customPrimPtrs, size_t a_customPrimCount)
{
  std::cout << "[BVHRT::UpdateGeom_AABB]: " << "not implemeted!" << std::endl; // not planned for this implementation (possible in general)
}

uint32_t BVHRT::AddGeom_RFScene(RFScene grid, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  //RF grid is always a unit cube
  float4 mn = float4(0, 0, 0,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataRF();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_RF_GRID);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_RFGridOffsets.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_RF_GRID;

  //fill grid-specific data arrays
  m_RFGridOffsets.push_back(m_RFGridData.size());
  m_RFGridSizes.push_back(grid.size);
  m_RFGridScales.push_back(grid.scale);

  m_RFGridFlags.push_back(1); // Do RF
  m_RFGridFlags.push_back(1); // Do fast rendering

  //create list of bboxes for BLAS
  std::vector<float> sparseGrid;
  std::vector<uint4> sparsePtrs;
  m_origNodes = GetBoxes_RFGrid(grid, sparseGrid, sparsePtrs);

  m_RFGridData.insert(m_RFGridData.end(), sparseGrid.begin(), sparseGrid.end());
  m_RFGridPtrs.insert(m_RFGridPtrs.end(), sparsePtrs.begin(), sparsePtrs.end());

  std::cout << "Using "
      << (m_RFGridData.size() * sizeof(float) / 1024 / 1024 + m_RFGridPtrs.size() * sizeof(uint4) / 1024 / 1024) 
      << " MB for model" << std::endl;
  
  return fake_this->AddGeom_AABB(AbstractObject::TAG_RF, (const CRT_AABB*)m_origNodes.data(), m_origNodes.size());
}

float4x4 Transpose(float4x4& a) {
    float4x4 b;

    for (int i = 0; i < 4; i++) {
        for (int j = 0; j < 4; j++) {
            b[j][i] = a[i][j];
        }
    }

    return b;
}

std::vector<float4x4> ComputeCovarianceMatrices(std::vector<float4x4>& m_gs_data_0) {
    std::vector<float4x4> covariance_matrices;

    for (int i = 0; i < m_gs_data_0.size(); ++i) {
        float4x4 S = float4x4(
            exp(m_gs_data_0[i][1][3]), 0.0f, 0.0f, 0.0f,
            0.0f, exp(m_gs_data_0[i][2][0]), 0.0f, 0.0f,
            0.0f, 0.0f, exp(m_gs_data_0[i][2][1]), 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f);

        float4 q = normalize(
            float4(m_gs_data_0[i][2][2], -m_gs_data_0[i][2][3], m_gs_data_0[i][3][0], m_gs_data_0[i][3][1]));

        float r = q.x;
        float x = q.y;
        float y = q.z;
        float z = q.w;

        float4x4 R = float4x4(
            1.0f - 2.0f * (y * y + z * z), 2.0f * (x * y - r * z), 2.0f * (x * z + r * y), 0.0f,
            2.0f * (x * y + r * z), 1.0f - 2.0f * (x * x + z * z), 2.0f * (y * z - r * x), 0.0f,
            2.0f * (x * z - r * y), 2.0f * (y * z + r * x), 1.0f - 2.0f * (x * x + y * y), 0.0f,
            0.0f, 0.0f, 0.0f, 0.0f);

        float4x4 M = S * R;
        float4x4 Sigma = Transpose(M) * M;

        covariance_matrices.push_back(Sigma);
    }

    return covariance_matrices;
}

std::vector<float4x4> InvertMatrices(std::vector<float4x4>&& matrices) {
    std::vector<float4x4> inverted_matrices;
    inverted_matrices.reserve(matrices.size());

    for (int i = 0; i < matrices.size(); ++i) {
        float determinant = matrices[i][0][0] * (matrices[i][1][1] * matrices[i][2][2] - matrices[i][2][1] * matrices[i][1][2]) -
                            matrices[i][0][1] * (matrices[i][1][0] * matrices[i][2][2] - matrices[i][1][2] * matrices[i][2][0]) +
                            matrices[i][0][2] * (matrices[i][1][0] * matrices[i][2][1] - matrices[i][1][1] * matrices[i][2][0]);

        if (determinant < 1e-9f) {
            matrices[i][0][0] += 1e-9f;
            matrices[i][1][1] += 1e-9f;
            matrices[i][2][2] += 1e-9f;

            determinant = matrices[i][0][0] * (matrices[i][1][1] * matrices[i][2][2] - matrices[i][2][1] * matrices[i][1][2]) -
                          matrices[i][0][1] * (matrices[i][1][0] * matrices[i][2][2] - matrices[i][1][2] * matrices[i][2][0]) +
                          matrices[i][0][2] * (matrices[i][1][0] * matrices[i][2][1] - matrices[i][1][1] * matrices[i][2][0]);
        }

        float4x4 inverse_matrix;

        inverse_matrix[0][0] = (matrices[i][1][1] * matrices[i][2][2] - matrices[i][2][1] * matrices[i][1][2]) / determinant;
        inverse_matrix[0][1] = (matrices[i][0][2] * matrices[i][2][1] - matrices[i][0][1] * matrices[i][2][2]) / determinant;
        inverse_matrix[0][2] = (matrices[i][0][1] * matrices[i][1][2] - matrices[i][0][2] * matrices[i][1][1]) / determinant;
        inverse_matrix[1][0] = (matrices[i][1][2] * matrices[i][2][0] - matrices[i][1][0] * matrices[i][2][2]) / determinant;
        inverse_matrix[1][1] = (matrices[i][0][0] * matrices[i][2][2] - matrices[i][0][2] * matrices[i][2][0]) / determinant;
        inverse_matrix[1][2] = (matrices[i][1][0] * matrices[i][0][2] - matrices[i][0][0] * matrices[i][1][2]) / determinant;
        inverse_matrix[2][0] = (matrices[i][1][0] * matrices[i][2][1] - matrices[i][2][0] * matrices[i][1][1]) / determinant;
        inverse_matrix[2][1] = (matrices[i][2][0] * matrices[i][0][1] - matrices[i][0][0] * matrices[i][2][1]) / determinant;
        inverse_matrix[2][2] = (matrices[i][0][0] * matrices[i][1][1] - matrices[i][1][0] * matrices[i][0][1]) / determinant;

        inverted_matrices.push_back(std::move(inverse_matrix));
    }

    return inverted_matrices;
}

uint32_t BVHRT::AddGeom_GSScene(GSScene grid, ISceneObject *fake_this, BuildOptions a_qualityLevel) 
{
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataGS();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_GS_PRIMITIVE);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = float4(-1.0f, -1.0f, -1.0f, 1.0f);
  m_geomData.back().boxMax = float4(1.0f, 1.0f, 1.0f, 1.0f);
  m_geomData.back().offset = uint2(grid.data_0.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_GS_PRIMITIVE;

  m_gs_data_0 = grid.data_0;
  m_gs_conic = InvertMatrices(ComputeCovarianceMatrices(m_gs_data_0));
  m_origNodes = GetBoxes_GSGrid(grid);
    
  return fake_this->AddGeom_AABB(AbstractObject::TAG_GS, (const CRT_AABB*)m_origNodes.data(), m_origNodes.size());
}

uint32_t BVHRT::AddGeom_SdfGrid(SdfGridView grid, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(grid.size.x*grid.size.y*grid.size.z > 0);
  assert(grid.size.x*grid.size.y*grid.size.z < (1u<<28)); //huge grids shouldn't be here
  //SDF grid is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataSdfGrid();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_SDF_GRID);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_SdfGridOffsets.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_SDF_GRID;

  //m_abstractObjectPtrs.push_back(m_abstractObjects.data() + m_abstractObjects.size() - 1); // WARNING! THSI ASSUME WE DON't realloc m_abstractObjects array!!! 
  //void* pPointer = m_abstractObjectPtrs.back();

  //fill grid-specific data arrays
  m_SdfGridOffsets.push_back(m_SdfGridData.size());
  m_SdfGridSizes.push_back(grid.size);
  m_SdfGridData.insert(m_SdfGridData.end(), grid.data, grid.data + grid.size.x*grid.size.y*grid.size.z);

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_SdfGrid(grid);
  
  return fake_this->AddGeom_AABB(AbstractObject::TAG_SDF_GRID, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1); // &pPointer, 1);
}

uint32_t BVHRT::AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataSdfNode();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_SDF_FRAME_OCTREE);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_SdfFrameOctreeRoots.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_SDF_FRAME_OCTREE;

  //fill octree-specific data arrays
  unsigned n_offset = m_SdfFrameOctreeNodes.size();
  m_SdfFrameOctreeRoots.push_back(n_offset);
  m_SdfFrameOctreeNodes.insert(m_SdfFrameOctreeNodes.end(), octree.nodes, octree.nodes + octree.size);
  for (int i=n_offset;i<m_SdfFrameOctreeNodes.size();i++)
    m_SdfFrameOctreeNodes[i].offset += n_offset;

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_SdfFrameOctree(octree);
  m_origNodes = orig_nodes;
  
  return fake_this->AddGeom_AABB(AbstractObject::TAG_SDF_NODE, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

uint32_t BVHRT::AddGeom_SdfSVS(SdfSVSView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //choose only those node that have both positive and negative values in distance array, i.e. borders
  std::vector<SdfSVSNode> border_nodes(octree.nodes, octree.nodes+octree.size);

/*border_nodes.reserve(octree.size);
  for (int i=0;i<octree.size;i++)
  {
    float sz = octree.nodes[i].pos_z_lod_size & 0x0000FFFF;
    float d_max = 2*1.73205081f/sz;

    bool less = false;
    bool more = false;
    for (int j=0;j<8;j++)
    {
      float val = -d_max + 2*d_max*(1.0/255.0f)*((octree.nodes[i].values[j/4] >> (8*(j%4))) & 0xFF);
      if (val <= 0)
        less = true;
      else if (val >= 0)
        more = true;
    }

    if (less)
      border_nodes.push_back(octree.nodes[i]);
  }*/ 

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataSdfNode();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_SDF_SVS);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_SdfSVSRoots.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_SDF_SVS;

  //fill octree-specific data arrays
  unsigned n_offset = m_SdfSVSNodes.size();
  m_SdfSVSRoots.push_back(n_offset);
  m_SdfSVSNodes.insert(m_SdfSVSNodes.end(), border_nodes.begin(), border_nodes.end());

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes(border_nodes.size());
  for (int i=0;i<border_nodes.size();i++)
  {
    float px = border_nodes[i].pos_xy >> 16;
    float py = border_nodes[i].pos_xy & 0x0000FFFF;
    float pz = border_nodes[i].pos_z_lod_size >> 16;
    float sz = border_nodes[i].pos_z_lod_size & 0x0000FFFF;
    orig_nodes[i].boxMin = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
    orig_nodes[i].boxMax = orig_nodes[i].boxMin + 2.0f*float3(1,1,1)/sz;
  }
  
  return fake_this->AddGeom_AABB(AbstractObject::TAG_SDF_NODE, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

uint32_t BVHRT::AddGeom_SdfSBS(SdfSBSView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(octree.size > 0 && octree.values_count > 0);
  assert(octree.size < (1u<<28) && octree.values_count < (1u<<28));

  uint32_t type = TYPE_SDF_SBS;
  uint32_t node_layout = octree.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK;
  if (node_layout == SDF_SBS_NODE_LAYOUT_UNDEFINED || node_layout == SDF_SBS_NODE_LAYOUT_DX) //only distances
  {
    type = TYPE_SDF_SBS;
  }
  else if (node_layout == SDF_SBS_NODE_LAYOUT_DX_UV16) //textured
  {
    type = TYPE_SDF_SBS_TEX;
  }
  else if (node_layout == SDF_SBS_NODE_LAYOUT_DX_RGB8 ||     //colored
           node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F || 
           node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN) 
  {
    type = TYPE_SDF_SBS_COL;
  }
  else
  {
    printf("unsupported node layout %u\n", node_layout);
  }

  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  uint32_t typeTag = type_to_tag(type);
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataSdfBrick();

  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = typeTag;


  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_SdfSBSRoots.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = type;

  //fill octree-specific data arrays
  unsigned n_offset = m_SdfSBSNodes.size();
  unsigned v_offset = m_SdfSBSData.size();
  m_SdfSBSRoots.push_back(n_offset);
  m_SdfSBSHeaders.push_back(octree.header);
  m_SdfSBSNodes.insert(m_SdfSBSNodes.end(), octree.nodes, octree.nodes + octree.size);
  m_SdfSBSData.insert(m_SdfSBSData.end(), octree.values, octree.values + octree.values_count);

  for (int i=n_offset; i<m_SdfSBSNodes.size(); i++)
    m_SdfSBSNodes[i].data_offset += v_offset;
  
  if (node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F|| 
      node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN) //indexed layout reqires float values
  {
    unsigned f_offset = m_SdfSBSDataF.size();
    m_SdfSBSDataF.insert(m_SdfSBSDataF.end(), octree.values_f, octree.values_f + octree.values_f_count);

    //all integer values are indices, so we need to remap them
    for (int i=v_offset; i<m_SdfSBSData.size(); i++)
      m_SdfSBSData[i] += f_offset;
  }
  else
  {
    assert(octree.values_f_count == 0);
  }

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes;

  //one node for each brick
    orig_nodes.resize(octree.size);
    for (int i=0;i<octree.size;i++)
    {
      float px = octree.nodes[i].pos_xy >> 16;
      float py = octree.nodes[i].pos_xy & 0x0000FFFF;
      float pz = octree.nodes[i].pos_z_lod_size >> 16;
      float sz = octree.nodes[i].pos_z_lod_size & 0x0000FFFF;

      orig_nodes[i].boxMin = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
      orig_nodes[i].boxMax = orig_nodes[i].boxMin + 2.0f*float3(1,1,1)/sz;
    }

  //edge case - one node for whole scene
  //add one more node, because embree breaks when
  //it tries to build BVH with only one node
  if (orig_nodes.size() == 1)
  {
    orig_nodes.resize(2);
    orig_nodes[1].boxMin = orig_nodes[0].boxMin - 0.001f*float3(1,1,1);
    orig_nodes[1].boxMax = orig_nodes[0].boxMin;
  }
  
  return fake_this->AddGeom_AABB(typeTag, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

uint32_t BVHRT::AddGeom_SdfSBSAdapt(SdfSBSAdaptView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(octree.size > 0 && octree.values_count > 0);
  assert(octree.size < (1u<<28) && octree.values_count < (1u<<28));

  uint32_t type = TYPE_SDF_SBS_ADAPT;
  uint32_t node_layout = octree.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK;
  if (node_layout == SDF_SBS_NODE_LAYOUT_UNDEFINED || node_layout == SDF_SBS_NODE_LAYOUT_DX) //only distances
  {
    type = TYPE_SDF_SBS_ADAPT;
  }
  else if (node_layout == SDF_SBS_NODE_LAYOUT_DX_UV16) //textured
  {
    type = TYPE_SDF_SBS_ADAPT_TEX;
  }
  else if (node_layout == SDF_SBS_NODE_LAYOUT_DX_RGB8 ||     //colored
           node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F || 
           node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN) 
  {
    type = TYPE_SDF_SBS_ADAPT_COL;
  }
  else
  {
    printf("unsupported node layout %u\n", node_layout);
  }

  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  uint32_t typeTag = type_to_tag(type);
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataSdfAdaptBrick();

  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = typeTag;


  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_SdfSBSAdaptRoots.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = type;

  //fill octree-specific data arrays
  unsigned n_offset = m_SdfSBSAdaptNodes.size();
  unsigned v_offset = m_SdfSBSAdaptData.size();
  m_SdfSBSAdaptNodes.insert(m_SdfSBSAdaptNodes.end(), octree.nodes, octree.nodes + octree.size);
  m_SdfSBSAdaptData.insert(m_SdfSBSAdaptData.end(), octree.values, octree.values + octree.values_count);
  m_SdfSBSAdaptRoots.push_back(n_offset);
  m_SdfSBSAdaptHeaders.push_back(octree.header);


  for (int i=n_offset; i<m_SdfSBSAdaptNodes.size(); i++)
    m_SdfSBSAdaptNodes[i].data_offset += v_offset;
  
  if (node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F|| 
      node_layout == SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F_IN) //indexed layout reqires float values
  {
    unsigned f_offset = m_SdfSBSAdaptDataF.size();
    m_SdfSBSAdaptDataF.insert(m_SdfSBSAdaptDataF.end(), octree.values_f, octree.values_f + octree.values_f_count);

    //all integer values are indices, so we need to remap them
    for (int i=v_offset; i<m_SdfSBSAdaptData.size(); i++)
      m_SdfSBSAdaptData[i] += f_offset;
  }
  else
  {
    assert(octree.values_f_count == 0);
  }

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes;
  orig_nodes.resize(octree.size);

  for (int i = 0; i < octree.size; i++)
  {
    float px = octree.nodes[i].pos_xy >> 16;
    float py = octree.nodes[i].pos_xy & 0x0000FFFF;
    float pz = octree.nodes[i].pos_z_vox_size >> 16;
    float vs = octree.nodes[i].pos_z_vox_size & 0x0000FFFF;

    float3 brick_size{
        (octree.nodes[i].vox_count_xyz_pad >> 16) & 0x000000FF,
        (octree.nodes[i].vox_count_xyz_pad >> 8) & 0x000000FF,
        (octree.nodes[i].vox_count_xyz_pad) & 0x000000FF};
    float3 brick_abs_size = (2.0f / SDF_SBS_ADAPT_MAX_UNITS) * vs * brick_size;

    orig_nodes[i].boxMin = float3(-1, -1, -1) + (2.0f / SDF_SBS_ADAPT_MAX_UNITS) * float3(px, py, pz);
    orig_nodes[i].boxMax = orig_nodes[i].boxMin + brick_abs_size;
  }

  //edge case - one node for whole scene
  //add one more node, because embree breaks when
  //it tries to build BVH with only one node
  if (orig_nodes.size() == 1)
  {
    orig_nodes.resize(2);
    orig_nodes[1].boxMin = orig_nodes[0].boxMin - 0.001f*float3(-1,-1,-1);
    orig_nodes[1].boxMax = orig_nodes[0].boxMin;
  }

  return fake_this->AddGeom_AABB(type_to_tag(type), (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

uint32_t BVHRT::AddGeom_SdfFrameOctreeTex(SdfFrameOctreeTexView octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataSdfNode();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_SDF_FRAME_OCTREE_TEX);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_SdfFrameOctreeTexRoots.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_SDF_FRAME_OCTREE_TEX;

  //fill octree-specific data arrays
  unsigned n_offset = m_SdfFrameOctreeTexNodes.size();
  m_SdfFrameOctreeTexRoots.push_back(n_offset);
  m_SdfFrameOctreeTexNodes.insert(m_SdfFrameOctreeTexNodes.end(), octree.nodes, octree.nodes + octree.size);
  for (int i=n_offset;i<m_SdfFrameOctreeTexNodes.size();i++)
    m_SdfFrameOctreeTexNodes[i].offset += n_offset;

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_SdfFrameOctreeTex(octree);
  m_origNodes = orig_nodes;
  
  return fake_this->AddGeom_AABB(AbstractObject::TAG_SDF_NODE, (const CRT_AABB*)m_origNodes.data(), m_origNodes.size());
}

uint32_t BVHRT::AddGeom_NURBS(const RBezierGrid &rbeziers, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  auto bbox = rbeziers.bbox;
  if (LiteMath::any_of(to_float3(bbox.boxMin) == to_float3(bbox.boxMax))) {
    bbox.boxMin -= 1e-4f;
    bbox.boxMax += 1e-4f;
  }

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataNURBS();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_NURBS);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = bbox.boxMin;
  m_geomData.back().boxMax = bbox.boxMax;
  m_geomData.back().offset = uint2(m_NURBSHeaders.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_NURBS;

  //save NURBS to headers and data
  uint32_t offset = m_NURBSData.size();
  auto [grid_rows, grid_cols] = rbeziers.grid.shape2D();
  auto [surf_rows, surf_cols] = rbeziers.grid[{0, 0}].weighted_points.shape2D();
  
  m_NURBSHeaders.push_back(NURBSHeader{
    static_cast<int>(offset), 
    static_cast<int>(surf_rows-1), static_cast<int>(surf_cols-1), 
    static_cast<int>(grid_rows+1), static_cast<int>(grid_cols+1)
  });
  
  for (int i = 0; i < rbeziers.grid.rows_count(); ++i)
  for (int j = 0; j < rbeziers.grid.cols_count(); ++j)
  {
    std::copy(
      reinterpret_cast<const float*>(
          rbeziers.grid[{i, j}].weighted_points.data()),
      reinterpret_cast<const float*>(
          rbeziers.grid[{i, j}].weighted_points.data()+surf_rows*surf_cols),
      std::back_inserter(m_NURBSData));
  }

  std::copy(
    rbeziers.uniq_uknots.begin(), rbeziers.uniq_uknots.end(), 
    std::back_inserter(m_NURBSData));
  std::copy(
    rbeziers.uniq_vknots.begin(), rbeziers.uniq_vknots.end(),
    std::back_inserter(m_NURBSData));

  auto [bboxes, uvs] = get_nurbs_bvh_leaves(rbeziers);
  std::vector<BVHNode> nodes(bboxes.size());
  for (int i = 0; i < bboxes.size(); ++i) {
    nodes[i].boxMin = to_float3(bboxes[i].boxMin);
    nodes[i].boxMax = to_float3(bboxes[i].boxMax);
    uint32_t approx_id = m_NURBS_approxes.size()/2;
    uint32_t packed = PackOffsetAndSize(approx_id, 1);
    m_primIdCount.push_back(packed);
    m_NURBS_approxes.push_back(uvs[i][0]);
    m_NURBS_approxes.push_back(uvs[i][1]);
  }

  if (bboxes.size() == 1) {
    nodes.resize(2);
    m_primIdCount.push_back(m_primIdCount.back());
  }

  return fake_this->AddGeom_AABB(AbstractObject::TAG_NURBS, (const CRT_AABB*)nodes.data(), nodes.size());
}

//////////////////// CATMUL_CLARK SECTION /////////////////////////////////////////////////////
uint32_t BVHRT::AddGeom_CatmulClark(const CatmulClark &surface, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  float4 mn = float4(-0.5,-0.5,-0.5,0.5);
  float4 mx = float4( 0.5, 0.5, 0.5,0.5);
  //TODO: find right BBox for given CatmulClark

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataCatmulClark();

  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_CATMUL_CLARK);

  //geomData of current CatmulClark
  m_geomData.emplace_back();

  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;

  // this offset is used in BVHRT::IntersectCatmulClark function
  m_geomData.back().offset = uint2(m_CatmulClarkHeaders.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_CATMUL_CLARK;

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes(2);
  orig_nodes[0].boxMin = to_float3(mn);
  orig_nodes[0].boxMin = to_float3(mx);
  //BVH can process only 2+ leaves, not one
  //So we need to add a small dummy box (it's a crutch)
  orig_nodes[1].boxMin = to_float3(mx);
  orig_nodes[1].boxMax = to_float3(mx+0.0001f);

  return fake_this->AddGeom_AABB(AbstractObject::TAG_CATMUL_CLARK, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size());
}
//////////////////// END CATMUL_CLARK SECTION /////////////////////////////////////////////////////

std::vector<BVHNode> getBoxes_OpenVDB_Grid(const OpenVDB_Grid& grid)
{
  std::vector<BVHNode> nodes;
  nodes.resize(2);
  nodes[0].boxMin = float3(-1,-1,-1);
  nodes[0].boxMax = float3(0.99,0.99,0.99);
  nodes[1].boxMin = float3(0.99,0.99,0.99);
  nodes[1].boxMax = float3(1,1,1);
  return nodes;
}

uint32_t BVHRT::AddGeom_OpenVDB_Grid(const OpenVDB_Grid &grid, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  #ifndef DISABLE_OPENVDB
  //SDF grid is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataOpenVDB_GRID();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_OPENVDB_GRID);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(m_VDBHeaders.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_OPENVDB_GRID;

  //m_abstractObjectPtrs.push_back(m_abstractObjects.data() + m_abstractObjects.size() - 1); // WARNING! THSI ASSUME WE DON't realloc m_abstractObjects array!!! 
  //void* pPointer = m_abstractObjectPtrs.back();

  uint32_t offset = m_VDBData.size();
  m_VDBHeaders.push_back({ offset });
  m_VDBData.push_back(grid);

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes(2);
  orig_nodes[0].boxMin = to_float3(mn);
  orig_nodes[0].boxMax = to_float3(mx);
  
  return fake_this->AddGeom_AABB(AbstractObject::TAG_OPENVDB_GRID, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1); // &pPointer, 1);
  #else
  return 0;
  #endif
}

//////////////////// RIBBON SECTION /////////////////////////////////////////////////////
uint32_t BVHRT::AddGeom_Ribbon(const Ribbon &rib, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  float4 mn = float4(-0.5,-0.5,-0.5,0.5);
  float4 mx = float4( 0.5, 0.5, 0.5,0.5);
  //TODO: find right BBox for given Ribbon

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataRibbon();

  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_RIBBON);

  //geomData of current Ribbon
  m_geomData.emplace_back();

  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;

  // this offset is used in BVHRT::Ribbon function
  m_geomData.back().offset = uint2(m_RibbonHeaders.size(), 0);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_RIBBON;

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes(2);
  orig_nodes[0].boxMin = to_float3(mn);
  orig_nodes[0].boxMin = to_float3(mx);
  //BVH can process only 2+ leaves, not one
  //So we need to add a small dummy box (it's a crutch)
  orig_nodes[1].boxMin = to_float3(mx);
  orig_nodes[1].boxMax = to_float3(mx+0.0001f);

  return fake_this->AddGeom_AABB(AbstractObject::TAG_RIBBON, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size());
}
//////////////////// END RIBBON SECTION /////////////////////////////////////////////////////


BVHNode getDiskAABB(float3 pt, float3 norm, float rad)
{
  BVHNode res_node{};
  norm = normalize(norm) * rad;

  float xy_len = std::sqrt(norm.x * norm.x + norm.y * norm.y);
  float xz_len = std::sqrt(norm.x * norm.x + norm.z * norm.z);
  float yz_len = std::sqrt(norm.y * norm.y + norm.z * norm.z);
  res_node.boxMin = pt - float3(yz_len, xz_len, xy_len);
  res_node.boxMax = pt + float3(yz_len, xz_len, xy_len);
  return res_node;
}

uint32_t BVHRT::AddGeom_GraphicsPrim(const GraphicsPrimView &prim_view, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  float3 mn = float3( 1, 1, 1);
  float3 mx = float3(-1,-1,-1);

  std::vector<BVHNode> orig_nodes;

  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataGraphicsPrim();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = AbstractObject::TAG_GRAPHICS_PRIM;

  m_geomData.emplace_back();
  m_geomData.back().offset = uint2(m_GraphicsPrimRoots.size(), m_GraphicsPrimRoots.size() + prim_view.size);
  m_geomData.back().bvhOffset = m_allNodePairs.size();
  m_geomData.back().type = TYPE_GRAPHICS_PRIM;

  m_GraphicsPrimHeaders.push_back(prim_view.header);
  m_GraphicsPrimRoots.push_back(m_GraphicsPrimPoints.size());
  m_GraphicsPrimPoints.insert(m_GraphicsPrimPoints.end(), prim_view.points, prim_view.points + prim_view.size);


  uint32_t arr_step = 2u;
  if (prim_view.header.prim_type == GRAPH_PRIM_POINT)
    arr_step = 1u;
  if (prim_view.header.prim_type >= GRAPH_PRIM_LINE_COLOR)
    arr_step = 3u;
  
  for (uint32_t i = 0u; i < prim_view.size; i += arr_step)
  {
    BVHNode new_node{};
    if (prim_view.header.prim_type == GRAPH_PRIM_POINT ||
        prim_view.header.prim_type == GRAPH_PRIM_POINT_COLOR)
    {
      float4 pt_rad = prim_view.points[i];
      new_node.boxMin = to_float3(pt_rad) - pt_rad.w;
      new_node.boxMax = to_float3(pt_rad) + pt_rad.w;
    }
    else if (prim_view.header.prim_type == GRAPH_PRIM_LINE ||
             prim_view.header.prim_type == GRAPH_PRIM_LINE_COLOR)
    {
      float4 pt1_rad = prim_view.points[i  ];
      float4 pt2     = prim_view.points[i+1];

      float3 pos = to_float3(pt2);
      float3 dir = normalize(to_float3(pt1_rad) - pos);
      float3 inv_dir = float3(dir.x > 1e-4f ? 1.f / dir.x : 0.f,
                              dir.y > 1e-4f ? 1.f / dir.y : 0.f,
                              dir.z > 1e-4f ? 1.f / dir.z : 0.f);

      // line-box intersection
      float2 t{0.f};
      bool pos_on_positive1 = false;
      bool pos_on_negative1 = false;

      for (int axis = 0; axis < 3; ++axis)
      {
        if (dir[axis] > 1e-4f)
        {
          float dist = 1.f - pos[axis];
          if (std::abs(dist) < 1e-4f)
            pos_on_positive1 = true;
          else
          {
            float t1 = dist / dir[axis];
            if (t.x == 0.f || std::abs(t1) < std::abs(t.x))
              t.x = t1;
          }

          dist = -1.f - pos[axis];
          if (std::abs(dist) < 1e-4f)
            pos_on_negative1 = true;
          else
          {
            float t2 = dist / dir[axis];
            if (t.y == 0.f || std::abs(t2) < std::abs(t.y))
              t.y = t2;
          }
        }
      }

      float3 intersect1 = pos_on_positive1 ? pos : pos + t.x * dir;
      float3 intersect2 = pos_on_negative1 ? pos : pos + t.y * dir;

      new_node = getDiskAABB(intersect1, dir, pt1_rad.w);

      BVHNode tmp_node = getDiskAABB(intersect2, dir, pt1_rad.w);
      new_node.boxMin = min(new_node.boxMin, tmp_node.boxMin);
      new_node.boxMax = max(new_node.boxMax, tmp_node.boxMax);
    }
    else if (prim_view.header.prim_type == GRAPH_PRIM_LINE_SEGMENT ||
             prim_view.header.prim_type == GRAPH_PRIM_LINE_SEGMENT_COLOR)
    {
      float4 pt1_rad = prim_view.points[i  ];
      float4 pt2     = prim_view.points[i+1];
      float3 dir = normalize(to_float3(pt2 - pt1_rad));

      new_node = getDiskAABB(to_float3(pt2), dir, pt1_rad.w);

      BVHNode tmp_node = getDiskAABB(to_float3(pt1_rad), dir, pt1_rad.w);
      new_node.boxMin = min(new_node.boxMin, tmp_node.boxMin);
      new_node.boxMax = max(new_node.boxMax, tmp_node.boxMax);
    }
    else if (prim_view.header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR ||
             prim_view.header.prim_type == GRAPH_PRIM_LINE_SEGMENT_DIR_COLOR)
    {
      float4 pt1_rad = prim_view.points[i];
      float3 pt2 = to_float3(prim_view.points[i+1]);
      float3 pt1 = to_float3(pt1_rad);
      float3 dir = pt2 - pt1;

      float cone_ra = length(pt2 - pt1) * 0.15f;
      float3 pa = pt2 * 0.7f + pt1 * 0.3f;

      new_node = getDiskAABB(pt1, dir, pt1_rad.w); // arrow start
      new_node.boxMin = min(new_node.boxMin, pt2); // arrow point
      new_node.boxMax = max(new_node.boxMax, pt2); // arrow point

      BVHNode tmp_node = getDiskAABB(pa, dir, cone_ra);
      new_node.boxMin = min(new_node.boxMin, tmp_node.boxMin); // arrow cone
      new_node.boxMax = max(new_node.boxMax, tmp_node.boxMax); // arrow cone
    }
    else if (prim_view.header.prim_type == GRAPH_PRIM_BOX ||
             prim_view.header.prim_type == GRAPH_PRIM_BOX_COLOR)
    {
      float4 pt_min_rad = prim_view.points[i];
      float3 pt_max = to_float3(prim_view.points[i+1]);
      new_node.boxMin = min(to_float3(pt_min_rad), pt_max) - pt_min_rad.w;
      new_node.boxMax = max(to_float3(pt_min_rad), pt_max) + pt_min_rad.w;
    }

    mn = min(mn, new_node.boxMin);
    mx = max(mx, new_node.boxMax);
    orig_nodes.push_back(new_node);
  }

  if (orig_nodes.size() == 1)
  {
    orig_nodes.resize(2);
    orig_nodes[1].boxMin = orig_nodes[0].boxMin - 0.001f*float3(-1,-1,-1);
    orig_nodes[1].boxMax = orig_nodes[0].boxMin;
  }

  m_geomData.back().boxMin = to_float4(mn, 1);
  m_geomData.back().boxMax = to_float4(mx, 1);

  return fake_this->AddGeom_AABB(AbstractObject::TAG_GRAPHICS_PRIM, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

uint32_t BVHRT::AddGeom_COctreeV1(COctreeV1View octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(m_SdfCompactOctreeV1Data.size() == 0); //only one compact octree per scene is supported
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataCOctreeSimple();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_COCTREE_V1);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(0, 0);
  m_geomData.back().bvhOffset = 0;
  m_geomData.back().type = TYPE_COCTREE_V1;

  //fill octree-specific data arrays
  m_SdfCompactOctreeV1Data = std::vector<SdfCompactOctreeNode>(octree.nodes, octree.nodes + octree.size);

  //create smallest possible list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes;
  orig_nodes.resize(2);
  orig_nodes[0].boxMin = float3(-1,-1,-1);
  orig_nodes[0].boxMax = float3(1,1,1);
  orig_nodes[0].leftOffset = 0;

  orig_nodes[1].boxMin = float3(-1.0001,-1.0001,-1.0001);
  orig_nodes[1].boxMax = float3(-1,-1,-1);
  orig_nodes[1].leftOffset = 0;

  m_origNodes = orig_nodes;
  
  return fake_this->AddGeom_AABB(m_abstractObjects.back().m_tag, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

uint32_t BVHRT::AddGeom_COctreeV2(COctreeV2View octree, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
  assert(m_SdfCompactOctreeV1Data.size() == 0); //only one compact octree per scene is supported
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataCOctreeSimple();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_COCTREE_V2);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(0, 0);
  m_geomData.back().bvhOffset = 0;
  m_geomData.back().type = TYPE_COCTREE_V2;

  //fill octree-specific data arrays
  m_SdfCompactOctreeV2Data = std::vector<uint32_t>(octree.data, octree.data + octree.size);

  //create smallest possible list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes;
  orig_nodes.resize(2);
  orig_nodes[0].boxMin = float3(-1,-1,-1);
  orig_nodes[0].boxMax = float3(1,1,1);
  orig_nodes[0].leftOffset = 0;

  orig_nodes[1].boxMin = float3(-1.0001,-1.0001,-1.0001);
  orig_nodes[1].boxMax = float3(-1,-1,-1);
  orig_nodes[1].leftOffset = 0;

  m_origNodes = orig_nodes;
  
  return fake_this->AddGeom_AABB(m_abstractObjects.back().m_tag, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

void add_border_nodes_rec(COctreeV3View octree, uint32_t max_bvh_level, 
                          std::vector<BVHNode> &nodes,
                          uint32_t nodeId, float3 p, float d, uint32_t level)
{
  unsigned childrenInfo = octree.data[nodeId + 0];
  unsigned children_leaves = childrenInfo & 0xFF00u;

  if (children_leaves || max_bvh_level == level)
  {
    //stop here
    nodes.emplace_back();
    nodes.back().leftOffset = nodeId;
    nodes.back().boxMin = float3(-1,-1,-1) + 2.0f*p*d;
    nodes.back().boxMax = float3(-1,-1,-1) + 2.0f*(p+float3(1,1,1))*d;
    //printf("added box (%f, %f, %f), (%f, %f, %f)\n", nodes.back().boxMin.x, nodes.back().boxMin.y, nodes.back().boxMin.z, nodes.back().boxMax.x, nodes.back().boxMax.y, nodes.back().boxMax.z);
  }
  else
  {
    int child_pos = 1;
    for (int i = 0; i < 8; i++)
    {
      // if child is active
      if ((childrenInfo & (1u << i)) > 0)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        add_border_nodes_rec(octree, max_bvh_level, nodes, octree.data[nodeId + child_pos], ch_p, ch_d, level+1);
        child_pos++;
      }
    }
  }
}

std::vector<BVHNode> GetBoxes_COctreeV3(COctreeV3View octree, uint32_t max_bvh_level, 
                                        uint32_t nodeId, float3 p, float d, uint32_t level)
{
  std::vector<BVHNode> nodes;
  add_border_nodes_rec(octree, max_bvh_level, nodes, 0, float3(0,0,0), 1.0f, 0);

  return nodes;
}

uint32_t BVHRT::AddGeom_COctreeV3(COctreeV3View octree, unsigned bvh_level, ISceneObject *fake_this, BuildOptions a_qualityLevel)
{
#if ON_CPU==1
  assert(COctreeV3::VERSION == 3); //if version is changed, this function should be revisited, as some changes may be needed
#endif

  scom::initialize_rot_transforms(m_SdfCompactOctreeRotVTransforms, octree.header.brick_size + 2 * octree.header.brick_pad + 1);
  scom::initialize_rot_transforms(m_SdfCompactOctreeRotPTransforms, octree.header.brick_size + 2 * octree.header.brick_pad);

  assert(m_SdfCompactOctreeV1Data.size() == 0); //only one compact octree per scene is supported
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here

  coctree_v3_header = octree.header;

  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill geom data array
  m_abstractObjects.resize(m_abstractObjects.size() + 1); 
  new (m_abstractObjects.data() + m_abstractObjects.size() - 1) GeomDataCOctreeBricked();
  m_abstractObjects.back().geomId = m_abstractObjects.size() - 1;
  m_abstractObjects.back().m_tag = type_to_tag(TYPE_COCTREE_V3);

  m_geomData.emplace_back();
  m_geomData.back().boxMin = mn;
  m_geomData.back().boxMax = mx;
  m_geomData.back().offset = uint2(0, 0);
  m_geomData.back().bvhOffset = 0;
  m_geomData.back().type = TYPE_COCTREE_V3;

  //fill octree-specific data arrays
  m_SdfCompactOctreeV3Data = std::vector<uint32_t>(octree.data, octree.data + octree.size);

  //list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_COctreeV3(octree, bvh_level, 0, float3(0,0,0), 1.0f, 0);

  if (orig_nodes.size() < 2)
  {
    orig_nodes.resize(2);
    orig_nodes[0].boxMin = float3(-1,-1,-1);
    orig_nodes[0].boxMax = float3(1,1,1);
    orig_nodes[0].leftOffset = 0;

    orig_nodes[1].boxMin = float3(-1.0001,-1.0001,-1.0001);
    orig_nodes[1].boxMax = float3(-1,-1,-1);
    orig_nodes[1].leftOffset = 0;
  }

  m_origNodes = orig_nodes;
  
  return fake_this->AddGeom_AABB(m_abstractObjects.back().m_tag, (const CRT_AABB*)orig_nodes.data(), orig_nodes.size(), nullptr, 1);
}

void BVHRT::set_debug_mode(bool enable)
{
  debug_cur_pixel = enable;
}

void BVHRT::ClearScene()
{
  m_instanceData.reserve(std::max<std::size_t>(reserveSize, m_instanceData.capacity()));
  m_instanceData.resize(0);

  m_firstSceneCommit = true;
}

void DebugPrintNodes(const std::vector<BVHNode>& nodes, const std::string& a_fileName)
{
  std::ofstream fout(a_fileName.c_str());

  for(size_t i=0;i<nodes.size();i++)
  {
    const auto& currBox = nodes[i];
    fout << "node[" << i << "]:" << std::endl;
    fout << "  bmin = { " << currBox.boxMin[0] << " " << currBox.boxMin[1] << " " << currBox.boxMin[2] << " } | " << currBox.leftOffset  << std::endl;
    fout << "  bmax = { " << currBox.boxMax[0] << " " << currBox.boxMax[1] << " " << currBox.boxMax[2] << " } | " << currBox.escapeIndex << std::endl;
  } 
}

void DebugPrintBoxes(const std::vector<Box4f>& nodes, const std::string& a_fileName)
{
  std::ofstream fout(a_fileName.c_str());

  for(size_t i=0;i<nodes.size();i++)
  {
    const auto& currBox = nodes[i];
    fout << "node[" << i << "]:" << std::endl;
    fout << "  bmin = { " << currBox.boxMin[0] << " " << currBox.boxMin[1] << " " << currBox.boxMin[2] << " " << currBox.boxMin[3]  << std::endl;
    fout << "  bmax = { " << currBox.boxMax[0] << " " << currBox.boxMax[1] << " " << currBox.boxMax[2] << " " << currBox.boxMax[3] << std::endl;
  } 
}

void BVHRT::CommitScene(uint32_t a_qualityLevel)
{
  assert(m_instanceData.size() > 0);

  //if there is only 1 instance, there is no need in TLAS
  if (m_instanceData.size() > 1)
  {
    std::vector<Box4f> instBoxes(m_instanceData.size());
    for (size_t i = 0; i < m_instanceData.size(); i++)
      instBoxes[i] = Box4f(m_instanceData[i].boxMin, m_instanceData[i].boxMax);
    
    BuilderPresets presets = {BVH2_LEFT_OFFSET, BVHQuality::HIGH, 1};
    m_nodesTLAS = BuildBVH((const BVHNode *)instBoxes.data(), instBoxes.size(), presets).nodes;
  }
  else
  {
    m_nodesTLAS.emplace_back();
    m_nodesTLAS[0].boxMin = to_float3(m_instanceData[0].boxMin);
    m_nodesTLAS[0].boxMax = to_float3(m_instanceData[0].boxMax);
    m_nodesTLAS[0].leftOffset = LEAF_BIT;
    m_nodesTLAS[0].escapeIndex = LEAF_NORMAL;
  }

  m_firstSceneCommit = false;

  //Create a vector of pointers from geom data
  //It is required to use virtual functions on GPU
  //And call different intersection shaders for each type

  m_abstractObjectPtrs.resize(m_abstractObjects.size());
  for (size_t i = 0; i < m_abstractObjects.size(); i++)
    m_abstractObjectPtrs[i] = m_abstractObjects.data() + i;
}

uint32_t BVHRT::AddInstance(uint32_t a_geomId, const float4x4 &a_matrix)
{
  if(a_geomId & CRT_GEOM_MASK_AABB_BIT) // TEMP SOLUTION !!!
    a_geomId = (a_geomId & 0x7fffffff); // TEMP SOLUTION !!!

  const auto &boxMin = m_geomData[a_geomId].boxMin;
  const auto &boxMax = m_geomData[a_geomId].boxMax;

  // (1) mult mesh bounding box vertices with matrix to form new bouding box for instance
  float4 boxVertices[8]{
      a_matrix * float4{boxMin.x, boxMin.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMin.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMin.x, boxMax.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMin.x, boxMin.y, boxMax.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMax.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMin.y, boxMax.z, 1.0f},
      a_matrix * float4{boxMin.x, boxMax.y, boxMax.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMax.y, boxMax.z, 1.0f},
  };

  Box4f newBox;
  for (size_t i = 0; i < 8; i++)
    newBox.include(boxVertices[i]);

  // (2) append bounding box and matrices
  const uint32_t oldSize = uint32_t(m_instanceData.size());
  InstanceData instance;
  instance.boxMin = newBox.boxMin;
  instance.boxMax = newBox.boxMax;
  instance.geomId = a_geomId;
  instance.transform = a_matrix;
  instance.transformInv = inverse4x4(a_matrix);
  instance.transformInvTransposed = transpose(inverse4x4(a_matrix));

  m_instanceData.push_back(instance);

  return oldSize;
}

void BVHRT::UpdateInstance(uint32_t a_instanceId, const float4x4 &a_matrix)
{
  if(a_instanceId > m_instanceData.size())
  {
    std::cout << "[BVHRT::UpdateInstance]: " << "bad instance id == " << a_instanceId << "; size == " << m_instanceData.size() << std::endl;
    return;
  }

  const uint32_t geomId = m_instanceData[a_instanceId].geomId;
  const float4 boxMin   = m_geomData[geomId].boxMin;
  const float4 boxMax   = m_geomData[geomId].boxMax;

  // (1) mult mesh bounding box vertices with matrix to form new bouding box for instance
  float4 boxVertices[8]{
      a_matrix * float4{boxMin.x, boxMin.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMin.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMin.x, boxMax.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMin.x, boxMin.y, boxMax.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMax.y, boxMin.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMin.y, boxMax.z, 1.0f},
      a_matrix * float4{boxMin.x, boxMax.y, boxMax.z, 1.0f},
      a_matrix * float4{boxMax.x, boxMax.y, boxMax.z, 1.0f},
  };

  Box4f newBox;
  for (size_t i = 0; i < 8; i++)
    newBox.include(boxVertices[i]);

  m_instanceData[a_instanceId].boxMin = newBox.boxMin;
  m_instanceData[a_instanceId].boxMax = newBox.boxMax;
  m_instanceData[a_instanceId].transform = a_matrix;
  m_instanceData[a_instanceId].transformInv = inverse4x4(a_matrix);
}

std::vector<BVHNode> BVHRT::GetBoxes_SdfGrid(SdfGridView grid)
{
  std::vector<BVHNode> nodes;
  nodes.resize(2);
  nodes[0].boxMin = float3(-1,-1,-1);
  nodes[0].boxMax = float3(1,1,0);
  nodes[1].boxMin = float3(-1,-1,0);
  nodes[1].boxMax = float3(1,1,1);
  return nodes;
}

std::array<float, CellSize> lerpCell(const float* v0, const float* v1, const float t)
{
  std::array<float, CellSize> ret = {};

  for (size_t i = 0; i < CellSize; i++)
    ((float*)ret.data())[i] = LiteMath::lerp(((float*)v0)[i], ((float*)v1)[i], t);

  return ret;
}

size_t indexGrid(size_t x, size_t y, size_t z, size_t gridSize)
{
  return x + y * gridSize + z * gridSize * gridSize;
}

std::array<float, CellSize> interpolate(float* grid, int3 nearCoords, int3 farCoords, float3 lerpFactors, size_t gridSize) {
  auto xy00 = lerpCell((float*)&grid[indexGrid(nearCoords[0], nearCoords[1], nearCoords[2], gridSize) * CellSize],
    (float*)&grid[indexGrid(farCoords[0], nearCoords[1], nearCoords[2], gridSize) * CellSize], lerpFactors.x);
  auto xy10 = lerpCell((float*)&grid[indexGrid(nearCoords[0], farCoords[1], nearCoords[2], gridSize) * CellSize],
    (float*)&grid[indexGrid(farCoords[0], farCoords[1], nearCoords[2], gridSize) * CellSize], lerpFactors.x);
  auto xy01 = lerpCell((float*)&grid[indexGrid(nearCoords[0], nearCoords[1], farCoords[2], gridSize) * CellSize],
    (float*)&grid[indexGrid(farCoords[0], nearCoords[1], farCoords[2], gridSize) * CellSize], lerpFactors.x);
  auto xy11 = lerpCell((float*)&grid[indexGrid(nearCoords[0], farCoords[1], farCoords[2], gridSize) * CellSize],
    (float*)&grid[indexGrid(farCoords[0], farCoords[1], farCoords[2], gridSize) * CellSize], lerpFactors.x);

  auto xyz0 = lerpCell(xy00.data(), xy10.data(), lerpFactors.y);
  auto xyz1 = lerpCell(xy01.data(), xy11.data(), lerpFactors.y);

  return lerpCell(xyz0.data(), xyz1.data(), lerpFactors.z);
}

void addToVector(std::vector<float>& v, float data[28]) {
  v.insert(v.end(), data, data + 28);
}

struct hashableFloat3 {
    float x, y, z;

    bool operator< (const hashableFloat3& other) const {
        return (x < other.x || (x == other.x && y < other.y) || (x == other.x && y == other.y && z < other.z));
    }
};

std::vector<BVHNode> BVHRT::GetBoxes_RFGrid(RFScene grid, std::vector<float>& sparseGrid, std::vector<uint4>& sparsePtrs)
{
  std::cout << "Optimizing model" << std::endl;
  std::map<hashableFloat3, uint> coordsToIdx;

  std::vector<BVHNode> nodes;
  nodes.reserve((grid.size - 1) * (grid.size - 1) * (grid.size - 1));

  auto getDensity = [&](BVHNode box)
  {
    float3 bbMin = float3(box.boxMin[0], box.boxMin[1], box.boxMin[2]);
    float3 bbMax = float3(box.boxMax[0], box.boxMax[1], box.boxMax[2]);

    float3 p = (bbMin + bbMax) / 2.0f;

    float3 lerpFactors = (p - bbMin) / (bbMax - bbMin);

    int3 nearCoords = LiteMath::clamp((int3)(bbMin * grid.size), 0, grid.size - 1);
    int3 farCoords = LiteMath::clamp(nearCoords + 1, 0, grid.size - 1);

    auto gridVal = interpolate((float*)grid.data.data(), nearCoords, farCoords, lerpFactors, grid.size);
    return gridVal[0] * grid.scale;
  };

  size_t i = 0;
  std::cout << "Added nodes: " << i << '\r' << std::flush;
  #pragma omp parallel for collapse(3)
  for (size_t z = 0; z < grid.size - 1; z++)
    for (size_t y = 0; y < grid.size - 1; y++)
      for (size_t x = 0; x < grid.size - 1; x++)
      {
        BVHNode node;
        node.boxMin = float3((float)x / (float)(grid.size), (float)y / (float)(grid.size), (float)z / (float)(grid.size));
        node.boxMax = float3((float)(x + 1) / (float)(grid.size), (float)(y + 1) / (float)(grid.size), (float)(z + 1) / (float)(grid.size));

        if (getDensity(node) > 0.0f) {
          auto addCell = [&](uint3 coords) {
            addToVector(sparseGrid, &grid.data[28 * (coords[0] + coords[1] * grid.size + coords[2] * grid.size * grid.size)]);
          };

          auto addPointer = [&](uint3 coords) {
            hashableFloat3 spaceCoords = {(float)coords[0] / (float) (grid.size - 1), (float)coords[1] / (float) (grid.size - 1), (float)coords[2] / (float) (grid.size - 1)};
            if (coordsToIdx.find(spaceCoords) == coordsToIdx.end()) {
              coordsToIdx[spaceCoords] = sparseGrid.size() / 28;
              addToVector(sparseGrid, &grid.data[28 * (coords[0] + coords[1] * grid.size + coords[2] * grid.size * grid.size)]);
            }
            return coordsToIdx[spaceCoords];
          };

          #pragma omp critical
          {
            if (m_RFGridFlags[1] == 1) {
              addCell(uint3(x, y, z));
              addCell(uint3(x + 1, y, z));
              addCell(uint3(x, y + 1, z));
              addCell(uint3(x, y, z + 1));

              addCell(uint3(x + 1, y + 1, z));
              addCell(uint3(x, y + 1, z + 1));
              addCell(uint3(x + 1, y, z + 1));
              addCell(uint3(x + 1, y + 1, z + 1));
            } else {
              uint4 ptrs;
              ptrs[0] = addPointer(uint3(x, y, z));
              ptrs[1] = addPointer(uint3(x + 1, y, z));
              ptrs[2] = addPointer(uint3(x, y + 1, z));
              ptrs[3] = addPointer(uint3(x, y, z + 1));

              sparsePtrs.push_back(ptrs);

              ptrs[0] = addPointer(uint3(x + 1, y + 1, z));
              ptrs[1] = addPointer(uint3(x, y + 1, z + 1));
              ptrs[2] = addPointer(uint3(x + 1, y, z + 1));
              ptrs[3] = addPointer(uint3(x + 1, y + 1, z + 1));

              sparsePtrs.push_back(ptrs);
            }

            nodes.push_back(node);
            std::cout << "Added nodes: " << ++i << '\r' << std::flush;
          }
        }
      }

  return nodes;
}

std::vector<BVHNode> BVHRT::GetBoxes_GSGrid(const GSScene& grid) {
  std::vector<BVHNode> nodes;
  nodes.reserve(grid.data_0.size());

  for (size_t i = 0; i < grid.data_0.size(); ++i) {
    float scale = exp(max(max(grid.data_0[i](1, 3), grid.data_0[i](2, 0)), grid.data_0[i](2, 1))) * 3.0f;

    BVHNode node;

    node.boxMin = float3(grid.data_0[i](0, 0) - scale, grid.data_0[i](0, 1) - scale, grid.data_0[i](0, 2) - scale);
    node.boxMax = float3(grid.data_0[i](0, 0) + scale, grid.data_0[i](0, 1) + scale, grid.data_0[i](0, 2) + scale);

    nodes.push_back(node);
  }

  return nodes;
}

void add_border_nodes_rec(const SdfFrameOctreeView &octree, std::vector<BVHNode> &nodes,
                          unsigned idx, float3 p, float d)
{
  unsigned ofs = octree.nodes[idx].offset;
  if (ofs == 0) 
  {
    bool less = false;
    bool more = false;
    for (int i=0;i<8;i++)
    {
      if (octree.nodes[idx].values[i] <= 0)
        less = true;
      else if (octree.nodes[idx].values[i] >= 0)
        more = true;
    }

    if (less && more)
    {
      float3 min_pos = 2.0f*(d*p) - 1.0f;
      float3 max_pos = min_pos + 2.0f*d*float3(1,1,1);
      nodes.emplace_back();
      nodes.back().boxMax = max_pos;
      nodes.back().boxMin = min_pos;
      nodes.back().leftOffset = idx; //just store idx here, it will be later replaced by real offset in BVHBuilder anyway
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      float ch_d = d / 2;
      float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      add_border_nodes_rec(octree, nodes, ofs + i, ch_p, ch_d);
    }
  }
}

std::vector<BVHNode> BVHRT::GetBoxes_SdfFrameOctree(SdfFrameOctreeView octree)
{
  std::vector<BVHNode> nodes;
  add_border_nodes_rec(octree, nodes, 0, float3(0,0,0), 1);
  return nodes;
}

void add_border_nodes_rec(const SdfFrameOctreeTexView &octree, std::vector<BVHNode> &nodes,
                          unsigned idx, float3 p, float d)
{
  unsigned ofs = octree.nodes[idx].offset;
  if (ofs == 0) 
  {
    bool less = false;
    bool more = false;
    for (int i=0;i<8;i++)
    {
      if (octree.nodes[idx].values[i] <= 0)
        less = true;
      else if (octree.nodes[idx].values[i] >= -sqrt(3)*d)
        more = true;
    }

    if (less && more)
    {
      float3 min_pos = 2.0f*(d*p) - 1.0f;
      float3 max_pos = min_pos + 2.0f*d*float3(1,1,1);
      nodes.emplace_back();
      nodes.back().boxMax = max_pos;
      nodes.back().boxMin = min_pos;
      nodes.back().leftOffset = idx; //just store idx here, it will be later replaced by real offset in BVHBuilder anyway
    }
  }
  else
  {
    for (int i = 0; i < 8; i++)
    {
      float ch_d = d / 2;
      float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
      add_border_nodes_rec(octree, nodes, ofs + i, ch_p, ch_d);
    }
  }
}

std::vector<BVHNode> BVHRT::GetBoxes_SdfFrameOctreeTex(SdfFrameOctreeTexView octree)
{
  std::vector<BVHNode> nodes;
  add_border_nodes_rec(octree, nodes, 0, float3(0,0,0), 1);

  return nodes;
}
  
float BVHRT::eval_distance(float3 pos)
{
  if (!m_SdfGridData.empty())
    return eval_distance_sdf_grid(0, pos);

  return 1e6; 
}

//SdfGridFunction interface implementation
void BVHRT::init(SdfGridView grid)
{
  m_SdfGridOffsets.push_back(m_SdfGridData.size());
  m_SdfGridSizes.push_back(grid.size);
  m_SdfGridData.insert(m_SdfGridData.end(), grid.data, grid.data + grid.size.x*grid.size.y*grid.size.z);
} 

std::shared_ptr<ISdfGridFunction> get_SdfGridFunction(SdfGridView scene)
{
  std::shared_ptr<ISdfGridFunction> rt(new BVHRT("", "")); 
  rt->init(scene);
  return rt;    
}

ISceneObject* MakeBVH2CommonRT(const char* a_implName, const char* a_buildName, const char* a_layoutName) 
{
  return new BVHRT(a_buildName, a_layoutName); 
}

std::shared_ptr<ISceneObject> CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName)
{
  return std::shared_ptr<ISceneObject>(MakeBVH2CommonRT(a_implName, a_buildName, a_layoutName));
}
