#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <algorithm>
#include <cassert>
#include <cfloat>
#include <memory>

#include "BVH2Common.h"

//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

constexpr size_t reserveSize = 1000;

void BVHRT::ClearGeom()
{
  m_vertPos.reserve(std::max<size_t>(100000, m_vertPos.capacity()));
  m_indices.reserve(std::max<size_t>(100000 * 3, m_indices.capacity()));
  m_primIndices.reserve(std::max<size_t>(100000, m_primIndices.capacity()));

  m_vertPos.resize(0);
  m_indices.resize(0);
  m_primIndices.resize(0);

  m_allNodePairs.reserve(std::max<size_t>(100000, m_allNodePairs.capacity()));
  m_allNodePairs.resize(0);

  m_geomOffsets.reserve(std::max(reserveSize, m_geomOffsets.capacity()));
  m_geomOffsets.resize(0);

  m_geomBoxes.reserve(std::max<size_t>(reserveSize, m_geomBoxes.capacity()));
  m_geomBoxes.resize(0);

  m_bvhOffsets.reserve(std::max<size_t>(reserveSize, m_bvhOffsets.capacity()));
  m_bvhOffsets.resize(0);

  m_SdfParameters.reserve(16);
  m_SdfParameters.resize(0);

  m_SdfObjects.reserve(16);
  m_SdfObjects.resize(0);

  m_SdfConjunctions.reserve(16);
  m_SdfConjunctions.resize(0);

  m_SdfNeuralProperties.reserve(16);
  m_SdfNeuralProperties.resize(0);

  m_ConjIndices.reserve(16);
  m_ConjIndices.resize(0);

  m_SdfGridData.reserve(16);
  m_SdfGridData.resize(0);  

  m_SdfGridOffsets.reserve(16);
  m_SdfGridOffsets.resize(0);  

  m_SdfGridSizes.reserve(16);
  m_SdfGridSizes.resize(0);  

  m_SdfOctreeNodes.reserve(16);
  m_SdfOctreeNodes.resize(0);  

  m_SdfOctreeRoots.reserve(16);
  m_SdfOctreeRoots.resize(0);  

  m_SdfFrameOctreeNodes.reserve(16);
  m_SdfFrameOctreeNodes.resize(0);  

  m_SdfFrameOctreeRoots.reserve(16);
  m_SdfFrameOctreeRoots.resize(0);  

  m_origNodes.reserve(16);
  m_origNodes.resize(0); 

  ClearScene();
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

uint32_t BVHRT::AddGeom_Triangles3f(const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride)
{
  const size_t vStride = vByteStride / 4;
  assert(vByteStride % 4 == 0);

  const uint32_t currGeomId = uint32_t(m_geomOffsets.size());
  const size_t oldSizeVert  = m_vertPos.size();
  const size_t oldSizeInd   = m_indices.size();

  m_geomOffsets.push_back(uint2(oldSizeInd, oldSizeVert));

  m_vertPos.resize(oldSizeVert + a_vertNumber);

  Box4f bbox;
  for (size_t i = 0; i < a_vertNumber; i++)
  {
    const float4 v = float4(a_vpos3f[i * vStride + 0], a_vpos3f[i * vStride + 1], a_vpos3f[i * vStride + 2], 1.0f);
    m_vertPos[oldSizeVert + i] = v;
    bbox.include(v);
  }

  m_geomBoxes.push_back(bbox);
  m_geomTypeByGeomId.push_back(TYPE_MESH_TRIANGLE);

  // Build BVH for each geom and append it to big buffer;
  // append data to global arrays and fix offsets
  //
  const size_t oldBvhSize = m_allNodePairs.size();
  m_bvhOffsets.push_back(oldBvhSize);
  
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFat((const float*)(m_vertPos.data() + oldSizeVert), a_vertNumber, 16, a_triIndices, a_indNumber, presets, layout);

  AppendTreeData(bvhData.nodes, bvhData.indices, a_triIndices, a_indNumber);

  return currGeomId;
}

void BVHRT::UpdateGeom_Triangles3f(uint32_t a_geomId, const float *a_vpos3f, size_t a_vertNumber, const uint32_t *a_triIndices, size_t a_indNumber, BuildQuality a_qualityLevel, size_t vByteStride)
{
  std::cout << "[BVHRT::UpdateGeom_Triangles3f]: " << "not implemeted!" << std::endl; // not planned for this implementation (possible in general)
}

uint32_t BVHRT::AddGeom_SdfScene(SdfSceneView scene, BuildQuality a_qualityLevel)
{
  assert(scene.conjunctions_count > 0);
  assert(scene.objects_count > 0);
  assert(scene.parameters_count > 0);
  float4 mn = scene.conjunctions[0].min_pos;
  float4 mx = scene.conjunctions[0].max_pos;
  for (int i=0; i<scene.conjunctions_count; i++) 
  {
    mn = min(mn, scene.conjunctions[i].min_pos);
    mx = max(mx, scene.conjunctions[i].max_pos);
  }
  m_geomOffsets.push_back(uint2(m_ConjIndices.size(), 0));
  m_geomBoxes.push_back(Box4f(mn, mx));
  m_geomTypeByGeomId.push_back(TYPE_SDF_PRIMITIVE);
  m_bvhOffsets.push_back(m_allNodePairs.size());

  unsigned p_offset = m_SdfParameters.size();
  unsigned o_offset = m_SdfObjects.size();
  unsigned c_offset = m_SdfConjunctions.size();
  unsigned np_offset = m_SdfNeuralProperties.size();

  m_SdfParameters.insert(m_SdfParameters.end(), scene.parameters, scene.parameters + scene.parameters_count);
  m_SdfObjects.insert(m_SdfObjects.end(), scene.objects, scene.objects + scene.objects_count);
  m_SdfConjunctions.insert(m_SdfConjunctions.end(), scene.conjunctions, scene.conjunctions + scene.conjunctions_count);
  m_SdfNeuralProperties.insert(m_SdfNeuralProperties.end(), scene.neural_properties, scene.neural_properties + scene.neural_properties_count);

  for (int i=o_offset;i<m_SdfObjects.size();i++)
  {
    m_SdfObjects[i].params_offset += p_offset;
    m_SdfObjects[i].neural_id += np_offset;
  }
  
  for (int i=c_offset;i<m_SdfConjunctions.size();i++)
    m_SdfConjunctions[i].offset += o_offset;

  std::vector<unsigned> conj_indices;
  std::vector<BVHNode> orig_nodes;
  for (int i=0;i<scene.conjunctions_count;i++)
  {
    auto &c = scene.conjunctions[i];
    conj_indices.push_back(c_offset + i);
    orig_nodes.emplace_back();
    orig_nodes.back().boxMin = to_float3(c.min_pos);
    orig_nodes.back().boxMax = to_float3(c.max_pos);
  }
  while (orig_nodes.size() < 2)
  {
    conj_indices.push_back(conj_indices.back());
    orig_nodes.emplace_back();
    orig_nodes.back().boxMin = float3(1000,1000,1000);
    orig_nodes.back().boxMax = float3(1000.1,1000.1,1000.1);
  }
  m_ConjIndices.insert(m_ConjIndices.end(), conj_indices.begin(), conj_indices.end());
  //orig_nodes.resize(1);
  //orig_nodes[0].boxMin = aabb.min_pos;
  //orig_nodes[0].boxMax = aabb.max_pos;

  // Build BVH for each geom and append it to big buffer;
  // append data to global arrays and fix offsets
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFatCustom(orig_nodes.data(), orig_nodes.size(), presets, layout);

  for (auto &i : bvhData.indices)
    printf("ind %d\n",(int)i);

  m_allNodePairs.insert(m_allNodePairs.end(), bvhData.nodes.begin(), bvhData.nodes.end());

  return m_geomTypeByGeomId.size()-1;
}

uint32_t BVHRT::AddGeom_SdfGrid(SdfGridView grid, BuildQuality a_qualityLevel)
{
  assert(grid.size.x*grid.size.y*grid.size.z > 0);
  assert(grid.size.x*grid.size.y*grid.size.z < (1u<<28)); //huge grids shouldn't be here
  //SDF grid is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill common data arrays
  m_geomOffsets.push_back(uint2(m_SdfGridOffsets.size(), 0));
  m_geomBoxes.push_back(Box4f(mn, mx));
  m_geomTypeByGeomId.push_back(TYPE_SDF_GRID);
  m_bvhOffsets.push_back(m_allNodePairs.size());

  //fill grid-specific data arrays
  m_SdfGridOffsets.push_back(m_SdfGridData.size());
  m_SdfGridSizes.push_back(grid.size);
  m_SdfGridData.insert(m_SdfGridData.end(), grid.data, grid.data + grid.size.x*grid.size.y*grid.size.z);

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_SdfGrid(grid);

  // Build BVH for each geom and append it to big buffer;
  // append data to global arrays and fix offsets
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFatCustom(orig_nodes.data(), orig_nodes.size(), presets, layout);

  for (auto &i : bvhData.indices)
    printf("grid ind %d\n",(int)i);

  m_allNodePairs.insert(m_allNodePairs.end(), bvhData.nodes.begin(), bvhData.nodes.end());

  return m_geomTypeByGeomId.size()-1;
}

uint32_t BVHRT::AddGeom_SdfOctree(SdfOctreeView octree, BuildQuality a_qualityLevel)
{
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill common data arrays
  m_geomOffsets.push_back(uint2(m_SdfOctreeRoots.size(), 0));
  m_geomBoxes.push_back(Box4f(mn, mx));
  m_geomTypeByGeomId.push_back(TYPE_SDF_OCTREE);
  m_bvhOffsets.push_back(m_allNodePairs.size());

  //fill octree-specific data arrays
  m_SdfOctreeRoots.push_back(m_SdfOctreeNodes.size());
  m_SdfOctreeNodes.insert(m_SdfOctreeNodes.end(), octree.nodes, octree.nodes + octree.size);
  for (int i=m_SdfOctreeRoots.back();i<m_SdfOctreeNodes.size();i++)
    m_SdfOctreeNodes[i].offset += m_SdfOctreeRoots.back();

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_SdfOctree(octree);

  // Build BVH for each geom and append it to big buffer;
  // append data to global arrays and fix offsets
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFatCustom(orig_nodes.data(), orig_nodes.size(), presets, layout);

  for (auto &i : bvhData.indices)
    printf("octree ind %d\n",(int)i);

  m_allNodePairs.insert(m_allNodePairs.end(), bvhData.nodes.begin(), bvhData.nodes.end());

  return m_geomTypeByGeomId.size()-1;
}

uint32_t BVHRT::AddGeom_SdfFrameOctree(SdfFrameOctreeView octree, BuildQuality a_qualityLevel)
{
  assert(octree.size > 0);
  assert(octree.size < (1u<<28)); //huge grids shouldn't be here
  //SDF octree is always a unit cube
  float4 mn = float4(-1,-1,-1,1);
  float4 mx = float4( 1, 1, 1,1);

  //fill common data arrays
  m_geomOffsets.push_back(uint2(m_SdfFrameOctreeRoots.size(), 0));
  m_geomBoxes.push_back(Box4f(mn, mx));
  m_geomTypeByGeomId.push_back(TYPE_SDF_FRAME_OCTREE);
  m_bvhOffsets.push_back(m_allNodePairs.size());

  //fill octree-specific data arrays
  unsigned n_offset = m_SdfFrameOctreeNodes.size();
  m_SdfFrameOctreeRoots.push_back(n_offset);
  m_SdfFrameOctreeNodes.insert(m_SdfFrameOctreeNodes.end(), octree.nodes, octree.nodes + octree.size);
  for (int i=n_offset;i<m_SdfFrameOctreeNodes.size();i++)
    m_SdfFrameOctreeNodes[i].offset += n_offset;

  //create list of bboxes for BLAS
  std::vector<BVHNode> orig_nodes = GetBoxes_SdfFrameOctree(octree);
  m_origNodes = orig_nodes;

  // Build BVH for each geom and append it to big buffer;
  // append data to global arrays and fix offsets
  auto presets = BuilderPresetsFromString(m_buildName.c_str());
  auto layout  = LayoutPresetsFromString(m_layoutName.c_str());
  auto bvhData = BuildBVHFatCustom(orig_nodes.data(), orig_nodes.size(), presets, layout);

  m_allNodePairs.insert(m_allNodePairs.end(), bvhData.nodes.begin(), bvhData.nodes.end());

  return m_geomTypeByGeomId.size()-1;
}

void BVHRT::ClearScene()
{
  m_instBoxes.reserve(std::max(reserveSize, m_instBoxes.capacity()));
  m_instMatricesInv.reserve(std::max(reserveSize, m_instMatricesInv.capacity()));
  m_instMatricesFwd.reserve(std::max(reserveSize, m_instMatricesFwd.capacity()));

  m_geomIdByInstId.reserve(std::max(reserveSize, m_geomIdByInstId.capacity()));

  m_instBoxes.resize(0);
  m_instMatricesInv.resize(0);
  m_instMatricesFwd.resize(0);
  m_geomIdByInstId.resize(0);

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

void BVHRT::CommitScene(BuildQuality a_qualityLevel)
{
  BuilderPresets presets = {BVH2_LEFT_OFFSET, BVHQuality::HIGH, 1};
  m_nodesTLAS = BuildBVH((const BVHNode *)m_instBoxes.data(), m_instBoxes.size(), presets).nodes;

  // DebugPrintNodes(m_nodesTLAS, "z01_tlas.txt");
  // DebugPrintBoxes(m_instBoxes, "y01_boxes.txt");

  m_firstSceneCommit = false;
}

uint32_t BVHRT::AddInstance(uint32_t a_geomId, const float4x4 &a_matrix)
{
  const auto &box = m_geomBoxes[a_geomId];

  // (1) mult mesh bounding box vertices with matrix to form new bouding box for instance
  float4 boxVertices[8]{
      a_matrix * float4{box.boxMin.x, box.boxMin.y, box.boxMin.z, 1.0f},
      a_matrix * float4{box.boxMax.x, box.boxMin.y, box.boxMin.z, 1.0f},
      a_matrix * float4{box.boxMin.x, box.boxMax.y, box.boxMin.z, 1.0f},
      a_matrix * float4{box.boxMin.x, box.boxMin.y, box.boxMax.z, 1.0f},
      a_matrix * float4{box.boxMax.x, box.boxMax.y, box.boxMin.z, 1.0f},
      a_matrix * float4{box.boxMax.x, box.boxMin.y, box.boxMax.z, 1.0f},
      a_matrix * float4{box.boxMin.x, box.boxMax.y, box.boxMax.z, 1.0f},
      a_matrix * float4{box.boxMax.x, box.boxMax.y, box.boxMax.z, 1.0f},
  };

  Box4f newBox;
  for (size_t i = 0; i < 8; i++)
    newBox.include(boxVertices[i]);

  // (2) append bounding box and matrices
  //
  const uint32_t oldSize = uint32_t(m_instBoxes.size());

  m_instBoxes.push_back(newBox);
  m_instMatricesFwd.push_back(a_matrix);
  m_instMatricesInv.push_back(inverse4x4(a_matrix));
  m_geomIdByInstId.push_back(a_geomId);

  return oldSize;
}

void BVHRT::UpdateInstance(uint32_t a_instanceId, const float4x4 &a_matrix)
{
  if(a_instanceId > m_geomIdByInstId.size())
  {
    std::cout << "[BVHRT::UpdateInstance]: " << "bad instance id == " << a_instanceId << "; size == " << m_geomIdByInstId.size() << std::endl;
    return;
  }

  const uint32_t geomId = m_geomIdByInstId[a_instanceId];
  const float4 boxMin   = m_geomBoxes[geomId].boxMin;
  const float4 boxMax   = m_geomBoxes[geomId].boxMax;

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

  m_instBoxes      [a_instanceId] = newBox;
  m_instMatricesFwd[a_instanceId] = a_matrix;
  m_instMatricesInv[a_instanceId] = inverse4x4(a_matrix);
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

std::vector<BVHNode> BVHRT::GetBoxes_SdfOctree(SdfOctreeView octree)
{
  std::vector<BVHNode> nodes;
  nodes.resize(2);
  nodes[0].boxMin = float3(-1,-1,-1);
  nodes[0].boxMax = float3(1,1,0);
  nodes[1].boxMin = float3(-1,-1,0);
  nodes[1].boxMax = float3(1,1,1);
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
  /*  nodes.resize(2);
  nodes[0].boxMin = float3(-1,-1,-1);
  nodes[0].boxMax = float3(1,1,0);
  nodes[1].boxMin = float3(-1,-1,0);
  nodes[1].boxMax = float3(1,1,1);
  return nodes;*/
  add_border_nodes_rec(octree, nodes, 0, float3(0,0,0), 1);
  return nodes;
}

//SdfSceneFunction interface implementation
void BVHRT::init(SdfSceneView scene)
{
  m_SdfParameters.insert(m_SdfParameters.end(), scene.parameters, scene.parameters + scene.parameters_count);
  m_SdfObjects.insert(m_SdfObjects.end(), scene.objects, scene.objects + scene.objects_count);
  m_SdfConjunctions.insert(m_SdfConjunctions.end(), scene.conjunctions, scene.conjunctions + scene.conjunctions_count);
  m_SdfNeuralProperties.insert(m_SdfNeuralProperties.end(), scene.neural_properties, scene.neural_properties + scene.neural_properties_count);
} 
  
float BVHRT::eval_distance(float3 pos)
{
  if (!m_SdfConjunctions.empty())
  {
    float dist = 1e6;
    for (int i=0; i<m_SdfConjunctions.size(); i++)
      dist = std::min(dist, eval_dist_sdf_conjunction(i, pos));
    return dist;
  }
  else if (!m_SdfOctreeNodes.empty())
    return eval_distance_sdf_octree(0, pos, 1000);
  else if (!m_SdfGridData.empty())
    return eval_distance_sdf_grid(0, pos);

  return 1e6; 
}

//SdfOctreeFunction interface implementation
void BVHRT::init(SdfOctreeView octree)
{
  m_SdfOctreeRoots.push_back(m_SdfOctreeNodes.size());
  m_SdfOctreeNodes.insert(m_SdfOctreeNodes.end(), octree.nodes, octree.nodes + octree.size);  
}

float BVHRT::eval_distance_level(float3 pos, unsigned max_level)
{
  return eval_distance_sdf_octree(0, pos, max_level);
}

std::vector<SdfOctreeNode> &BVHRT::get_nodes()
{
  return m_SdfOctreeNodes;
}

const std::vector<SdfOctreeNode> &BVHRT::get_nodes() const
{
  return m_SdfOctreeNodes;
}

//SdfGridFunction interface implementation
void BVHRT::init(SdfGridView grid)
{
  m_SdfGridOffsets.push_back(m_SdfGridData.size());
  m_SdfGridSizes.push_back(grid.size);
  m_SdfGridData.insert(m_SdfGridData.end(), grid.data, grid.data + grid.size.x*grid.size.y*grid.size.z);
} 

//implementation of different constructor-like functions

std::shared_ptr<ISdfSceneFunction> get_SdfSceneFunction(SdfSceneView scene)
{
  std::shared_ptr<ISdfSceneFunction> rt(new BVHRT("", "")); 
  rt->init(scene);
  return rt;
}

std::shared_ptr<ISdfOctreeFunction> get_SdfOctreeFunction(SdfOctreeView scene)
{
  std::shared_ptr<ISdfOctreeFunction> rt(new BVHRT("", "")); 
  rt->init(scene);
  return rt;  
}

std::shared_ptr<ISdfGridFunction> get_SdfGridFunction(SdfGridView scene)
{
  std::shared_ptr<ISdfGridFunction> rt(new BVHRT("", "")); 
  rt->init(scene);
  return rt;    
}

ISceneObject* MakeBruteForceRT(const char* a_implName);
ISceneObject* MakeBVH2CommonRT(const char* a_implName, const char* a_buildName, const char* a_layoutName) 
{
  return new BVHRT(a_buildName, a_layoutName); 
}
std::shared_ptr<ISceneObject> CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName)
{
  const std::string className(a_implName); //
  if (className.find("BruteForce") != std::string::npos)
    return std::shared_ptr<ISceneObject>(MakeBruteForceRT(a_implName));
  else if (className.find("BVH2Common") != std::string::npos || className.find("BVH2") != std::string::npos)
    return std::shared_ptr<ISceneObject>(MakeBVH2CommonRT(a_implName, a_buildName, a_layoutName));
  return nullptr;
}
