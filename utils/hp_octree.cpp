#include "hp_octree.h"
#include "hp_octree_precomputed_tables.h"
#include <thread>

HPOctreeBuilder::HPOctreeBuilder()
{
  //if these asserts fail, the tables have changed
  //regenerate them with hp_octree_generate_tables
  assert(sizeof(NormalisedLengths)/sizeof(float) == (BASIS_MAX_DEGREE + 1)*(TREE_MAX_DEPTH + 1));
  assert(sizeof(NormalisedLengths[0])/sizeof(float) == TREE_MAX_DEPTH + 1);
}

HPOctreeBuilder::NodeLegacy::NodeLegacy()
{
  childIdx = -1;

  aabb.m_min = float3(0, 0, 0);
  aabb.m_max = float3(0, 0, 0);

  basis.coeffs = nullptr;
  basis.degree = BASIS_MAX_DEGREE + 1;

  depth = TREE_MAX_DEPTH + 1;
}

HPOctreeBuilder::ConfigLegacy::ConfigLegacy()
{
  targetErrorThreshold = pow(10, -10);
  nearnessWeighting.type = NearnessWeighting::Type::None;
  continuity.enforce = true;
  continuity.strength = 8.0;
  threadCount = std::thread::hardware_concurrency() != 0 ? std::thread::hardware_concurrency() : 1;
  root = Box3Legacy(float3(-0.5, -0.5, -0.5), float3(0.5, 0.5, 0.5));
  enableLogging = false;
}

void HPOctreeBuilder::ConfigLegacy::IsValid() const
{
  assert(targetErrorThreshold > 0.0);
  assert(threadCount > 0);
  float3 size = root.m_max - root.m_min;
  float volume = size.x * size.y * size.z;
  assert(volume > 0.0);

  if (nearnessWeighting.type != ConfigLegacy::NearnessWeighting::None)
  {
    assert(nearnessWeighting.strength > 0.0);
  }

  if (continuity.enforce)
  {
    assert(continuity.strength > 0.0);
  }
}

void HPOctreeBuilder::readLegacy(const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);

  unsigned size = 0;
  fs.read((char *)&size, sizeof(unsigned));
  std::vector<unsigned char> bytes(size);
  fs.read((char *)bytes.data(), size);

  printf("size: %u\n", size);

  assert(size > 0);

  unsigned nCoeffs = *((unsigned *)(bytes.data() + 0));
  printf("nCoeffs: %u\n", nCoeffs);
  coeffStore.resize(nCoeffs);
  memcpy(coeffStore.data(), bytes.data() + sizeof(unsigned), sizeof(double) * nCoeffs);

  unsigned nNodes = *((unsigned *)(bytes.data() + sizeof(unsigned) + sizeof(double) * nCoeffs));
  nodes.resize(nNodes);
  memcpy(nodes.data(), bytes.data() + sizeof(unsigned) + sizeof(double) * nCoeffs + sizeof(unsigned), sizeof(NodeLegacy) * nNodes);

  config = *(ConfigLegacy *)(bytes.data() + sizeof(unsigned) + sizeof(double) * nCoeffs + sizeof(unsigned) + sizeof(NodeLegacy) * nNodes);

  config.IsValid();
  configRootCentre = 0.5f * (config.root.m_min + config.root.m_max);
  configRootInvSizes = 1.0f / (config.root.m_max - config.root.m_min);

  readLegacy(coeffStore, nodes);
}

double HPOctreeBuilder::QueryLegacy(const float3 &pt_) const
{
  // Move to unit cube
  const float3 pt = (pt_ - configRootCentre) * configRootInvSizes;

  // Not in volume
  if (pt.x < nodes[0].aabb.m_min.x || pt.x > nodes[0].aabb.m_max.x ||
      pt.y < nodes[0].aabb.m_min.y || pt.y > nodes[0].aabb.m_max.y ||
      pt.z < nodes[0].aabb.m_min.z || pt.z > nodes[0].aabb.m_max.z)
  {
    return std::numeric_limits<double>::max();
  }

  // Start at root
  uint32_t curNodeIdx = 0;
  while (1)
  {
    const float3 &aabbMin = nodes[curNodeIdx].aabb.m_min;
    const float3 &aabbMax = nodes[curNodeIdx].aabb.m_max;
    const float curNodeAABBHalf = (aabbMax.x - aabbMin.x) * 0.5f;

    const uint32_t xIdx = (pt.x >= (aabbMin.x + curNodeAABBHalf));
    const uint32_t yIdx = (pt.y >= (aabbMin.y + curNodeAABBHalf)) << 1;
    const uint32_t zIdx = (pt.z >= (aabbMin.z + curNodeAABBHalf)) << 2;

    const uint32_t childIdx = nodes[curNodeIdx].childIdx + xIdx + yIdx + zIdx;
    const NodeLegacy &curChild = nodes[childIdx];
    if (curChild.basis.degree != (BASIS_MAX_DEGREE + 1))
    {
      // Leaf
      NodeLegacy::Basis basis;
      basis.degree = curChild.basis.degree;
      basis.coeffs = (double *)(coeffStore.data() + curChild.basis.coeffsStart);

      float d1 = FApprox(basis, curChild.aabb, pt, curChild.depth);
      float d2 = FApprox(childIdx, pt);
      printf("d1: %f, d2: %f\n", d1, d2);
      return d1;
    }
    else
    {
      // Not leaf
      curNodeIdx = childIdx;
    }
  }
}

double HPOctreeBuilder::FApprox(const NodeLegacy::Basis &basis_, const Box3Legacy &aabb_, const float3 &pt_, const uint32_t depth_) const
{
  // Move pt_ to unit cube
  const float3 unitPt = (pt_ - 0.5f * (aabb_.m_min + aabb_.m_max)) * (2 << depth_);
  printf("aabb: (%f, %f, %f) (%f, %f, %f)\n", aabb_.m_min.x, aabb_.m_min.y, aabb_.m_min.z, aabb_.m_max.x, aabb_.m_max.y, aabb_.m_max.z);
  printf("unitPt: %f, %f, %f\n", unitPt.x, unitPt.y, unitPt.z);

  // Create lookup table for pt_
  double LpXLookup[BASIS_MAX_DEGREE][3];
  for (uint32_t i = 0; i < 3; ++i)
  {
    // Constant
    LpXLookup[0][i] = NormalisedLengths[0][depth_];

    // Initial values for recurrence
    double LjMinus2 = 0.0;
    double LjMinus1 = 1.0;
    double Lj = 1.0;

    // Determine remaining values
    for (uint32_t j = 1; j <= basis_.degree; ++j)
    {
      Lj = LegendreCoefficent[j][0] * unitPt[i] * LjMinus1 - LegendreCoefficent[j][1] * LjMinus2;
      LjMinus2 = LjMinus1;
      LjMinus1 = Lj;

      LpXLookup[j][i] = Lj * NormalisedLengths[j][depth_];
    }
  }

  // Sum up basis coeffs
  double fApprox = 0.0;
  for (uint32_t i = 0; i < LegendreCoeffientCount[basis_.degree]; ++i)
  {
    double Lp = 1.0;
    for (uint32_t j = 0; j < 3; ++j)
    {
      Lp *= LpXLookup[BasisIndexValues[i][j]][j];
    }

    fApprox += basis_.coeffs[i] * Lp;
  }

  return fApprox;
}

float HPOctreeBuilder::FApprox(uint32_t nodeId, const float3& pt) const
{
  // Move pt_ to unit cube [-1,1]
  float px = octree.nodes[nodeId].pos_xy >> 16;
  float py = octree.nodes[nodeId].pos_xy & 0x0000FFFF;
  float pz = octree.nodes[nodeId].pos_z_lod_size >> 16;
  float sz = octree.nodes[nodeId].pos_z_lod_size & 0x0000FFFF;

  const float3 min_pos = float3(-1,-1,-1) + 2.0f*float3(px,py,pz)/sz;
  const float3 max_pos = min_pos + 2.0f*float3(1,1,1)/sz;
  const float3 half_size = 0.5f*(max_pos - min_pos);
  const float3 center = 0.5f*(max_pos + min_pos);
  const float3 unitPt = (pt - center) * half_size;
  printf("min_pos: %f, %f, %f\n", min_pos.x, min_pos.y, min_pos.z);
  printf("max_pos: %f, %f, %f\n", max_pos.x, max_pos.y, max_pos.z);
  printf("unitPt: %f, %f, %f\n", unitPt.x, unitPt.y, unitPt.z);

  unsigned depth = octree.nodes[nodeId].degree_lod & 0x0000FFFF;
  unsigned degree = octree.nodes[nodeId].degree_lod >> 16;

  // Create lookup table for pt_
  double LpXLookup[BASIS_MAX_DEGREE][3];
  for (uint32_t i = 0; i < 3; ++i)
  {
    // Constant
    LpXLookup[0][i] = NormalisedLengths[0][depth];

    // Initial values for recurrence
    double LjMinus2 = 0.0;
    double LjMinus1 = 1.0;
    double Lj = 1.0;

    // Determine remaining values
    for (uint32_t j = 1; j <= degree; ++j)
    {
      Lj = LegendreCoefficent[j][0] * unitPt[i] * LjMinus1 - LegendreCoefficent[j][1] * LjMinus2;
      LjMinus2 = LjMinus1;
      LjMinus1 = Lj;

      LpXLookup[j][i] = Lj * NormalisedLengths[j][depth];
    }
  }

  // Sum up basis coeffs
  double fApprox = 0.0;
  for (uint32_t i = 0; i < LegendreCoeffientCount[degree]; ++i)
  {
    double Lp = 1.0;
    for (uint32_t j = 0; j < 3; ++j)
    {
      Lp *= LpXLookup[BasisIndexValues[i][j]][j];
    }

    fApprox += octree.data[octree.nodes[nodeId].data_offset + i] * Lp;
  }

  return fApprox;
}

void HPOctreeBuilder::readLegacy(const std::vector<double> &coeffStore, const std::vector<NodeLegacy> &nodes)
{
  octree.data.resize(coeffStore.size());
  octree.nodes.resize(nodes.size());

  for (int i = 0; i < coeffStore.size(); i++)
    octree.data[i] = coeffStore[i];
  
  for (int i = 0; i < nodes.size(); i++)
  {
    SdfHPOctreeNode &node = octree.nodes[i];
    float sz = pow(2, nodes[i].depth);
    uint3 p = uint3(sz*0.5f*(nodes[i].aabb.m_min + 1.0f));
    assert(p.x < (1 << 16) && p.y < (1 << 16) && p.z < (1 << 16));

    node.pos_xy = (p.x << 16) | p.y;
    node.pos_z_lod_size = (p.z << 16) | (2 << nodes[i].depth);
    node.degree_lod = (nodes[i].basis.degree << 16) | nodes[i].depth;
    node.data_offset = nodes[i].basis.coeffsStart;
  }
}