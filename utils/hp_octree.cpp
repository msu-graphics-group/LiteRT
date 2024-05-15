#include "hp_octree.h"
#include "hp_octree_precomputed_tables.h"
#include <thread>

static constexpr uint32_t BasisIndexValues[455][3] = {
  { 0, 0, 0 },
  { 0, 0, 1 },
  { 0, 1, 0 },
  { 1, 0, 0 },
  { 0, 0, 2 },
  { 0, 1, 1 },
  { 0, 2, 0 },
  { 1, 0, 1 },
  { 1, 1, 0 },
  { 2, 0, 0 },
  { 0, 0, 3 },
  { 0, 1, 2 },
  { 0, 2, 1 },
  { 0, 3, 0 },
  { 1, 0, 2 },
  { 1, 1, 1 },
  { 1, 2, 0 },
  { 2, 0, 1 },
  { 2, 1, 0 },
  { 3, 0, 0 },
  { 0, 0, 4 },
  { 0, 1, 3 },
  { 0, 2, 2 },
  { 0, 3, 1 },
  { 0, 4, 0 },
  { 1, 0, 3 },
  { 1, 1, 2 },
  { 1, 2, 1 },
  { 1, 3, 0 },
  { 2, 0, 2 },
  { 2, 1, 1 },
  { 2, 2, 0 },
  { 3, 0, 1 },
  { 3, 1, 0 },
  { 4, 0, 0 },
  { 0, 0, 5 },
  { 0, 1, 4 },
  { 0, 2, 3 },
  { 0, 3, 2 },
  { 0, 4, 1 },
  { 0, 5, 0 },
  { 1, 0, 4 },
  { 1, 1, 3 },
  { 1, 2, 2 },
  { 1, 3, 1 },
  { 1, 4, 0 },
  { 2, 0, 3 },
  { 2, 1, 2 },
  { 2, 2, 1 },
  { 2, 3, 0 },
  { 3, 0, 2 },
  { 3, 1, 1 },
  { 3, 2, 0 },
  { 4, 0, 1 },
  { 4, 1, 0 },
  { 5, 0, 0 },
  { 0, 0, 6 },
  { 0, 1, 5 },
  { 0, 2, 4 },
  { 0, 3, 3 },
  { 0, 4, 2 },
  { 0, 5, 1 },
  { 0, 6, 0 },
  { 1, 0, 5 },
  { 1, 1, 4 },
  { 1, 2, 3 },
  { 1, 3, 2 },
  { 1, 4, 1 },
  { 1, 5, 0 },
  { 2, 0, 4 },
  { 2, 1, 3 },
  { 2, 2, 2 },
  { 2, 3, 1 },
  { 2, 4, 0 },
  { 3, 0, 3 },
  { 3, 1, 2 },
  { 3, 2, 1 },
  { 3, 3, 0 },
  { 4, 0, 2 },
  { 4, 1, 1 },
  { 4, 2, 0 },
  { 5, 0, 1 },
  { 5, 1, 0 },
  { 6, 0, 0 },
  { 0, 0, 7 },
  { 0, 1, 6 },
  { 0, 2, 5 },
  { 0, 3, 4 },
  { 0, 4, 3 },
  { 0, 5, 2 },
  { 0, 6, 1 },
  { 0, 7, 0 },
  { 1, 0, 6 },
  { 1, 1, 5 },
  { 1, 2, 4 },
  { 1, 3, 3 },
  { 1, 4, 2 },
  { 1, 5, 1 },
  { 1, 6, 0 },
  { 2, 0, 5 },
  { 2, 1, 4 },
  { 2, 2, 3 },
  { 2, 3, 2 },
  { 2, 4, 1 },
  { 2, 5, 0 },
  { 3, 0, 4 },
  { 3, 1, 3 },
  { 3, 2, 2 },
  { 3, 3, 1 },
  { 3, 4, 0 },
  { 4, 0, 3 },
  { 4, 1, 2 },
  { 4, 2, 1 },
  { 4, 3, 0 },
  { 5, 0, 2 },
  { 5, 1, 1 },
  { 5, 2, 0 },
  { 6, 0, 1 },
  { 6, 1, 0 },
  { 7, 0, 0 },
  { 0, 0, 8 },
  { 0, 1, 7 },
  { 0, 2, 6 },
  { 0, 3, 5 },
  { 0, 4, 4 },
  { 0, 5, 3 },
  { 0, 6, 2 },
  { 0, 7, 1 },
  { 0, 8, 0 },
  { 1, 0, 7 },
  { 1, 1, 6 },
  { 1, 2, 5 },
  { 1, 3, 4 },
  { 1, 4, 3 },
  { 1, 5, 2 },
  { 1, 6, 1 },
  { 1, 7, 0 },
  { 2, 0, 6 },
  { 2, 1, 5 },
  { 2, 2, 4 },
  { 2, 3, 3 },
  { 2, 4, 2 },
  { 2, 5, 1 },
  { 2, 6, 0 },
  { 3, 0, 5 },
  { 3, 1, 4 },
  { 3, 2, 3 },
  { 3, 3, 2 },
  { 3, 4, 1 },
  { 3, 5, 0 },
  { 4, 0, 4 },
  { 4, 1, 3 },
  { 4, 2, 2 },
  { 4, 3, 1 },
  { 4, 4, 0 },
  { 5, 0, 3 },
  { 5, 1, 2 },
  { 5, 2, 1 },
  { 5, 3, 0 },
  { 6, 0, 2 },
  { 6, 1, 1 },
  { 6, 2, 0 },
  { 7, 0, 1 },
  { 7, 1, 0 },
  { 8, 0, 0 },
  { 0, 0, 9 },
  { 0, 1, 8 },
  { 0, 2, 7 },
  { 0, 3, 6 },
  { 0, 4, 5 },
  { 0, 5, 4 },
  { 0, 6, 3 },
  { 0, 7, 2 },
  { 0, 8, 1 },
  { 0, 9, 0 },
  { 1, 0, 8 },
  { 1, 1, 7 },
  { 1, 2, 6 },
  { 1, 3, 5 },
  { 1, 4, 4 },
  { 1, 5, 3 },
  { 1, 6, 2 },
  { 1, 7, 1 },
  { 1, 8, 0 },
  { 2, 0, 7 },
  { 2, 1, 6 },
  { 2, 2, 5 },
  { 2, 3, 4 },
  { 2, 4, 3 },
  { 2, 5, 2 },
  { 2, 6, 1 },
  { 2, 7, 0 },
  { 3, 0, 6 },
  { 3, 1, 5 },
  { 3, 2, 4 },
  { 3, 3, 3 },
  { 3, 4, 2 },
  { 3, 5, 1 },
  { 3, 6, 0 },
  { 4, 0, 5 },
  { 4, 1, 4 },
  { 4, 2, 3 },
  { 4, 3, 2 },
  { 4, 4, 1 },
  { 4, 5, 0 },
  { 5, 0, 4 },
  { 5, 1, 3 },
  { 5, 2, 2 },
  { 5, 3, 1 },
  { 5, 4, 0 },
  { 6, 0, 3 },
  { 6, 1, 2 },
  { 6, 2, 1 },
  { 6, 3, 0 },
  { 7, 0, 2 },
  { 7, 1, 1 },
  { 7, 2, 0 },
  { 8, 0, 1 },
  { 8, 1, 0 },
  { 9, 0, 0 },
  { 0, 0, 10 },
  { 0, 1, 9 },
  { 0, 2, 8 },
  { 0, 3, 7 },
  { 0, 4, 6 },
  { 0, 5, 5 },
  { 0, 6, 4 },
  { 0, 7, 3 },
  { 0, 8, 2 },
  { 0, 9, 1 },
  { 0, 10, 0 },
  { 1, 0, 9 },
  { 1, 1, 8 },
  { 1, 2, 7 },
  { 1, 3, 6 },
  { 1, 4, 5 },
  { 1, 5, 4 },
  { 1, 6, 3 },
  { 1, 7, 2 },
  { 1, 8, 1 },
  { 1, 9, 0 },
  { 2, 0, 8 },
  { 2, 1, 7 },
  { 2, 2, 6 },
  { 2, 3, 5 },
  { 2, 4, 4 },
  { 2, 5, 3 },
  { 2, 6, 2 },
  { 2, 7, 1 },
  { 2, 8, 0 },
  { 3, 0, 7 },
  { 3, 1, 6 },
  { 3, 2, 5 },
  { 3, 3, 4 },
  { 3, 4, 3 },
  { 3, 5, 2 },
  { 3, 6, 1 },
  { 3, 7, 0 },
  { 4, 0, 6 },
  { 4, 1, 5 },
  { 4, 2, 4 },
  { 4, 3, 3 },
  { 4, 4, 2 },
  { 4, 5, 1 },
  { 4, 6, 0 },
  { 5, 0, 5 },
  { 5, 1, 4 },
  { 5, 2, 3 },
  { 5, 3, 2 },
  { 5, 4, 1 },
  { 5, 5, 0 },
  { 6, 0, 4 },
  { 6, 1, 3 },
  { 6, 2, 2 },
  { 6, 3, 1 },
  { 6, 4, 0 },
  { 7, 0, 3 },
  { 7, 1, 2 },
  { 7, 2, 1 },
  { 7, 3, 0 },
  { 8, 0, 2 },
  { 8, 1, 1 },
  { 8, 2, 0 },
  { 9, 0, 1 },
  { 9, 1, 0 },
  { 10, 0, 0 },
  { 0, 0, 11 },
  { 0, 1, 10 },
  { 0, 2, 9 },
  { 0, 3, 8 },
  { 0, 4, 7 },
  { 0, 5, 6 },
  { 0, 6, 5 },
  { 0, 7, 4 },
  { 0, 8, 3 },
  { 0, 9, 2 },
  { 0, 10, 1 },
  { 0, 11, 0 },
  { 1, 0, 10 },
  { 1, 1, 9 },
  { 1, 2, 8 },
  { 1, 3, 7 },
  { 1, 4, 6 },
  { 1, 5, 5 },
  { 1, 6, 4 },
  { 1, 7, 3 },
  { 1, 8, 2 },
  { 1, 9, 1 },
  { 1, 10, 0 },
  { 2, 0, 9 },
  { 2, 1, 8 },
  { 2, 2, 7 },
  { 2, 3, 6 },
  { 2, 4, 5 },
  { 2, 5, 4 },
  { 2, 6, 3 },
  { 2, 7, 2 },
  { 2, 8, 1 },
  { 2, 9, 0 },
  { 3, 0, 8 },
  { 3, 1, 7 },
  { 3, 2, 6 },
  { 3, 3, 5 },
  { 3, 4, 4 },
  { 3, 5, 3 },
  { 3, 6, 2 },
  { 3, 7, 1 },
  { 3, 8, 0 },
  { 4, 0, 7 },
  { 4, 1, 6 },
  { 4, 2, 5 },
  { 4, 3, 4 },
  { 4, 4, 3 },
  { 4, 5, 2 },
  { 4, 6, 1 },
  { 4, 7, 0 },
  { 5, 0, 6 },
  { 5, 1, 5 },
  { 5, 2, 4 },
  { 5, 3, 3 },
  { 5, 4, 2 },
  { 5, 5, 1 },
  { 5, 6, 0 },
  { 6, 0, 5 },
  { 6, 1, 4 },
  { 6, 2, 3 },
  { 6, 3, 2 },
  { 6, 4, 1 },
  { 6, 5, 0 },
  { 7, 0, 4 },
  { 7, 1, 3 },
  { 7, 2, 2 },
  { 7, 3, 1 },
  { 7, 4, 0 },
  { 8, 0, 3 },
  { 8, 1, 2 },
  { 8, 2, 1 },
  { 8, 3, 0 },
  { 9, 0, 2 },
  { 9, 1, 1 },
  { 9, 2, 0 },
  { 10, 0, 1 },
  { 10, 1, 0 },
  { 11, 0, 0 },
  { 0, 0, 12 },
  { 0, 1, 11 },
  { 0, 2, 10 },
  { 0, 3, 9 },
  { 0, 4, 8 },
  { 0, 5, 7 },
  { 0, 6, 6 },
  { 0, 7, 5 },
  { 0, 8, 4 },
  { 0, 9, 3 },
  { 0, 10, 2 },
  { 0, 11, 1 },
  { 0, 12, 0 },
  { 1, 0, 11 },
  { 1, 1, 10 },
  { 1, 2, 9 },
  { 1, 3, 8 },
  { 1, 4, 7 },
  { 1, 5, 6 },
  { 1, 6, 5 },
  { 1, 7, 4 },
  { 1, 8, 3 },
  { 1, 9, 2 },
  { 1, 10, 1 },
  { 1, 11, 0 },
  { 2, 0, 10 },
  { 2, 1, 9 },
  { 2, 2, 8 },
  { 2, 3, 7 },
  { 2, 4, 6 },
  { 2, 5, 5 },
  { 2, 6, 4 },
  { 2, 7, 3 },
  { 2, 8, 2 },
  { 2, 9, 1 },
  { 2, 10, 0 },
  { 3, 0, 9 },
  { 3, 1, 8 },
  { 3, 2, 7 },
  { 3, 3, 6 },
  { 3, 4, 5 },
  { 3, 5, 4 },
  { 3, 6, 3 },
  { 3, 7, 2 },
  { 3, 8, 1 },
  { 3, 9, 0 },
  { 4, 0, 8 },
  { 4, 1, 7 },
  { 4, 2, 6 },
  { 4, 3, 5 },
  { 4, 4, 4 },
  { 4, 5, 3 },
  { 4, 6, 2 },
  { 4, 7, 1 },
  { 4, 8, 0 },
  { 5, 0, 7 },
  { 5, 1, 6 },
  { 5, 2, 5 },
  { 5, 3, 4 },
  { 5, 4, 3 },
  { 5, 5, 2 },
  { 5, 6, 1 },
  { 5, 7, 0 },
  { 6, 0, 6 },
  { 6, 1, 5 },
  { 6, 2, 4 },
  { 6, 3, 3 },
  { 6, 4, 2 },
  { 6, 5, 1 },
  { 6, 6, 0 },
  { 7, 0, 5 },
  { 7, 1, 4 },
  { 7, 2, 3 },
  { 7, 3, 2 },
  { 7, 4, 1 },
  { 7, 5, 0 },
  { 8, 0, 4 },
  { 8, 1, 3 },
  { 8, 2, 2 },
  { 8, 3, 1 },
  { 8, 4, 0 },
  { 9, 0, 3 },
  { 9, 1, 2 },
  { 9, 2, 1 },
  { 9, 3, 0 },
  { 10, 0, 2 },
  { 10, 1, 1 },
  { 10, 2, 0 },
  { 11, 0, 1 },
  { 11, 1, 0 },
  { 12, 0, 0 },
};

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
  fs.close();

  readLegacy(bytes.data(), size);
}
void HPOctreeBuilder::readLegacy(unsigned char *bytes, unsigned size)
{
  printf("size: %u\n", size);

  assert(size > 0);

  unsigned nCoeffs = *((unsigned *)(bytes + 0));
  printf("nCoeffs: %u\n", nCoeffs);
  coeffStore.resize(nCoeffs);
  memcpy(coeffStore.data(), bytes + sizeof(unsigned), sizeof(double) * nCoeffs);

  unsigned nNodes = *((unsigned *)(bytes + sizeof(unsigned) + sizeof(double) * nCoeffs));
  nodes.resize(nNodes);
  memcpy(nodes.data(), bytes + sizeof(unsigned) + sizeof(double) * nCoeffs + sizeof(unsigned), sizeof(NodeLegacy) * nNodes);

  config = *(ConfigLegacy *)(bytes + sizeof(unsigned) + sizeof(double) * nCoeffs + sizeof(unsigned) + sizeof(NodeLegacy) * nNodes);

  config.IsValid();
  configRootCentre = 0.5f * (config.root.m_min + config.root.m_max);
  configRootInvSizes = 1.0f / (config.root.m_max - config.root.m_min);

  readLegacy(coeffStore, nodes);
}

double HPOctreeBuilder::QueryLegacy(const float3 &pt_) const
{
  // Move to unit cube
  const float3 pt = (pt_ - configRootCentre) * configRootInvSizes;
  //printf("centre: %f %f %f\n", configRootCentre.x, configRootCentre.y, configRootCentre.z);
  //printf("invSizes: %f %f %f\n", configRootInvSizes.x, configRootInvSizes.y, configRootInvSizes.z);

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
      float d2 = FApprox(allToLeafRemap[childIdx], pt);
      //printf("d1: %f, d2: %f\n", d1, d2);
      return FApprox(allToLeafRemap[childIdx], pt);
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
  //printf("aabb: (%f, %f, %f) (%f, %f, %f)\n", aabb_.m_min.x, aabb_.m_min.y, aabb_.m_min.z, aabb_.m_max.x, aabb_.m_max.y, aabb_.m_max.z);
  //printf("unitPt: %f, %f, %f\n", unitPt.x, unitPt.y, unitPt.z);

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
  const float3 unitPt = 2.0f*(pt - min_pos) / (max_pos - min_pos) - 1.0f;
  //printf("px py pz sz: %f, %f, %f, %f\n", px, py, pz, sz);
  //printf("min_pos: %f, %f, %f\n", min_pos.x, min_pos.y, min_pos.z);
  //printf("max_pos: %f, %f, %f\n", max_pos.x, max_pos.y, max_pos.z);
  //printf("unitPt: %f, %f, %f\n", unitPt.x, unitPt.y, unitPt.z);

  unsigned depth = octree.nodes[nodeId].degree_lod & 0x0000FFFF;
  unsigned degree = octree.nodes[nodeId].degree_lod >> 16;

  // Create lookup table for pt_
  float LpXLookup[BASIS_MAX_DEGREE][3];
  for (uint32_t i = 0; i < 3; ++i)
  {
    // Constant
    LpXLookup[0][i] = NormalisedLengths[0][depth];

    // Initial values for recurrence
    float LjMinus2 = 0.0;
    float LjMinus1 = 1.0;
    float Lj = 1.0;

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
  float fApprox = 0.0;
  int valuesIdx = 0;
  for (int p = 0; p <= BASIS_MAX_DEGREE; ++p)
  {
    for (int k1 = 0; k1 <= p; ++k1)
    {
      for (int k2 = 0; k2 <= p - k1; ++k2)
      {
        int k3 = p - k1 - k2;
        float Lp = LpXLookup[k1][0]*LpXLookup[k2][1]*LpXLookup[k3][2];

        fApprox += octree.data[octree.nodes[nodeId].data_offset + valuesIdx] * Lp;
        valuesIdx++;
        if (valuesIdx >= LegendreCoeffientCount[degree])
          return fApprox;
      }
    }
  }
  return 1000.0f;
}

void HPOctreeBuilder::readLegacy(const std::vector<double> &coeffStore, const std::vector<NodeLegacy> &nodes)
{
  octree.data.reserve(coeffStore.size());
  octree.nodes.reserve(nodes.size());
  allToLeafRemap.resize(nodes.size());
  
  for (int i = 0; i < nodes.size(); i++)
  {
    if (nodes[i].basis.degree != (BASIS_MAX_DEGREE + 1))
    {
      SdfHPOctreeNode node;
      float sz = 1 << (nodes[i].depth + 1);
      uint3 p = uint3(0.5f*sz*(nodes[i].aabb.m_min + 1.0f));
      assert(p.x < (1 << 16) && p.y < (1 << 16) && p.z < (1 << 16));

      node.pos_xy = (p.x << 16) | p.y;
      node.pos_z_lod_size = (p.z << 16) | (1 << (nodes[i].depth + 1));
      node.degree_lod = (nodes[i].basis.degree << 16) | nodes[i].depth;
      node.data_offset = octree.data.size();

      allToLeafRemap[i] = octree.nodes.size();
      octree.nodes.push_back(node);
      for (int j=0;j<LegendreCoeffientCount[nodes[i].basis.degree];j++)
        octree.data.push_back(coeffStore[nodes[i].basis.coeffsStart+j]);
      
    }
  }

  printf("new size: %d\n nCoeffs: %d\n", (int)octree.nodes.size(), (int)octree.data.size());
}