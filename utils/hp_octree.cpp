#include "hp_octree.h"
#include <thread>

/*
  Constexpr-satisfying versions of Pow and Sqrt.
*/
constexpr double PowConst(const double base_, const size_t pow_)
{
  if (pow_ == 0)
  {
    return 1.0;
  }
  else
  {
    return base_ * PowConst(base_, pow_ - 1);
  }
}
constexpr double SqrtConst(const double x_)
{
  // Newton's Method
  double guess = x_;
  for (size_t i = 0; i < 100; ++i)
  {
    guess = 0.5 * (guess + x_ / guess);
  }

  return guess;
}

/*
  Standard sum to N. store[n] = n * (n + 1) / 2
*/
struct SumToNCalc
{
  constexpr SumToNCalc() : values()
  {
    for (uint32_t i = 0; i <= (4 * BASIS_MAX_DEGREE + 1); ++i)
    {
      uint32_t sum = 0;
      for (uint32_t j = 0; j <= i; ++j)
      {
        sum += j;
      }

      values[i] = sum;
    }
  }
  uint32_t values[(4 * BASIS_MAX_DEGREE + 1) + 1];
};
static constexpr const uint32_t (&SumToN)[(4 * BASIS_MAX_DEGREE + 1) + 1] = SumToNCalc().values;

/*
  In equation (4), precomputes the sqrt portions under the assumption that the internal octree
  volume is taken to be the unit cube.
*/
struct NormalisedLengthsCalc
{
  constexpr NormalisedLengthsCalc() : values()
  {
    for (uint32_t i = 0; i <= BASIS_MAX_DEGREE; ++i)
    {
      for (uint32_t j = 0; j <= TREE_MAX_DEPTH; ++j)
      {
        // Sizes are 1 at level 0, 0.5 at level 1, ... , 2^-n at level n
        values[i][j] = SqrtConst((2.0 * i + 1.0) * PowConst(2.0, j));
      }
    }
  }
  double values[BASIS_MAX_DEGREE + 1][TREE_MAX_DEPTH + 1];
};
static constexpr const double (&NormalisedLengths)[BASIS_MAX_DEGREE + 1][TREE_MAX_DEPTH + 1] = NormalisedLengthsCalc().values;

/*
  Stores the number of coefficient in each basis polynomial. I.e. store[n] tells us that
  the nth Legendre polynomial contains store[n] coefficients. If the dimension is M and
  the max basis degree is N, then the formula is:

          store_M^N[n] = 1 / M! * (N + 1) * (N + 2) * ... * (N + M)
*/
struct LegendreCoefficientCountCalc
{
  constexpr LegendreCoefficientCountCalc() : values()
  {
    const double f = 1.0 / 6.0;

    for (uint32_t i = 0; i <= BASIS_MAX_DEGREE; ++i)
    {
      values[i] = (uint32_t)(f * (i + 1) * (i + 2) * (i + 3));
    }
  }
  uint32_t values[BASIS_MAX_DEGREE + 1];

  static constexpr const uint32_t Size()
  {
    const double f = 1.0 / 6.0;
    return (uint32_t)(f * (BASIS_MAX_DEGREE + 1) * (BASIS_MAX_DEGREE + 2) * (BASIS_MAX_DEGREE + 3));
  }
};
static constexpr const uint32_t (&LegendreCoeffientCount)[BASIS_MAX_DEGREE + 1] = LegendreCoefficientCountCalc().values;

/*
  Stores the constants used in each term of the reccurence relation definition for Legendre polynomials.
  Removes the need to perform repeated divides when querying the tree.
*/
struct LegendreCoefficientCalc
{
  constexpr LegendreCoefficientCalc() : values()
  {
    values[0][0] = 0.0;
    values[0][1] = 0.0;

    for (uint32_t i = 1; i <= BASIS_MAX_DEGREE; ++i)
    {
      values[i][0] = (2.0 * i - 1.0) / i;
      values[i][1] = (i - 1.0) / i;
    }
  }
  double values[BASIS_MAX_DEGREE + 1][2];
};
static constexpr const double (&LegendreCoefficent)[BASIS_MAX_DEGREE + 1][2] = LegendreCoefficientCalc().values;

/*
  Stores all possible combinations of BasisIndex for dimension M and max basis degree N. Saves having
  to compute these on the fly or store them inside the node (both very bad ideas!).
*/
struct BasisIndexValuesCalc
{
  constexpr BasisIndexValuesCalc() : values()
  {
    uint32_t valuesIdx = 0;
    for (uint32_t p = 0; p <= BASIS_MAX_DEGREE; ++p)
    {
      for (uint32_t i = 0; i <= p; ++i)
      {
        for (uint32_t j = 0; j <= p - i; ++j)
        {
          for (uint32_t k = 0; k <= p - i - j; ++k)
          {
            if ((i + j + k) == p)
            {
              values[valuesIdx][0] = i;
              values[valuesIdx][1] = j;
              values[valuesIdx][2] = k;
              valuesIdx++;
            }
          }
        }
      }
    }
  }
  uint32_t values[LegendreCoefficientCountCalc::Size()][3];
};
static constexpr const uint32_t (&BasisIndexValues)[LegendreCoefficientCountCalc::Size()][3] = BasisIndexValuesCalc().values;

/*
    Stores adjacent face pairs used during the continuity correction. The lookup table is additive (i.e. the lookup table for
  dim_{n - 1} is a subset of the lookup table for dim_{n}) so we iterate through dims 0, 1 and 2 to get the final set.
*/
struct SharedFaceLookupCalc
{
  constexpr SharedFaceLookupCalc() : values()
  {
    constexpr unsigned char ourDim = 3;

    for (unsigned char i = 0; i < ourDim; ++i)
    {
      // Axis = 0, MAX_DIM = 3 => modVal1 = 1, modVal = 2, valsPerMod = 4
      unsigned char valueIdxs[2] = {0};
      unsigned char modVal1 = (1 << i);
      unsigned char modVal = (1 << (i + 1));
      unsigned char valsPerMod = (1 << (ourDim - 1 - i));

      for (unsigned char j = 0; j < valsPerMod; ++j)
      {
        for (unsigned char k = 0; k < modVal1; ++k)
        {
          values[i][valueIdxs[0]++][0] = k + j * modVal;
        }

        for (unsigned char k = modVal1; k < modVal; ++k)
        {
          values[i][valueIdxs[1]++][1] = k + j * modVal;
        }
      }
    }
  }
  size_t values[3][4][2];
};
static constexpr const size_t (&SharedFaceLookup)[3][4][2] = SharedFaceLookupCalc().values;

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
}

double HPOctreeBuilder::Query(const float3 &pt_) const
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

      return FApprox(basis, curChild.aabb, pt, curChild.depth);
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