#pragma once

#include <vector>
#include <functional>
#include <fstream>
#include "../sdfScene/sdf_scene.h"
#include "../utils/mesh.h"

constexpr static unsigned BASIS_MAX_DEGREE = 12;
constexpr static unsigned TREE_MAX_DEPTH   = 10;

void hp_octree_generate_tables();

class HPOctreeBuilder
{
public:
  HPOctreeBuilder();
  double QueryLegacy(const float3& pt_) const;
  void readLegacy(const std::string &path);
  
private:
  // format from original hp-Adaptive-Signed-Distance-Field-Octree implementation
  struct Box3Legacy
  {
    Box3Legacy() = default;
    Box3Legacy(const float3 &min_, const float3 &max_)
    {
      m_min = min_;
      m_max = max_;
    }
    float3 m_min;
    float3 m_max;
  };
  
  struct NodeLegacy
  {
    NodeLegacy();

    // Nodes are allocated in blocks of 8, so the ith child's ID is childIdx + i
    uint32_t childIdx;

    // Standard AABB store
    Box3Legacy aabb;

    // Stores LegendreCoeffientCount[degree] doubles
    struct Basis
    {
      union
      {
        double *coeffs = nullptr;
        size_t coeffsStart;
      };
      unsigned char degree;
    } basis;

    // Since we're storing an odd number of bytes with basis.degree, we may as well store depth as it'll be padded in anyway
    unsigned char depth;
  };

  struct ConfigLegacy
  {
    /// Reasonable defaults
    ConfigLegacy();

    struct NearnessWeighting
    {
      enum Type : unsigned char
      {
        None = 0,
        Polynomial = 1,
        Exponential = 2
      } type;

      double strength;
    } nearnessWeighting;

    struct Continuity
    {
      bool enforce;
      double strength;
    } continuity;

    bool enableLogging;
    double targetErrorThreshold;
    uint32_t threadCount;

    Box3Legacy root;

    /// Checks that the config has valid parameters
    void IsValid() const;
  };

  void readLegacy(const std::vector<double> &coeffStore, const std::vector<NodeLegacy> &nodes);
  double FApprox(const NodeLegacy::Basis& basis_, const Box3Legacy& aabb_, const float3& pt_, const uint32_t depth_) const;
  float FApprox(uint32_t idx, const float3& pt) const;


  std::vector<double> coeffStore;
	std::vector<NodeLegacy> nodes;

  SdfHPOctree octree;

  ConfigLegacy config;
  float3 configRootCentre;
  float3 configRootInvSizes;
};