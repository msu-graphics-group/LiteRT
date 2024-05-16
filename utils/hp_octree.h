#pragma once

#include <vector>
#include <functional>
#include <fstream>
#include "../sdfScene/sdf_scene.h"
#include "../utils/mesh.h"

void hp_octree_generate_tables();

/*
  HPOctreeBuilder creates hp-adaptive octree from arbitrary distance function or mesh
  It is a wrapper around the hp_octree library https://github.com/jw007123/hp-Adaptive-Signed-Distance-Field-Octree
  Based on paper "An hp-Adaptive Discretization Algorithm for Signed Distance Field Generation."
  It produces an octree with Legandre polinomials of different degrees in every leaf node
*/
class HPOctreeBuilder
{
public:
  struct BuildSettings
  {
    enum class NearnessWeighting
    {
      None = 0,
      Polynomial = 1,
      Exponential = 2
    }; 
    unsigned threads = 1; // number of threads to use, if >1 sdf function must support multi-threading
    float target_error = 5e-7; //distance error to stop expanding the octree
    NearnessWeighting nearness_weighting = NearnessWeighting::Exponential;
    float nearness_weight = 20.0;
    bool enforce_continuity = false; // might crash for larger octrees
    float continuity_strength = 8.0; //only used if enforce_continuity is true
  };
  HPOctreeBuilder();
  double QueryLegacy(const float3& pt_) const;
  void readLegacy(const std::string &path);
  
  //construct hp-adaptive octree from arbitrary distance function in [-1,1]^3 region
  void construct(std::function<float(const float3 &, unsigned thread_idx)> sdf,
                 BuildSettings settings = BuildSettings{1u, 5e-7f, BuildSettings::NearnessWeighting::Exponential, 20.0f, false, 8.0f});
  
  //construct hp-adaptive octree from mesh in [-1,1]^3 region
  //mesh should fit in [-1,1]^3 cube, otherwise octree can be messed up
  //mesh should be watertight, not self-intersecting and have normals pointing out
  //TODO: fix normals direction, detect and fill holes
  void construct(const cmesh4::SimpleMesh &mesh,
                 BuildSettings settings = BuildSettings{8u, 5e-7f, BuildSettings::NearnessWeighting::Exponential, 20.0f, false, 8.0f});
  
  SdfHPOctree octree;
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

  void readLegacy(unsigned char *data, unsigned size);
  void readLegacy(const std::vector<double> &coeffStore, const std::vector<NodeLegacy> &nodes);
  double FApprox(const NodeLegacy::Basis& basis_, const Box3Legacy& aabb_, const float3& pt_, const uint32_t depth_) const;
  float FApprox(uint32_t idx, const float3& pt) const;


  std::vector<double> coeffStore;
	std::vector<NodeLegacy> nodes;
  std::vector<unsigned> allToLeafRemap; 

  ConfigLegacy config;
  float3 configRootCentre;
  float3 configRootInvSizes;
};