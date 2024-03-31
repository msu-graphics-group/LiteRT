#pragma once
#include "gpuReady/sdf_scene.h"
#include <string>

#ifndef KERNEL_SLICER

struct SdfGridView
{
  uint3 size;
  const float *data; //size.x*size.y*size.z values 
};

struct SdfOctreeView
{
  unsigned size;
  const SdfOctreeNode *nodes;
};

struct SdfScene
{
  std::vector<float> parameters;
  std::vector<SdfObject> objects;
  std::vector<SdfConjunction> conjunctions;
  std::vector<NeuralProperties> neural_properties;
};

// all interfaces use SdfSceneView to be independant of how exactly
// SDF scenes are stored
struct SdfSceneView
{
  SdfSceneView() = default;
  SdfSceneView(const SdfScene &scene)
  {
    parameters = scene.parameters.data();
    objects = scene.objects.data();
    conjunctions = scene.conjunctions.data();
    neural_properties = scene.neural_properties.data();

    parameters_count = scene.parameters.size();
    objects_count = scene.objects.size();
    conjunctions_count = scene.conjunctions.size();
    neural_properties_count = scene.neural_properties.size();
  }
  SdfSceneView(const std::vector<float> &_parameters,
               const std::vector<SdfObject> &_objects,
               const std::vector<SdfConjunction> &_conjunctions,
               const std::vector<NeuralProperties> &_neural_properties)
  {
    parameters = _parameters.data();
    objects = _objects.data();
    conjunctions = _conjunctions.data();
    neural_properties = _neural_properties.data();

    parameters_count = _parameters.size();
    objects_count = _objects.size();
    conjunctions_count = _conjunctions.size();
    neural_properties_count = _neural_properties.size();
  }

  const float *parameters;
  const SdfObject *objects;
  const SdfConjunction *conjunctions;
  const NeuralProperties *neural_properties;

  unsigned parameters_count;
  unsigned objects_count;
  unsigned conjunctions_count;
  unsigned neural_properties_count;
};

// perform sphere tracing to find ray intersection with a whole scene and inside given bbox
// dir vector MUST be normalized
float eval_dist_scene(const SdfSceneView &sdf, float3 p);
bool sdf_sphere_tracing(const SdfSceneView &sdf, const LiteMath::float3 &min_pos, const LiteMath::float3 &max_pos, const LiteMath::float3 &pos, const LiteMath::float3 &dir,
                        LiteMath::float3 *surface_pos = nullptr);

// save/load scene
void save_sdf_scene_hydra(const SdfScene &scene, const std::string &folder, const std::string &name);
void save_sdf_scene(const SdfScene &scene, const std::string &path);
void load_sdf_scene(SdfScene &scene, const std::string &path);
void load_neural_sdf_scene_SIREN(SdfScene &scene, const std::string &path); // loads scene from raw SIREN weights file
#endif