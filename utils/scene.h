#pragma once

#include "HydraCore3/include/cglobals.h"
#include "HydraCore3/include/crandom.h"
#include "HydraCore3/include/clight.h"
#include "HydraCore3/include/cmaterial.h"
#include "LiteScene/cmesh4.h"
#include "LiteMath/Image2d.h"
#include "utils/Camera.h"

#include <memory>

//It is a structure to store all the scene data
//It can be read from a .xml file (with accompanying binaries)
//It can be saved to an .xml + binaries and it will be a valid Hydra scene
//Currently it supports only basic stuff, so no SDFs, spectral features etc.
struct InstancedMesh
{
  cmesh4::SimpleMesh mesh;
  std::vector<LiteMath::float4x4> transforms;
};

struct HydraScene
{
  std::vector<std::shared_ptr<LiteImage::ICombinedImageSampler>> textures;
  std::vector<Material> materials;
  std::vector<InstancedMesh> meshes;
  std::vector<LightSource> lights;
  std::vector<Camera> cameras;
};

void load_hydra_scene_xml(const std::string &path,       HydraScene &scene);
void save_hydra_scene_xml(const std::string &path, const HydraScene &scene);

void save_hydra_scene_gltf(const std::string &path, const HydraScene &scene);
