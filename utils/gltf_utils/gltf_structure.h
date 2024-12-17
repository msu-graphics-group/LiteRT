#pragma once

#include <string>
#include <vector>
#include "LiteMath/LiteMath.h"
#include <map>

namespace gltf
{
  using LiteMath::float2;
  using LiteMath::float3;
  using LiteMath::float4;
  using LiteMath::float4x4;
  using LiteMath::int2;
  using LiteMath::uint2;

  struct BinaryFile;
  struct Texture;
  struct Node;
  struct TextureFile;

  enum AccessorComponentType
  {
    BYTE = 5120,
    UNSIGNED_BYTE,
    SHORT,
    UNSIGNED_SHORT,
    UNSIGNED_INT = 5125,
    FLOAT
  };

  enum AccessorType
  {
    SCALAR,
    VEC2,
    VEC3,
    VEC4,
    MAT2,
    MAT3,
    MAT4
  };

  enum BufferViewTargetType
  {
    ARRAY_BUFFER = 34962,
    ELEMENT_ARRAY_BUFFER
  };
  enum primitiveAttributeType
  {
    POSITION,
    NORMAL,
    TANGENT,
    TEXCOORD_0,
    TEXCOORD_1,
    COLOR_0,
    JOINTS_0,
    WEIGHTS_0
  };

  enum primitiveRenderingMode
  {
    POINTS,
    LINES,
    LINE_LOOP,
    LINE_STRIP,
    TRIANGLES,
    TRIANGLE_STRIP,
    TRIANGLE_FAN
  };

  enum magFilterType
  {
    MAG_NEAREST = 9728,
    MAG_LINEAR = 9729
  };
  enum minFilterType
  {
    NEAREST = 9728,
    LINEAR = 9729,
    NEAREST_MIPMAP_NEAREST = 9984,
    LINEAR_MIPMAP_NEAREST,
    NEAREST_MIPMAP_LINEAR,
    LINEAR_MIPMAP_LINEAR
  };

  enum wrappingMode
  {
    CLAMP_TO_EDGE = 33071,
    MIRRORED_REPEAT = 33648,
    REPEAT = 10497
  };

  enum cameraType
  {
    PERSPECTIVE,
    ORTHOGRAPHIC
  };

  enum materialAlphaMode
  {
    OPAQUE,
    MASK,
    BLEND
  };
  struct Asset
  {
    const std::string copyright{"2021 (c) Sammael"};
    const std::string version{"2.0"};
    const std::string generator{"Sammael's glTF writer"};
  };

  struct Scene
  {
    std::vector<int> nodes;
  };
  struct Node
  {
    int camera = -1;
    std::vector<int> child_nodes;
    float4x4 transform = float4x4();
    int mesh = -1;
    float4 rotation = float4(0, 0, 0, 1);
    float3 scale = float3(0.1, 0.1, 0.1);
    float3 translation = float3(0, 0, 0);
  };
  struct Primitive
  {
    std::map<primitiveAttributeType, int> attributes; // int is a number of accessor with data
    int indicies;
    int material = -1;
    primitiveRenderingMode mode = primitiveRenderingMode::TRIANGLES;
  };
  struct Mesh
  {
    std::vector<Primitive> primitives;
  };
  struct Buffer
  {
    int byte_length;
    BinaryFile *data;
  };
  struct BufferView
  {
    int buffer;
    int byte_length;
    int byte_offset = 0;
    int byte_stride = -1;
    BufferViewTargetType target;
  };
  struct Accessor
  {
    int buffer_view;
    int byte_offset = 0;
    AccessorComponentType componentType;
    AccessorType type;
    int count;
    std::vector<float> max_values;
    std::vector<float> min_values;
  };
  struct PerspectiveCamera
  {
    float yfov = LiteMath::M_PI / 2;
    float znear = 0.1;
    float zfar = 10000;
  };
  struct OrthographicCamera
  {
    float xmag;
    float ymag;
    float zfar;
    float znear;
  };
  struct Camera
  {
    cameraType type;
    PerspectiveCamera persp;
    OrthographicCamera ortho;
  };
  struct Image
  {
    TextureFile *picture;
  };
  struct TextureUsage
  {
    int texture_index = -1;
    int texCoord = 1;
  };
  struct Material
  {
    float3 emissive_factor = float3(0, 0, 0);
    float alpha_cutoff = 0.5;
    bool double_sided = false;
    materialAlphaMode alpha_mode = materialAlphaMode::MASK;

    TextureUsage baseColorTex;
    TextureUsage metallicRoughnessTex;
    TextureUsage normalTex;
    TextureUsage occlusionTex;
    TextureUsage emissiveTex;

    float roughness = 0.95;
    float metallic = 0.25;
  };
  struct Sampler
  {
    magFilterType magFilter = magFilterType::MAG_LINEAR;
    minFilterType minFilter = minFilterType::LINEAR_MIPMAP_LINEAR;
    wrappingMode wrapS = wrappingMode::CLAMP_TO_EDGE;
    wrappingMode wrapT = wrappingMode::CLAMP_TO_EDGE;
  };
  struct Texture
  {
    int sampler;
    int image;
  };
  struct File
  {
    Asset asset;
    int main_scene = 0;
    std::vector<Scene> scenes;
    std::vector<Node> nodes;
    std::vector<Mesh> meshes;
    std::vector<Buffer> buffers;
    std::vector<BufferView> buffer_views;
    std::vector<Accessor> accessors;
    std::vector<Texture> textures;
    std::vector<Sampler> samplers;
    std::vector<Material> materials;
    std::vector<Image> images;
    std::vector<Camera> cameras;
  };
  struct BinaryFile
  {
    char *data = nullptr;
    int cur_size = 0;
    int max_size;
    std::string file_name;
  };
  struct TextureFile
  {
    bool existed_texture = true;
    std::string file_name;
  };
  struct glBFile
  {
  };
  struct FullData
  {
    File gltf_file;
    std::vector<BinaryFile> pos_binary_files;
    std::vector<BinaryFile> ind_binary_files;
    std::vector<BinaryFile> norm_binary_files;
    std::vector<BinaryFile> tc_binary_files;
    std::vector<BinaryFile> other_binary_files;
    std::vector<TextureFile> textures_files;
    std::vector<glBFile> glb_files;
  };
}