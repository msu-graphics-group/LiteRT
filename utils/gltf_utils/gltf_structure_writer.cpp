#include "gltf_structure.h"
#include <iostream>
#include <fstream>
#include "utils/common/matrix_transform.h"

namespace gltf
{
  int accessorComponentTypeSizes[6] = {1, 1, 2, 2, 4, 4};
  int AccessorTypeComponents[7] = {1, 2, 3, 4, 4, 9, 16};
  std::string AccessorTypeNames[7] = {"SCALAR", "VEC2", "VEC3", "VEC4", "MAT2", "MAT3", "MAT4"};
  std::string primitiveAttributeTypeNames[8] = {"POSITION", "NORMAL", "TANGENT", "TEXCOORD_0", "TEXCOORD_1",
                                                "COLOR_0", "JOINTS_0", "WEIGHTS_0"};
  std::string cameraTypeNames[2] = {"perspective", "orthographic"};
  std::string materialAlphaModeNames[3] = {"OPAQUE", "MASK", "BLEND"};
  bool write_to_json(FullData &fullData, std::string name)
  {
    std::string str;
    auto &a = fullData.gltf_file;
    str += "{\n";

    str += "\"asset\" : { \n";
    str += "\"copyright\" :\"" + a.asset.copyright + "\",\n";
    str += "\"generator\" :\"" + a.asset.generator + "\",\n";
    str += "\"version\" :\"" + a.asset.version + "\"\n";
    str += "}";

    if (!a.buffers.empty())
    {
      str += ",\n\"buffers\" :[\n";
      for (int i = 0; i < a.buffers.size(); i++)
      {
        str += "{";
        auto &b = a.buffers[i];
        str += "\"byteLength\" :" + std::to_string(b.byte_length) + ",\n";
        str += "\"uri\" :\"" + b.data->file_name + "\"\n";
        str += "}";
        if (i != a.buffers.size() - 1)
          str += ",\n";
      }

      str += "]";
    }

    if (!a.buffer_views.empty())
    {
      str += ",\n\"bufferViews\" :[\n";
      for (int i = 0; i < a.buffer_views.size(); i++)
      {
        str += "{";
        auto &b = a.buffer_views[i];
        str += "\"buffer\" :" + std::to_string(b.buffer) + ",\n";
        str += "\"byteOffset\" :" + std::to_string(b.byte_offset) + ",\n";
        str += "\"byteLength\" :" + std::to_string(b.byte_length) + ",\n";
        if (b.byte_stride >= 0)
          str += "\"byteStride\" :" + std::to_string(b.byte_stride) + ",\n";
        str += "\"target\" :" + std::to_string((uint)b.target) + "\n";
        str += "}";
        if (i != a.buffer_views.size() - 1)
          str += ",\n";
      }

      str += "]";
    }

    if (!a.cameras.empty())
    {
      str += ",\n\"cameras\" :[\n";
      for (int i = 0; i < a.cameras.size(); i++)
      {
        str += "{";
        auto &b = a.cameras[i];

        str += "}";
        if (i != a.cameras.size() - 1)
          str += ",\n";
      }

      str += "]";
    }

    if (!a.nodes.empty())
    {
      str += ",\n\"nodes\" :[\n";
      for (int i = 0; i < a.nodes.size(); i++)
      {
        str += "{";
        auto &b = a.nodes[i];
        bool first = true;
        if (b.mesh >= 0)
        {
          if (!first)
          {
            str += ",\n";
          }
          str += "\"mesh\" :" + std::to_string(b.mesh);
          first = false;
        }
        if (b.camera >= 0)
        {
          if (!first)
          {
            str += ",\n";
          }
          str += "\"camera\" :" + std::to_string(b.camera);
          first = false;
        }
        if (b.mesh >= 0)
        {
          if (b.use_RTS == false)
          {
            float4x4 tr = b.transform;
            // tr *= LiteMath::translate(tr, b.translation);
            tr = LiteMath::scale(float4x4(), b.scale) * tr;
            if (!first)
            {
              str += ",\n";
            }
            str += "\"matrix\" : [";
            for (int j = 0; j < 4; j++)
            {
              for (int k = 0; k < 4; k++)
              {
                str += std::to_string(tr(k, j));
                if (j < 3 || k < 3)
                  str += ", ";
              }
            }
            str += "]";
            first = false;
          }
          else
          {
            if (!first)
            {
              str += ",\n";
            }
            str += "\"rotation\" : [" + std::to_string(b.rotation.x) + ", " + std::to_string(b.rotation.y) + ", " +
                  std::to_string(b.rotation.z) + ", " + std::to_string(b.rotation.w) + "]";
            first = false;

            if (!first)
            {
              str += ",\n";
            }
            str += "\"translation\" : [" + std::to_string(b.translation.x) + ", " + std::to_string(b.translation.y) + ", " +
                  std::to_string(b.translation.z) + "]";
            first = false;
  
            if (!first)
            {
              str += ",\n";
            }
            str += "\"scale\" : [" + std::to_string(b.scale.x) + ", " + std::to_string(b.scale.y) + ", " +
                  std::to_string(b.scale.z) + "]";
            first = false;
          }
        }
        if (!b.child_nodes.empty())
        {
          str += "\"children\" :";
          str += "[";
          for (int j = 0; j < b.child_nodes.size(); j++)
          {
            str += std::to_string(b.child_nodes[j]);
            if (j != b.child_nodes.size() - 1)
              str += ",";
          }
          str += "]\n";
        }
        str += "\n}";
        if (i != a.nodes.size() - 1)
          str += ",\n";
      }

      str += "]";
    }

    if (!a.scenes.empty())
    {
      str += ",\n\"scenes\" :[\n";
      for (int i = 0; i < a.scenes.size(); i++)
      {
        str += "{\n";
        auto &b = a.scenes[i];
        if (!b.nodes.empty())
        {
          str += "\"nodes\" :";
          str += "[";
          for (int j = 0; j < b.nodes.size(); j++)
          {
            str += std::to_string(b.nodes[j]);
            if (j != b.nodes.size() - 1)
              str += ",";
          }
          str += "]\n";
        }
        str += "}";
        if (i != a.scenes.size() - 1)
          str += ",\n";
      }

      str += "]";
    }

    if (!a.accessors.empty())
    {
      str += ",\n\"accessors\" :[\n";
      for (int i = 0; i < a.accessors.size(); i++)
      {
        str += "{";
        auto &b = a.accessors[i];
        str += "\"bufferView\" :" + std::to_string(b.buffer_view) + ",\n";
        str += "\"byteOffset\" :" + std::to_string(b.byte_offset) + ",\n";
        str += "\"componentType\" :" + std::to_string((int)b.componentType) + ",\n";
        str += "\"count\" :" + std::to_string(b.count) + ",\n";
        str += "\"type\" :\"" + AccessorTypeNames[(int)(b.type)] + "\"";

        if (b.max_values.size() == b.min_values.size() && b.max_values.size() > 0)
        {
          str += ",\n\"max\" : [";
          for (int j = 0; j < b.max_values.size(); j++)
          {
            str += std::to_string(b.max_values[j]);
            if (j != b.max_values.size() - 1)
              str += ",";
          }
          str += "],\n";

          str += "\"min\" : [";

          for (int j = 0; j < b.min_values.size(); j++)
          {
            str += std::to_string(b.min_values[j]);
            if (j != b.min_values.size() - 1)
              str += ",";
          }
          str += "]\n";
        }
        str += "}";
        if (i != a.accessors.size() - 1)
          str += ",\n";
      }

      str += "]";
    }

    if (!a.meshes.empty())
    {
      str += ",\n\"meshes\" :[\n";
      for (int i = 0; i < a.meshes.size(); i++)
      {
        str += "{";
        auto &b = a.meshes[i];

        if (!b.primitives.empty())
        {
          str += "\"primitives\": [\n";
          for (int i = 0; i < b.primitives.size(); i++)
          {
            str += "{";
            auto &c = b.primitives[i];

            if (!c.attributes.empty())
            {
              str += "\"attributes\": {\n";
              int i = 0;
              for (auto attr : c.attributes)
              {
                std::string attr_name = primitiveAttributeTypeNames[(int)attr.first];
                str += "\"" + attr_name + "\" : " + std::to_string(attr.second);
                if (i != c.attributes.size() - 1)
                  str += ",\n";
                i++;
              }

              str += "}";
            }

            str += " ,\n\"indices\" :" + std::to_string(c.indicies);
            if (c.material >= 0)
            {
              str += " ,\n\"material\" :" + std::to_string(c.material);
            }
            if (c.mode != primitiveRenderingMode::TRIANGLES)
            {
              str += " ,\n\"mode\" :" + std::to_string((int)c.mode);
            }
            str += "\n}";
            if (i != b.primitives.size() - 1)
              str += ",\n";
          }

          str += "]";
        }

        str += "}";
        if (i != a.meshes.size() - 1)
          str += ",\n";
      }

      str += "]";
    }
    if (!a.images.empty())
    {
      str += ",\n\"images\" :[\n";
      for (int i = 0; i < a.images.size(); i++)
      {
        str += "{\n";
        auto &b = a.images[i];
        str += "\"uri\": \"" + b.picture->file_name + "\"";
        str += "}";
        if (i != a.images.size() - 1)
          str += ",\n";
      }

      str += "]";
    }
    if (!a.textures.empty())
    {
      str += ",\n\"textures\" :[\n";
      for (int i = 0; i < a.textures.size(); i++)
      {
        str += "{\n";
        auto &b = a.textures[i];
        str += "\"sampler\":" + std::to_string(b.sampler) + ",\n";
        str += "\"source\":" + std::to_string(b.image);
        str += "}";
        if (i != a.textures.size() - 1)
          str += ",\n";
      }

      str += "]";
    }
    if (!a.samplers.empty())
    {
      str += ",\n\"samplers\" :[\n";
      for (int i = 0; i < a.samplers.size(); i++)
      {
        str += "{\n";
        auto &b = a.samplers[i];
        str += "\"magFilter\":" + std::to_string((int)b.magFilter) + ",\n";
        str += "\"minFilter\":" + std::to_string((int)b.minFilter) + ",\n";
        str += "\"wrapS\":" + std::to_string((int)b.wrapS) + ",\n";
        str += "\"wrapT\":" + std::to_string((int)b.wrapT) + "\n";
        str += "}";
        if (i != a.samplers.size() - 1)
          str += ",\n";
      }

      str += "]";
    }
    if (!a.materials.empty())
    {
      str += ",\n\"materials\" :[\n";
      for (int i = 0; i < a.materials.size(); i++)
      {
        str += "{\n";
        auto &b = a.materials[i];

        str += "\"emissiveFactor\": [" + std::to_string(b.emissive_factor.x) + "," + std::to_string(b.emissive_factor.y) +
               "," + std::to_string(b.emissive_factor.z) + "],\n";
        if (b.alpha_mode == materialAlphaMode::MASK)
          str += "\"alphaCutoff\": " + std::to_string(b.alpha_cutoff) + ",\n";
        str += "\"doubleSided\": ";
        b.double_sided ? str += "true,\n" : str += "false,\n";
        str += "\"alphaMode\": \"" + materialAlphaModeNames[(int)b.alpha_mode] + "\",\n";

        if (b.normalTex.texture_index >= 0)
        {
          str += "\"normalTexture\": {\n";
          str += "\"index\": " + std::to_string(b.normalTex.texture_index) + ",\n";
          str += "\"texCoord\": " + std::to_string(b.normalTex.texCoord) + "\n";
          str += "},";
        }
        if (b.occlusionTex.texture_index >= 0)
        {
          str += "\"occlusionTexture\": {\n";
          str += "\"index\": " + std::to_string(b.occlusionTex.texture_index) + ",\n";
          str += "\"texCoord\": " + std::to_string(b.occlusionTex.texCoord) + "\n";
          str += "},";
        }
        if (b.emissiveTex.texture_index >= 0)
        {
          str += "\"emissiveTexture\": {\n";
          str += "\"index\": " + std::to_string(b.emissiveTex.texture_index) + ",\n";
          str += "\"texCoord\": " + std::to_string(b.emissiveTex.texCoord) + "\n";
          str += "},";
        }

        str += "\"pbrMetallicRoughness\": {\n";
        str += "\"baseColorFactor\": [" + 
              std::to_string(b.baseColorFactor.x) + "," + std::to_string(b.baseColorFactor.y) + "," + 
              std::to_string(b.baseColorFactor.z) + "," + std::to_string(b.baseColorFactor.w)+ "],\n";
        if (b.baseColorTex.texture_index >= 0)
        {
          str += "\"baseColorTexture\": {\n";
          str += "\"index\": " + std::to_string(b.baseColorTex.texture_index) + ",\n";
          str += "\"texCoord\": " + std::to_string(b.baseColorTex.texCoord) + "\n";
          str += "},";
        }
        if (b.metallicRoughnessTex.texture_index >= 0)
        {
          str += "\"metallicRoughnessTexture\": {\n";
          str += "\"index\": " + std::to_string(b.metallicRoughnessTex.texture_index) + ",\n";
          str += "\"texCoord\": " + std::to_string(b.metallicRoughnessTex.texCoord) + "\n";
          str += "},";
        }
        str += "\"metallicFactor\": " + std::to_string(b.metallic) + ",\n";
        str += "\"roughnessFactor\": " + std::to_string(b.roughness) + "\n";
        str += "}";
        str += "}";
        if (i != a.materials.size() - 1)
          str += ",\n";
      }

      str += "]";
    }
    str += "}";

    std::ofstream out(name + ".gltf");
    out << str;
    out.close();

    return true;
  }
}