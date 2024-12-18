#include "general_gltf_writer.h"

#include <cstring>
#include <fstream>
#include <iostream>
#include <map>

namespace gltf
{
  void convert_texture(const LiteImage::ICombinedImageSampler *in_tex,
                       std::string tex_file_name,
                       TextureFile &out_tex,
                       Image &out_image,
                       Sampler &out_sampler)
  {
    out_tex.existed_texture = true;
    out_tex.file_name = tex_file_name;

    out_image.picture = &out_tex;

    out_sampler = Sampler();
    //TODO: load sampler
  }

  void convert_material(const ::Material &in_mat, 
                        Material &out_mat)
  {
    out_mat.alpha_cutoff = 0.5;
    out_mat.alpha_mode = materialAlphaMode::OPAQUE;
    out_mat.double_sided = false;
    out_mat.baseColorFactor = float4(1,0,1,1); //purple color marks that something is wrong

    if (in_mat.mtype == MAT_TYPE_GLTF)
    {
      out_mat.baseColorFactor = in_mat.colors[GLTF_COLOR_BASE];
      out_mat.roughness = in_mat.data[GLTF_FLOAT_GLOSINESS];
      out_mat.metallic  = in_mat.data[GLTF_FLOAT_ALPHA];

      //texId=0 in Hydra is reserved to stub empty texture that should always exist
      out_mat.baseColorTex.texCoord = 0; //we have only one set of texture coordinates
      out_mat.baseColorTex.texture_index = in_mat.texid[0];

      //we have either both roughness/metallness textures or none
      if (in_mat.cflags & FLAG_FOUR_TEXTURES)
      {
        //TODO: store all in one texture or support two textures
        printf("glTF saver does not support roughness/metallness textures\n");
      }
    }
    else if (in_mat.mtype == MAT_TYPE_LIGHT_SOURCE)
    {
      out_mat.baseColorFactor = float4(1,1,1,1);
      out_mat.emissive_factor = in_mat.data[EMISSION_MULT] * to_float3(in_mat.colors[EMISSION_COLOR]);
    }
    else
    {
      printf("Hydra mat type %u is not compatible with glTF\n",in_mat.mtype);
    }
  }

  void GeneralGltfWriter::convert_to_gltf(const HydraScene &scene, std::string asset_name)
  {
    FullData fullData;

    fullData.gltf_file.main_scene = 0;
    fullData.gltf_file.scenes.emplace_back();

    // create textures, samplers and materials
    if (!scene.textures.empty())
    {
      fullData.textures_files.resize(scene.textures.size());
      fullData.gltf_file.images.resize(scene.textures.size());
      fullData.gltf_file.textures.resize(scene.textures.size());
      fullData.gltf_file.samplers.resize(scene.textures.size());

      for (int i = 0; i < scene.textures.size(); i++)
      {
        auto *t = scene.textures[i].get();
        
        if (t->bpp() != 4 && t->bpp() != 16)
        {
          printf("texture %d bpp is not 4 or 16\n", i);
        }
        bool is_exr = t->bpp() == 16;
        std::string save_format = is_exr ? ".exr" : ".png";
        std::string tex_file_name = asset_name + "_tex" + std::to_string(i) + save_format;
        
        if (is_exr)
          printf("texture %d is exr, unable to save it\n", i);
        else
        {
          LiteImage::Image2D<uint32_t> t_img(t->width(), t->height(), (const uint32_t *)t->data());
          LiteImage::SaveImage(tex_file_name.c_str(), t_img);
        }
          
        fullData.gltf_file.textures[i].image = i;
        fullData.gltf_file.textures[i].sampler = i;

        convert_texture(t, tex_file_name, fullData.textures_files[i], 
                        fullData.gltf_file.images[i], fullData.gltf_file.samplers[i]);
      }
    }

    assert(scene.materials.size() > 0);

    fullData.gltf_file.materials.resize(scene.materials.size());
    for (int i = 0; i < scene.materials.size(); i++)
    {
      convert_material(scene.materials[i], fullData.gltf_file.materials[i]);
    }

    // if (!scene.textures.empty())
    // {
    //   fullData.gltf_file.samplers.push_back(Sampler());
    //   fullData.textures_files.resize(scene.textures.size());
    //   fullData.gltf_file.images.resize(scene.textures.size());
    //   fullData.gltf_file.textures.resize(scene.textures.size());
    //   fullData.gltf_file.materials.resize(scene.textures.size());

    //   for (int i = 0; i < scene.textures.size(); i++)
    //   {
    //     auto *t = scene.textures[i].get();

    //     fullData.textures_files[i].existed_texture = true;
    //     fullData.textures_files[i].file_name = "unknown"; //TODO:set name here and then save texture by this name

    //     fullData.gltf_file.images[i].picture = &fullData.textures_files[i];

    //     fullData.gltf_file.textures[i].image = i;
    //     fullData.gltf_file.textures[i].sampler = 0;

    //     fullData.gltf_file.materials[i].alpha_cutoff = 0.5;
    //     fullData.gltf_file.materials[i].alpha_mode = materialAlphaMode::MASK;
    //     fullData.gltf_file.materials[i].double_sided = true;
    //     fullData.gltf_file.materials[i].baseColorTex.texCoord = 0;
    //     fullData.gltf_file.materials[i].baseColorTex.texture_index = i;
    //   }
    // }
    int bin_file_id = 0;
    int model_n = 0;
    int max_model = std::min<int>(settings.max_models, scene.meshes.size());
    while (model_n < max_model)
    {
      int start = model_n;
      int end = model_n;
      int ind_count = 0;
      int vert_count = 0;
      bool correct_patch = true;
      while (end < max_model)
      {
        cmesh4::SimpleMesh m = scene.meshes[end].mesh;

        if (m.vPos4f.empty())
        {
          fprintf(stderr, "glTF_writer: cannot convert model, it is empty");
          correct_patch = false;
        }
        if (m.indices.empty())
        {
          fprintf(stderr, "glTF_writer: cannot convert model, only indexed models are supported");
          correct_patch = false;
        }
        if (m.vNorm4f.size() != 0 && m.vNorm4f.size() != m.vPos4f.size())
        {
          fprintf(stderr, "glTF_writer: cannot convert model, normals does not match positions");
          correct_patch = false;
        }
        if (std::max(sizeof(float) * m.vPos4f.size(), sizeof(uint32_t) * m.indices.size()) > settings.max_binary_file_size)
        {
          fprintf(stderr, "glTF_writer: cannot convert model, model is too large");
          correct_patch = false;
        }

        ind_count += m.indices.size();
        vert_count += m.vPos4f.size();

        if (std::max(sizeof(uint32_t) * ind_count, 3 * sizeof(float) * vert_count) > settings.max_binary_file_size)
        {
          break;
        }
        end++;
      }

      model_n = end;

      if (correct_patch)
      {
        fullData.pos_binary_files.emplace_back();
        auto &pbf = fullData.pos_binary_files.back();
        pbf.max_size = 3 * sizeof(float) * vert_count;
        pbf.cur_size = 0;
        pbf.data = new char[pbf.max_size];
        pbf.file_name = asset_name + "_" + std::to_string(bin_file_id) + "_pos.bin";

        fullData.norm_binary_files.emplace_back();
        auto &nbf = fullData.norm_binary_files.back();
        nbf.max_size = 3 * sizeof(float) * vert_count;
        nbf.cur_size = 0;
        nbf.data = new char[nbf.max_size];
        nbf.file_name = asset_name + "_" + std::to_string(bin_file_id) + "_norm.bin";

        fullData.tc_binary_files.emplace_back();
        auto &tbf = fullData.tc_binary_files.back();
        tbf.max_size = 2 * sizeof(float) * vert_count;
        tbf.cur_size = 0;
        tbf.data = new char[tbf.max_size];
        tbf.file_name = asset_name + "_" + std::to_string(bin_file_id) + "_tc.bin";

        fullData.ind_binary_files.emplace_back();
        auto &ibf = fullData.ind_binary_files.back();
        ibf.max_size = sizeof(uint32_t) * ind_count;
        ibf.cur_size = 0;
        ibf.data = new char[ibf.max_size];
        ibf.file_name = asset_name + "_" + std::to_string(bin_file_id) + "_ind.bin";

        for (int i = start; i < end; i++)
        {
          correct_patch = correct_patch && model_to_gltf(scene.meshes[i].mesh, fullData, bin_file_id);
        }

        if (correct_patch)
        {
          bool b = write_to_binary_file(pbf.data, pbf.cur_size, pbf.file_name);
          correct_patch = correct_patch && b;

          b = write_to_binary_file(nbf.data, nbf.cur_size, nbf.file_name);
          correct_patch = correct_patch && b;

          b = write_to_binary_file(tbf.data, tbf.cur_size, tbf.file_name);
          correct_patch = correct_patch && b;

          b = write_to_binary_file(ibf.data, ibf.cur_size, ibf.file_name);
          correct_patch = correct_patch && b;
        }
        delete[] pbf.data;
        delete[] nbf.data;
        delete[] tbf.data;
        delete[] ibf.data;

        // create buffers and buffer views for patch
        if (correct_patch && settings.debug)
        {
          printf("glTF_writer: successfully saved models binary data. Models %d - %d. %d verts, %d ind\n",
                 start, end - 1, vert_count, ind_count);
          int pos_bv;
          int norm_bv;
          int ind_bv;
          int tc_bv;

          // positions
          fullData.gltf_file.buffers.emplace_back();
          fullData.gltf_file.buffers.back().data = &pbf;
          fullData.gltf_file.buffers.back().byte_length = pbf.cur_size;

          fullData.gltf_file.buffer_views.emplace_back();
          fullData.gltf_file.buffer_views.back().buffer = fullData.gltf_file.buffers.size() - 1;
          fullData.gltf_file.buffer_views.back().byte_offset = 0;
          fullData.gltf_file.buffer_views.back().byte_stride = 3 * sizeof(float);
          fullData.gltf_file.buffer_views.back().byte_length = fullData.gltf_file.buffers.back().byte_length;
          fullData.gltf_file.buffer_views.back().target = BufferViewTargetType::ARRAY_BUFFER;
          pos_bv = fullData.gltf_file.buffer_views.size() - 1;

          // normals
          fullData.gltf_file.buffers.emplace_back();
          fullData.gltf_file.buffers.back().data = &nbf;
          fullData.gltf_file.buffers.back().byte_length = nbf.cur_size;

          fullData.gltf_file.buffer_views.emplace_back();
          fullData.gltf_file.buffer_views.back().buffer = fullData.gltf_file.buffers.size() - 1;
          fullData.gltf_file.buffer_views.back().byte_offset = 0;
          fullData.gltf_file.buffer_views.back().byte_stride = 3 * sizeof(float);
          fullData.gltf_file.buffer_views.back().byte_length = fullData.gltf_file.buffers.back().byte_length;
          fullData.gltf_file.buffer_views.back().target = BufferViewTargetType::ARRAY_BUFFER;
          norm_bv = fullData.gltf_file.buffer_views.size() - 1;

          // texture coordinates
          fullData.gltf_file.buffers.emplace_back();
          fullData.gltf_file.buffers.back().data = &tbf;
          fullData.gltf_file.buffers.back().byte_length = tbf.cur_size;

          fullData.gltf_file.buffer_views.emplace_back();
          fullData.gltf_file.buffer_views.back().buffer = fullData.gltf_file.buffers.size() - 1;
          fullData.gltf_file.buffer_views.back().byte_offset = 0;
          fullData.gltf_file.buffer_views.back().byte_stride = 2 * sizeof(float);
          fullData.gltf_file.buffer_views.back().byte_length = fullData.gltf_file.buffers.back().byte_length;
          fullData.gltf_file.buffer_views.back().target = BufferViewTargetType::ARRAY_BUFFER;
          tc_bv = fullData.gltf_file.buffer_views.size() - 1;

          // indices
          fullData.gltf_file.buffers.emplace_back();
          fullData.gltf_file.buffers.back().data = &ibf;
          fullData.gltf_file.buffers.back().byte_length = ibf.cur_size;

          fullData.gltf_file.buffer_views.emplace_back();
          fullData.gltf_file.buffer_views.back().buffer = fullData.gltf_file.buffers.size() - 1;
          fullData.gltf_file.buffer_views.back().byte_offset = 0;
          fullData.gltf_file.buffer_views.back().byte_length = fullData.gltf_file.buffers.back().byte_length;
          fullData.gltf_file.buffer_views.back().target = BufferViewTargetType::ELEMENT_ARRAY_BUFFER;
          ind_bv = fullData.gltf_file.buffer_views.size() - 1;
          // create accessors and meshes for each model

          int ind_byte_offset = 0;
          int pos_byte_offset = 0;
          int tc_byte_offset = 0;
          for (int i = start; i < end; i++)
          {
            const cmesh4::SimpleMesh &m = scene.meshes[i].mesh;

            int verts = m.vPos4f.size();
            int ind_acc_n, pos_acc_n, norm_acc_n, tc_acc_n;

            float3 max_bounds = float3(settings.max_bound);
            float3 min_bounds = float3(-settings.max_bound);
            if (settings.calc_exact_bbox && verts > 0)
            {
              max_bounds = float3(-1e6);
              min_bounds = float3(1e6);

              for (int j = 0; j < m.vPos4f.size(); j++)
              {
                max_bounds.x = std::max(max_bounds.x, m.vPos4f[j].x);
                max_bounds.y = std::max(max_bounds.y, m.vPos4f[j].y);
                max_bounds.z = std::max(max_bounds.z, m.vPos4f[j].z);

                min_bounds.x = std::min(min_bounds.x, m.vPos4f[j].x);
                min_bounds.y = std::min(min_bounds.y, m.vPos4f[j].y);
                min_bounds.z = std::min(min_bounds.z, m.vPos4f[j].z);
              }
            }
            // positions accessor
            fullData.gltf_file.accessors.emplace_back();
            Accessor &pos_acc = fullData.gltf_file.accessors.back();
            pos_acc.buffer_view = pos_bv;
            pos_acc.byte_offset = pos_byte_offset;
            pos_acc.count = verts;
            pos_acc.componentType = AccessorComponentType::FLOAT;
            pos_acc.type = AccessorType::VEC3;
            pos_acc.max_values = {max_bounds.x, max_bounds.y, max_bounds.z};
            pos_acc.min_values = {min_bounds.x, min_bounds.y, min_bounds.z};
            pos_acc_n = fullData.gltf_file.accessors.size() - 1;

            // normals accessor
            fullData.gltf_file.accessors.emplace_back();
            Accessor &norm_acc = fullData.gltf_file.accessors.back();
            norm_acc.buffer_view = norm_bv;
            norm_acc.byte_offset = pos_byte_offset;
            norm_acc.count = verts;
            norm_acc.componentType = AccessorComponentType::FLOAT;
            norm_acc.type = AccessorType::VEC3;
            norm_acc_n = fullData.gltf_file.accessors.size() - 1;

            // tc accessor
            fullData.gltf_file.accessors.emplace_back();
            Accessor &tc_acc = fullData.gltf_file.accessors.back();
            tc_acc.buffer_view = tc_bv;
            tc_acc.byte_offset = tc_byte_offset;
            tc_acc.count = verts;
            tc_acc.componentType = AccessorComponentType::FLOAT;
            tc_acc.type = AccessorType::VEC2;
            tc_acc_n = fullData.gltf_file.accessors.size() - 1;

            // indices accessor
            fullData.gltf_file.accessors.emplace_back();
            Accessor &ind_acc = fullData.gltf_file.accessors.back();
            ind_acc.buffer_view = ind_bv;
            ind_acc.byte_offset = ind_byte_offset;
            ind_acc.count = m.indices.size();
            ind_acc.componentType = AccessorComponentType::UNSIGNED_INT;
            ind_acc.type = AccessorType::SCALAR;
            ind_acc.min_values = {0};
            ind_acc.max_values = {(float)(verts - 1)};
            ind_acc_n = fullData.gltf_file.accessors.size() - 1;

            fullData.gltf_file.meshes.emplace_back();
            fullData.gltf_file.meshes.back().primitives.emplace_back();
            auto &pr = fullData.gltf_file.meshes.back().primitives.back();
            pr.indicies = ind_acc_n;
            pr.attributes.emplace(primitiveAttributeType::POSITION, pos_acc_n);
            pr.attributes.emplace(primitiveAttributeType::NORMAL, norm_acc_n);
            pr.attributes.emplace(primitiveAttributeType::TEXCOORD_0, tc_acc_n);

            //TODO: support multiple materials
            int base_material_id = m.matIndices[0];
            for (unsigned mi : m.matIndices)
            {
              if (mi != base_material_id)
              {
                fprintf(stderr, "Multiple materials not supported\n");
                break;
              }
            }
            pr.material = base_material_id;

            ind_byte_offset += sizeof(uint) * m.indices.size();
            pos_byte_offset += 3 * sizeof(float) * verts;
            tc_byte_offset += 2 * sizeof(float) * verts;
          }
        }
        bin_file_id++;
      }
    }

    // create a node for each set of transforms
    auto &main_gltf_scene = fullData.gltf_file.scenes[fullData.gltf_file.main_scene];
    for (int i = 0; i < scene.meshes.size(); i++)
    {
      auto &t = scene.meshes[i].transforms;
      if (t.size() == 0)
        continue;
      int mesh_id = i;
      fullData.gltf_file.nodes.emplace_back();
      Node &n = fullData.gltf_file.nodes.back();
      main_gltf_scene.nodes.push_back(fullData.gltf_file.nodes.size() - 1);

      if (t.size() == 1)
      {
        n.mesh = mesh_id;
        n.transform = t[0];
      }
      else
      {
        n.child_nodes = {};
        n.transform = float4x4();
        n.scale = float3(1, 1, 1);
        int nn = fullData.gltf_file.nodes.size() - 1;
        for (auto &tr : t)
        {
          fullData.gltf_file.nodes.emplace_back();
          fullData.gltf_file.nodes[nn].child_nodes.push_back(fullData.gltf_file.nodes.size() - 1);
          fullData.gltf_file.nodes.back().mesh = mesh_id;
          fullData.gltf_file.nodes.back().transform = tr;
        }
      }
    }

    int camera_id = 0;
    for (const ::Camera &c : scene.cameras)
    {
      if (camera_to_gltf(c, fullData, camera_id))
        camera_id++;
    }

    GltfStructureWriter gsw;
    gsw.write_to_json(fullData, asset_name);
  }
  bool GeneralGltfWriter::model_to_gltf(const cmesh4::SimpleMesh &m, FullData &full_data, int bin_file_id)
  {
    bool ok = true;

    int sz = m.vPos4f.size();
    std::vector<float> buf(3*sz);

    for (int i = 0; i < sz; i++)
    {
      buf[3 * i + 0] = m.vPos4f[i].x;
      buf[3 * i + 1] = m.vPos4f[i].y;
      buf[3 * i + 2] = m.vPos4f[i].z;
    }
    ok = ok && add_to_binary_file((const char *)buf.data(), 3 * sz * sizeof(float), full_data.pos_binary_files[bin_file_id]);

    for (int i = 0; i < sz; i++)
    {
      buf[3 * i + 0] = m.vNorm4f[i].x;
      buf[3 * i + 1] = m.vNorm4f[i].y;
      buf[3 * i + 2] = m.vNorm4f[i].z;
    }
    ok = ok && add_to_binary_file((const char *)buf.data(), 3 * sz * sizeof(float), full_data.norm_binary_files[bin_file_id]);
    
    for (int i = 0; i < sz; i++)
    {
      buf[2 * i + 0] = m.vTexCoord2f[i].x;
      buf[2 * i + 1] = m.vTexCoord2f[i].y;
    }
    ok = ok && add_to_binary_file((const char *)buf.data(), 2 * sz * sizeof(float), full_data.tc_binary_files[bin_file_id]);

    const char *ind_data = reinterpret_cast<const char *>(m.indices.data());
    ok = ok && add_to_binary_file(ind_data, m.indices.size() * sizeof(uint32_t), full_data.ind_binary_files[bin_file_id]);

    return ok;
  }
  bool GeneralGltfWriter::camera_to_gltf(const ::Camera &c, FullData &full_data, int id)
  {
    return true;
  }
  bool GeneralGltfWriter::add_to_binary_file(const char *data, int size, BinaryFile &b_file)
  {
    if (size + b_file.cur_size <= b_file.max_size)
    {
      memcpy(b_file.data + b_file.cur_size, data, size);
      b_file.cur_size += size;
      return true;
    }
    return false;
  }
  bool GeneralGltfWriter::write_to_binary_file(const char *data, int size, std::string file_name)
  {
    std::ofstream fs(file_name, std::ios::out | std::ios::binary);
    fs.write(data, size);
    fs.close();
    return !fs.fail();
  }
}