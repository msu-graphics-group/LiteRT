#include "sdf_scene.h"
#include <cassert>
#include <fstream>

void save_sdf_scene(const SdfScene &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  unsigned c_count = scene.conjunctions.size();
  unsigned o_count = scene.objects.size();
  unsigned p_count = scene.parameters.size();

  fs.write((const char *)(&c_count), sizeof(unsigned));
  fs.write((const char *)(&o_count), sizeof(unsigned));
  fs.write((const char *)(&p_count), sizeof(unsigned));

  fs.write((const char *)scene.conjunctions.data(), c_count * sizeof(SdfConjunction));
  fs.write((const char *)scene.objects.data(), o_count * sizeof(SdfObject));
  fs.write((const char *)scene.parameters.data(), p_count * sizeof(float));
  fs.flush();
  fs.close();
}

void load_sdf_scene(SdfScene &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned c_count = 0;
  unsigned o_count = 0;
  unsigned p_count = 0;

  fs.read((char *)(&c_count), sizeof(unsigned));
  fs.read((char *)(&o_count), sizeof(unsigned));
  fs.read((char *)(&p_count), sizeof(unsigned));

  assert(c_count > 0);
  assert(o_count > 0);
  assert(p_count > 0);
  scene.conjunctions.resize(c_count);
  scene.objects.resize(o_count);
  scene.parameters.resize(p_count);

  fs.read((char *)scene.conjunctions.data(), c_count * sizeof(SdfConjunction));
  fs.read((char *)scene.objects.data(), o_count * sizeof(SdfObject));
  fs.read((char *)scene.parameters.data(), p_count * sizeof(float));
  fs.close();
}

void save_sdf_grid(const SdfGridView &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, 3 * sizeof(unsigned));
  fs.write((const char *)scene.data, scene.size.x*scene.size.y*scene.size.z * sizeof(float));
  fs.flush();
  fs.close();
}

void load_sdf_grid(SdfGrid &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  fs.read((char *)&scene.size, 3 * sizeof(unsigned));
  scene.data.resize(scene.size.x*scene.size.y*scene.size.z);
  fs.read((char *)scene.data.data(), scene.size.x*scene.size.y*scene.size.z * sizeof(float));
  fs.close();
}

void save_sdf_octree(const SdfOctreeView &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, sizeof(unsigned));
  fs.write((const char *)scene.nodes, scene.size * sizeof(SdfOctreeNode));
  fs.flush();
  fs.close();
}

void load_sdf_octree(std::vector<SdfOctreeNode> &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.resize(sz);
  fs.read((char *)scene.data(), scene.size() * sizeof(SdfOctreeNode));
  fs.close();
}

void save_sdf_frame_octree(const SdfFrameOctreeView &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, sizeof(unsigned));
  fs.write((const char *)scene.nodes, scene.size * sizeof(SdfFrameOctreeNode));
  fs.flush();
  fs.close();
}

void load_sdf_frame_octree(std::vector<SdfFrameOctreeNode> &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.resize(sz);
  fs.read((char *)scene.data(), scene.size() * sizeof(SdfFrameOctreeNode));
  fs.close();
}

void save_sdf_SVS(const SdfSVSView &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, sizeof(unsigned));
  fs.write((const char *)scene.nodes, scene.size * sizeof(SdfSVSNode));
  fs.flush();
  fs.close();
}

void load_sdf_SVS(std::vector<SdfSVSNode> &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.resize(sz);
  fs.read((char *)scene.data(), scene.size() * sizeof(SdfSVSNode));
  fs.close();
}

void save_sdf_SBS(const SdfSBSView &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.header, sizeof(SdfSBSHeader));
  fs.write((const char *)&scene.size, sizeof(unsigned));
  fs.write((const char *)scene.nodes, scene.size * sizeof(SdfSBSNode));
  fs.write((const char *)&scene.values_count, sizeof(unsigned));
  fs.write((const char *)scene.values, scene.values_count * sizeof(unsigned));
  fs.flush();
  fs.close();
}

void  load_sdf_SBS(SdfSBS &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  fs.read((char *)&scene.header, sizeof(SdfSBSHeader));
  unsigned sz, cnt;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.nodes.resize(sz);
  fs.read((char *)scene.nodes.data(), sz * sizeof(SdfSBSNode));
  fs.read((char *)&cnt, sizeof(unsigned));
  scene.values.resize(cnt);
  fs.read((char *)scene.values.data(), cnt * sizeof(unsigned));
  fs.close();
}

void save_sdf_frame_octree_tex(const SdfFrameOctreeTexView &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&scene.size, sizeof(unsigned));
  fs.write((const char *)scene.nodes, scene.size * sizeof(SdfFrameOctreeTexNode));
  fs.flush();
  fs.close();
}

void load_sdf_frame_octree_tex(std::vector<SdfFrameOctreeTexNode> &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  unsigned sz = 0;
  fs.read((char *)&sz, sizeof(unsigned));
  scene.resize(sz);
  fs.read((char *)scene.data(), scene.size() * sizeof(SdfFrameOctreeTexNode));
  fs.close();
}

void load_neural_sdf_scene_SIREN(SdfScene &scene, const std::string &path)
{
  constexpr unsigned layers = 4;
  constexpr unsigned sz = 64;

  unsigned p_cnt = 3 * sz + 1 * sz + (layers - 1) * sz + 1 + (layers - 2) * sz * sz;
  scene.parameters.resize(p_cnt, 0.0f);
  std::ifstream fs(path, std::ios::binary);
  fs.read((char *)(scene.parameters.data()), sizeof(float) * p_cnt);

  scene.neural_properties.emplace_back();
  scene.neural_properties[0].layer_count = layers;
  scene.neural_properties[0].layers[0].in_size = 3;
  for (int i = 1; i < layers; i++)
    scene.neural_properties[0].layers[i].in_size = sz;
  for (int i = 0; i < layers - 1; i++)
    scene.neural_properties[0].layers[i].out_size = sz;
  scene.neural_properties[0].layers[layers - 1].out_size = 1;
  unsigned off = 0;
  for (int i = 0; i < layers; i++)
  {
    scene.neural_properties[0].layers[i].offset = off;
    off += (scene.neural_properties[0].layers[i].in_size + 1) * scene.neural_properties[0].layers[i].out_size;
  }

  scene.objects.emplace_back();
  scene.objects[0].type = SDF_PRIM_SIREN;
  scene.objects[0].params_offset = 0;
  scene.objects[0].params_count = p_cnt;
  scene.objects[0].neural_id = 0;
  scene.objects[0].distance_add = 0.0f;
  scene.objects[0].distance_mult = 1.0f;
  scene.objects[0].complement = 0;
  scene.objects[0].min_pos = float4(-1,-1,-1, 1);
  scene.objects[0].max_pos = float4(1,1,1, 1);
  scene.objects[0].transform.identity();

  scene.conjunctions.emplace_back();
  scene.conjunctions[0].offset = 0;
  scene.conjunctions[0].size = 1;
  scene.conjunctions[0].min_pos = scene.objects[0].min_pos;
  scene.conjunctions[0].max_pos = scene.objects[0].max_pos;
}

// Saves SdfScene as a separate hydra-xml file with no lights, materials and textures
// It's a lazy hack to do it without using Hydra API
// It saves both binary (<folder>/<name>.bin) and xml (<folder>/<name>.xml) files
void save_sdf_scene_hydra(const SdfScene &scene, const std::string &folder, const std::string &name)
{
  std::string bin_path = folder + "/" + name + ".bin";
  std::string path = folder + "/" + name + ".xml";
  save_sdf_scene(scene, bin_path);
  int bytesize = 3 * sizeof(unsigned) + sizeof(SdfConjunction) * scene.conjunctions.size() + sizeof(SdfObject) * scene.objects.size() +
                 sizeof(float) * scene.parameters.size();
  char buf[2 << 12];
  snprintf(buf, 2 << 12, R""""(
    <?xml version="1.0"?>
    <textures_lib>
    </textures_lib>
    <materials_lib>
    </materials_lib>
    <geometry_lib>
      <sdf id="0" name="sdf" type="bin" bytesize="%d" loc="%s">
      </sdf>
    </geometry_lib>
    <lights_lib>
    </lights_lib>
    <cam_lib>
      <camera id="0" name="my camera" type="uvn">
        <fov>60</fov>
        <nearClipPlane>0.01</nearClipPlane>
        <farClipPlane>100.0</farClipPlane>
        <up>0 1 0</up>
        <position>0 0 3</position>
        <look_at>0 0 0</look_at>
      </camera>
    </cam_lib>
    <render_lib>
      <render_settings type="HydraModern" id="0">
        <width>512</width>
        <height>512</height>
        <method_primary>pathtracing</method_primary>
        <method_secondary>pathtracing</method_secondary>
        <method_tertiary>pathtracing</method_tertiary>
        <method_caustic>pathtracing</method_caustic>
        <trace_depth>6</trace_depth>
        <diff_trace_depth>3</diff_trace_depth>
        <maxRaysPerPixel>1024</maxRaysPerPixel>
        <qmc_variant>7</qmc_variant>
      </render_settings>
    </render_lib>
    <scenes>
      <scene id="0" name="my scene" discard="1" bbox="-10 -10 -10 10 10 10">
        <instance id="0" mesh_id="0" rmap_id="0" scn_id="0" scn_sid="0" matrix="1 0 0 0   0 1 0 0   0 0 1 0   0 0 0 1 " />
        <instance id="1" mesh_id="1" rmap_id="0" scn_id="0" scn_sid="0" matrix="0.7 0 0 1    0 0.7 0 0   0 0 0.7 0   0 0 0 1 " />
      </scene>
    </scenes>
  )"""",
           bytesize, (name + ".bin").c_str());

  std::ofstream fs(path);
  fs << std::string(buf);
  fs.flush();
  fs.close();
}

SdfSBSAdaptNode convert_sbs_node_to_adapt(const SdfSBSNode &sbs_node, uint32_t sbs_header_brick_size)
{
  assert(sbs_header_brick_size < 256u);

  uint32_t vox_count_xyz_pad = (sbs_header_brick_size << 16) + (sbs_header_brick_size << 8) + sbs_header_brick_size;
  uint32_t pos_x =    (sbs_node.pos_xy >> 16) & 0x0000FFFF;
  uint32_t pos_y =    (sbs_node.pos_xy      ) & 0x0000FFFF;
  uint32_t pos_z =    (sbs_node.pos_z_lod_size >> 16) & 0x0000FFFF;
  uint32_t lod_size = (sbs_node.pos_z_lod_size      ) & 0x0000FFFF;
  uint32_t d_off =     sbs_node.data_offset;

  {
    int bit_count = 0;
    for (int j = 0; j < 16; ++j)
      bit_count += (lod_size >> j) & 1u;
    assert(bit_count == 1);
      // throw std::runtime_error("Error: convert SBS to adapt - lod size is not a power of two");
  }
  uint32_t voxel_size_in_units = SDF_SBS_ADAPT_MAX_UNITS / lod_size;

  pos_x *= voxel_size_in_units;
  pos_y *= voxel_size_in_units;
  pos_z *= voxel_size_in_units;

  assert(pos_x < SDF_SBS_ADAPT_MAX_UNITS);
  assert(pos_y < SDF_SBS_ADAPT_MAX_UNITS);
  assert(pos_z < SDF_SBS_ADAPT_MAX_UNITS);

  SdfSBSAdaptNode res_node;
  res_node.pos_xy = (pos_x << 16) + pos_y;
  res_node.pos_z_vox_size = (pos_z << 16) + voxel_size_in_units;
  res_node.data_offset = d_off;
  res_node.vox_count_xyz_pad = vox_count_xyz_pad;

  return res_node;
}

SdfSBSAdaptView convert_sbs_to_adapt(SdfSBSAdapt &adapt_scene, const SdfSBSView &scene)
{
  assert(scene.header.brick_size < 256u);
  const uint32_t vox_count_xyz_pad = (scene.header.brick_size << 16) + (scene.header.brick_size << 8) + scene.header.brick_size;

  adapt_scene.header.brick_pad = scene.header.brick_pad;
  adapt_scene.header.bytes_per_value = scene.header.bytes_per_value;
  adapt_scene.header.aux_data = scene.header.aux_data;
  adapt_scene.values.insert(adapt_scene.values.end(), scene.values, scene.values + scene.values_count);
  adapt_scene.values_f.insert(adapt_scene.values_f.end(), scene.values_f, scene.values_f + scene.values_f_count);

  for (uint32_t i = 0u; i < scene.size; ++i)
  {
    uint32_t pos_x =    (scene.nodes[i].pos_xy >> 16) & 0x0000FFFF;
    uint32_t pos_y =    (scene.nodes[i].pos_xy      ) & 0x0000FFFF;
    uint32_t pos_z =    (scene.nodes[i].pos_z_lod_size >> 16) & 0x0000FFFF;
    uint32_t lod_size = (scene.nodes[i].pos_z_lod_size      ) & 0x0000FFFF;
    uint32_t d_off =     scene.nodes[i].data_offset;

    {
      int bit_count = 0;
      for (int j = 0; j < 16; ++j)
        bit_count += (lod_size >> j) & 1u;
      assert(bit_count == 1);
        // throw std::runtime_error("Error: convert SBS to adapt - lod size is not a power of two");
    }
    uint32_t voxel_size_in_units = SDF_SBS_ADAPT_MAX_UNITS / lod_size;

    pos_x *= voxel_size_in_units;
    pos_y *= voxel_size_in_units;
    pos_z *= voxel_size_in_units;

    assert(pos_x < SDF_SBS_ADAPT_MAX_UNITS);
    assert(pos_y < SDF_SBS_ADAPT_MAX_UNITS);
    assert(pos_z < SDF_SBS_ADAPT_MAX_UNITS);

    SdfSBSAdaptNode new_node;
    new_node.pos_xy = (pos_x << 16) + pos_y;
    new_node.pos_z_vox_size = (pos_z << 16) + voxel_size_in_units;
    new_node.data_offset = d_off;
    new_node.vox_count_xyz_pad = vox_count_xyz_pad;

    adapt_scene.nodes.push_back(new_node);
  }
  return adapt_scene;
}