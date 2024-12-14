#include "sdf_scene.h"
#include "cmesh4.h"
#include "utils/mesh/mesh.h"

#include <cassert>
#include <fstream>
#include <filesystem>

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

ModelInfo get_info_sdf_scene(const SdfScene &scene)
{
  ModelInfo info;

  info.name = "sdf";
  info.bytesize = scene.conjunctions.size() * sizeof(SdfConjunction) + sizeof(unsigned) +
                  scene.objects.size() * sizeof(SdfObject) + sizeof(unsigned) +
                  scene.parameters.size() * sizeof(float) + sizeof(unsigned);
  info.num_primitives = scene.objects.size();

  return info;
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

ModelInfo get_info_sdf_grid(const SdfGridView &scene)
{
  ModelInfo info;
  
  info.name = "sdf_grid";
  info.bytesize = scene.size.x*scene.size.y*scene.size.z * sizeof(float) + sizeof(unsigned);
  info.num_primitives = 1;

  return info;
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

ModelInfo get_info_sdf_frame_octree(const SdfFrameOctreeView &scene)
{
  ModelInfo info;

  info.name = "sdf_frame_octree";
  info.bytesize = scene.size * sizeof(SdfFrameOctreeNode) + sizeof(unsigned);
  info.num_primitives = scene.size;

  return info;
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

ModelInfo get_info_sdf_SVS(const SdfSVSView &scene)
{
  ModelInfo info;

  info.name = "sdf_svs";
  info.bytesize = scene.size * sizeof(SdfSVSNode) + sizeof(unsigned);
  info.num_primitives = scene.size;

  return info;
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

ModelInfo get_info_sdf_SBS(const SdfSBSView &scene)
{
  ModelInfo info;

  info.name = "sdf_sbs";
  info.bytesize = sizeof(SdfSBSHeader) + 
                  scene.size * sizeof(SdfSBSNode) + sizeof(unsigned) + 
                  scene.values_count * sizeof(unsigned);
  info.num_primitives = scene.size;

  return info;
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

ModelInfo get_info_sdf_frame_octree_tex(const SdfFrameOctreeTexView &scene)
{
  ModelInfo info;

  info.name = "sdf_frame_octree_tex";
  info.bytesize = scene.size * sizeof(SdfFrameOctreeTexNode) + sizeof(unsigned);
  info.num_primitives = scene.size;

  return info;
}

void save_coctree_v3(const COctreeV3View &scene, const std::string &path)
{
  std::ofstream fs(path, std::ios::binary);
  fs.write((const char *)&COctreeV3::VERSION, sizeof(uint32_t));
  fs.write((const char *)&scene.header, sizeof(COctreeV3Header));
  fs.write((const char *)&scene.size, sizeof(uint32_t));
  fs.write((const char *)scene.data, scene.size * sizeof(uint32_t));
  fs.flush();
  fs.close();
}

void load_coctree_v3(COctreeV3 &scene, const std::string &path)
{
  std::ifstream fs(path, std::ios::binary);
  uint32_t version = 0;
  fs.read((char *)&version, sizeof(uint32_t));
  if (version != COctreeV3::VERSION) {
    printf("COctreeV3 version mismatch (save is version %u, current version is %u)\n",
           version, COctreeV3::VERSION);
  }
  fs.read((char *)&scene.header, sizeof(COctreeV3Header));
  uint32_t sz = 0;
  fs.read((char *)&sz, sizeof(uint32_t));
  scene.data.resize(sz);
  fs.read((char *)scene.data.data(), sz * sizeof(uint32_t));
  fs.close();
}

ModelInfo get_info_coctree_v3(const COctreeV3View &scene)
{
  ModelInfo info;

  info.name = "sdf_coctree_v3";
  info.bytesize = sizeof(uint32_t) + sizeof(COctreeV3Header) + 
                  scene.size * sizeof(uint32_t) + sizeof(uint32_t);
  info.num_primitives = scene.size;

  return info;
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

std::string insert_in_demo_scene_cornell_box(const std::string &geom_info_str, const std::string &scenes_relative_path)
{
  constexpr unsigned MAX_BUF_SIZE = 8192;
  char buf[MAX_BUF_SIZE];

    snprintf(buf, MAX_BUF_SIZE, R""""(
    <?xml version="1.0"?>
    <textures_lib total_chunks="4">
      <texture id="0" name="Map#0" loc="%s/01_simple_scenes/data/chunk_00000.image4ub" offset="8" bytesize="16" width="2" height="2" dl="0" />
    </textures_lib>
    <materials_lib>
      <material id="0" name="mysimplemat" type="hydra_material">
        <diffuse brdf_type="lambert">
          <color val="0.5 0.5 0.5" />
        </diffuse>
      </material>
      <material id="1" name="red" type="hydra_material">
        <diffuse brdf_type="lambert">
          <color val="0.5 0.0 0.0" />
        </diffuse>
      </material>
      <material id="2" name="green" type="hydra_material">
        <diffuse brdf_type="lambert">
          <color val="0.0 0.5 0.0" />
        </diffuse>
      </material>
      <material id="3" name="white" type="hydra_material">
        <diffuse brdf_type="lambert">
          <color val="0.5 0.5 0.5" />
        </diffuse>
      </material>
      <material id="4" name="gold" type="hydra_material">
        <diffuse brdf_type="lambert">
          <color val="0.40 0.4 0" />
        </diffuse>
        <reflectivity brdf_type="ggx">
          <color val="0.20 0.20 0" />
          <glossiness val="0.750000024" />
        </reflectivity>
      </material>
      <material id="5" name="my_area_light_material" type="hydra_material" light_id="0" visible="1">
        <emission>
          <color val="25 25 25" />
        </emission>
      </material>
      <material id="6" name="Silver2_SG" type="hydra_material">
        <emission>
          <color val="0 0 0" />
          <cast_gi val="1" />
          <multiplier val="1" />
        </emission>
        <diffuse brdf_type="lambert">
          <color val="0.27 0.29 0.29" />
          <roughness val="0" />
        </diffuse>
        <reflectivity brdf_type="phong">
          <extrusion val="maxcolor" />
          <color val="0.85 0.85 0.85" />
          <glossiness val="0.8" />
          <fresnel val="1" />
          <fresnel_ior val="1.5" />
        </reflectivity>
      </material>
    </materials_lib>
    <geometry_lib total_chunks="4">
      <mesh id="0" name="my_box" type="vsgf" bytesize="1304" loc="%s/01_simple_scenes/data/cornell_open.vsgf" offset="0" vertNum="20" triNum="10" dl="0" path="" bbox="    -4 4 -4 4 -4 4">
        <positions type="array4f" bytesize="320" offset="24" apply="vertex" />
        <normals type="array4f" bytesize="320" offset="344" apply="vertex" />
        <tangents type="array4f" bytesize="320" offset="664" apply="vertex" />
        <texcoords type="array2f" bytesize="160" offset="984" apply="vertex" />
        <indices type="array1i" bytesize="120" offset="1144" apply="tlist" />
        <matindices type="array1i" bytesize="40" offset="1264" apply="primitive" />
      </mesh>
      %s
      <mesh id="2" name="my_area_light_lightmesh" type="vsgf" bytesize="280" loc="%s/01_simple_scenes/data/chunk_00003.vsgf" offset="0" vertNum="4" triNum="2" dl="0" path="" light_id="0" bbox="    -1 1 0 0 -1 1">
        <positions type="array4f" bytesize="64" offset="24" apply="vertex" />
        <normals type="array4f" bytesize="64" offset="88" apply="vertex" />
        <tangents type="array4f" bytesize="64" offset="152" apply="vertex" />
        <texcoords type="array2f" bytesize="32" offset="216" apply="vertex" />
        <indices type="array1i" bytesize="24" offset="248" apply="tlist" />
        <matindices type="array1i" bytesize="8" offset="272" apply="primitive" />
      </mesh>
    </geometry_lib>
    <lights_lib>
      <light id="0" name="my_area_light" type="area" shape="rect" distribution="diffuse" visible="1" mat_id="5" mesh_id="2">
        <size half_length="1" half_width="1" />
        <intensity>
          <color val="1 1 1" />
          <multiplier val="25" />
        </intensity>
      </light>
    </lights_lib>
    <cam_lib>
      <camera id="0" name="my camera" type="uvn">
        <fov>45</fov>
        <nearClipPlane>0.01</nearClipPlane>
        <farClipPlane>100.0</farClipPlane>
        <up>0 1 0</up>
        <position>0 0 14</position>
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
        <trace_depth>4</trace_depth>
        <diff_trace_depth>4</diff_trace_depth>
        <maxRaysPerPixel>1024</maxRaysPerPixel>
        <qmc_variant>7</qmc_variant>
      </render_settings>
    </render_lib>
    <scenes>
      <scene id="0" name="my scene" discard="1" bbox="    -4 4 -4.137 4 -4 4">
        <remap_lists>
          <remap_list id="0" size="2" val="0 4 " />
        </remap_lists>
        <instance id="0" mesh_id="1" rmap_id="0" scn_id="0" scn_sid="0" matrix="3 0 0 0   0 3 0 -1.4   0 0 -3 0   0 0 0 1 " />
        <instance id="1" mesh_id="0" rmap_id="-1" scn_id="0" scn_sid="0" matrix="-1 0 -8.74228e-08 0 0 1 0 0 8.74228e-08 0 -1 0 0 0 0 1 " />
        <instance_light id="0" light_id="0" matrix="1 0 0 0 0 1 0 3.85 0 0 1 0 0 0 0 1 " lgroup_id="-1" />
        <instance id="2" mesh_id="2" rmap_id="-1" matrix="1 0 0 0 0 1 0 3.85 0 0 1 0 0 0 0 1 " light_id="0" linst_id="0" />
      </scene>
    </scenes>
    )"""",
    scenes_relative_path.c_str(), scenes_relative_path.c_str(), geom_info_str.c_str(), scenes_relative_path.c_str());

    return std::string(buf, strlen(buf));
}

std::string insert_in_demo_scene_single_object(std::string geom_info_str, const std::string &scenes_relative_path)
{
constexpr unsigned MAX_BUF_SIZE = 8192;
  char buf[MAX_BUF_SIZE];

  snprintf(buf, MAX_BUF_SIZE, R""""(
<?xml version="1.0"?>
<textures_lib total_chunks="4">
  <texture id="0" name="Map#0" loc="data/chunk_00000.image4ub" offset="8" bytesize="16" width="2" height="2" dl="0" />
</textures_lib>
<materials_lib>
  <material id="0" name="mysimplemat" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.5 0.5 0.5" />
    </diffuse>
  </material>
  <material id="1" name="red" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.5 0.0 0.0" />
    </diffuse>
  </material>
  <material id="2" name="green" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.0 0.5 0.0" />
    </diffuse>
  </material>
  <material id="3" name="white" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.5 0.5 0.5" />
    </diffuse>
  </material>
  <material id="4" name="gold" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.40 0.4 0" />
    </diffuse>
    <reflectivity brdf_type="torranse_sparrow">
      <color val="0.10 0.10 0" />
      <glossiness val="0.850000024" />
    </reflectivity>
  </material>
  <material id="5" name="my_area_light_material" type="hydra_material" light_id="0" visible="1">
    <emission>
      <color val="25 25 25" />
    </emission>
  </material>
  <material id="6" name="Silver2_SG" type="hydra_material">
    <emission>
      <color val="0 0 0" />
      <cast_gi val="1" />
      <multiplier val="1" />
    </emission>
    <diffuse brdf_type="lambert">
      <color val="0.27 0.29 0.29" />
      <roughness val="0" />
    </diffuse>
    <reflectivity brdf_type="phong">
      <extrusion val="maxcolor" />
      <color val="0.85 0.85 0.85" />
      <glossiness val="0.8" />
      <fresnel val="1" />
      <fresnel_ior val="1.5" />
    </reflectivity>
  </material>
</materials_lib>
<geometry_lib total_chunks="4">
  %s
</geometry_lib>
<lights_lib>
  <light id="0" name="my_area_light" type="area" shape="rect" distribution="diffuse" visible="1" mat_id="5" mesh_id="2">
    <size half_length="1" half_width="1" />
    <intensity>
      <color val="1 1 1" />
      <multiplier val="25" />
    </intensity>
  </light>
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
  <scene id="0" name="my scene" discard="1" bbox="   -10 10 -4.137 0.7254 -10 10">
    <remap_lists>
      <remap_list id="0" size="2" val="0 4 " />
    </remap_lists>
    <instance id="0" mesh_id="0" rmap_id="0" scn_id="0" scn_sid="0" matrix="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 " />
    <instance_light id="0" light_id="0" matrix="1 0 0 0 0 1 0 3.85 0 0 1 0 0 0 0 1 " lgroup_id="-1" />
  </scene>
</scenes>
    )"""",
           geom_info_str.c_str());

  return std::string(buf, strlen(buf));
}

std::string insert_in_demo_scene_single_object_cubemap(std::string geom_info_str, const std::string &scenes_relative_path)
{
constexpr unsigned MAX_BUF_SIZE = 8192;
  char buf[MAX_BUF_SIZE];

  snprintf(buf, MAX_BUF_SIZE, R""""(
<?xml version="1.0"?>
<textures_lib total_chunks="4">
  <texture id="0" name="Map#0" loc="data/chunk_00000.image4ub" offset="8" bytesize="16" width="2" height="2" dl="0" />
  <texture id="1" name="cubemap" path="%s/textures/23_antwerp_night.exr" loc="%s/textures/23_antwerp_night.exr" offset="8" bytesize="128000000" width="4000" height="2000" channels="4" dl="0" />
</textures_lib>
<materials_lib>
  <material id="0" name="mysimplemat" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.5 0.5 0.5" />
    </diffuse>
  </material>
  <material id="1" name="red" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.5 0.0 0.0" />
    </diffuse>
  </material>
  <material id="2" name="green" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.0 0.5 0.0" />
    </diffuse>
  </material>
  <material id="3" name="white" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.5 0.5 0.5" />
    </diffuse>
  </material>
  <material id="4" name="gold" type="hydra_material">
    <diffuse brdf_type="lambert">
      <color val="0.40 0.4 0" />
    </diffuse>
    <reflectivity brdf_type="torranse_sparrow">
      <color val="0.10 0.10 0" />
      <glossiness val="0.850000024" />
    </reflectivity>
  </material>
  <material id="5" name="my_area_light_material" type="hydra_material" light_id="0" visible="1">
    <emission>
      <color val="25 25 25" />
    </emission>
  </material>
  <material id="6" name="Silver2_SG" type="hydra_material">
    <emission>
      <color val="0 0 0" />
      <cast_gi val="1" />
      <multiplier val="1" />
    </emission>
    <diffuse brdf_type="lambert">
      <color val="0.27 0.29 0.29" />
      <roughness val="0" />
    </diffuse>
    <reflectivity brdf_type="phong">
      <extrusion val="maxcolor" />
      <color val="0.85 0.85 0.85" />
      <glossiness val="0.8" />
      <fresnel val="1" />
      <fresnel_ior val="1.5" />
    </reflectivity>
  </material>
</materials_lib>
<geometry_lib total_chunks="4">
  %s
</geometry_lib>
<lights_lib>
  <light id="0" name="sky" type="sky" shape="point" distribution="map" visible="1" mat_id="2">
    <intensity>
      <color val="1 1 1">
        <texture id="1" type="texref" matrix="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1" addressing_mode_u="wrap" addressing_mode_v="wrap" />
      </color>
      <multiplier val="1.0" />
    </intensity>
  </light>
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
  <scene id="0" name="my scene" discard="1" bbox="   -10 10 -4.137 0.7254 -10 10">
    <remap_lists>
      <remap_list id="0" size="2" val="0 4 " />
    </remap_lists>
    <instance id="0" mesh_id="0" rmap_id="0" scn_id="0" scn_sid="0" matrix="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 " />
    <instance_light id="0" light_id="0" matrix="1 0 0 0 0 1 0 0 0 0 1 0 0 0 0 1 " lgroup_id="-1">
      <transform_sequence transformation="scale * rotation * position" rotation="Euler in dergees" />
    </instance_light>
  </scene>
</scenes>
    )"""",
           scenes_relative_path.c_str(), scenes_relative_path.c_str(), geom_info_str.c_str());

  return std::string(buf, strlen(buf));
}

std::string insert_in_demo_scene(std::string geom_info_str, const std::string &scenes_relative_path, DemoScene scene_type)
{
  switch (scene_type)
  {
  case DemoScene::CORNELL_BOX:
    return insert_in_demo_scene_cornell_box(geom_info_str, scenes_relative_path);
    break;
  case DemoScene::SINGLE_OBJECT:
    return insert_in_demo_scene_single_object(geom_info_str, scenes_relative_path);
    break;
  case DemoScene::SINGLE_OBJECT_CUBEMAP:
    return insert_in_demo_scene_single_object_cubemap(geom_info_str, scenes_relative_path);
    break;
  
  default:
    assert("false");
    return "";
    break;
  }
}

std::string get_xml_string_model_demo_scene(std::string bin_file_name, std::string scenes_relative_path, ModelInfo info, int mat_id, DemoScene scene)
{
  constexpr unsigned MAX_BUF_SIZE = 1024;
  char buf[MAX_BUF_SIZE];

    snprintf(buf, MAX_BUF_SIZE, R""""(
    <%s id="1" name="demo_model" type="%s" bytesize="%d" loc="%s" num_primitives="%d" mat_id="%d">
    </%s>
    )"""",
    info.name.c_str(), info.name.c_str(), info.bytesize, bin_file_name.c_str(), info.num_primitives, mat_id, info.name.c_str());
  
  std::string geom_info_str = std::string(buf, strlen(buf));



  return insert_in_demo_scene(geom_info_str, scenes_relative_path, scene);
}

std::string get_xml_string_model_demo_scene(std::string bin_file_name, std::string scenes_relative_path, const cmesh4::SimpleMesh &mesh, DemoScene scene)
{
  constexpr unsigned MAX_BUF_SIZE = 1024;
  char buf[MAX_BUF_SIZE];

  unsigned bytesize = mesh.SizeInBytes();
  unsigned vertNum = mesh.VerticesNum();
  unsigned triNum = mesh.TrianglesNum();
  float3 bbox_min, bbox_max;
  cmesh4::get_bbox(mesh, &bbox_min, &bbox_max);
  unsigned positions_bytesize = 4*sizeof(float)*vertNum;
  unsigned normals_bytesize = 4*sizeof(float)*vertNum;
  unsigned tangents_bytesize = 4*sizeof(float)*vertNum;
  unsigned texcoords_bytesize = 2*sizeof(float)*vertNum;
  unsigned indices_bytesize = sizeof(unsigned)*triNum*3;
  unsigned matIndices_bytesize = sizeof(unsigned)*triNum;

  unsigned base_offset = 6*sizeof(unsigned);

    snprintf(buf, MAX_BUF_SIZE, R""""(
      <mesh id="1" name="demo_mesh" type="vsgf" bytesize="%d" loc="%s" offset="0" vertNum="%d" triNum="%d" dl="0" path="" bbox="%f %f %f %f %f %f">
        <positions type="array4f" bytesize="%d" offset="%d" apply="vertex" />
        <normals type="array4f" bytesize="%d" offset="%d" apply="vertex" />
        <tangents type="array4f" bytesize="%d" offset="%d" apply="vertex" />
        <texcoords type="array2f" bytesize="%d" offset="%d" apply="vertex" />
        <indices type="array1i" bytesize="%d" offset="%d" apply="tlist" />
        <matindices type="array1i" bytesize="%d" offset="%d" apply="primitive" />
      </mesh>
    )"""",
    bytesize, bin_file_name.c_str(), vertNum, triNum, bbox_min.x, bbox_max.x, bbox_min.y, bbox_max.y, bbox_min.z, bbox_max.z,
    positions_bytesize, base_offset, 
    normals_bytesize, base_offset+positions_bytesize,
    tangents_bytesize, base_offset+positions_bytesize+normals_bytesize,
    texcoords_bytesize, base_offset+positions_bytesize+normals_bytesize+tangents_bytesize,
    indices_bytesize, base_offset+positions_bytesize+normals_bytesize+tangents_bytesize+texcoords_bytesize,
    matIndices_bytesize, base_offset+positions_bytesize+normals_bytesize+tangents_bytesize+texcoords_bytesize+indices_bytesize
    );
  
  std::string geom_info_str = std::string(buf, strlen(buf));

  return insert_in_demo_scene(geom_info_str, scenes_relative_path, scene);
}

void save_xml_string(const std::string xml_string, const std::string &path)
{
  std::ofstream xml_file(path);
  xml_file << xml_string;
  xml_file.close();
}

std::string get_scenes_relative_path(std::string bin_file_name)
{
  std::string scenes_absolute_path = std::filesystem::current_path().string() + "/scenes";
  std::string xml_absolute_path = std::filesystem::absolute(std::filesystem::path(bin_file_name).parent_path()).string();
  std::string scenes_relative_path = std::filesystem::relative(scenes_absolute_path, xml_absolute_path).string();

  return scenes_relative_path;
}
void save_scene_xml(std::string path, std::string bin_file_name, ModelInfo info, int mat_id,
                    DemoScene scene)
{
  std::string scenes_relative_path = get_scenes_relative_path(path);
  save_xml_string(get_xml_string_model_demo_scene(bin_file_name, scenes_relative_path, info, mat_id, scene), path);
}

void save_scene_xml(std::string path, std::string bin_file_name, const cmesh4::SimpleMesh &mesh,
                    DemoScene scene)
{
  std::string scenes_relative_path = get_scenes_relative_path(path);
  save_xml_string(get_xml_string_model_demo_scene(bin_file_name, scenes_relative_path, mesh, scene), path);
}

SdfSBSAdaptView convert_sbs_to_adapt(SdfSBSAdapt &adapt_scene, const SdfSBSView &scene)
{
  assert(scene.header.brick_size > 0u && scene.header.brick_size < 256u);

  adapt_scene.header.brick_pad = scene.header.brick_pad;
  adapt_scene.header.bytes_per_value = scene.header.bytes_per_value;
  adapt_scene.header.aux_data = scene.header.aux_data;
  adapt_scene.values.insert(adapt_scene.values.end(), scene.values, scene.values + scene.values_count);
  adapt_scene.values_f.insert(adapt_scene.values_f.end(), scene.values_f, scene.values_f + scene.values_f_count);

  for (uint32_t i = 0u; i < scene.size; ++i)
  {
    uint3 pos;
    pos.x = (scene.nodes[i].pos_xy >> 16) & 0x0000FFFF;
    pos.y = (scene.nodes[i].pos_xy      ) & 0x0000FFFF;
    pos.z = (scene.nodes[i].pos_z_lod_size >> 16) & 0x0000FFFF;

    uint32_t lod_size = (scene.nodes[i].pos_z_lod_size) & 0x0000FFFF;
    uint32_t d_off    =  scene.nodes[i].data_offset;

    {
      int bit_count = 0;
      for (int j = 0; j < 16; ++j)
        bit_count += (lod_size >> j) & 1u;
      assert(bit_count == 1);
        // throw std::runtime_error("Error: convert SBS to adapt - lod size is not a power of two");
    }
    uint32_t brick_size_in_units = SDF_SBS_ADAPT_MAX_UNITS / lod_size;

    pos.x *= brick_size_in_units;
    pos.y *= brick_size_in_units;
    pos.z *= brick_size_in_units;

    assert(pos.x < SDF_SBS_ADAPT_MAX_UNITS);
    assert(pos.y < SDF_SBS_ADAPT_MAX_UNITS);
    assert(pos.z < SDF_SBS_ADAPT_MAX_UNITS);

    uint16_t voxel_size_in_units = float(brick_size_in_units) / scene.header.brick_size;

      uint32_t vox_count_xyz_pad = (scene.header.brick_size << 16) + (scene.header.brick_size << 8) + scene.header.brick_size;

      SdfSBSAdaptNode new_node;
      new_node.pos_xy = (pos.x << 16) + pos.y;
      new_node.pos_z_vox_size = (pos.z << 16) + voxel_size_in_units;
      new_node.data_offset = d_off;
      new_node.vox_count_xyz_pad = vox_count_xyz_pad;

      adapt_scene.nodes.push_back(new_node);
  }
  return adapt_scene;
}

SdfSBSAdaptView convert_sbs_to_adapt_with_split(SdfSBSAdapt &adapt_scene, const SdfSBSView &scene)
{
  assert(scene.header.brick_size > 1u && scene.header.brick_size < 256u);
  assert(scene.header.brick_pad == 0u);

  adapt_scene.header.brick_pad = scene.header.brick_pad;
  adapt_scene.header.bytes_per_value = scene.header.bytes_per_value;
  adapt_scene.header.aux_data = scene.header.aux_data;

  for (uint32_t i = 0u; i < scene.size; ++i)
  {
    uint3 pos;
    pos.x = (scene.nodes[i].pos_xy >> 16) & 0x0000FFFF;
    pos.y = (scene.nodes[i].pos_xy      ) & 0x0000FFFF;
    pos.z = (scene.nodes[i].pos_z_lod_size >> 16) & 0x0000FFFF;

    uint32_t lod_size = (scene.nodes[i].pos_z_lod_size) & 0x0000FFFF;
    uint32_t d_off    =  scene.nodes[i].data_offset;

    {
      int bit_count = 0;
      for (int j = 0; j < 16; ++j)
        bit_count += (lod_size >> j) & 1u;
      assert(bit_count == 1);
        // throw std::runtime_error("Error: convert SBS to adapt - lod size is not a power of two");
    }
    uint32_t brick_size_in_units = SDF_SBS_ADAPT_MAX_UNITS / lod_size;

    pos.x *= brick_size_in_units;
    pos.y *= brick_size_in_units;
    pos.z *= brick_size_in_units;

    assert(pos.x < SDF_SBS_ADAPT_MAX_UNITS);
    assert(pos.y < SDF_SBS_ADAPT_MAX_UNITS);
    assert(pos.z < SDF_SBS_ADAPT_MAX_UNITS);

    uint16_t voxel_size_in_units = float(brick_size_in_units) / scene.header.brick_size;

    // choose split axis: {0, 1, 2}
    uint split_axis = 0;
    {
      double axis_rand = double(std::rand()) / (RAND_MAX + 1.);
      split_axis = 3u * axis_rand;
      // printf("Split Axis: %d\n", split_axis);
    }

    // choose split size (brick_size of first brick along split axis): [1, header.brick_size - 1]
    uint3 brick_size{scene.header.brick_size};
    {
      double split_size_rand = double(std::rand()) / (RAND_MAX + 1.);
      brick_size[split_axis] = 1u + split_size_rand * (scene.header.brick_size - 1u);
      // printf("Split Size: %d\n", brick_size[split_axis]);
    }
    uint32_t vox_count_xyz_pad = (brick_size.x << 16) + (brick_size.y << 8) + brick_size.z;

    // set brick 1
    SdfSBSAdaptNode new_node;
    new_node.pos_xy = (pos.x << 16) + pos.y;
    new_node.pos_z_vox_size = (pos.z << 16) + voxel_size_in_units;
    new_node.data_offset = adapt_scene.values.size();
    new_node.vox_count_xyz_pad = vox_count_xyz_pad;

    adapt_scene.nodes.push_back(new_node);


    // update data for brick 2
    if (scene.header.aux_data != SDF_SBS_NODE_LAYOUT_ID32F_IRGB32F)
    {
      uint32_t v_size = scene.header.brick_size + 1;
      uint3 v_size_adapt{brick_size + 1};
      uint32_t vals_per_int = 4 / scene.header.bytes_per_value; 
      uint32_t bits         = 8 * scene.header.bytes_per_value;
      uint32_t max_val = scene.header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
      uint32_t adapt_d_off = adapt_scene.values.size();

      // brick 1 values
      for (uint32_t v_id = 0u; v_id < v_size*v_size*v_size; ++v_id)
      {
        uint3 v_pos{v_id/(v_size*v_size), v_id/v_size%v_size, v_id%v_size};

        if (v_pos[split_axis] <= brick_size[split_axis])
        {
          uint32_t single_value = (scene.values[d_off + v_id/vals_per_int] >> (bits*(v_id%vals_per_int))) & max_val;
          uint32_t v_id_adapt = v_pos.x*v_size_adapt.y*v_size_adapt.z + v_pos.y*v_size_adapt.z + v_pos.z;
          uint32_t adapt_val_ind = adapt_d_off + v_id_adapt/vals_per_int;
          while (adapt_val_ind >= adapt_scene.values.size())
            adapt_scene.values.push_back(0u);
          adapt_scene.values[adapt_val_ind] += single_value << (bits*(v_id_adapt%vals_per_int));
        }
      }
      adapt_d_off = adapt_scene.values.size();
      v_size_adapt[split_axis] = scene.header.brick_size - brick_size[split_axis] + 1;

      for (uint32_t v_id = 0u; v_id < v_size*v_size*v_size; ++v_id)
      {
        uint3 v_pos{v_id/(v_size*v_size), v_id/v_size%v_size, v_id%v_size};

        if (v_pos[split_axis] >= brick_size[split_axis])
        {
          v_pos[split_axis] -= brick_size[split_axis];
          uint32_t single_value = (scene.values[d_off + v_id/vals_per_int] >> (bits*(v_id%vals_per_int))) & max_val;
          uint32_t v_id_adapt = v_pos.x*v_size_adapt.y*v_size_adapt.z + v_pos.y*v_size_adapt.z + v_pos.z;
          uint32_t adapt_val_ind = adapt_d_off + v_id_adapt/vals_per_int;
          while (adapt_val_ind >= adapt_scene.values.size())
            adapt_scene.values.push_back(0u);
          adapt_scene.values[adapt_val_ind] += single_value << (bits*(v_id_adapt%vals_per_int));
        }
      }
      d_off = adapt_d_off;
    } // else not implemented

    pos[split_axis] += brick_size[split_axis] * voxel_size_in_units;
    assert(pos[split_axis] < SDF_SBS_ADAPT_MAX_UNITS);

    brick_size[split_axis] = scene.header.brick_size - brick_size[split_axis];
    vox_count_xyz_pad = (brick_size.x << 16) + (brick_size.y << 8) + brick_size.z;


    // set brick 2
    new_node.pos_xy = (pos.x << 16) + pos.y;
    new_node.pos_z_vox_size = (pos.z << 16) + voxel_size_in_units;
    new_node.data_offset = d_off;
    new_node.vox_count_xyz_pad = vox_count_xyz_pad;

    adapt_scene.nodes.push_back(new_node);
  }
  return adapt_scene;
}