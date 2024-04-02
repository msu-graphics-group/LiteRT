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