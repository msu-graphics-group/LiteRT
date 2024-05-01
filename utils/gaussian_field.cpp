#include "gaussian_field.h"

#include "happly.h"

void parse_element_data(GSScene& scene, happly::Element& element) {
  scene.count = element.count;

  scene.x = element.getProperty<float>("x");
  scene.y = element.getProperty<float>("y");
  scene.z = element.getProperty<float>("z");

  scene.nx = element.getProperty<float>("nx");
  scene.ny = element.getProperty<float>("ny");
  scene.nz = element.getProperty<float>("nz");

  scene.f_dc_0 = element.getProperty<float>("f_dc_0");
  scene.f_dc_1 = element.getProperty<float>("f_dc_1");
  scene.f_dc_2 = element.getProperty<float>("f_dc_2");

  scene.f_rest_0 = element.getProperty<float>("f_rest_0");
  scene.f_rest_1 = element.getProperty<float>("f_rest_1");
  scene.f_rest_2 = element.getProperty<float>("f_rest_2");
  scene.f_rest_3 = element.getProperty<float>("f_rest_3");
  scene.f_rest_4 = element.getProperty<float>("f_rest_4");
  scene.f_rest_5 = element.getProperty<float>("f_rest_5");
  scene.f_rest_6 = element.getProperty<float>("f_rest_6");
  scene.f_rest_7 = element.getProperty<float>("f_rest_7");
  scene.f_rest_8 = element.getProperty<float>("f_rest_8");
  scene.f_rest_9 = element.getProperty<float>("f_rest_9");
  scene.f_rest_10 = element.getProperty<float>("f_rest_10");
  scene.f_rest_11 = element.getProperty<float>("f_rest_11");
  scene.f_rest_12 = element.getProperty<float>("f_rest_12");
  scene.f_rest_13 = element.getProperty<float>("f_rest_13");
  scene.f_rest_14 = element.getProperty<float>("f_rest_14");
  scene.f_rest_15 = element.getProperty<float>("f_rest_15");
  scene.f_rest_16 = element.getProperty<float>("f_rest_16");
  scene.f_rest_17 = element.getProperty<float>("f_rest_17");
  scene.f_rest_18 = element.getProperty<float>("f_rest_18");
  scene.f_rest_19 = element.getProperty<float>("f_rest_19");
  scene.f_rest_20 = element.getProperty<float>("f_rest_20");
  scene.f_rest_21 = element.getProperty<float>("f_rest_21");
  scene.f_rest_22 = element.getProperty<float>("f_rest_22");
  scene.f_rest_23 = element.getProperty<float>("f_rest_23");
  scene.f_rest_24 = element.getProperty<float>("f_rest_24");
  scene.f_rest_25 = element.getProperty<float>("f_rest_25");
  scene.f_rest_26 = element.getProperty<float>("f_rest_26");
  scene.f_rest_27 = element.getProperty<float>("f_rest_27");
  scene.f_rest_28 = element.getProperty<float>("f_rest_28");
  scene.f_rest_29 = element.getProperty<float>("f_rest_29");
  scene.f_rest_30 = element.getProperty<float>("f_rest_30");
  scene.f_rest_31 = element.getProperty<float>("f_rest_31");
  scene.f_rest_32 = element.getProperty<float>("f_rest_32");
  scene.f_rest_33 = element.getProperty<float>("f_rest_33");
  scene.f_rest_34 = element.getProperty<float>("f_rest_34");
  scene.f_rest_35 = element.getProperty<float>("f_rest_35");
  scene.f_rest_36 = element.getProperty<float>("f_rest_36");
  scene.f_rest_37 = element.getProperty<float>("f_rest_37");
  scene.f_rest_38 = element.getProperty<float>("f_rest_38");
  scene.f_rest_39 = element.getProperty<float>("f_rest_39");
  scene.f_rest_40 = element.getProperty<float>("f_rest_40");
  scene.f_rest_41 = element.getProperty<float>("f_rest_41");
  scene.f_rest_42 = element.getProperty<float>("f_rest_42");
  scene.f_rest_43 = element.getProperty<float>("f_rest_43");
  scene.f_rest_44 = element.getProperty<float>("f_rest_44");

  scene.opacity = element.getProperty<float>("opacity");

  scene.scale_0 = element.getProperty<float>("scale_0");
  scene.scale_1 = element.getProperty<float>("scale_1");
  scene.scale_2 = element.getProperty<float>("scale_2");

  scene.rot_0 = element.getProperty<float>("rot_0");
  scene.rot_1 = element.getProperty<float>("rot_1");
  scene.rot_2 = element.getProperty<float>("rot_2");
  scene.rot_3 = element.getProperty<float>("rot_3");

  scene.base_color_0 = element.getProperty<float>("base_color_0");
  scene.base_color_1 = element.getProperty<float>("base_color_1");
  scene.base_color_2 = element.getProperty<float>("base_color_2");

  scene.roughness = element.getProperty<float>("roughness");

  scene.metallic = element.getProperty<float>("metallic");

  scene.incidents_dc_0 = element.getProperty<float>("incidents_dc_0");
  scene.incidents_dc_1 = element.getProperty<float>("incidents_dc_1");
  scene.incidents_dc_2 = element.getProperty<float>("incidents_dc_2");

  scene.incidents_rest_0 = element.getProperty<float>("incidents_rest_0");
  scene.incidents_rest_1 = element.getProperty<float>("incidents_rest_1");
  scene.incidents_rest_2 = element.getProperty<float>("incidents_rest_2");
  scene.incidents_rest_3 = element.getProperty<float>("incidents_rest_3");
  scene.incidents_rest_4 = element.getProperty<float>("incidents_rest_4");
  scene.incidents_rest_5 = element.getProperty<float>("incidents_rest_5");
  scene.incidents_rest_6 = element.getProperty<float>("incidents_rest_6");
  scene.incidents_rest_7 = element.getProperty<float>("incidents_rest_7");
  scene.incidents_rest_8 = element.getProperty<float>("incidents_rest_8");
  scene.incidents_rest_9 = element.getProperty<float>("incidents_rest_9");
  scene.incidents_rest_10 = element.getProperty<float>("incidents_rest_10");
  scene.incidents_rest_11 = element.getProperty<float>("incidents_rest_11");
  scene.incidents_rest_12 = element.getProperty<float>("incidents_rest_12");
  scene.incidents_rest_13 = element.getProperty<float>("incidents_rest_13");
  scene.incidents_rest_14 = element.getProperty<float>("incidents_rest_14");
  scene.incidents_rest_15 = element.getProperty<float>("incidents_rest_15");
  scene.incidents_rest_16 = element.getProperty<float>("incidents_rest_16");
  scene.incidents_rest_17 = element.getProperty<float>("incidents_rest_17");
  scene.incidents_rest_18 = element.getProperty<float>("incidents_rest_18");
  scene.incidents_rest_19 = element.getProperty<float>("incidents_rest_19");
  scene.incidents_rest_20 = element.getProperty<float>("incidents_rest_20");
  scene.incidents_rest_21 = element.getProperty<float>("incidents_rest_21");
  scene.incidents_rest_22 = element.getProperty<float>("incidents_rest_22");
  scene.incidents_rest_23 = element.getProperty<float>("incidents_rest_23");
  scene.incidents_rest_24 = element.getProperty<float>("incidents_rest_24");
  scene.incidents_rest_25 = element.getProperty<float>("incidents_rest_25");
  scene.incidents_rest_26 = element.getProperty<float>("incidents_rest_26");
  scene.incidents_rest_27 = element.getProperty<float>("incidents_rest_27");
  scene.incidents_rest_28 = element.getProperty<float>("incidents_rest_28");
  scene.incidents_rest_29 = element.getProperty<float>("incidents_rest_29");
  scene.incidents_rest_30 = element.getProperty<float>("incidents_rest_30");
  scene.incidents_rest_31 = element.getProperty<float>("incidents_rest_31");
  scene.incidents_rest_32 = element.getProperty<float>("incidents_rest_32");
  scene.incidents_rest_33 = element.getProperty<float>("incidents_rest_33");
  scene.incidents_rest_34 = element.getProperty<float>("incidents_rest_34");
  scene.incidents_rest_35 = element.getProperty<float>("incidents_rest_35");
  scene.incidents_rest_36 = element.getProperty<float>("incidents_rest_36");
  scene.incidents_rest_37 = element.getProperty<float>("incidents_rest_37");
  scene.incidents_rest_38 = element.getProperty<float>("incidents_rest_38");
  scene.incidents_rest_39 = element.getProperty<float>("incidents_rest_39");
  scene.incidents_rest_40 = element.getProperty<float>("incidents_rest_40");
  scene.incidents_rest_41 = element.getProperty<float>("incidents_rest_41");
  scene.incidents_rest_42 = element.getProperty<float>("incidents_rest_42");
  scene.incidents_rest_43 = element.getProperty<float>("incidents_rest_43");
  scene.incidents_rest_44 = element.getProperty<float>("incidents_rest_44");

  scene.visibility_dc_0 = element.getProperty<float>("visibility_dc_0");

  scene.visibility_rest_0 = element.getProperty<float>("visibility_rest_0");
  scene.visibility_rest_1 = element.getProperty<float>("visibility_rest_1");
  scene.visibility_rest_2 = element.getProperty<float>("visibility_rest_2");
  scene.visibility_rest_3 = element.getProperty<float>("visibility_rest_3");
  scene.visibility_rest_4 = element.getProperty<float>("visibility_rest_4");
  scene.visibility_rest_5 = element.getProperty<float>("visibility_rest_5");
  scene.visibility_rest_6 = element.getProperty<float>("visibility_rest_6");
  scene.visibility_rest_7 = element.getProperty<float>("visibility_rest_7");
  scene.visibility_rest_8 = element.getProperty<float>("visibility_rest_8");
  scene.visibility_rest_9 = element.getProperty<float>("visibility_rest_9");
  scene.visibility_rest_10 = element.getProperty<float>("visibility_rest_10");
  scene.visibility_rest_11 = element.getProperty<float>("visibility_rest_11");
  scene.visibility_rest_12 = element.getProperty<float>("visibility_rest_12");
  scene.visibility_rest_13 = element.getProperty<float>("visibility_rest_13");
  scene.visibility_rest_14 = element.getProperty<float>("visibility_rest_14");
}

void load_gs_scene(GSScene& scene, const std::string& path) {
  auto data = happly::PLYData(path, true);
  parse_element_data(scene, data.getElement("vertex"));
}
