#include "gaussian_field.h"

#include "happly.h"

void parse_element_data(GSScene& scene, happly::Element& element) {
  scene.data.reserve(element.count);

  const auto x = element.getProperty<float>("x");
  const auto y = element.getProperty<float>("y");
  const auto z = element.getProperty<float>("z");

  const auto f_dc_0 = element.getProperty<float>("f_dc_0");
  const auto f_dc_1 = element.getProperty<float>("f_dc_1");
  const auto f_dc_2 = element.getProperty<float>("f_dc_2");

  const auto f_rest_0 = element.getProperty<float>("f_rest_0");
  const auto f_rest_1 = element.getProperty<float>("f_rest_1");
  const auto f_rest_2 = element.getProperty<float>("f_rest_2");
  const auto f_rest_3 = element.getProperty<float>("f_rest_3");
  const auto f_rest_4 = element.getProperty<float>("f_rest_4");
  const auto f_rest_5 = element.getProperty<float>("f_rest_5");
  const auto f_rest_6 = element.getProperty<float>("f_rest_6");
  const auto f_rest_7 = element.getProperty<float>("f_rest_7");
  const auto f_rest_8 = element.getProperty<float>("f_rest_8");
  const auto f_rest_9 = element.getProperty<float>("f_rest_9");
  const auto f_rest_10 = element.getProperty<float>("f_rest_10");
  const auto f_rest_11 = element.getProperty<float>("f_rest_11");
  const auto f_rest_12 = element.getProperty<float>("f_rest_12");
  const auto f_rest_13 = element.getProperty<float>("f_rest_13");
  const auto f_rest_14 = element.getProperty<float>("f_rest_14");
  const auto f_rest_15 = element.getProperty<float>("f_rest_15");
  const auto f_rest_16 = element.getProperty<float>("f_rest_16");
  const auto f_rest_17 = element.getProperty<float>("f_rest_17");
  const auto f_rest_18 = element.getProperty<float>("f_rest_18");
  const auto f_rest_19 = element.getProperty<float>("f_rest_19");
  const auto f_rest_20 = element.getProperty<float>("f_rest_20");
  const auto f_rest_21 = element.getProperty<float>("f_rest_21");
  const auto f_rest_22 = element.getProperty<float>("f_rest_22");
  const auto f_rest_23 = element.getProperty<float>("f_rest_23");
  const auto f_rest_24 = element.getProperty<float>("f_rest_24");
  const auto f_rest_25 = element.getProperty<float>("f_rest_25");
  const auto f_rest_26 = element.getProperty<float>("f_rest_26");
  const auto f_rest_27 = element.getProperty<float>("f_rest_27");
  const auto f_rest_28 = element.getProperty<float>("f_rest_28");
  const auto f_rest_29 = element.getProperty<float>("f_rest_29");
  const auto f_rest_30 = element.getProperty<float>("f_rest_30");
  const auto f_rest_31 = element.getProperty<float>("f_rest_31");
  const auto f_rest_32 = element.getProperty<float>("f_rest_32");
  const auto f_rest_33 = element.getProperty<float>("f_rest_33");
  const auto f_rest_34 = element.getProperty<float>("f_rest_34");
  const auto f_rest_35 = element.getProperty<float>("f_rest_35");
  const auto f_rest_36 = element.getProperty<float>("f_rest_36");
  const auto f_rest_37 = element.getProperty<float>("f_rest_37");
  const auto f_rest_38 = element.getProperty<float>("f_rest_38");
  const auto f_rest_39 = element.getProperty<float>("f_rest_39");
  const auto f_rest_40 = element.getProperty<float>("f_rest_40");
  const auto f_rest_41 = element.getProperty<float>("f_rest_41");
  const auto f_rest_42 = element.getProperty<float>("f_rest_42");
  const auto f_rest_43 = element.getProperty<float>("f_rest_43");
  const auto f_rest_44 = element.getProperty<float>("f_rest_44");

  const auto opacity = element.getProperty<float>("opacity");

  const auto scale_0 = element.getProperty<float>("scale_0");
  const auto scale_1 = element.getProperty<float>("scale_1");
  const auto scale_2 = element.getProperty<float>("scale_2");

  const auto rot_0 = element.getProperty<float>("rot_0");
  const auto rot_1 = element.getProperty<float>("rot_1");
  const auto rot_2 = element.getProperty<float>("rot_2");
  const auto rot_3 = element.getProperty<float>("rot_3");

  for (std::size_t i = 0; i < element.count; ++i) {
    scene.data.emplace_back(
      LiteMath::float4x4(x[i], y[i], z[i], opacity[i],
                         rot_0[i], rot_1[i], rot_2[i], rot_3[i],
                         scale_0[i], scale_1[i], scale_2[i], 0.0f,
                         0.0f, 0.0f, 0.0f, 0.0f)
    );

    scene.data_r.emplace_back(
      LiteMath::float4x4(f_dc_0[i], f_rest_0[i], f_rest_1[i], f_rest_2[i],
                         f_rest_3[i], f_rest_4[i], f_rest_5[i], f_rest_6[i],
                         f_rest_7[i], f_rest_8[i], f_rest_9[i], f_rest_10[i],
                         f_rest_11[i], f_rest_12[i], f_rest_13[i], f_rest_14[i])
    );

    scene.data_g.emplace_back(
      LiteMath::float4x4(f_dc_1[i], f_rest_15[i], f_rest_16[i], f_rest_17[i],
                         f_rest_18[i], f_rest_19[i], f_rest_20[i], f_rest_21[i],
                         f_rest_22[i], f_rest_23[i], f_rest_24[i], f_rest_25[i],
                         f_rest_26[i], f_rest_27[i], f_rest_28[i], f_rest_29[i])
    );

    scene.data_b.emplace_back(
      LiteMath::float4x4(f_dc_2[i], f_rest_30[i], f_rest_31[i], f_rest_32[i],
                         f_rest_33[i], f_rest_34[i], f_rest_35[i], f_rest_36[i],
                         f_rest_37[i], f_rest_38[i], f_rest_39[i], f_rest_40[i],
                         f_rest_41[i], f_rest_42[i], f_rest_43[i], f_rest_44[i])
    );
  }
}

void load_gs_scene(GSScene& scene, const std::string& points_path) {
  auto data = happly::PLYData(points_path, true);
  parse_element_data(scene, data.getElement("vertex"));
}
