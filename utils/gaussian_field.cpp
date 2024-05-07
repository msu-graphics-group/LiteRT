#include "gaussian_field.h"

#include "happly.h"

void parse_element_data(GSScene& scene, happly::Element& element) {
  scene.data_0.reserve(element.count);

  const auto x = element.getProperty<float>("x");
  const auto y = element.getProperty<float>("y");
  const auto z = element.getProperty<float>("z");

  const auto f_dc_0 = element.getProperty<float>("f_dc_0");
  const auto f_dc_1 = element.getProperty<float>("f_dc_1");
  const auto f_dc_2 = element.getProperty<float>("f_dc_2");

  const auto opacity = element.getProperty<float>("opacity");

  const auto scale_0 = element.getProperty<float>("scale_0");
  const auto scale_1 = element.getProperty<float>("scale_1");
  const auto scale_2 = element.getProperty<float>("scale_2");

  const auto rot_0 = element.getProperty<float>("rot_0");
  const auto rot_1 = element.getProperty<float>("rot_1");
  const auto rot_2 = element.getProperty<float>("rot_2");
  const auto rot_3 = element.getProperty<float>("rot_3");

  for (std::size_t i = 0; i < element.count; ++i) {
    scene.data_0.emplace_back(
      LiteMath::float4x4(x[i], y[i], z[i], f_dc_0[i],
                         f_dc_1[i], f_dc_2[i], opacity[i], scale_0[i],
                         scale_1[i], scale_2[i], rot_0[i], rot_1[i],
                         rot_2[i], rot_3[i], 0.0f, 0.0f)
    );
  }
}

void load_gs_scene(GSScene& scene, const std::string& path) {
  auto data = happly::PLYData(path, true);
  parse_element_data(scene, data.getElement("vertex"));
}
