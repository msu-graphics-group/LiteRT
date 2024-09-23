#pragma once

#include <cstdint>
#include <cstddef>
#include <unordered_set>

#include "LiteMath.h"
#include "dependencies/HydraCore3/external/CrossRT/CrossRT.h"
#include "sdfScene/sdf_scene.h"
#include "utils/radiance_field.h"
#include "utils/gaussian_field.h"
#include "render_settings.h"

std::shared_ptr<ISceneObject> CreateSceneRT(const char* a_implName, const char* a_buildName, const char* a_layoutName);
