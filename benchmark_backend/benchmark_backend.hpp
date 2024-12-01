#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <vector>
#include <cstdlib>

#include "../utils/mesh.h"
#include "../utils/sdf_converter.h"
#include "../render_settings.h"
#include "../Renderer/eye_ray.h"
#include "../utils/image_metrics.h"
#include "../utils/mesh_bvh.h"

namespace BenchmarkBackend
{
    MultiRenderPreset createPreset(const std::string& render_mode, const int spp);
    int getDevice(const std::string backend);
    int getFileSize(std::string file_name);
    void calcMetrics(float& min, float& max, float& average, const float& new_val);

    void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
                float3 pos, float3 target, float3 up, 
                MultiRenderPreset preset, int a_passNum = 1);

    void getInfoMesh();

    void getInfoGrid();

    void getInfoSVS();

    void getInfoSBS();

    //  very slooooow
    void getInfoAdaptSBS();
};