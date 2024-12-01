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

    void getMetrics(const char** argv);

    void getInfoMesh(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

    void getInfoGrid(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

    void getInfoSVS(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

    void getInfoSBS(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);

    //  very slooooow
    void getInfoAdaptSBS(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras);
};