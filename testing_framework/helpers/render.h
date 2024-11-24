#pragma once
#include <string>
#include "../../utils/mesh.h"
#include "LiteMath/Image2d.h"
#include "../../Renderer/eye_ray.h"

namespace testing
{

    using Mesh = cmesh4::SimpleMesh;
    using Image = LiteImage::Image2D<uint32_t>;

    /*
        Loads mesh from <scenes-dir>/<scene>
    */
    Mesh load_vsgf_mesh_unfitted(const std::string&scene);

    /*
        Transforms mesh to fit into bounding box
    */
    void refit_mesh(Mesh&mesh, float3 box_min, float3 box_max);

    /*
        Loads mesh
        Transforms it to fit inside box [-box_size, box_size]
    */
    Mesh load_vsgf_mesh(const std::string& path, float box_size = 0.9);

    /*
        Creates image with size <image-width>*<image-height>
    */
    LiteImage::Image2D<uint32_t> create_image();

    /*
        Renders image and measures time
    */
    void render(
        LiteImage::Image2D<uint32_t> &image,
        std::shared_ptr<MultiRenderer> pRender,
        MultiRenderPreset preset,
        float3 pos = float3(0, 0, 3),
        float3 target = float3(0, 0, 0),
        float3 up = float3(0, 1, 0)
    );

    void save_image_by_path(Image&image, const std::string&path, std::string_view comment = "");

    /*
        Saves image to path
        <saves-dir>/<test-name>/<name>.png
        Prints "Saving <comment> image to <path>"
    */
    void save_image(Image&image, const std::string&name, std::string_view comment = "");

    /*
        Creates renderer with given type and sets scene
        Renders
    */
    template<typename T>
    void render_scene(
        LiteImage::Image2D<uint32_t> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const T&scene,
        float3 pos = float3(0, 0, 3),
        float3 target = float3(0, 0, 0),
        float3 up = float3(0, 1, 0)
    )
    {
        auto renderer = CreateMultiRenderer(renderer_type);
        renderer->SetScene(scene);
        render(image, renderer, preset, pos, target, up);
    }

    /*
        Calls renderer->LoadSceneHydra(...)
    */
   void render_hydra_scene(
        LiteImage::Image2D<uint32_t> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const std::string&scene_path
    );

    /*
        Calls renderer->CreateSceneFromHydra
        If <comment> is not empty runs ScopedTimer(<comment>) on renderer->CreateSceneFromHydra(...)
    */
    void render_hydra_scene(
        LiteImage::Image2D<uint32_t> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const std::string&scene_path,
        unsigned int type,
        SparseOctreeSettings settings,
        std::string comment = ""
    );

}
