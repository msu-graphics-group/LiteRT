#pragma once
#include <string>
#include <testing_framework/helpers/image.h>
#include <testing_framework/helpers/format.h>
#include <testing_framework/helpers/scoped_timer.h>
#include "../Renderer/eye_ray.h"
#include "utils/sdf_converter.h"

namespace testing
{

    template <typename T>
    void call_render(
        Image<T> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        float4x4 world_view,
        float4x4 proj, 
        size_t passes);

    template <typename T>
    void render(
        Image<T> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        float4x4 world_view,
        float4x4 proj)
    {
        renderer->SetPreset(preset);

        size_t passes = renderings_count();

        float timings[4];

        log(bar_info) << "Rendering "
                      << foreground(highlight_color_1) << passes << default_color
                      << " times" << std::endl;

        call_render(image, renderer, preset, world_view, proj, passes);

        renderer->GetExecutionTime("CastRaySingleBlock", timings);

        float time_ms = timings[0] / passes;

        log(bar_info)
            << begin_aligned(MESSAGE_WIDTH, -1, 0) << "Finished rendering " << end_aligned
            << foreground(highlight_color_3) << format_time(time_ms) << default_color
            << std::endl;
    }

    template <typename T>
    void render(
        Image<T> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        float3 pos = float3(0, 0, 3),
        float3 target = float3(0, 0, 0),
        float3 up = float3(0, 1, 0))
    {
        float fov_degrees = 60;
        float z_near = 0.1f;
        float z_far = 100.0f;
        float aspect = (float)image.width() / image.height();
        auto proj = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
        auto worldView = LiteMath::lookAt(pos, target, up);
        render(image, renderer, preset, worldView, proj);
    }

    template <typename T, typename Scene>
    void render_scene(
        Image<T> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        const Scene &scene,
        float3 pos = float3(0, 0, 3),
        float3 target = float3(0, 0, 0),
        float3 up = float3(0, 1, 0))
    {
        renderer->SetScene(scene);
        render(image, renderer, preset, pos, target, up);
    }

    /*
        Creates renderer with given type and sets scene
    */
    template <typename T, typename Scene>
    void render_scene(
        Image<T> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const Scene &scene,
        float3 pos = float3(0, 0, 3),
        float3 target = float3(0, 0, 0),
        float3 up = float3(0, 1, 0))
    {
        return render_scene(image, CreateMultiRenderer(renderer_type), preset, scene, pos, target, up);
    }

    /*
        Calls renderer->LoadSceneHydra(...)
    */
    template <typename T>
    void render_hydra_scene(
        Image<T> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const std::string &scene_path,
        source_location loc = source_location::current())
    {
        std::string path = scenes_directory() + "/" + scene_path;
        log(bar_info) << "Loading Hydra Scene from "
                      << foreground(highlight_color_2) << path << default_color
                      << std::endl;
        assert_file_existance(path, true, loc);

        auto renderer = CreateMultiRenderer(renderer_type);
        renderer->SetPreset(preset);
        renderer->SetViewport(0, 0, image.width(), image.height());
        renderer->LoadSceneHydra(path);

        render(image, renderer, preset, renderer->getWorldView(), renderer->getProj());
    }

    /*
        Calls renderer->CreateSceneFromHydra
        If <comment> is not empty runs ScopedTimer(<comment>) on renderer->CreateSceneFromHydra(...)
    */
    template <typename T>
    void render_hydra_scene(
        Image<T> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const std::string &scene_path,
        unsigned int type,
        SparseOctreeSettings settings,
        std::string comment = "",
        source_location loc = source_location::current())
    {
        std::string path = scenes_directory() + "/" + scene_path;
        log(bar_info) << "Loading Hydra Scene from "
                      << foreground(highlight_color_2) << path << default_color
                      << std::endl;
        assert_file_existance(path, true, loc);

        auto renderer = CreateMultiRenderer(renderer_type);
        auto timer = comment.length() > 0 ? std::optional{ScopedTimer(comment)} : std::nullopt;
        renderer->SetPreset(preset);
        renderer->SetViewport(0, 0, image.width(), image.height());
        renderer->CreateSceneFromHydra(path, type, settings);
        if (timer)
        {
            timer->end();
        }

        render(image, renderer, preset, renderer->getWorldView(), renderer->getProj());
    }

}
