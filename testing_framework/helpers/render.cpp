#include <testing_framework/helpers/render.h>
#include <testing_framework/core/environment.h>
#include <testing_framework/helpers/options.h>
#include <testing_framework/core/logging.h>
#include <testing_framework/helpers/format.h>
#include <testing_framework/helpers/scoped_timer.h>
#include <iomanip>
#include <filesystem>

#include "../../utils/sdf_converter.h"

namespace testing
{

    static void check_file_existance(const std::string &path)
    {
        if (!std::filesystem::exists(path))
        {
            log(bar_error) << "File "
                           << foreground(highlight_color_2) << path << default_color
                           << " does not exist" << std::endl;
            skip();
        }
    }

    Mesh load_vsgf_mesh_unfitted(const std::string &scene)
    {
        std::string path = scenes_directory() + "/" + scene;
        log(bar_info) << "Loading VSGF mesh from "
                      << foreground(highlight_color_2) << path << default_color
                      << std::endl;
        check_file_existance(path);
        auto mesh = cmesh4::LoadMeshFromVSGF(path.c_str());
        log(bar_info) << "Mesh triangles count: "
                      << foreground(highlight_color_3) << mesh.TrianglesNum()
                      << std::endl;
        return mesh;
    }

    void refit_mesh(Mesh &mesh, float3 box_min, float3 box_max)
    {
        float3 min, max;
        get_bbox(mesh, &min, &max);
        std::cout << std::fixed << std::setprecision(3);
        log(bar_info) << "Mesh bounding box: "
                      << "["
                      << foreground(highlight_color_1) << min << default_color
                      << " - "
                      << foreground(highlight_color_1) << max << default_color
                      << "]" << std::endl;
        log(bar_info) << "Transforming mesh to fit into bounding box "
                      << "["
                      << foreground(highlight_color_1) << box_min << default_color
                      << " - "
                      << foreground(highlight_color_1) << box_max << default_color
                      << "]" << std::endl;
        rescale_mesh(mesh, box_min, box_max);
        get_bbox(mesh, &min, &max);
        log(bar_info) << "Mesh bounding box after transformation: "
                      << "["
                      << foreground(highlight_color_1) << min << default_color
                      << " - "
                      << foreground(highlight_color_1) << max << default_color
                      << "]" << std::endl;
    }

    Mesh load_vsgf_mesh(const std::string &scene, float box_size)
    {
        auto mesh = load_vsgf_mesh_unfitted(scene);
        float3 box = float3(1, 1, 1) * box_size;
        refit_mesh(mesh, -box, box);
        return mesh;
    }

    LiteImage::Image2D<uint32_t> create_image()
    {
        return LiteImage::Image2D<uint32_t>(image_width(), image_height());
    }

    void save_image_by_path(Image&image, const std::string&path, std::string_view comment)
    {
        std::filesystem::create_directories(std::filesystem::path(path).parent_path());
        log(bar_info) << "Saving " << comment << (comment.length() > 0 ? " " : "") << "image to "
                      << foreground(highlight_color_2) << path << default_color
                      << std::endl;
        LiteImage::SaveImage<uint32_t>(path.c_str(), image);
    }

    void save_image(Image&image, const std::string &name, std::string_view comment)
    {
        save_image_by_path(
            image,
            saves_directory() + "/" + std::string(get_test_name()) + "/" + name + ".png", comment
        );
    }

    void render(
        LiteImage::Image2D<uint32_t> &image,
        std::shared_ptr<MultiRenderer> pRender,
        MultiRenderPreset preset,
        float4x4 proj,
        float4x4 world_view
    )
    {
        pRender->SetPreset(preset);

        size_t passes = renderings_count();

        float timings[4];

        log(bar_info) << "Rendering "
                      << foreground(highlight_color_1) << passes << default_color
                      << " times" << std::endl;

        pRender->Render(image.data(), image.width(), image.height(), world_view, proj, preset, passes);

        pRender->GetExecutionTime("CastRaySingleBlock", timings);

        float time_ms = timings[0] / passes;

        log(bar_info)
            << begin_aligned(MESSAGE_WIDTH, -1, 0) << "Finished rendering " << end_aligned
            << foreground(highlight_color_3) << format_time(time_ms) << default_color
            << std::endl;
    }

    void render(
        LiteImage::Image2D<uint32_t> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        float3 pos,
        float3 target,
        float3 up)
    {
        float fov_degrees = 60;
        float z_near = 0.1f;
        float z_far = 100.0f;
        float aspect = (float)image.width() / image.height();
        auto proj = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
        auto worldView = LiteMath::lookAt(pos, target, up);
        render(image, renderer, preset, proj, worldView);
    }

    void render_hydra_scene(
        LiteImage::Image2D<uint32_t> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const std::string&scene_path
    )
    {
        std::string path = scenes_directory() + "/" + scene_path;
        log(bar_info) << "Loading Hydra Scene from "
            << foreground(highlight_color_2) << path << default_color
            << std::endl;
        check_file_existance(path);

        auto renderer = CreateMultiRenderer(renderer_type);
        renderer->SetPreset(preset);
        renderer->SetViewport(0, 0, image.width(), image.height());
        renderer->LoadSceneHydra(path);

        render(image, renderer, preset, renderer->getProj(), renderer->getWorldView());
    }

    void render_hydra_scene(
        LiteImage::Image2D<uint32_t> &image,
        int renderer_type,
        MultiRenderPreset preset,
        const std::string &scene_path,
        unsigned int type,
        SparseOctreeSettings settings,
        std::string comment
    )
    {
        std::string path = scenes_directory() + "/" + scene_path;
        log(bar_info) << "Loading Hydra Scene from "
            << foreground(highlight_color_2) << path << default_color
            << std::endl;
        check_file_existance(path);
        
        auto renderer = CreateMultiRenderer(renderer_type);
        auto timer = comment.length() > 0 ? std::optional{ScopedTimer(comment)} : std::nullopt;
        renderer->SetPreset(preset);
        renderer->SetViewport(0, 0, image.width(), image.height());
        renderer->CreateSceneFromHydra(path, type, settings);
        if (timer)
        {
            timer->end();
        }

        render(image, renderer, preset, renderer->getProj(), renderer->getWorldView());
    }

}