#include <testing_framework/helpers/render.h>

namespace testing
{

    template <>
    void call_render<uint32_t>(
        Image<uint32_t> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        float4x4 world_view,
        float4x4 proj,
        size_t passes
        )
    {
        renderer->Render(image.data(), image.width(), image.height(), world_view, proj, preset, passes);
    }

    template <>
    void call_render<LiteMath::float4>(
        Image<LiteMath::float4> &image,
        std::shared_ptr<MultiRenderer> renderer,
        MultiRenderPreset preset,
        float4x4 world_view,
        float4x4 proj,
        size_t passes)
    {
        renderer->RenderFloat(image.data(), image.width(), image.height(), world_view, proj, preset, passes);
    }

}