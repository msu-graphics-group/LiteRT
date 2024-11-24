#include <testing_framework/core/environment.h>
#include <testing_framework/core/add_test.h>

#include <testing_framework/helpers/render.h>
#include <testing_framework/helpers/scoped_timer.h>
#include <testing_framework/helpers/check.h>
#include <testing_framework/helpers/mesh.h>
/*
    Includes for code
*/
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"
#include "../utils/sdf_converter.h"
#include "../utils/sparse_octree_builder.h"
#include "../utils/marching_cubes.h"
#include "../utils/sdf_smoother.h"
#include "../utils/demo_meshes.h"
#include "../utils/image_metrics.h"
#include "../diff_render/MultiRendererDR.h"
#include "../utils/iou.h"
#include "../nurbs/nurbs_common_host.h"

namespace litert_tests
{
    const char TEAPOT_MESH[] = "01_simple_scenes/data/teapot.vsgf";

    const char TEAPOT_HYDRA[] = "01_simple_scenes/teapot.xml";

    // former test 1
    ADD_TEST(FramedOctree, "Testing if framed octree is working")
    {
        constexpr size_t OCTREE_DEPTH = 9;
        auto mesh = testing::load_vsgf_mesh(TEAPOT_MESH);

        // Creating framed octree
        testing::ScopedTimer timer("creating SDF framed octree from mesh");
        SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, OCTREE_DEPTH);
        std::vector<SdfFrameOctreeNode> frame_nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
        timer.end();

        auto image = testing::create_image();

        std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ST,
                                            SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ST};

        std::vector<std::string> names = {"bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

        for (int i = 0; i < presets_oi.size(); i++)
        {
            MultiRenderPreset preset = getDefaultPreset();
            preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
            preset.sdf_node_intersect = presets_oi[i];

            testing::render_scene(image, DEVICE_GPU, preset, frame_nodes);
            testing::save_image(image, names[i]);
        }
    }

    // former test 2
    ADD_TEST(SVS, "Testing if SVS is working")
    {
        constexpr size_t OCTREE_DEPTH = 9;
        auto mesh = testing::load_vsgf_mesh(TEAPOT_MESH);

        // Creating SVS
        testing::ScopedTimer timer("creating SDF SVS");
        SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, OCTREE_DEPTH);
        std::vector<SdfSVSNode> frame_nodes = sdf_converter::create_sdf_SVS(settings, mesh);
        timer.end();

        auto image = testing::create_image();

        std::vector<unsigned> presets_oi = {SDF_OCTREE_NODE_INTERSECT_ST, SDF_OCTREE_NODE_INTERSECT_ANALYTIC,
                                            SDF_OCTREE_NODE_INTERSECT_NEWTON, SDF_OCTREE_NODE_INTERSECT_BBOX};

        std::vector<std::string> names = {"bvh_sphere_tracing", "bvh_analytic", "bvh_newton", "bvh_bboxes"};

        for (int i = 0; i < presets_oi.size(); i++)
        {
            MultiRenderPreset preset = getDefaultPreset();
            preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
            preset.sdf_node_intersect = presets_oi[i];

            testing::render_scene(image, DEVICE_GPU, preset, frame_nodes);
            testing::save_image(image, names[i]);
        }
    }

    // former 3
    ADD_TEST(SVS_and_SBS_verification, "Testing if SVS and SBS are correct")
    {

        constexpr size_t OCTREE_DEPTH = 8;

        MultiRenderPreset preset = getDefaultPreset();
        preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
        preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

        auto mesh = testing::load_vsgf_mesh(TEAPOT_MESH);

        SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, OCTREE_DEPTH);

        struct sbs_type
        {
            unsigned int voxels, bytes;
        };

        std::vector<sbs_type> sbs_types = {
            {1, 1},
            {1, 2},
            {2, 1},
            {2, 2},
            {2, 4}};

        std::vector<SdfSBS> sbs;
        std::vector<std::string> sbs_names;
        std::vector<std::string> sbs_file_names;

        for (auto [voxels, bytes] : sbs_types)
        {
            sbs_names.push_back(
                "SBS " + std::to_string(voxels * voxels * voxels) + "-voxels " + std::to_string(bytes) + "-bytes");
            sbs_file_names.push_back(
                "sbs_" + std::to_string(voxels) + "_" + std::to_string(bytes));
            testing::ScopedTimer timer("creating SDF " + sbs_names.back());
            sbs.push_back(sdf_converter::create_sdf_SBS(settings, SdfSBSHeader{voxels, 0, bytes, SDF_SBS_NODE_LAYOUT_DX}, mesh));
        }

        testing::ScopedTimer timer("creating SDF SVS");
        std::vector<SdfSVSNode> svs_nodes = sdf_converter::create_sdf_SVS(settings, mesh);
        timer.end();

        auto image = testing::create_image();
        auto ref_image = testing::create_image();
        auto svs_image = testing::create_image();

        {
            testing::render_scene(ref_image, DEVICE_CPU, preset, mesh);
            testing::save_image(ref_image, "ref_cpu");
        }
        {
            testing::render_scene(svs_image, DEVICE_CPU, preset, svs_nodes);
            testing::save_image(svs_image, "svs_cpu");

            testing::check_psnr(ref_image, svs_image, "mesh", "SVS", 40);
        }
        {
            testing::render_scene(image, DEVICE_CPU, preset, sbs[0]);
            testing::save_image(image, sbs_file_names[0] + "_cpu");

            testing::check_psnr(ref_image, image, "mesh", sbs_names[0], 40);
            testing::check_psnr(svs_image, image, "SVS", sbs_names[0], 40);
        }
        for (size_t i = 0; i < sbs.size(); i++)
        {
            testing::render_scene(image, DEVICE_CPU, preset, sbs[i]);
            testing::save_image(image, sbs_file_names[i] + "_gpu");

            testing::check_psnr(ref_image, image, "mesh", sbs_names[i] + " GPU", 40);
            testing::check_psnr(svs_image, image, "SVS", sbs_names[i] + " GPU", 40);
        }
    }

    // former 6
    ADD_TEST(FasterBVHBuild, "MESH_TLO SDF BVH build")
    {
        auto scene = TEAPOT_HYDRA;

        MultiRenderPreset ref_preset = getDefaultPreset();
        MultiRenderPreset bvh_preset = getDefaultPreset();
        bvh_preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;
        MultiRenderPreset tlo_bvh_preset = getDefaultPreset();
        tlo_bvh_preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_NEWTON;

        auto ref_image = testing::create_image();
        auto bvh_image = testing::create_image();
        auto tlo_bvh_image = testing::create_image();

        {
            testing::render_hydra_scene(ref_image, DEVICE_GPU, ref_preset, scene);
            testing::save_image(ref_image, "ref");
        }

        {
            testing::render_hydra_scene(
                bvh_image,
                DEVICE_GPU,
                bvh_preset,
                scene,
                TYPE_SDF_SVS,
                SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9),
                "creating BVH from Hydra scene");
            testing::save_image(bvh_image, "bvh");
        }
        {
            testing::render_hydra_scene(
                tlo_bvh_image,
                DEVICE_GPU,
                tlo_bvh_preset,
                scene,
                TYPE_SDF_SVS,
                SparseOctreeSettings(SparseOctreeBuildType::MESH_TLO, 9),
                "creating mesh TLO BVH from Hydra scene");
            testing::save_image(tlo_bvh_image, "tlo_bvh");
        }

        testing::check_psnr(ref_image, tlo_bvh_image, "mesh", "mesh TLO BVH", 45);
        testing::check_psnr(bvh_image, tlo_bvh_image, "BVH", "mesh TLO BVH", 30);
    }

    // former 8
    ADD_TEST(SDF_grid, "Testing SDF grid")
    {
        auto scene = TEAPOT_HYDRA;

        MultiRenderPreset preset = getDefaultPreset();
        preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
        preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ANALYTIC;

        auto ref_image = testing::create_image();
        auto image = testing::create_image();

        testing::render_hydra_scene(
            ref_image,
            DEVICE_GPU,
            preset,
            scene);
        testing::save_image(ref_image, "ref");

        testing::render_hydra_scene(
            image,
            DEVICE_GPU,
            preset,
            scene,
            TYPE_SDF_GRID,
            SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 7),
            "creating SDF grid from Hydra scene");
        testing::save_image(image, "grid");

        testing::check_psnr(ref_image, image, "mesh", "SDF grid", 30);
    }

    // former 9
    ADD_TEST(MeshRender, "Testing if mesh are rendered on cpu and gpu")
    {
        auto scene = TEAPOT_HYDRA;

        MultiRenderPreset preset = getDefaultPreset();
        auto cpu_image = testing::create_image();
        auto gpu_image = testing::create_image();

        testing::render_hydra_scene(cpu_image, DEVICE_CPU, preset, scene);
        testing::save_image(cpu_image, "cpu");

        testing::render_hydra_scene(gpu_image, DEVICE_GPU, preset, scene);
        testing::save_image(gpu_image, "gpu");

        testing::check_psnr(cpu_image, gpu_image, "cpu", "gpu", 45);

        testing::saved_reference_check_psnr(cpu_image, "CPU", "cpu", 90);
        testing::saved_reference_check_psnr(gpu_image, "GPU", "gpu", 90);
    }

}