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
#include "../utils/ball_tree.h"

namespace litert_tests
{
    const char TEAPOT_MESH[] = "01_simple_scenes/data/teapot.vsgf";
    const char BUNNY_MESH[] = "01_simple_scenes/data/bunny.vsgf";

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

        // WTF? repeating
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

    // former 15
    ADD_TEST(FrameOctreeNodeRemoval, "Testing frame octree node removal")
    {
        // auto mesh = cmesh4::LoadMeshFromVSGF(std::string("./scenes/01_simple_scenes/data/teapot.vsgf").c_str());

        auto mesh = testing::load_vsgf_mesh(
            TEAPOT_MESH,
            0.999 // heavily changes PSNR
        );

        // WTF?
        // cmesh4::normalize_mesh(mesh);   // does nothing
        // MeshBVH mesh_bvh;               // is not used
        // mesh_bvh.init(mesh);            // is not used

        std::vector<SdfFrameOctreeNode> octree_nodes_6;
        std::vector<SdfFrameOctreeNode> octree_nodes_7;
        std::vector<SdfFrameOctreeNode> octree_nodes_8;
        const unsigned level_6_nodes = 21603;

        {
            testing::ScopedTimer timer("creating octree with depth-6, trimmed to " + std::to_string(level_6_nodes));
            SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 6);
            octree_nodes_6 = sdf_converter::create_sdf_frame_octree(settings, mesh);
            sdf_converter::frame_octree_limit_nodes(octree_nodes_6, level_6_nodes, false);
        }

        {
            testing::ScopedTimer timer("creating octree with depth-7, trimmed to " + std::to_string(level_6_nodes));
            SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 7);
            octree_nodes_7 = sdf_converter::create_sdf_frame_octree(settings, mesh);
            sdf_converter::frame_octree_limit_nodes(octree_nodes_7, level_6_nodes, false);
        }

        {
            testing::ScopedTimer timer("creating octree with depth-8, trimmed to " + std::to_string(level_6_nodes));
            SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
            octree_nodes_8 = sdf_converter::create_sdf_frame_octree(settings, mesh);
            sdf_converter::frame_octree_limit_nodes(octree_nodes_8, level_6_nodes, false);
        }

        MultiRenderPreset preset = getDefaultPreset();
        preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;

        auto image_6 = testing::create_image();
        auto image_7 = testing::create_image();
        auto image_8 = testing::create_image();

        testing::render_scene(image_6, DEVICE_GPU, preset, octree_nodes_6);
        testing::save_image(image_6, "6");

        testing::render_scene(image_7, DEVICE_GPU, preset, octree_nodes_7);
        testing::save_image(image_7, "trimmed_7");

        testing::render_scene(image_8, DEVICE_GPU, preset, octree_nodes_8);
        testing::save_image(image_8, "trimmed_8");

        testing::check_psnr(image_6, image_7, "octree-6", "octree-7", 45);
        testing::check_psnr(image_6, image_8, "octree-6", "octree-8", 45);
    }

    // former 16
    ADD_TEST(SVS_Nodes_Removal, "Testing removing nodes from SVS")
    {

        auto mesh = testing::load_vsgf_mesh(
            TEAPOT_MESH, 0.999 // WTF? changing from 0.9 to 0.999, made PSNR 100 insdead of 47.2
        );
        /* WTF?
        cmesh4::normalize_mesh(mesh);
        MeshBVH mesh_bvh;
        mesh_bvh.init(mesh);
        */

        std::vector<SdfSVSNode> octree_nodes_6;
        std::vector<SdfSVSNode> octree_nodes_7;
        std::vector<SdfSVSNode> octree_nodes_8;
        const unsigned level_6_nodes = 11215;

        {
            testing::ScopedTimer timer("creating SVS from octree with depth-6, trimmed to " + std::to_string(level_6_nodes));
            SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 6);
            std::vector<SdfFrameOctreeNode> nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
            sdf_converter::frame_octree_limit_nodes(nodes, level_6_nodes, true);
            sdf_converter::frame_octree_to_SVS_rec(nodes, octree_nodes_6, 0, uint3(0, 0, 0), 1);
        }

        {
            testing::ScopedTimer timer("creating SVS from octree with depth-7, trimmed to " + std::to_string(level_6_nodes));
            SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 7);
            std::vector<SdfFrameOctreeNode> nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
            sdf_converter::frame_octree_limit_nodes(nodes, level_6_nodes, true);
            sdf_converter::frame_octree_to_SVS_rec(nodes, octree_nodes_7, 0, uint3(0, 0, 0), 1);
        }

        {
            testing::ScopedTimer timer("creating SVS from octree with depth-8, trimmed to " + std::to_string(level_6_nodes));
            SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 8);
            std::vector<SdfFrameOctreeNode> nodes = sdf_converter::create_sdf_frame_octree(settings, mesh);
            sdf_converter::frame_octree_limit_nodes(nodes, level_6_nodes, true);
            sdf_converter::frame_octree_to_SVS_rec(nodes, octree_nodes_8, 0, uint3(0, 0, 0), 1);
        }

        MultiRenderPreset preset = getDefaultPreset();
        preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;

        auto image_6 = testing::create_image();
        auto image_7 = testing::create_image();
        auto image_8 = testing::create_image();

        testing::render_scene(image_6, DEVICE_GPU, preset, octree_nodes_6);
        testing::save_image(image_6, "6");

        testing::render_scene(image_7, DEVICE_GPU, preset, octree_nodes_7);
        testing::save_image(image_7, "trimmed_7");

        testing::render_scene(image_8, DEVICE_GPU, preset, octree_nodes_8);
        testing::save_image(image_8, "trimmed_8");

        testing::check_psnr(image_6, image_7, "SVS octree-6", "SVS octree-7", 45);
        testing::check_psnr(image_6, image_8, "SVS octree-6", "SVS octree-8", 45);
    }

    // former 17
    ADD_TEST(AllTypesSanityCheck, "Testing all")
    {
        auto mesh = testing::load_vsgf_mesh(TEAPOT_MESH, 0.999);
        // cmesh4::normalize_mesh(mesh);

        // constexpr size_t WIDTH = 512;
        // constexpr size_t HEIGHT = 512;

        MultiRenderPreset preset = getDefaultPreset();
        preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;

        testing::ScopedTimer timer("crearing SDF grid from mesh");
        auto grid = sdf_converter::create_sdf_grid(GridSettings(64), mesh);
        timer.end();

        timer = testing::ScopedTimer("creating frame octree from mesh");
        auto octree = sdf_converter::create_sdf_frame_octree(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64 * 64 * 64), mesh);
        timer.end();

        timer = testing::ScopedTimer("creating SVS from mesh");
        auto svs = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64 * 64 * 64), mesh);
        timer.end();

        SdfSBSHeader header;
        header.brick_size = 2;
        header.brick_pad = 0;
        header.bytes_per_value = 1;
        header.aux_data = SDF_SBS_NODE_LAYOUT_DX;
        timer = testing::ScopedTimer("creating SDF SBS from mesh");
        auto sbs = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 8, 64 * 64 * 64), header, mesh);
        timer.end();

        std::vector<unsigned> modes = {DEVICE_CPU, DEVICE_GPU, DEVICE_GPU_RTX};
        std::vector<std::string> render_device_names = {"cpu", "gpu", "rtx"};

        std::vector image_mesh(3, testing::create_image());
        std::vector image_grid(3, testing::create_image());
        std::vector image_frame_octree(3, testing::create_image());
        std::vector image_svs(3, testing::create_image());
        std::vector image_sbs(3, testing::create_image());

        for (int i = 0; i < 3; i++)
        {
            testing::render_scene(image_mesh[i], modes[i], preset, mesh);
            testing::save_image(image_mesh[i], "mesh_" + render_device_names[i]);

            testing::render_scene(image_grid[i], modes[i], preset, grid);
            testing::save_image(image_grid[i], "grid_" + render_device_names[i]);

            testing::render_scene(image_frame_octree[i], modes[i], preset, octree);
            testing::save_image(image_frame_octree[i], "octree_" + render_device_names[i]);

            testing::render_scene(image_svs[i], modes[i], preset, svs);
            testing::save_image(image_svs[i], "SVS_" + render_device_names[i]);

            testing::render_scene(image_sbs[i], modes[i], preset, sbs);
            testing::save_image(image_sbs[i], "SBS_" + render_device_names[i]);
        }

        testing::check_psnr(image_mesh[0], image_mesh[1], "Mesh CPU", "Mesh GPU", 50);
        testing::check_psnr(image_mesh[0], image_mesh[2], "Mesh CPU", "Mesh RTX", 50);

        std::vector<std::string> method_names = {"Grid", "Framed Octree", "SVS", "SBS"};
        std::vector images = {&image_grid, &image_frame_octree, &image_svs, &image_sbs};
        std::vector mesh_threshold = {25, 30, 30, 30};
        std::vector self_threshold = {50, 50, 50, 50};

        for (int i = 0; i < images.size(); i++)
        {
            testing::check_psnr(image_mesh[0], (*images[i])[0], "Mesh CPU", method_names[i] + " CPU", mesh_threshold[i]);
            testing::check_psnr(image_mesh[0], (*images[i])[1], "Mesh CPU", method_names[i] + " GPU", mesh_threshold[i]);
            testing::check_psnr(image_mesh[0], (*images[i])[2], "Mesh CPU", method_names[i] + " RTX", mesh_threshold[i]);
            testing::check_psnr((*images[i])[0], (*images[i])[1], method_names[i] + " CPU", method_names[i] + " GPU", self_threshold[i]);
            testing::check_psnr((*images[i])[0], (*images[i])[2], method_names[i] + " CPU", method_names[i] + " RTX", self_threshold[i]);
        }
    }

    // former 18
    ADD_TEST(MeshNormalization, "Normalizing mesh")
    {

        MultiRenderPreset preset = getDefaultPreset();
        auto mesh = testing::load_vsgf_mesh(TEAPOT_MESH, 0.999);

        std::vector<std::string> mod_names = {"filled", "compressed", "fixed_normals", "normalized"};
        std::vector<std::function<void(testing::Mesh &)>> mods = {};

        mods.push_back([](testing::Mesh &mesh)
                       {
            testing::ScopedTimer timer("creating watertight mesh");
            int ind = -1;
            bool fl = false;
            auto mesh_filled = cmesh4::check_watertight_mesh(mesh, true) ? mesh : cmesh4::removing_holes(mesh, ind, fl);
            // WTF?
            // mesh_filled = mesh; // why?!
            mesh = std::move(mesh_filled); });
        mods.push_back([](testing::Mesh &mesh)
                       {
                           testing::ScopedTimer timer("creating compressed mesh");

                           cmesh4::compress_close_vertices(mesh, 1e-9f, true); });
        mods.push_back([](testing::Mesh &mesh)
                       {
                           testing::ScopedTimer timer("creating mesh with fixed normals");

                           cmesh4::fix_normals(mesh, true); });

        std::vector<testing::Mesh> mod_meshes(mods.size());

        auto ref = testing::create_image();
        auto ref_svs = testing::create_image();

        auto image = testing::create_image();

        testing::render_scene(ref, DEVICE_GPU, preset, mesh, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
        testing::save_image(ref, "ref");

        testing::ScopedTimer timer("creating SDF SVS for Mesh");
        auto svs = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9), mesh);
        timer.end();

        testing::render_scene(ref_svs, DEVICE_GPU, preset, svs, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
        testing::save_image(ref_svs, "ref_svs");

        for (size_t i = 0; i < mods.size(); i++)
        {
            /*
                WTF?
                Should be applied to original mesh or chained?
            */
            // doing chained as in original code
            mod_meshes[i] = (i == 0 ? mesh : mod_meshes[i - 1]); // copy
            mods[i](mod_meshes[i]);                              // apply modification
            testing::log(testing::bar_info)
                << "Triangles in " << mod_names[i] << " mesh: " << mod_meshes[i].TrianglesNum() << std::endl;
            // WTF?
            // may be make default camera?
            testing::render_scene(image, DEVICE_GPU, preset, mod_meshes[i], float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
            testing::save_image(image, mod_names[i]);
            testing::check_psnr(ref, image, "Mesh", mod_names[i] + " Mesh", 45);

            testing::ScopedTimer timer("creating SDF SVS for " + mod_names[i] + " Mesh");
            auto svs = sdf_converter::create_sdf_SVS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 9), mod_meshes[i]);
            timer.end();

            testing::render_scene(image, DEVICE_GPU, preset, svs, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
            testing::save_image(image, mod_names[i] + "_svs");
            testing::check_psnr(ref_svs, image, "SVS", mod_names[i] + " SVS", 45);
        }
    }

    // former 25
    ADD_TEST(FloatImages, "Testing if float images are working")
    {
        auto scene = TEAPOT_HYDRA;

        auto cpu = testing::create_image<float4>();
        auto gpu = testing::create_image<float4>();

        MultiRenderPreset preset = getDefaultPreset();

        testing::render_hydra_scene(cpu, DEVICE_CPU, preset, scene);
        testing::save_image(cpu, "cpu");

        testing::render_hydra_scene(gpu, DEVICE_GPU, preset, scene);
        testing::save_image(gpu, "gpu");

        testing::check_psnr(cpu, gpu, "CPU", "GPU", 45);
    }

    // former 32
    ADD_TEST(Smooth_SBS_normals, "Testing smooth SBS normals")
    {

        auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.95);

        MultiRenderPreset preset = getDefaultPreset();
        preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
        SparseOctreeSettings settings(SparseOctreeBuildType::MESH_TLO, 5);

        SdfSBSHeader header;
        header.brick_size = 4;
        header.brick_pad = 0;
        header.bytes_per_value = 1;

        auto image_mesh = testing::create_image();
        auto image_sbs = testing::create_image();
        auto image_smooth = testing::create_image();

        preset.normal_mode = NORMAL_MODE_VERTEX;
        testing::render_scene(image_mesh, DEVICE_GPU, preset, mesh);
        testing::save_image(image_mesh, "mesh");

        {
            auto renderer = CreateMultiRenderer(DEVICE_GPU);

            testing::ScopedTimer timer("creating SDF SBS indexed with neighbours");
            auto indexed_SBS = sdf_converter::create_sdf_SBS_indexed_with_neighbors(
                settings,
                header,
                mesh,
                0,
                renderer->getMaterials(),
                renderer->getTextures());
            timer.end();

            /*
                Can not SetScene twice
            */
            renderer->SetScene(indexed_SBS);

            preset.normal_mode = NORMAL_MODE_GEOMETRY;
            testing::render(image_sbs, renderer, preset);
            testing::save_image(image_sbs, "sbs");

            preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;
            testing::render(image_smooth, renderer, preset);
            testing::save_image(image_smooth, "smooth_normals");
        }

        
        float psnr1 = testing::check_psnr(image_mesh, image_sbs, "Mesh", "SBS", 30);
        float psnr2 = testing::check_psnr(image_mesh, image_smooth, "Mesh", "Smoothed normals", 30);
        testing::check_greater(psnr2, psnr1, "PSNR SBS with smoothed normals", "PSNR default SBS", true);
    }

    ADD_TEST(BallTreeNN, "Testing Ball Tree nearest neighbors search")
    {
      srand(time(NULL));
      int points_count = 100000;
      constexpr int DIM = 8;
      scom::Dataset dataset;
      dataset.data_points.resize(points_count);
      dataset.all_points.resize(points_count*DIM);

      for (int i = 0; i < points_count; ++i) 
      {
        dataset.data_points[i].data_offset = i*DIM;
        dataset.data_points[i].original_id = i;
        dataset.data_points[i].rotation_id = 0;
        for (int j = 0; j < DIM; ++j)
          dataset.all_points[i*DIM + j] = 2*(float) rand() / (float) RAND_MAX + 1;
      }

      auto t1 = std::chrono::high_resolution_clock::now();
      scom::BallTree tree;
      tree.build(dataset, 8);
      auto t2 = std::chrono::high_resolution_clock::now();
      printf("build took %d us\n", (int)std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count());

      scom::LinearSearchAS naive;
      naive.build(dataset, 8);
      
      double search_time_naive = 0;
      double search_time_tree = 0;

      int tries = 1000;
      int found_count = 0;

      int error_count = 0;
      for (int i = 0; i < tries; ++i) 
      {
        float point[DIM];
        for (int j = 0; j < DIM; ++j)
          point[j] = 2*(float) rand() / (float) RAND_MAX + 1;

        float limit = 0.1f*DIM*((float)rand() / (float) RAND_MAX);
        //limit = 10;

      auto t1 = std::chrono::high_resolution_clock::now();
     
        float min_distance_naive = 1000;
        const float *min_point_naive = naive.get_closest_point(point, limit, &min_distance_naive);

      auto t2 = std::chrono::high_resolution_clock::now();

        float min_distance = 1000;
        //const float *min_point = tree.find_nearest_neighbor(point, limit, &min_distance);
        const float *min_point = tree.get_closest_point(point, limit, &min_distance);

      auto t3 = std::chrono::high_resolution_clock::now();

        search_time_naive += std::chrono::duration_cast<std::chrono::microseconds>(t2-t1).count();
        search_time_tree += std::chrono::duration_cast<std::chrono::microseconds>(t3-t2).count();

        bool found_naive = min_point_naive != nullptr;
        bool found_tree = min_point != nullptr;

        if (found_naive != found_tree)
        {
          error_count++;
          printf("point %f %f %f, ", point[0], point[1], point[2]);
          if (min_point_naive)
            printf("naive %f %f %f, ", min_point_naive[0], min_point_naive[1], min_point_naive[2]);
          if (min_point)
            printf("tree %f %f %f", min_point[0], min_point[1], min_point[2]);
          printf("\n");
        }
        else if (found_naive && found_tree && std::abs(min_distance - min_distance_naive) > 1e-6f)
        {
          error_count++;
          printf("point %f %f %f, ", point[0], point[1], point[2]);
          printf("naive %f %f %f, ", min_point_naive[0], min_point_naive[1], min_point_naive[2]);
          printf("tree %f %f %f, ", min_point[0], min_point[1], min_point[2]);
          printf("dist %f %f\n", min_distance, min_distance_naive);
        }
        else
        {
          found_count += (found_naive && found_tree);
        }
      }

      search_time_naive /= tries;
      search_time_tree /= tries;

      //printf("Search time naive = %.2f us\n", search_time_naive);
      //printf("Search time tree = %.2f us\n", search_time_tree);

      int scan_errors = 0;
      for (int i = 0; i < tries; ++i) 
      {
        float point[DIM];
        for (int j = 0; j < DIM; ++j)
          point[j] = 2*(float) rand() / (float) RAND_MAX + 1;

        float limit = 0.1f*DIM*((float)rand() / (float) RAND_MAX);

        std::vector<bool> points_found_naive(points_count, false);
        std::vector<bool> points_found_tree(points_count, false);

        naive.scan_near(point, limit, [&points_found_naive](float dist, unsigned idx, const scom::DataPoint &point, const float *data) 
        {
          points_found_naive[idx] = true;
        });

        tree.scan_near(point, limit, [&points_found_tree](float dist, unsigned idx, const scom::DataPoint &point, const float *data) 
        {
          points_found_tree[idx] = true;
        });

        for (int j = 0; j < points_count; ++j)
        {
          if (points_found_naive[j] != points_found_tree[j])
          {
            scan_errors++;
            printf("point %f %f %f, ", point[0], point[1], point[2]);
            printf("found %d %d\n",(int)points_found_naive[j], (int)points_found_tree[j]);
          }
        }
      }

      testing::check_greater(search_time_naive, search_time_tree, "Linear search time (us)", "Ball tree search time (us)");
      testing::check_equal(error_count, 0, "Search errors", "0");
      testing::check_equal(scan_errors, 0, "Scan errors", "0");
    }
}