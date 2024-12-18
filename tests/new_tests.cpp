#include <testing_framework/core/environment.h>
#include <testing_framework/core/add_test.h>

#include <testing_framework/helpers/render.h>
#include <testing_framework/helpers/scoped_timer.h>
#include <testing_framework/helpers/check.h>
#include <testing_framework/helpers/mesh.h>
/*
    Includes for code
*/
#include "IRenderer.h"
#include "Renderer/eye_ray.h"
#include "utils/mesh/mesh_bvh.h"
#include "utils/mesh/mesh.h"
#include "LiteScene/hydraxml.h"
#include "LiteMath/Image2d.h"
#include "utils/sdf/sdf_converter.h"
#include "utils/sdf/sparse_octree_builder.h"
#include "utils/mesh/marching_cubes.h"
#include "utils/sdf/sdf_smoother.h"
#include "utils/mesh/demo_meshes.h"
#include "utils/image_metrics.h"
#include "diff_render/MultiRendererDR.h"
#include "utils/sdf/iou.h"
#include "nurbs/nurbs_common_host.h"
#include "utils/coctree/ball_tree.h"
#include "hydra_integration.h"
#include "utils/scene.h"
#include "utils/gltf_utils/gltf_writer.h"

namespace litert_tests
{
  const char TEAPOT_MESH[] = "01_simple_scenes/data/teapot.vsgf";
  const char BUNNY_MESH[] = "01_simple_scenes/data/bunny.vsgf";
  const char SPHERE_MESH[] = "01_simple_scenes/data/sphere.vsgf";

  const char TEAPOT_HYDRA[] = "01_simple_scenes/teapot.xml";

  // former 3
  ADD_TEST(SVS_and_SBS_verification, "Testing if SVS and SBS are correct")
  {

    constexpr size_t OCTREE_DEPTH = 8;

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_LINEAR_DEPTH;
    preset.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;

    auto mesh = testing::load_vsgf_mesh(TEAPOT_MESH);

    SparseOctreeSettings settings(OCTREE_DEPTH);

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
          SparseOctreeSettings(9),
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
          SparseOctreeSettings(9),
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
        SparseOctreeSettings(7),
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

    testing::saved_reference_check_psnr(cpu_image, "CPU", "cpu", 75);
    testing::saved_reference_check_psnr(gpu_image, "GPU", "gpu", 75);
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
    auto octree = sdf_converter::create_sdf_frame_octree(SparseOctreeSettings(8), mesh);
    timer.end();

    timer = testing::ScopedTimer("creating SVS from mesh");
    auto svs = sdf_converter::create_sdf_SVS(SparseOctreeSettings(8), mesh);
    timer.end();

    SdfSBSHeader header;
    header.brick_size = 2;
    header.brick_pad = 0;
    header.bytes_per_value = 1;
    header.aux_data = SDF_SBS_NODE_LAYOUT_DX;
    timer = testing::ScopedTimer("creating SDF SBS from mesh");
    auto sbs = sdf_converter::create_sdf_SBS(SparseOctreeSettings(8), header, mesh);
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
    auto svs = sdf_converter::create_sdf_SVS(SparseOctreeSettings(9), mesh);
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
      auto svs = sdf_converter::create_sdf_SVS(SparseOctreeSettings(9), mod_meshes[i]);
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
    SparseOctreeSettings settings(5);

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
    dataset.all_points.resize(points_count * DIM);

    for (int i = 0; i < points_count; ++i)
    {
      dataset.data_points[i].data_offset = i * DIM;
      dataset.data_points[i].original_id = i;
      dataset.data_points[i].rotation_id = 0;
      for (int j = 0; j < DIM; ++j)
        dataset.all_points[i * DIM + j] = 2 * (float)rand() / (float)RAND_MAX + 1;
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    scom::BallTree tree;
    tree.build(dataset, 8);
    auto t2 = std::chrono::high_resolution_clock::now();
    printf("build took %d us\n", (int)std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count());

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
        point[j] = 2 * (float)rand() / (float)RAND_MAX + 1;

      float limit = 0.1f * DIM * ((float)rand() / (float)RAND_MAX);
      // limit = 10;

      auto t1 = std::chrono::high_resolution_clock::now();

      float min_distance_naive = 1000;
      const float *min_point_naive = naive.get_closest_point(point, limit, &min_distance_naive);

      auto t2 = std::chrono::high_resolution_clock::now();

      float min_distance = 1000;
      // const float *min_point = tree.find_nearest_neighbor(point, limit, &min_distance);
      const float *min_point = tree.get_closest_point(point, limit, &min_distance);

      auto t3 = std::chrono::high_resolution_clock::now();

      search_time_naive += std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
      search_time_tree += std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();

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

    // printf("Search time naive = %.2f us\n", search_time_naive);
    // printf("Search time tree = %.2f us\n", search_time_tree);

    int scan_errors = 0;
    for (int i = 0; i < tries; ++i)
    {
      float point[DIM];
      for (int j = 0; j < DIM; ++j)
        point[j] = 2 * (float)rand() / (float)RAND_MAX + 1;

      float limit = 0.1f * DIM * ((float)rand() / (float)RAND_MAX);

      std::vector<bool> points_found_naive(points_count, false);
      std::vector<bool> points_found_tree(points_count, false);

      naive.scan_near(point, limit, [&points_found_naive](float dist, unsigned idx, const scom::DataPoint &point, const float *data)
                      { points_found_naive[idx] = true; });

      tree.scan_near(point, limit, [&points_found_tree](float dist, unsigned idx, const scom::DataPoint &point, const float *data)
                     { points_found_tree[idx] = true; });

      for (int j = 0; j < points_count; ++j)
      {
        if (points_found_naive[j] != points_found_tree[j])
        {
          scan_errors++;
          printf("point %f %f %f, ", point[0], point[1], point[2]);
          printf("found %d %d\n", (int)points_found_naive[j], (int)points_found_tree[j]);
        }
      }
    }

    testing::check_greater(search_time_naive, search_time_tree, "Linear search time (us)", "Ball tree search time (us)");
    testing::check_equal(error_count, 0, "Search errors", "0");
    testing::check_equal(scan_errors, 0, "Scan errors", "0");
  }

  ADD_TEST(PlyReader, "Testing reader for .ply files")
  {
    MultiRenderPreset preset = getDefaultPreset();
    preset.normal_mode = NORMAL_MODE_GEOMETRY;
    auto image = testing::create_image();

    std::string path = "./scenes/01_simple_scenes/test_1.ply";
    testing::assert_file_existance(path, true, testing::source_location::current());

    auto mesh = cmesh4::LoadMesh(path.c_str(), true, true);

    testing::render_scene(image, DEVICE_CPU, preset, mesh, float3(3, 0, 3));
    testing::save_image(image, "res");

    testing::saved_reference_check_psnr(image, "res", "res", 75);
  }

  ADD_TEST(InstancesRender, "Render simple scene with instances from Hydra file")
  {
    std::string scene = "01_simple_scenes/instanced_objects.xml";

    MultiRenderPreset preset = getDefaultPreset();
    auto cpu_image = testing::create_image();
    auto gpu_image = testing::create_image();

    testing::render_hydra_scene(cpu_image, DEVICE_CPU, preset, scene);
    testing::save_image(cpu_image, "cpu");

    testing::render_hydra_scene(gpu_image, DEVICE_CPU, preset, scene);
    testing::save_image(gpu_image, "gpu");

    testing::check_psnr(cpu_image, gpu_image, "cpu", "gpu", 50);

    testing::saved_reference_check_psnr(cpu_image, "CPU", "cpu", 50);
    testing::saved_reference_check_psnr(gpu_image, "GPU", "gpu", 50);
  }

  ADD_TEST(GlobalOctreeCreator, "Creates Global Octree using mesh and sdf")
  {
    MultiRenderPreset preset = getDefaultPreset();
    auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.999);

    constexpr size_t OCTREE_DEPTH = 7;

    int max_threads = 6;

    std::vector<MeshBVH> bvh(max_threads);
    for (unsigned i = 0; i < max_threads; i++)
      bvh[i].init(mesh);
    auto sdf = [&](const float3 &p, unsigned idx) -> float 
    { return bvh[idx].get_signed_distance(p); };

    sdf_converter::GlobalOctree global_mesh, global_sdf;
    global_mesh.header.brick_pad = 0;
    global_mesh.header.brick_size = 1;

    global_sdf.header = global_mesh.header;


    SparseOctreeSettings settings(OCTREE_DEPTH), 
                         settings2(OCTREE_DEPTH);
    auto tlo = cmesh4::create_triangle_list_octree(mesh, OCTREE_DEPTH, 0, 1.0f);

    
    sdf_converter::mesh_octree_to_global_octree(mesh, tlo, global_mesh, 0.0f, OCTREE_DEPTH, OCTREE_DEPTH, false);
    sdf_converter::sdf_to_global_octree(settings2, sdf, max_threads, global_sdf);

    std::vector<SdfFrameOctreeNode> frame_mesh, frame_sdf;
    sdf_converter::global_octree_to_frame_octree(global_mesh, frame_mesh);
    sdf_converter::global_octree_to_frame_octree(global_sdf, frame_sdf);

    auto img_mesh = testing::create_image();
    auto img_sdf = testing::create_image();

    testing::render_scene(img_mesh, DEVICE_GPU, preset, frame_mesh, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_mesh, "frame_octree_from_mesh");

    testing::render_scene(img_sdf, DEVICE_GPU, preset, frame_sdf, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_sdf, "frame_octree_from_sdf");

    float psnr = testing::check_psnr(img_mesh, img_sdf, "global octree mesh", "global octree sdf", 75);
  }

  ADD_TEST(CompactOctree, "Tests different settings for COctreeV3 rendering")
  {
    const unsigned device = DEVICE_GPU;

    auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.95);

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    preset.spp = 4;
    SparseOctreeSettings settings(6);

    auto img_mesh = testing::create_image();

    testing::render_scene(img_mesh, device, preset, mesh, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_mesh, "ref");

    int default_size = 0;
    {
      auto img = testing::create_image();
      
      COctreeV3Settings co_settings;
      co_settings.brick_size = 3;
      co_settings.brick_pad  = 0;

      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings, co_settings, mesh);
      default_size = coctree.data.size();
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "default");
      float psnr = image_metrics::PSNR(img_mesh, img);
      testing::check_psnr(img_mesh, img, "COctreeV3 (default)", "Mesh", 40); 
    }   

    {
      auto img = testing::create_image();
      
      COctreeV3Settings co_settings;
      co_settings.brick_size = 3;
      co_settings.brick_pad  = 0;
      co_settings.default_leaf_type = COCTREE_LEAF_TYPE_GRID;
      co_settings.allow_fallback_to_unpacked_leaves = false;

      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings, co_settings, mesh);
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "grid");
      float psnr = image_metrics::PSNR(img_mesh, img);
      testing::check_psnr(img_mesh, img, "COctreeV3 (grid)", "Mesh", 40); 
      testing::check_less(default_size, coctree.data.size(), "size (default)", "size (grid)");
    }   

    {
      auto img = testing::create_image();
      
      COctreeV3Settings co_settings;
      co_settings.brick_size = 3;
      co_settings.brick_pad  = 0;
      co_settings.default_leaf_type = COCTREE_LEAF_TYPE_BIT_PACK;
      co_settings.allow_fallback_to_unpacked_leaves = false;

      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings, co_settings, mesh);
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "bitpack");
      float psnr = image_metrics::PSNR(img_mesh, img);
      testing::check_psnr(img_mesh, img, "COctreeV3 (bitpack)", "Mesh", 40); 
      testing::check_less(default_size, coctree.data.size(), "size (default)", "size (bitpack)");
    }   

    {
      auto img = testing::create_image();
      
      COctreeV3Settings co_settings;
      co_settings.brick_size = 3;
      co_settings.brick_pad  = 0;
      co_settings.default_leaf_type = COCTREE_LEAF_TYPE_SLICES;
      co_settings.allow_fallback_to_unpacked_leaves = false;

      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings, co_settings, mesh);
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "slices");
      float psnr = image_metrics::PSNR(img_mesh, img);
      testing::check_psnr(img_mesh, img, "COctreeV3 (slices)", "Mesh", 40); 
      testing::check_less(default_size, coctree.data.size(), "size (default)", "size (slices)");
    }   
  }

  ADD_TEST(SimilarityCompression, "Tests different settings for COctreeV3 similarity compression")
  {
    auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.95);

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    preset.spp = 4;
    SparseOctreeSettings settings(6);

    auto img_mesh = testing::create_image();

    testing::render_scene(img_mesh, DEVICE_GPU, preset, mesh, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_mesh, "ref");

    {
      auto img_orig = testing::create_image();
      auto img_comp = testing::create_image();
      float psnr_orig, psnr_comp;
      COctreeV3 coctree_comp;
      
      COctreeV3Settings header_orig;
      header_orig.brick_size = 2;
      header_orig.brick_pad  = 0;

      COctreeV3 coctree_orig = sdf_converter::create_COctree_v3(settings, header_orig, mesh);
      testing::render_scene(img_orig, DEVICE_GPU, preset, coctree_orig, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img_orig, "orig");
      psnr_orig = image_metrics::PSNR(img_mesh, img_orig);

      COctreeV3Settings header_comp = header_orig;
      header_comp.sim_compression = true;

      scom::Settings scom_settings;
      scom_settings.similarity_threshold = 0.05f;
      scom_settings.search_algorithm = scom::SearchAlgorithm::BALL_TREE;

      //CA::REPLACEMENT
      scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::REPLACEMENT;
      coctree_comp = sdf_converter::create_COctree_v3(settings, header_comp, scom_settings, mesh);
      testing::render_scene(img_comp, DEVICE_GPU, preset, coctree_comp, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img_comp, "comp_replacement");
      psnr_comp = image_metrics::PSNR(img_mesh, img_comp);
      int replacement_size = coctree_comp.data.size();

      testing::check_psnr(img_orig, img_comp, "Compressed (CA::REPLACEMENT)", "Original", 40);
      testing::check_less(psnr_orig - psnr_comp, 1.5, "PSNR loss (CA::REPLACEMENT)", "threshold");

      //CA::COMPONENTS_RECURSIVE_FILL
      scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::COMPONENTS_RECURSIVE_FILL;
      coctree_comp = sdf_converter::create_COctree_v3(settings, header_comp, scom_settings, mesh);
      testing::render_scene(img_comp, DEVICE_GPU, preset, coctree_comp, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img_comp, "comp_recursive_fill");
      psnr_comp = image_metrics::PSNR(img_mesh, img_comp);

      testing::check_psnr(img_orig, img_comp, "Compressed (CA::COMPONENTS_RECURSIVE_FILL)", "Original", 40);
      testing::check_less(psnr_orig - psnr_comp, 1.5, "PSNR loss (CA::COMPONENTS_RECURSIVE_FILL)", "threshold");

      //CA::HIERARCHICAL
      scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::HIERARCHICAL;
      coctree_comp = sdf_converter::create_COctree_v3(settings, header_comp, scom_settings, mesh);
      testing::render_scene(img_comp, DEVICE_GPU, preset, coctree_comp, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img_comp, "comp_hierarchical");
      psnr_comp = image_metrics::PSNR(img_mesh, img_comp);

      testing::check_psnr(img_orig, img_comp, "Compressed (CA::HIERARCHICAL)", "Original", 40);
      testing::check_less(psnr_orig - psnr_comp, 1.5, "PSNR loss (CA::HIERARCHICAL)", "threshold");
    }
  }

  ADD_TEST(COctreeLODs, "Rendering COctrees with level of detail")
  {
    const unsigned device = DEVICE_GPU;

    auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.95);

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    preset.spp = 4;
    SparseOctreeSettings settings_5(5);
    SparseOctreeSettings settings_6(6);

    auto img_mesh = testing::create_image();

    testing::render_scene(img_mesh, device, preset, mesh, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_mesh, "ref");

    auto img_no_lods_5 = testing::create_image();
    auto img_no_lods_6 = testing::create_image();
    
    COctreeV3Settings co_settings;
    co_settings.brick_size = 3;
    co_settings.brick_pad  = 0;
    
    {
      co_settings.use_lods = false;

      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings_6, co_settings, mesh);
      testing::render_scene(img_no_lods_6, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img_no_lods_6, "no_lods_6");
    } 
    {
      co_settings.use_lods = false;

      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings_5, co_settings, mesh);
      testing::render_scene(img_no_lods_5, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img_no_lods_5, "no_lods_5");
    }  
    {
      co_settings.use_lods = true;
      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings_6, co_settings, mesh);
      auto img = testing::create_image();

      preset.fixed_lod = true;
      preset.level_of_detail = 7;
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "lod_6");
      testing::check_psnr(img_no_lods_6, img, "LOD 6 fixed", "Original", 50);

      preset.fixed_lod = true;
      preset.level_of_detail = 6;
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "lod_5");
      testing::check_psnr(img_no_lods_5, img, "LOD 5 fixed", "Original", 50);

      preset.fixed_lod = false;
      preset.level_of_detail = 16;
      testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "lod_dynamic");
      testing::check_psnr(img_no_lods_6, img, "dynamic LOD", "Original", 50);
      testing::check_psnr(img_mesh, img, "LOD 6 fixed", "Mesh", 30);

      //save_coctree_v3(coctree, "saves/coctree.bin");
      //save_scene_xml("saves/coctree.xml", "coctree.bin", get_info_coctree_v3(coctree), 0, DemoScene::SINGLE_OBJECT);
    }   

    {
      co_settings.use_lods = true;
      co_settings.sim_compression = true;

      scom::Settings scom_settings;
      scom_settings.similarity_threshold = 0.1f;
      scom_settings.search_algorithm = scom::SearchAlgorithm::BALL_TREE;
      scom_settings.cluster_non_leafs = true;

      auto coctree_comp = sdf_converter::create_COctree_v3(settings_6, co_settings, scom_settings, mesh);
      auto img = testing::create_image();

      preset.fixed_lod = true;
      preset.level_of_detail = 7;
      testing::render_scene(img, device, preset, coctree_comp, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "lod_6_scom");
      testing::check_psnr(img_no_lods_6, img, "LOD 6 fixed + scom", "Original", 35);
      
      preset.fixed_lod = true;
      preset.level_of_detail = 6;
      testing::render_scene(img, device, preset, coctree_comp, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
      testing::save_image(img, "lod_5_scom");
      testing::check_psnr(img_no_lods_5, img, "LOD 5 fixed + scom", "Original", 35);

      //save_coctree_v3(coctree_comp, "saves/coctree_comp.bin");
      //save_scene_xml("saves/coctree_comp.xml", "coctree_comp.bin", get_info_coctree_v3(coctree_comp), 0, DemoScene::SINGLE_OBJECT);
    }
  }

  ADD_TEST(PrecisedOctreeCreation, "Compare precised framed octree and standart framed octree")
  {
    MultiRenderPreset preset = getDefaultPreset();
    auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.95);
    constexpr size_t OCTREE_DEPTH = 7;

    sdf_converter::GlobalOctree global_framed, global_prec;
    global_framed.header.brick_pad = 0;
    global_framed.header.brick_size = 1;

    global_prec.header = global_framed.header;


    SparseOctreeSettings settings(OCTREE_DEPTH), 
                         settings2(OCTREE_DEPTH);
    auto tlo = cmesh4::create_triangle_list_octree(mesh, OCTREE_DEPTH, 0, 1.0f);

    
    sdf_converter::mesh_octree_to_global_octree(mesh, tlo, global_framed, 0.0f, OCTREE_DEPTH, OCTREE_DEPTH, false);
    sdf_converter::mesh_octree_to_global_octree(mesh, tlo, global_prec, 0.001, 2, 7, false);

    int nc_1 = global_framed.nodes.size();
    int nc_2 = global_prec.nodes.size();
    testing::check_less(nc_2, nc_1, "Node Count (with Early Stop)", "Node Count (Default)");

    std::vector<SdfFrameOctreeNode> frame, frame_prec;
    sdf_converter::global_octree_to_frame_octree(global_framed, frame);
    sdf_converter::global_octree_to_frame_octree(global_prec, frame_prec);

    auto img_mesh = testing::create_image();
    auto img_prec = testing::create_image();

    testing::render_scene(img_mesh, DEVICE_GPU, preset, frame, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_mesh, "frame_octree");

    testing::render_scene(img_prec, DEVICE_GPU, preset, frame_prec, float3(2, 0, 2), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_prec, "precised_frame_octree");

    float psnr = testing::check_psnr(img_mesh, img_prec, "framed octree", "precised framed octree", 35);
  }

  ADD_TEST(SaveToObj, "Save mesh to obj")
  {
    MultiRenderPreset preset = getDefaultPreset();
    preset.normal_mode = NORMAL_MODE_GEOMETRY;

    auto mesh_1 = cmesh4::LoadMesh("scenes/01_simple_scenes/data/bunny.vsgf");

    auto img_1 = testing::create_image();
    auto img_2 = testing::create_image();
    testing::render_scene(img_1, DEVICE_GPU, preset, mesh_1);
    testing::save_image(img_1, "ref");

    cmesh4::SaveMeshToObj("saves/SaveToObj/test_mesh.obj", mesh_1);
    auto mesh_2 = cmesh4::LoadMesh("saves/SaveToObj/test_mesh.obj");
    testing::render_scene(img_2, DEVICE_GPU, preset, mesh_2);
    testing::save_image(img_2, "saved");

    testing::check_psnr(img_1, img_2, "Orignal mesh", "Saved and Loaded mesh", 50);
  }

  ADD_TEST(COctreeSmoothNormals, "Testing COctree with Smooth normals")
  {
    const unsigned device = DEVICE_GPU;

    auto mesh = testing::load_vsgf_mesh(BUNNY_MESH, 0.95);

    MultiRenderPreset preset = getDefaultPreset();
    preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
    preset.spp = 4;
    SparseOctreeSettings settings(5);

    auto img_mesh = testing::create_image();
    auto img = testing::create_image();
    auto img_smooth = testing::create_image();

    testing::render_scene(img_mesh, device, preset, mesh, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
    testing::save_image(img_mesh, "ref");

    std::vector<int> brick_sizes = {1, 2, 3};
    std::vector<int> lods = {4, 5, 6, 7};
    for (int brick_size : brick_sizes)
    {
    COctreeV3Settings co_settings;
    co_settings.brick_size = brick_size;
    co_settings.brick_pad  = 1;
    
    {
      co_settings.use_lods = true;
      COctreeV3 coctree = sdf_converter::create_COctree_v3(settings, co_settings, mesh);
      for (int lod : lods)
      {
        preset.fixed_lod = true;
        preset.level_of_detail = lod;

        preset.normal_mode = NORMAL_MODE_GEOMETRY;
        testing::render_scene(img, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
        testing::save_image(img, "base_" + std::to_string(brick_size) + "_" + std::to_string(lod));

        preset.normal_mode = NORMAL_MODE_SDF_SMOOTHED;
        testing::render_scene(img_smooth, device, preset, coctree, float3(0, 0, 3), float3(0, 0, 0), float3(0, 1, 0));
        testing::save_image(img_smooth, "smooth_" + std::to_string(brick_size) + "_" + std::to_string(lod));
        
        float psnr_1 = image_metrics::PSNR(img_mesh, img);
        float psnr_2 = image_metrics::PSNR(img_mesh, img_smooth);
        testing::check_greater(psnr_2, psnr_1, "PSNR with smooth normals", "PSNR with default normals");
      }
    } 
    }
  }

  ADD_TEST(HydraReflections, "HydraReflections")
  {
    int W  = 256;
    int H = 256;
    LiteImage::Image2D<uint32_t> img1(W, H);
    LiteImage::Image2D<uint32_t> img2(W, H);

    HydraRenderPreset preset = getDefaultHydraRenderPreset();
    preset.spp = 1024;

    {
      HydraRenderer renderer(DEVICE_GPU);
      renderer.SetPreset(img1.width(), img1.height(), preset);
      renderer.LoadScene("./refs/HydraReflections/MESH/default.xml");
      renderer.SetViewport(0,0, img1.width(), img1.height());
      renderer.CommitDeviceData();
      renderer.Clear(img1.width(), img1.height(), "color");
      renderer.Render(img1.data(), img1.width(), img1.height(), "color", 1); 
      testing::save_image(img1, "ref");
    }
    {
      HydraRenderer renderer(DEVICE_GPU);
      renderer.SetPreset(img2.width(), img2.height(), preset);
      renderer.LoadScene("./refs/HydraReflections/SDF_FRAME_OCTREE_COMPACT/medium.xml");
      renderer.SetViewport(0,0, img2.width(), img2.height());
      renderer.CommitDeviceData();
      renderer.Clear(img2.width(), img2.height(), "color");
      renderer.Render(img2.data(), img2.width(), img2.height(), "color", 1); 
      testing::save_image(img2, "res");
    }
    testing::check_psnr(img1, img2, "ref", "res", 40);
  }

  ADD_TEST(glTFWiter, "glTFWiter")
  {
    HydraScene scene;
    load_hydra_scene_xml("saves/gltf/input/GLTF_sphere_metal_hydra3.xml", scene);
    cmesh4::set_mat_id(scene.meshes[1].mesh, 4);
    gltf::convert_to_gltf(scene, "saves/gltf/test");
  }
}