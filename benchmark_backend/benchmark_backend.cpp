#include "benchmark_backend.hpp"

namespace BenchmarkBackend
{
    void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender, 
            float3 pos, float3 target, float3 up, 
            MultiRenderPreset preset, int a_passNum)
    {
        float fov_degrees = 60;
        float z_near = 0.1f;
        float z_far = 100.0f;
        float aspect   = 1.0f;
        auto proj      = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
        auto worldView = LiteMath::lookAt(pos, target, up);

        pRender->Render(image.data(), image.width(), image.height(), worldView, proj, preset, a_passNum);
    }

    void
    getMetrics(const char** argv)
    {
        if (std::string(argv[5]) == "MESH")
        {
            BenchmarkBackend::getInfoMesh(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
        }
        else if (std::string(argv[5]) == "SDF_GRID")
        {
            BenchmarkBackend::getInfoGrid(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
        }
        else if (std::string(argv[5]) == "SDF_SVS")
        {
            BenchmarkBackend::getInfoSVS(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
        }
        else if (std::string(argv[5]) == "SDF_SBS")
        {
            BenchmarkBackend::getInfoSBS(argv[2], argv[3], argv[4], argv[5], argv[6], atoi(argv[7]), atoi(argv[8]), atoi(argv[9]), atoi(argv[10]));
        }
    }

    void 
    getInfoMesh(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
    {
        std::fstream f;
        f.open("benchmark/results/results.csv", std::ios::app);

        auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
        cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

        MultiRenderPreset preset = createPreset(renderer, spp);
        
        auto pRender = CreateMultiRenderer(getDevice(backend));
        pRender->SetPreset(preset);

        float min_time = 1e4, max_time = -1, average_time = 0;
        float memory = 0;
        float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
        float min_flip = 1000, max_flip = -1, average_flip = 0;

        memory = sizeof(float) * (float)(mesh.IndicesNum() + mesh.VerticesNum()) / 1024.f / 1024.f;
        pRender->SetScene(mesh);

        for (int camera = 0; camera < cameras; camera++)
        {
            const float dist = 2;
            float angle = 2.0f * LiteMath::M_PI * camera / (float)cameras;
            const float3 pos = dist*float3(sin(angle), 0, cos(angle));

            LiteImage::Image2D<uint32_t> image(width, height);

            auto t1 = std::chrono::steady_clock::now();
            render(image, pRender, pos, float3(0,0,0), float3(0,1,0), preset, 1);
            auto t2 = std::chrono::steady_clock::now();

            //  Time calculation
            float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
            calcMetrics(min_time, max_time, average_time, t);

            std::string img_name = "benchmark/saves/" + type + "_" + std::to_string(camera) + ".bmp";
            LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);

            //  calculate metrics
            calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, image));
            calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, image));
        }

        average_time /= (float)cameras;
        average_psnr /= (float)cameras;
        average_flip /= (float)cameras;

        f << model << ", " << backend << ", " << renderer << ", " << type << ", " << lod << ", " << memory << ", " << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
        f.close();
    }

        // for (const auto &lod : config.lods)
        // {
        //     auto mesh = cmesh4::LoadMeshFromVSGF(model.c_str());
        //     cmesh4::rescale_mesh(mesh, float3(-0.95, -0.95, -0.95), float3(0.95, 0.95, 0.95));

        //     MultiRenderPreset preset = createPreset(renderer, config.spp);
            
        //     auto pRender = CreateMultiRenderer(getDevice(backend));
        //     pRender->SetPreset(preset);

        //     float min_time = 1e4, max_time = -1, average_time = 0;
        //     float memory = 0;
        //     float min_psnr = 1000, max_psnr = -1, average_psnr = 0;
        //     float min_flip = 1000, max_flip = -1, average_flip = 0;

        //     if (lod == "low")
        //     {

        //     }
        //     else if (lod == "mid")
        //     {

        //     }
        //     else if (lod == "high")
        //     {
        //     //  Load model into chosen structure
        //     if (repr_type == "MESH")
        //     {
        //         memory = sizeof(float) * (float)(mesh.IndicesNum() + mesh.VerticesNum()) / 1024.f / 1024.f;
        //         pRender->SetScene(mesh);
        //     }
        //     else if (repr_type == "SDF_GRID")
        //     {
        //         auto grid = sdf_converter::create_sdf_grid(GridSettings(64), mesh);
        //         memory = sizeof(float) * (float)grid.data.size() / 1024.f / 1024.f;

        //         pRender->SetScene(grid);
        //     }
        //     else if (repr_type == "SDF_SVS")
        //     {
        //         SparseOctreeSettings settings(SparseOctreeBuildType::DEFAULT, 7);
        //         std::vector<SdfSVSNode> frame_nodes = sdf_converter::create_sdf_SVS(settings, mesh);
        //         memory = sizeof(SdfSVSNode) * frame_nodes.size() / 1024.f / 1024.f;

        //         pRender->SetScene(frame_nodes);
        //     }
        //     else if (repr_type == "SDF_SBS")
        //     {
        //         SdfSBSHeader header;
        //         header.brick_size = 3;
        //         header.brick_pad = 0;
        //         header.bytes_per_value = 1;
        //         header.aux_data = SDF_SBS_NODE_LAYOUT_DX;

        //         auto sbs = sdf_converter::create_sdf_SBS(SparseOctreeSettings(SparseOctreeBuildType::DEFAULT, 5), header, mesh);

        //         pRender->SetScene(sbs);
        //     }
        //     else if (repr_type == "SDF_SBS_ADAPT")
        //     {
        //         std::vector<MeshBVH> bvh(1);
        //         for (unsigned i = 0; i < 1; i++)
        //         bvh[i].init(mesh);
        //         auto real_sdf = [&](const float3 &p, unsigned idx) -> float 
        //         { return bvh[idx].get_signed_distance(p);};
        //         //  Неработающая ..... 
        //         SdfSBSAdapt sbs_adapt = sdf_converter::greed_sbs_adapt(real_sdf, 5);

        //         pRender->SetScene(sbs_adapt);
        //     }
        //     }

        //     for (int camera = 0; camera < config.cameras; camera++)
        //     {
        //     const float dist = 2;
        //     float angle = 2.0f * LiteMath::M_PI * camera / (float)config.cameras;
        //     const float3 pos = dist*float3(sin(angle), 0, cos(angle));

        //     LiteImage::Image2D<uint32_t> image(config.width, config.height);

        //     auto t1 = std::chrono::steady_clock::now();
        //     render(image, pRender, pos, float3(0,0,0), float3(0,1,0), preset, 1);
        //     auto t2 = std::chrono::steady_clock::now();

        //     //  Time calculation
        //     float t = std::chrono::duration_cast<std::chrono::milliseconds>(t2 - t1).count() / 1000.f;
        //     calcMetrics(min_time, max_time, average_time, t);
            
        //     if (repr_type == "MESH")
        //     {
        //         ref_images.push_back(image);
        //     }

        //     std::string img_name = "benchmark/saves/" + repr_type + "_" + std::to_string(camera) + ".bmp";
        //     LiteImage::SaveImage<uint32_t>(img_name.c_str(), image);

        //     //  calculate metrics
        //     calcMetrics(min_psnr, max_psnr, average_psnr, image_metrics::PSNR(image, ref_images[camera]));
        //     calcMetrics(min_flip, max_flip, average_flip, image_metrics::FLIP(image, ref_images[camera]));
        //     }

        //     average_time /= (float)config.cameras;
        //     average_psnr /= (float)config.cameras;
        //     average_flip /= (float)config.cameras;

        //     f << model << ", " << backend << ", " << renderer << ", " << repr_type << ", " << lod << ", " << memory << ", " << min_time << ", " << max_time << ", " << average_time << ", " << min_psnr << ", " << max_psnr << ", " << average_psnr << ", " << min_flip << ", " << max_flip << ", " << average_flip << std::endl;
        // }

    void 
    getInfoGrid(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
    {
        
    }

    void 
    getInfoSVS(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
    {

    }

    void
    getInfoSBS(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
    {

    }

    //  very slooooow (huin ia)
    void
    getInfoAdaptSBS(const std::string &model, const std::string &backend, const std::string &renderer, 
        const std::string &type, const std::string &lod, const int width, const int height, const int spp, const int cameras)
    {

    }

    MultiRenderPreset 
    createPreset(const std::string& render_mode, const int spp)
    {
        MultiRenderPreset preset = getDefaultPreset();

        if (render_mode == "LAMBERT")
        {
            preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
        }

        preset.spp = spp;

        return preset;
    }

    int 
    getDevice(const std::string backend)
    {
        if (backend == "CPU")
        {
            return DEVICE_CPU;
        }
        else if (backend == "GPU")
        {
            return DEVICE_GPU;
        }
        else if (backend == "RTX")
        {
            return DEVICE_GPU_RTX;
        }

        //  if GPU_RQ -> skip
        return -1;
    }

    void 
    calcMetrics(float& min, float& max, float& average, const float& new_val)
    {
        if (new_val < min)
        {
            min = new_val;
        }
        if (new_val > max)
        {
            max = new_val;
        }

        average += new_val;
    }
};