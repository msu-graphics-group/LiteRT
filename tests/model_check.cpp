#include "tests.h"
#include "../IRenderer.h"
#include "../Renderer/eye_ray.h"
#include "../utils/mesh_bvh.h"
#include "../utils/mesh.h"
#include "../utils/sdf_converter.h"
#include "../utils/image_metrics.h"
#include "LiteScene/hydraxml.h"
#include "../utils/sparse_octree_builder.h"
#include "../utils/similarity_compression.h"
#include "LiteMath/Image2d.h"

#include <functional>
#include <cassert>
#include <chrono>
#include <filesystem>
#include <map>
#include <algorithm>
namespace cmesh4
{
	std::vector<std::string> explode(std::string aStr, char aDelim);
};
void render(LiteImage::Image2D<uint32_t> &image, std::shared_ptr<MultiRenderer> pRender,
			float3 pos, float3 target, float3 up,
			MultiRenderPreset preset, int a_passNum);

void check_model(const std::string &path)
{
	printf("[check_model::INFO] Checking model %s\n", path.c_str());

	auto mesh = cmesh4::LoadMesh(path.c_str(), true, true);

	if (mesh.indices.size() == 0)
	{
		printf("[check_model::ERROR] Failed to load mesh %s\n", path.c_str());
		return;
	}

	printf("[check_model::INFO] Loaded mesh\n");

	std::string model_name = cmesh4::explode(cmesh4::explode(path, '/').back(), '.')[0];
	printf("[check_model::INFO] Model name: %s\n", model_name.c_str());

	MultiRenderPreset preset = getDefaultPreset();
	preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
	preset.spp = 4;

	unsigned W = 4096, H = 4096;
	LiteImage::Image2D<uint32_t> image_ref(W, H);
	LiteImage::Image2D<uint32_t> image(W, H);

  {
    auto pRender = CreateMultiRenderer(DEVICE_GPU);
    preset.normal_mode = NORMAL_MODE_VERTEX;
    pRender->SetPreset(preset);
    pRender->SetScene(mesh);
    //render(image, pRender, float3(0.4,-0.8,-0.4), float3(0,-0.81,-0.4), float3(0,1,0), preset, 1); //for sponza, pRender, float3(0.4,-0.8,-0.4), float3(0,-0.81,-0.4), float3(0,1,0), preset, 1); //for sponza
    //render(image_ref, pRender, float3(-0.75,-0.75,1.25), float3(-0.75,-0.75,0), float3(0,1,0), preset, 1); //for HMS_Daring_Type_45.obj
    render(image_ref, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset, 1);
    LiteImage::SaveImage<uint32_t>(("saves/check_" + model_name + "_ref.png").c_str(), image_ref); 

    long mesh_total_bytes = 0;
    BVHRT *bvh = dynamic_cast<BVHRT*>(pRender->GetAccelStruct()->UnderlyingImpl(0));
    mesh_total_bytes = bvh->m_allNodePairs.size()*sizeof(BVHNodePair) + 
                       bvh->m_primIdCount.size()* sizeof(uint32_t) +
                       bvh->m_vertPos.size()* sizeof(float4) +
                       bvh->m_vertNorm.size()* sizeof(float4) +
                       bvh->m_indices.size()* sizeof(uint32_t) +
                       bvh->m_primIndices.size()* sizeof(uint32_t);
    printf("mesh %6.1f Mb\n", mesh_total_bytes/(1024.0f*1024.0f));
  }

	printf("[check_model::INFO] Rendered mesh\n");

	constexpr int max_depth = 9;
	std::array<float, max_depth> PSNRs = {0, 0, 0, 0, 0, 0, 0, 0};
	float max_psnr = 0;

  for (int depth = 3; depth < max_depth; depth++)
  {
    printf("[check_model::INFO] Building SDF with depth %d\n", depth);
    SparseOctreeSettings settings = SparseOctreeSettings(SparseOctreeBuildType::MESH_TLO, depth);
    sdf_converter::GlobalOctree g;
    g.header.brick_size = 4;
    g.header.brick_pad = 0;
    
    COctreeV3 coctree;
    coctree.header.bits_per_value = 8;
    coctree.header.brick_size = g.header.brick_size;
    coctree.header.brick_pad = g.header.brick_pad;
    coctree.header.uv_size = 0;
    coctree.header.sim_compression = 1;
    
    scom::Settings scom_settings;
    scom_settings.similarity_threshold = 0.075f;
    scom_settings.search_algorithm = scom::SearchAlgorithm::BALL_TREE;
    scom_settings.clustering_algorithm = scom::ClusteringAlgorithm::REPLACEMENT;

		auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
		printf("[check_model::INFO]   Built TLO\n");
		sdf_converter::mesh_octree_to_global_octree(mesh, tlo, g);
		printf("[check_model::INFO]   Built Global Octree\n");
		sdf_converter::global_octree_to_compact_octree_v3(g, coctree, 8, scom_settings);
		printf("[check_model::INFO]   Built Compact Octree\n");

    auto pRender = CreateMultiRenderer(DEVICE_GPU);
    preset.normal_mode = g.header.brick_pad == 1 ? NORMAL_MODE_SDF_SMOOTHED : NORMAL_MODE_VERTEX;
    pRender->SetPreset(preset);
    pRender->SetScene(coctree, 0);
    //render(image, pRender, float3(0.4,-0.8,-0.4), float3(0,-0.81,-0.4), float3(0,1,0), preset, 1); //for sponza
    //render(image, pRender, float3(-0.75,-0.75,1.25), float3(-0.75,-0.75,0), float3(0,1,0), preset, 1); //for HMS_Daring_Type_45.obj
    render(image, pRender, float3(0,0,3), float3(0,0,0), float3(0,1,0), preset, 1);
    LiteImage::SaveImage<uint32_t>(("saves/check_" + model_name + "_depth_" + std::to_string(depth) + ".png").c_str(), image); 

		float psnr = image_metrics::PSNR(image, image_ref);
		max_psnr = std::max(max_psnr, psnr);
		PSNRs[depth] = psnr;
		printf("[check_model::INFO] Rendered Compact Octree. PSNR = %.2f\n", psnr);
	}

	printf("[check_model::INFO] ==================================================================\n");
	if (max_psnr < 30)
		printf("[check_model::INFO] OUR ALGORITHM FAILED TO CONVERT THIS MODEL. DO NOT USE IT\n");
	else if (max_psnr < 40)
		printf("[check_model::INFO] OUR ALGORITHM CONVERTED WITH MODEL WITH FLAWS. USE IT WITH CAUTION\n");
	else if (max_psnr < 50)
		printf("[check_model::INFO] OUR ALGORITHM SUCCEEDED TO CONVERT THIS MODEL\n");
	else
		printf("[check_model::INFO] PSNR IS TOO HIGH. CHECK THE IMAGES\n");
	printf("[check_model::INFO] ==================================================================\n");
}


namespace model_validator
{

	using image_t = LiteImage::Image2D<uint32_t>;

	void save_image(const image_t &image, const std::string &name, const std::string &dir)
	{
		auto path = dir + "/" + name + ".png";
		std::cout << "Saving image to '" << path << "'" << std::endl;
		LiteImage::SaveImage(path.c_str(), image);
	}

	constexpr size_t WIDTH = 2048;
	constexpr size_t HEIGHT = 2048;

	void render(
		image_t &image,
		MultiRenderer *renderer,
		MultiRenderPreset preset,
		float3 pos = float3(0, 0, 3),
		float3 target = float3(0, 0, 0),
		float3 up = float3(0, 1, 0),
		int a_passNum = 1)
	{

		float fov_degrees = 60;
		float z_near = 0.1f;
		float z_far = 100.0f;
		float aspect = 1.0f;
		auto proj = LiteMath::perspectiveMatrix(fov_degrees, aspect, z_near, z_far);
		auto worldView = LiteMath::lookAt(pos, target, up);

		renderer->Render(image.data(), image.width(), image.height(), worldView, proj, preset, a_passNum);
	}

	template <typename T>
	void render_scene_and_save(
		image_t &image,
		const T &scene,
		const std::string &log_name,  // name, which would be logged
		const std::string &file_name, // name of file
		const std::string &dir,
		MultiRenderPreset preset,
		float3 pos = float3(0, 0, 3),
		float3 target = float3(0, 0, 0),
		float3 up = float3(0, 1, 0))
	{
		auto renderer = CreateMultiRenderer(DEVICE_GPU);
		if constexpr (std::is_same_v<T, COctreeV3>)
		{
			renderer->SetScene(scene, 0);
		}
		else
		{
			renderer->SetScene(scene);
		}
		std::cout << "Started rendering '" << log_name << "'" << std::endl;
		render(image, renderer.get(), preset, pos, target, up);
		std::cout << "Ended rendering '" << log_name << "'" << std::endl;

		std::filesystem::create_directories(dir);
		save_image(image, file_name, dir);
	}

	template <typename Scene>
	float render_and_measure(
		const image_t &reference,
		image_t &scene_image,
		const Scene &scene,
		const std::string &log_name,
		const std::string &file_name,
		const std::string &dir,
		MultiRenderPreset preset,
		float3 pos = float3(0, 0, 3),
		float3 target = float3(0, 0, 0),
		float3 up = float3(0, 1, 0))
	{
		render_scene_and_save(scene_image, scene, log_name, file_name, dir, preset, pos, target, up);
		float psnr = image_metrics::PSNR(reference, scene_image);
		std::cout << "PSNR of 'Reference' and '" << log_name << "' = " << psnr << std::endl;
		return psnr;
	}

	template <typename F>
	float check_method(
		const image_t &reference,
		image_t &scene_image,
		F f,
		const std::string &log_name,
		const std::string &file_name,
		const std::string &dir,
		MultiRenderPreset preset,
		float3 pos = float3(0, 0, 3),
		float3 target = float3(0, 0, 0),
		float3 up = float3(0, 1, 0))
	{
		std::cout << "Started creating '" << log_name << "'" << std::endl;
		auto scene = f();
		std::cout << "Ended creating '" << log_name << "'" << std::endl;
		return render_and_measure(reference, scene_image, scene, log_name, file_name, dir, preset, pos, target, up);
	}

	/*
		Returns quality of model
		0 means best
	*/
	size_t check_model(const cmesh4::SimpleMesh &mesh, const std::string &image_dir)
	{
		float max_psnr = 0;

		MultiRenderPreset preset = getDefaultPreset();
		preset.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
		preset.spp = 4;

		image_t ref_image(WIDTH, HEIGHT);
		image_t tmp_image(WIDTH, HEIGHT);

		render_scene_and_save(ref_image, mesh, "Reference", "ref", image_dir, preset);

		constexpr size_t MIN_DEPTH = 3;
		constexpr size_t MAX_DEPTH = 7;

		for (size_t depth = MIN_DEPTH; depth <= MAX_DEPTH; depth++)
		{

			SparseOctreeSettings settings = SparseOctreeSettings(SparseOctreeBuildType::MESH_TLO, depth);
			sdf_converter::GlobalOctree g;
			g.header.brick_size = 4;
			g.header.brick_pad = 1;

			COctreeV3 coctree;
			coctree.header.bits_per_value = 8;
			coctree.header.brick_size = g.header.brick_size;
			coctree.header.brick_pad = g.header.brick_pad;
			coctree.header.uv_size = 0;
			coctree.header.sim_compression = 0;

			preset.normal_mode = g.header.brick_pad == 1 ? NORMAL_MODE_SDF_SMOOTHED : NORMAL_MODE_VERTEX;

			auto create_coctree = [&]()
			{
				auto tlo = cmesh4::create_triangle_list_octree(mesh, settings.depth, 0, 1.0f);
				std::cout << "Finished TLO..." << std::endl;
				sdf_converter::mesh_octree_to_global_octree(mesh, tlo, g);
				std::cout << "Finished global octree..." << std::endl;
				sdf_converter::global_octree_to_compact_octree_v3(g, coctree, 8);
				return coctree;
			};

			float psnr = check_method(ref_image, tmp_image, create_coctree, "Compact octree with depth " + std::to_string(depth), "coctree_depth_" + std::to_string(depth), image_dir, preset);
			max_psnr = std::max(max_psnr, psnr);
		}
		size_t q = 0;
		if (max_psnr < 30)
		{
			q = 2;
		}
		else if (max_psnr < 40)
		{
			q = 1;
		}
		else if (max_psnr < 50)
		{
			q = 0;
		}
		else
		{
			q = 3;
		}
		auto msg = q == 0 ? "OUR ALGORITHM SUCCEEDED TO CONVERT THIS MODEL" : q == 1 ? "OUR ALGORITHM CONVERTED WITH MODEL WITH FLAWS. USE IT WITH CAUTION"
																		  : q == 2	 ? "OUR ALGORITHM FAILED TO CONVERT THIS MODEL. DO NOT USE IT"
																					 : "PSNR IS TOO HIGH. CHECK THE IMAGES";

		std::cout << msg << std::endl;
		return q == 3 ? 0 : q;
	}

	
	void validate_model(
		const std::string&model_path,
		const std::string&model_root,
		const std::string&out_dir)
	{
		
		

		auto mesh = cmesh4::LoadMesh(model_path.c_str());

		if (mesh.indices.size() == 0)
		{
			std::cerr << "Error: failed to load '" << model_path << "'." << std::endl;
			return;
		}

		std::cout << "Started validating '" << model_root << "'" << std::endl;

		std::string name = std::filesystem::path(model_root).filename().string();

		std::filesystem::create_directories(out_dir + "/images/" + name);
		std::filesystem::create_directories(out_dir + "/best");
		std::filesystem::create_directories(out_dir + "/mid");

		size_t q = check_model(mesh, out_dir + "/images/" + name);

		std::string dst = out_dir + "/" + (q == 0 ? "best" : "mid") + "/" + name;

		if (q < 2)
		{
			std::cout << "Model is copied into '" << dst << "'" << std::endl;
			std::filesystem::copy(model_root, dst, std::filesystem::copy_options::recursive);
		}
		std::cout << "Ended validating '" << model_root << "'" << std::endl;
	}

	void validate_models(const std::string&models_dir, const std::string&out_dir)
	{
		static std::vector<std::string> model_extentions = {".obj", ".ply", ".vsgf"};
		
		static auto is_model_file_name = [&](const std::string&x){
			return model_extentions.end() != std::find(model_extentions.begin(), model_extentions.end(), std::filesystem::path(x).extension().string());
		};
		
		for (auto file : std::filesystem::directory_iterator(models_dir))
		{
			if (file.is_directory())
			{
				std::string path;
				for (auto i : std::filesystem::directory_iterator(file.path()))
				{
					if (i.is_regular_file() && is_model_file_name(i.path().string()))
					{
						if (path != "")
						{
							std::cerr << "Error: lready found model '" << path << "' inside '" << file.path().string() <<"'."<< std::endl;
							return;
						}
						else
						{
							path = i.path().string();
						}
					}
				}
				if (path == "")
				{
					std::cerr << "Error: did not find any models inside '" << file.path().string() << "'." << std::endl;
				}
				validate_model(path, file.path().string(), out_dir);
			}
			else if(file.is_regular_file())
			{
				auto path = file.path().string();
				if (is_model_file_name(path))
				{
					validate_model(path, path, out_dir);
				}
				else
				{
					std::cerr << "Error: '" << path << "' is not a model." << std::endl;
					return;
				}
			}
		}
	}

}

void check_models(const std::string &dir, const std::string &saves)
{
	model_validator::validate_models(dir, saves);
}