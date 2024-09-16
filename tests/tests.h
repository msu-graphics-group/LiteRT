#pragma once
#include <vector>
#include <string>

extern std::string scenes_folder_path;

void perform_tests_litert(const std::vector<int> &test_ids);
void benchmark_framed_octree_intersection();

enum BenchmarkFlags
{
  BENCHMARK_FLAG_NONE = 0,
  BENCHMARK_FLAG_BUILD = 1,        //build structures and save on disk
  BENCHMARK_FLAG_RENDER_RT = 2,    //render with eye_ray renderer, primary rays, phong shading model
  BENCHMARK_FLAG_RENDER_DEPTH = 4, //render to depth
  BENCHMARK_FLAG_RENDER_HYDRA = 8  //render with hydra, realistic path tracer
};

void main_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags = BENCHMARK_FLAG_BUILD | BENCHMARK_FLAG_RENDER_RT, 
                    const std::string &supported_type = "");
void SBS_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags);
void rtx_benchmark(const std::string &path, const std::string &mesh_name, unsigned flags = BENCHMARK_FLAG_BUILD | BENCHMARK_FLAG_RENDER_RT, 
                   const std::string &supported_type = "", const std::string &device = "GPU");
void quality_check(const char *path);