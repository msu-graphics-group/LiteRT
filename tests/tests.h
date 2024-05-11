#pragma once
#include <vector>
#include <string>

extern std::string scenes_folder_path;

void perform_tests_litert(const std::vector<int> &test_ids);
void benchmark_framed_octree_intersection();
void quality_check(const char *path);