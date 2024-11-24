#pragma once
#include <filesystem>

namespace testing
{

    std::filesystem::path current_executable_path();
    std::filesystem::path current_executable_name();

}