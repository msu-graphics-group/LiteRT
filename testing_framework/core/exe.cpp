#include "exe.h"

namespace testing
{

    std::filesystem::path current_executable_path()
    {
        return std::filesystem::canonical("/proc/self/exe");
    }

    std::filesystem::path current_executable_name()
    {
        return current_executable_path().filename();
    }

}