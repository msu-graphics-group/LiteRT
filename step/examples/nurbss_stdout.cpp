#include <filesystem>
#include <iostream>

#include "utils/common/timer.h"
#include <LiteMath.h>
#include <step.h>

using namespace STEP;
using namespace profiling;

int main(int argc, const char** argv) {
    if (argc != 2) {
        std::cout << "Usage: nurbss_stdout <path_to_step>" << std::endl;
        return 0;
    }
    std::filesystem::path path = argv[1];
    std::cout << "Parsing started..." << std::endl;
    auto timer = Timer();

    bool exists;
    STEP::Parser parser(path, exists);
    if (!exists) {
        std::cout << "[Error] Parsing failed. File does not exist." << std::endl;
        return 1;
    }

    auto nurbsTable = parser.allIDNurbs();

    auto time = timer.getElapsedTime();
    std::cout << "Parsing finished successfully." << std::endl;
    std::cout << "Parsing time: " << time.asSeconds() << "s." << std::endl;

    for (auto& entry : nurbsTable) {
        auto ID    = entry.first;
        auto nurbs = entry.second;
        std::cout << "### NURBS " << ID << " ###" << std::endl;
        std::cout << nurbs << std::endl;
    }
    return 0;
}
