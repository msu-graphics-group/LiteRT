#include <fstream>
#include <chrono>
#include <filesystem>
#include <sstream>

#include <stp_parser.hpp>
#include <LiteMath.h>

using namespace STEP;

void nurbs2file(const RawNURBS &nurbs, const std::string &filename) {
    std::ofstream cout(filename);

    // Control points dimensions
    uint32_t n = nurbs.points.rows_count();
    cout << "n = " << n-1 << std::endl;

    uint32_t m = nurbs.points.cols_count();
    cout << "m = " << m-1 << std::endl;

    // Control points
    cout << "points:" << std::endl;
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            auto index = std::make_pair(i, j);
            auto point = nurbs.points[index];
            cout << "{" << point.x << " " << point.z << " " << point.y << "}\t";
        }
        cout << std::endl;
    }

    // Weights
    cout << "weights:" << std::endl;
    for (size_t i = 0; i < n; i++) {
        for (size_t j = 0; j < m; j++) {
            auto index = std::make_pair(i, j);
            auto point = nurbs.points[index];
            cout << nurbs.weights[index] << " ";
        }
        cout << std::endl;
    }

    // Degrees
    cout << "u_degree: " << nurbs.u_degree << std::endl;
    cout << "v_degree: " << nurbs.v_degree << std::endl;

    // Knots
    cout << "u_knots: ";
    for (auto knot : nurbs.u_knots)
        cout << knot << " ";
    cout << std::endl;
    
    cout << "v_knots: ";
    for (auto knot : nurbs.v_knots)
        cout << knot << " ";
    cout << std::endl;
}


int main(int argc, const char **argv) {
  if (argc != 2) {
    std::cout << "Usage: parse_and_div <path_to_stp_file>" << std::endl;
    return 0;
  }
  std::filesystem::path stp_path = argv[1];
  std::cout << "Parsing started..." << std::endl;
  auto tick_start = std::chrono::high_resolution_clock::now();
  auto entities = STEP::parse(stp_path);
  auto nurbsV = STEP::allNURBS(entities);
  auto tick_end = std::chrono::high_resolution_clock::now();
  std::cout << "Parsing finished successfully." << std::endl;
  float time = 
      std::chrono::duration_cast<std::chrono::milliseconds>(tick_end-tick_start).count()/1000.0f;
  std::cout << "Parsing time: " << time << "s." << std::endl;

  int counter = 0;
  for (auto &nurbs: nurbsV) {
    std::stringstream ss;
    ss << stp_path.filename().replace_extension("").c_str() << counter++ << ".nurbss";
    nurbs2file(nurbs, ss.str());
  }
  
  return 0;
}
