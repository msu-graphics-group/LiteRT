#include "functions.h"
#include "lutworks.h"
#include <spec/conversions.h>
#include <internal/serialization/parsers.h>
#include <internal/serialization/csv.h>
#include <internal/common/format.h>
#include <vector>
#include <iostream>
#include <fstream>
#include <glog/logging.h>

const std::string EMISS_LUT_FILENAME = "output/f_emission_lut.eflf";
using namespace spec;

void load_dataset(std::vector<vec3i> &rgbs, std::vector<Float> &wavelenghts, std::vector<std::vector<Float>> &values)
{
    std::ifstream in_spectra("output/dataset_spectra.csv");
    std::ifstream in_rgbs("output/dataset_rgb.csv");

    wavelenghts = std::get<0>(*csv::parse_line_m<Float>(in_spectra));


    auto data = csv::load_as_vector_m<Float>(in_spectra);
    in_spectra.close();
    auto rgb_data = csv::load_as_vector<int, int, int>(in_rgbs, ',', 1);
    in_rgbs.close();

    values.resize(data.size());
    rgbs.reserve(data.size());

    for(unsigned i = 0; i < data.size(); ++i) {
        values[i] = std::move(std::get<0>(data[i]));

        const auto &e = rgb_data[i];
        rgbs.emplace_back(std::get<0>(e), std::get<1>(e), std::get<2>(e));
    }
}

int main(int argc, char **argv)
{   
    google::InitGoogleLogging(argv[0]);

    //Parameters
    unsigned param_step = 4;
    unsigned param_knearest = 4;
    if(argc >= 2) {
        param_step = parse<unsigned>(argv[1]);
        if(argc == 3) {
            param_knearest = parse<unsigned>(argv[2]);
        }
    }

    std::vector<vec3i> ds_rgbs;
    std::vector<Float> ds_wavelenghts;
    std::vector<std::vector<Float>> ds_spectra;
    load_dataset(ds_rgbs, ds_wavelenghts, ds_spectra);

    FourierLUT lut = generate_lut(ds_wavelenghts, ds_spectra, ds_rgbs, param_step, param_knearest);

    std::ofstream output{EMISS_LUT_FILENAME};

    std::cout << "Writing data to " << EMISS_LUT_FILENAME << "." << std::endl;
    write_header(output);
    write_lut(output, lut);
    std::cout << "Successfully written data." << std::endl;

    return 0;
} 
