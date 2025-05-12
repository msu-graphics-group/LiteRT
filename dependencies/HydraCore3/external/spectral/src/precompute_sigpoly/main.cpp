#include "functions.h"
#include "lutworks.h"
#include <internal/serialization/parsers.h>
#include <internal/common/format.h>
#include <iostream>
#include <fstream>
#include <glog/logging.h>

int main(int argc, char **argv)
{   
    google::InitGoogleLogging(argv[0]);

    if(argc == 2) {
        int zeroed_idx = spec::parse<int>(argv[1]);

        SigpolyLUT lut = generate_lut(zeroed_idx, 4, 24);
        std::string output_path = spec::format("output/sp_lut%d.slf", zeroed_idx);
        std::ofstream output{output_path};

        std::cout << "Writing data to " << output_path << "." << std::endl;
        write_header(output);
        write_lut(output, lut);
        std::cout << "Successfully written data." << std::endl;
    }
    else {
        for(int i = 0; i < 3; ++i) {
            SigpolyLUT lut = generate_lut(i, 4, 24);
            std::string output_path = spec::format("output/sp_lut%d.slf", i);
            std::ofstream output{output_path};

            std::cout << "Writing data to " << output_path << "." << std::endl;
            write_header(output);
            write_lut(output, lut);
            std::cout << "Successfully written data." << std::endl;
        }
    }

    return 0;
} 
