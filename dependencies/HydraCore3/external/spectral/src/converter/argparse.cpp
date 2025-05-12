#include "argparse.h"
#include <cstring>
#include <getopt.h>
#include <iostream> 
#include <filesystem>
#include <internal/serialization/csv.h>

using namespace spec;

namespace fs = std::filesystem;

namespace {
    enum InputType
    {
        NONE,
        COLOR,
        FILE
    };

    Pixel load_color_vec(const std::string &str) 
    {
        const auto [r, g, b] = *csv::parse_line<Float, Float, Float>(str + "\n", ' ');
        return Pixel::from_vec3({r, g, b});
    }

}



bool parse_args(int argc, char **argv, Args &args)
{
    static struct option long_options[] = {
        {"downsample", no_argument, nullptr, 1},
        {"ior", no_argument, nullptr, 2},
        {nullptr, 0, nullptr, 0}
    };

    InputType input_type = InputType::NONE;
    int c;
    while((c = getopt_long(argc, argv, "n:D:c:v:f:m:", long_options, nullptr)) != -1) {
        switch(c) {
        case 1:
            args.downsample_mode = true;
            break;
        case 2:
            args.ior_mode = true;
            break;
        case 'c':
            if(input_type != InputType::NONE) return false;
            args.color = Pixel::from_rgb(std::stoi(optarg, nullptr, 16));
            input_type = InputType::COLOR;
            break;
        case 'v':
            if(input_type != InputType::NONE) return false;
            args.color = load_color_vec(std::string(optarg));
            input_type = InputType::COLOR;
            break;
        case 'f':
            if(input_type != InputType::NONE) return false;
            args.input_path = optarg;
            input_type = InputType::FILE;
            break;
        case 'm':
            args.method.emplace(optarg);
            break;
        case 'D':
            args.output_dir = optarg;
            break;
        case 'n':
            args.output_name.emplace(optarg);
            break;
        case '?':
            std::cerr << "[!] Unknown argument." << std::endl; 
            return false;
        }
    }  
    if(input_type == InputType::NONE) {
        std::cerr << "[!] No input specified." << std::endl;
        return false;
    }
    if(!args.output_name) {
        fs::path p1{args.input_path};
        args.output_name.emplace(p1.stem());
    }

    if(args.ior_mode && input_type != InputType::COLOR) {
        std::cerr << "[!] IOR conversion is supported only for colors." << std::endl;
        return false;
    }
    //validate
    if(args.downsample_mode) {
        return input_type == InputType::FILE && !args.method.has_value();
    }
    else {
        if(!args.method) {
            std::cerr << "[!] No method specified." << std::endl;
            return false;
        }

        return input_type == InputType::FILE || args.output_name.has_value();
    }
}