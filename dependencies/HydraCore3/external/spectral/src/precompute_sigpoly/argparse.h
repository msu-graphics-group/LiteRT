#ifndef ARGPARSE_H
#define ARGPARSE_H
#include <imageutil/pixel.h>
#include <string>
#include <optional>

struct Args {
    bool refresh = false;
    std::optional<spec::Pixel> color_in;
    std::string output_path = "output/sigpolylut.spl";
};

bool parse_args(int argc, char **argv, Args &args);


#endif