#include "argparse.h"
#include <upsample/glassner_naive.h>
#include <upsample/smits.h>
#include <upsample/sigpoly.h>
#include <imageutil/image.h>
#include <spec/basic_spectrum.h>
#include <spec/spectral_util.h>
#include <spec/conversions.h>
#include <internal/common/format.h>
#include <internal/common/util.h>
#include <chrono>
#include <stdexcept>
#include <filesystem>
#include <iostream>
#include <fstream>
#include <memory>

using std::chrono::high_resolution_clock;
using std::chrono::duration_cast;

using namespace spec;
namespace fs = std::filesystem;

std::unique_ptr<IUpsampler> get_upsampler_by_name(const std::string &method_name)
{
    IUpsampler *ptr = nullptr;
    if(method_name == "glassner") {
        ptr = new GlassnerUpsampler();
    }
    else if(method_name == "sigpoly") {
        ptr = new SigPolyUpsampler();
    }
    else if(method_name == "smits") {
        ptr = new SmitsUpsampler();
    }
    return std::unique_ptr<IUpsampler>(ptr);
}

inline bool string_ends_with(const std::string &value, const std::string &ending)
{
    if (ending.size() > value.size()) return false;
    return std::equal(ending.rbegin(), ending.rend(), value.rbegin());
}

int downsample(const Args &args) {

    std::cout << "Loading file..." << std::endl;
    ISpectralImage::ptr spec_img;
    ISpectrum::ptr spec;
    ISpectrum::csptr illum;
    const std::string output_path = (fs::path(args.output_dir) / fs::path(*args.output_name)).string();

    if(!spec::util::load_spectral_image(args.input_path, spec_img, illum)) {
        if(!spec::util::load_spectrum(args.input_path, spec, illum)) {
            std::cerr << "Unknown file format" << std::endl;
            return 2;
        }        
        std::cout << "Converting spectum to RGB..." << std::endl;
        vec3 downsampled_rgb = xyz2rgb(spectre2xyz(*spec, *illum));
        std::ofstream output{output_path + ".txt"};
        output << downsampled_rgb.x << " " << downsampled_rgb.y << " " << downsampled_rgb.z << std::endl; 
        return 0;
    }

    const int w = spec_img->get_width();
    const int h = spec_img->get_height();
    Image img{w, h};


    std::cout << "Converting spectral image to png..." << std::endl;
    init_progress_bar(w * h, 1000);

    for(int j = 0; j < h; ++j) {
        for(int i = 0; i < w; ++i) {
            img.at(i, j) = Pixel::from_vec3(xyz2rgb(spectre2xyz(spec_img->at(i, j), *illum)));
            print_progress(j * w + i);
        }
    }
    finish_progress_bar();

    img.save(output_path + ".png");

    return 0;
}

int upsample_color(const Args &args, IUpsampler &upsampler)
{
    const Pixel rgb = *args.color;
    std::cout << "Converting color to spectrum with " << *args.method << " method..." << std::endl;
    ISpectrum::ptr spectrum = upsampler.upsample_pixel(rgb);


    if(args.ior_mode) {
        BasicSpectrum eta_p;
        BasicSpectrum k_p;

        std::cout << "Calculating ior..." << std::endl;
        for(int wl = WAVELENGHTS_START; wl <= WAVELENGHTS_END; wl += WAVELENGHTS_STEP) {
            auto [eta, k] = color2ior(spectrum->get_or_interpolate(wl));
            eta_p.set(wl, eta);
            k_p.set(wl, k);
        }

        if(!spec::util::save(args.output_dir, *args.output_name + "_eta", eta_p)) {
            std::cerr << "[!] Error saving spetrum." << std::endl;
            return 2;
        }
        if(!spec::util::save(args.output_dir, *args.output_name + "_eta", k_p)) {
            std::cerr << "[!] Error saving spectrum." << std::endl;
            return 2;
        }
        return 0;
    }
    if(!spec::util::save(args.output_dir, *args.output_name, *spectrum)) {
        std::cerr << "[!] Error saving spectrum." << std::endl;
        return 2;
    }
    return 0;
}

int upsample(const Args &args) {
    auto upsampler = get_upsampler_by_name(*args.method);
    if(!upsampler) {
        std::cerr << "[!] Unknown method." << std::endl;
        return 1;
    }

    if(args.color) {
        return upsample_color(args, *upsampler);
    }

    try {

        Image image = Image(args.input_path);
        std::cout << "Converting image to spectral with " << *args.method << " method..." << std::endl;
        auto t1 = high_resolution_clock::now();
        ISpectralImage::ptr spectral_img = upsampler->upsample(image);
        auto t2 = high_resolution_clock::now();
        std::cout << "Upsampling took " << duration_cast<std::chrono::milliseconds>(t2 - t1).count() << " ms." << std::endl;

        std::cout << "Saving..." << std::endl;

        if(!spec::util::save(args.output_dir, *args.output_name, *spectral_img)) {
            std::cerr << "[!] Error saving image." << std::endl;
            return 3;
        }

    } catch (std::exception &ex) {
        std::cerr << "[!] " << ex.what() << std::endl;
        return 1;
    }
    return 0;
}

/**
 *  -c (hex):    use color code instead of texture
 *  -f (path):   path to texture
 *  -m (method): method to use
 */
int main(int argc, char **argv)
{
    Args args;
    if(!parse_args(argc, argv, args)) return 1;

    return args.downsample_mode ? downsample(args) : upsample(args);
} 
