#include <spec/fourier_spectrum.h>
#include <spec/basic_spectrum.h>
#include <spec/spectral_util.h>
#include <spec/conversions.h>
#include <imageutil/pixel.h>
#include <upsample/functional/fourier.h>
#include <internal/common/constants.h>
#include <internal/common/format.h>
#include <fstream>
#include <cstring>
#include <iostream>

using namespace spec;

FourierLUT load_lut(const std::string &path) {
    std::ifstream file{path};
    return FourierLUT::load_from(file);
}

int main(int argc, char **argv)
{
    if(argc == 1) return 1;

    const Pixel color = Pixel::from_rgb(std::stoi(argv[1], nullptr, 16));

    std::string path = "resources/f_emission_lut.eflf";
    if(argc == 3) {
        path = argv[2];
    }

    FourierLUT lut = load_lut(path);

    FourierEmissionSpectrum fspec = upsample::fourier_emiss_int(color, 25.0f, lut);

    BasicSpectrum spec;
    for(int wl = WAVELENGHTS_START; wl <= WAVELENGHTS_END; wl += WAVELENGHTS_STEP)
    {
        spec.set(wl, fspec.get_or_interpolate(wl));
    }

    util::save_spd(format("output/spd/%s.spd", argv[1]), spec);
    std::cout << "Effective color " << xyz2rgb_unsafe(spectre2xyz0(fspec)) << std::endl;

    return 0;
}