#ifndef INCLUDE_SPECTRAL_SPEC_SPECTRAL_UTIL_H
#define INCLUDE_SPECTRAL_SPEC_SPECTRAL_UTIL_H
#include <spectral/spec/basic_spectrum.h>
#include <spectral/spec/sigpoly_spectrum.h>
#include <string>
#include <vector>
#include <ostream>
#include <istream>
#include <cinttypes>

namespace spec::util
{

    extern const BasicSpectrum CIE_D6500;

    extern const std::string META_FILENAME;

    constexpr uint64_t SIGPOLY_FILE_MARKER = 0xfafa0000ab0bab0b;

    using XYZArray_t = Float[CURVES_ARRAY_LEN];

    template<const XYZArray_t& arr>
    Float _interp(Float wl) noexcept(true)
    {
        const int a = (int(wl) / WAVELENGHTS_STEP) * WAVELENGHTS_STEP;
        const int a_idx = (a - WAVELENGHTS_START) / WAVELENGHTS_STEP;
        if(a > WAVELENGHTS_END) return 0.0f;

        const Float f_a = arr[a_idx];
        if(a == wl) return f_a;


        const int b_idx = a_idx + 1;
        if(b_idx < WAVELENGHTS_START / WAVELENGHTS_STEP) return 0.0f;

        const Float f_b = arr[b_idx];

        return f_a + (f_b - f_a) * (wl - a) / WAVELENGHTS_STEP;
    }

    Float get_cie_y_integral();
    Float get_cie_y_integral(const ISpectrum &light);


    BasicSpectrum convert_to_spd(const ISpectrum &spectrum, const std::vector<Float> &wavelenghts = {});


    struct SavingResult
    {
        bool success;
        int channels_used;
        Float norm_min;
        Float norm_range;
    };

    struct MetadataEntry
    {
        std::string filename;
        std::vector<Float> targets;
        Float norm_min_val;
        Float norm_range;
    };

    struct Metadata
    {
        int width;
        int height;
        std::string format;
        std::vector<MetadataEntry> wavelenghts;

        void save(std::ostream &dest) const;
        static Metadata load(std::istream &src);
    };


    void save_spd(const std::string &path, const BasicSpectrum &spectre);

    /**
     *  Saves specified wavelenghts of spectral image in multichannel png file (up to 4 channels).
     * Writes saving data to res. If wavelenghts is empty or contains more than 4 
     * elements exception is thrown.
     */
    void save_wavelenghts_to_png_multichannel(std::ostream &stream, const BasicSpectralImage &img, const std::vector<Float> &wavelenghts, SavingResult &res, int requested_channels = 0);

    void save_wavelenght_to_png1(std::ostream &stream, const BasicSpectralImage &img, Float wavelenght, SavingResult &res);

    bool save_as_png1(const BasicSpectralImage &image, const std::string &dir, const std::string &meta_filename = META_FILENAME, const ISpectrum &lightsource = CIE_D6500);

    bool save_sigpoly(const std::string path, const SigPolySpectrum &spectrum);
    bool save_sigpoly_img(const std::string path, const SigPolySpectralImage &img);

    bool save(const std::string &directory_path, const std::string &input_filename, const ISpectrum &s);
    bool save(const std::string &directory_path, const std::string &input_filename, const ISpectralImage &s);


    BasicSpectrum load_spd(const std::string &path);
    BasicSpectrum load_spd(const std::string &path, ISpectrum::csptr &lightsource);

    BasicSpectralImage load_jsonmeta(const std::string &meta_path, ISpectrum::csptr &lightsource);

    BasicSpectralImage load_envi_hdr(const std::string &meta_path, const std::string &raw_path, ISpectrum::csptr &lightsource);
    BasicSpectralImage load_envi_hdr(const std::string &meta_path, ISpectrum::csptr &lightsource);
    SigPolySpectrum load_sigpoly(const std::string &path, ISpectrum::csptr &lightsource);
    SigPolySpectralImage load_sigpoly_img(const std::string &path,  ISpectrum::csptr &lightsource);

    bool load_spectrum(const std::string path, ISpectrum::ptr &s, ISpectrum::csptr &lightsource);
    bool load_spectral_image(const std::string path, ISpectralImage::ptr &img, ISpectrum::csptr &lightsource);

}

#endif