#ifndef INCLUDE_SPECTRAL_SPEC_BASIC_SPECTRUM_H
#define INCLUDE_SPECTRAL_SPEC_BASIC_SPECTRUM_H
#include <spectral/spec/spectrum.h>
#include <spectral/spec/spectral_image.h>
#include <spectral/internal/common/constants.h>
#include <initializer_list>
#include <unordered_map>
#include <set>

namespace spec {

    class BasicSpectrum : public ISpectrum
    {
    public: 
        INJECT_REFL(BasicSpectrum);

        BasicSpectrum()
            : spectre() {}

        BasicSpectrum(std::initializer_list<std::pair<const Float, Float>> l) 
            : spectre(l) {get_wavelenghts();}

        BasicSpectrum(const BasicSpectrum &s) = default;

        BasicSpectrum(BasicSpectrum &&s)
            : ISpectrum(s), cached_wavelenghts(std::move(s.cached_wavelenghts)), modified(std::move(s.modified)), spectre(std::move(s.spectre)) {}

        void set(Float wavelenght, Float value);

        const std::set<Float> &get_wavelenghts() const;

        Float &operator[](Float w);

        Float &get_or_create(Float w);

        Float operator[](Float w) const;

        Float get_or_interpolate(Float w) const override;

        BasicSpectrum &operator=(const BasicSpectrum &other) = default;
        BasicSpectrum &operator=(BasicSpectrum &&other);

        void reserve(size_t count);

        const std::unordered_map<Float, Float> &get_map() const {
            return spectre;
        }

        static const BasicSpectrum none;

    private:
        mutable std::set<Float> cached_wavelenghts{};
        mutable bool modified = true;
        std::unordered_map<Float, Float> spectre;
    };

    /**
     * Formats: 
     *          png1, json          - monochrnomatic png for each wavelenght and metadata file (default)
     *          png1_zip, zip       - same as 'png1', but packed in zip archive
     *          png3, json3         - 3 channel png for each triplet of wavelenghts and metadata file
     *          png3_zip            - same as 'png3', but packet in zip archive
     *          one_spectre         - saves spectre of (0, 0) pixel in text format
     * 
     *  If format is not specified (empty string is supplied), format will be choosen from name of file. 
     * If 'png1', or 'png3' format is used, path is interpreted as directory to save those files to,
     * otherwise, path is considered to be filename. If 'json' or 'json3' format is used, all
     * correspinding pngs are saved in the same directory as file. 
     */

    extern template class SpectralImage<BasicSpectrum>;

    class BasicSpectralImage : public SpectralImage<BasicSpectrum>
    {
    public:
        using SpectralImage<BasicSpectrum>::SpectralImage;
        static const UniqueId class_id;
        //SpectralImage(const std::string &path);

        BasicSpectralImage(int w, int h, const BasicSpectrum &p)
            : SpectralImage<BasicSpectrum>(w, h, p), wavelenghts(p.get_wavelenghts()) {} 

        BasicSpectralImage(const BasicSpectralImage &img)
            : SpectralImage<BasicSpectrum>(img), wavelenghts(img.wavelenghts) {}

        BasicSpectralImage(BasicSpectralImage &&image)
            : SpectralImage<BasicSpectrum>(std::move(image)), wavelenghts(std::move(image.wavelenghts)) {}

        BasicSpectralImage &operator=(BasicSpectralImage &&other);

        void add_wavelenght(Float w);
        
        void remove_wavelenght(Float w);

        const std::set<Float> &get_wavelenghts() const
        {
            return wavelenghts;
        }

        void set_wavelenghts(const std::set<Float> &wl) {
            wavelenghts = wl;
        }

        void set_wavelenghts(std::set<Float> &&wl) {
            wavelenghts = std::move(wl);
        }

        bool save(const std::string &path, const std::string &format = "") const;

        bool validate() const;

        

    private:
        std::set<Float> wavelenghts;
    };


}
#endif