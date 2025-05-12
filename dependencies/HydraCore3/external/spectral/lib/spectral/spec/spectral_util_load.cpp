#include <spec/spectral_util.h>
#include <internal/math/math.h>
#include <internal/serialization/csv.h>
#include <internal/serialization/envi.h>
#include <internal/serialization/binary.h>
#include <internal/common/format.h>
#include <stb_image.h>
#include <fstream>
#include <stdexcept>
#include <vector>
#include <cmath>
#include <tuple>
#include <unordered_map>
#include <filesystem>
#include <iostream>

#include <cstring>

#include <internal/common/util.h>

using namespace spec;
namespace fs = std::filesystem;

namespace spec::util
{

    namespace {
        inline void _d6500ptr(ISpectrum::csptr &dest)
        {
            dest.reset(&CIE_D6500, [](ISpectrum const *) -> void {});
        }
    }

    BasicSpectrum load_spd(const std::string &path)
    {
        std::ifstream file{path};
        if(!file) throw std::runtime_error("Cannot open file");
        BasicSpectrum sp;

        std::vector<std::tuple<Float, Float>> loaded = csv::load_as_vector<Float, Float>(file, ' ');
        for(const auto &p : loaded) {
            sp.set(std::get<0>(p), std::get<1>(p));
        }

        return sp;
    }

    BasicSpectrum load_spd(const std::string &path, ISpectrum::csptr &lightsource)
    {
        _d6500ptr(lightsource);
        return load_spd(path);
    }

    namespace {

        int _read_stream(void *context, char *data, int size)
        {
            std::istream *stream = reinterpret_cast<std::istream *>(context);
            stream->read(data, size);
            return stream->gcount(); 
        }   

        void _skip_stream(void *context, int n)
        {
            std::istream *stream = reinterpret_cast<std::istream *>(context);
            if(n > 0) {
                stream->ignore(n);
            }
            else {
                stream->seekg(n, std::ios_base::cur);
            }
        }

        int _eof_stream(void *context)
        {
            std::istream *stream = reinterpret_cast<std::istream *>(context);
            stream->peek();
            return stream->eof();
        }   

        const stbi_io_callbacks _callbacks{_read_stream, _skip_stream, _eof_stream};

        using _DeleterType = void (*)(unsigned char *);
        using STBImageUniquePtr = std::unique_ptr<unsigned char[], _DeleterType>;


        void _stb_deleter(unsigned char *ptr) 
        {
            stbi_image_free(ptr);
        }

        STBImageUniquePtr _load_image(std::istream &stream, int *width, int *height, int desired_channels)
        {
            int channels;
            STBImageUniquePtr data{
                stbi_load_from_callbacks(&_callbacks, reinterpret_cast<void *>(&stream), width, height, &channels, desired_channels),
                _stb_deleter};
            //if(channels != desired_channels) throw std::runtime_error(format("Incorrect number of channels (%d)", channels));
            return data;
        }

        void _load_from_file(const fs::path &directory, const Metadata &meta, const MetadataEntry &entry, BasicSpectralImage &img)
        {
            std::ifstream file{directory / entry.filename, std::ios::binary | std::ios::in};
            int w, h;
            STBImageUniquePtr data = _load_image(file, &w, &h, entry.targets.size());
            if(w != meta.width || h != meta.height) {
                throw std::runtime_error("Incorrect image shape");
            }

            const unsigned char *ptr = data.get();
            const int wl_count = entry.targets.size();

            for(int j = 0; j < meta.height; ++j) {
                for(int i = 0; i < meta.width; ++i) {
                    BasicSpectrum &spec = img.at(i, j);
                    for(int k = 0; k < wl_count; ++k) {
                        const Float val = std::fma<Float>(*(ptr++) / 255.0f, entry.norm_range, entry.norm_min_val); 
                        spec.set(entry.targets[k], val);
                    }
                }
            }

        }

    }


    BasicSpectralImage load_json_meta(const std::string &meta_path, ISpectrum::csptr &lightsource)
    {
        fs::path p{meta_path};
        std::ifstream meta_file{meta_path};
        Metadata meta = Metadata::load(meta_file);
        meta_file.close();

        if(meta.format != "png1") {
            throw std::runtime_error("Unsupported image format");
        }

        BasicSpectralImage image{meta.width, meta.height};


        for(const MetadataEntry &entry : meta.wavelenghts) {
            if(entry.targets.size() < 1 && entry.targets.size() > 4) {
                throw std::runtime_error("Too many wavelenghts per image");
            }
            _load_from_file(p.parent_path(), meta, entry, image);
        }

        lightsource.reset(new BasicSpectrum(load_spd(p.parent_path() / "light.spd")));

        return image;
    }

    namespace {

        template<typename T>
        void _load_bsq(const MetaENVI &meta, std::istream &str, BasicSpectralImage &img)
        {   
            init_progress_bar(meta.bands * meta.lines * meta.samples, 1000);

            size_t count = 0;
            for(int w = 0; w < meta.bands; ++w) {
                Float wl = meta.wavelength[w];
                img.add_wavelenght(wl);
                for(int j = 0; j < meta.lines; ++j) {
                    for(int i = 0; i < meta.samples; ++i) {
                        Float value = binary::read_ordered<T>(str, meta.byte_order == MetaENVI::ByteOrder::BIG_ENDIAN_ORDER);
                        img.at(i, j).set(wl, value);
                        print_progress(++count);
                    }
                }

            }
            finish_progress_bar();

        }

        template<typename T>
        void _load_bil(const MetaENVI &meta, std::istream &str, BasicSpectralImage &img)
        {
            for(Float wl : meta.wavelength) {
                img.add_wavelenght(wl);
                for(int j = 0; j < meta.lines; ++j) {
                    for(int i = 0; i < meta.samples; ++i) {
                        Float value = binary::read_ordered<T>(str, meta.byte_order == MetaENVI::ByteOrder::BIG_ENDIAN_ORDER);
                        img.at(i, j).set(wl, value);
                    }
                }

            }
        }

        template<typename T>
        void _load_bip(const MetaENVI &meta, std::istream &str, BasicSpectralImage &img)
        {
            for(int j = 0; j < meta.lines; ++j) {
                for(Float wl : meta.wavelength) {
                    for(int i = 0; i < meta.samples; ++i) {
                        Float value = binary::read_ordered<T>(str, meta.byte_order == MetaENVI::ByteOrder::BIG_ENDIAN_ORDER);
                        img.at(i, j).set(wl, value);
                    }
                }
            }
            for(Float wl : meta.wavelength) {
                img.add_wavelenght(wl);
            }

        }


    }

    BasicSpectralImage load_envi_hdr(const std::string &meta_path, const std::string &raw_path, ISpectrum::csptr &lightsource)
    {
        MetaENVI meta = MetaENVI::load(meta_path);

        //meta.byte_order = MetaENVI::ByteOrder::LITTLE_ENDIAN_ORDER;

        std::cout << meta_path << " " << raw_path << std::endl;

        if(meta.wavelength_units == MetaENVI::UnitType::UNSUPPORTED || meta.data_type == MetaENVI::DataType::UNSUPPORTED) {
            throw std::runtime_error("Unsupported file format");
        }
        std::ifstream file{raw_path, std::ios::in | std::ios::binary};
        file.ignore(meta.header_offset); //Skip header offset specified in metadata

        std::cout << "Loading file with width " << meta.lines << " height " << meta.samples << std::endl;

        BasicSpectralImage img{meta.lines, meta.samples};
        BasicSpectrum *raw_ptr = img.raw_data();
        for(long i = 0; i < img.get_width() * img.get_height(); ++i) {
            raw_ptr[i].reserve(meta.bands);
        }

        switch(meta.interleave) {
        case MetaENVI::Interleave::BSQ:
            if(meta.data_type == MetaENVI::DataType::FLOAT32) {
                _load_bsq<float>(meta, file, img);
            }
            else {
                _load_bsq<double>(meta, file, img);
            }
            break;
        case MetaENVI::Interleave::BIL:
            if(meta.data_type == MetaENVI::DataType::FLOAT32) {
                _load_bil<float>(meta, file, img);
            }
            else {
                _load_bil<double>(meta, file, img);
            }
            break;
        case MetaENVI::Interleave::BIP:
            if(meta.data_type == MetaENVI::DataType::FLOAT32) {
                _load_bip<float>(meta, file, img);
            }
            else {
                _load_bip<double>(meta, file, img);
            }
            break;
        }

        BasicSpectrum *ls = new BasicSpectrum();

        for(int w = 0; w < meta.bands; ++w) {
            ls->set(meta.wavelength[w], Float(meta.illuminant[w]));
        }
        lightsource.reset(ls);
        return img;
    }

    BasicSpectralImage load_envi_hdr(const std::string &meta_path, ISpectrum::csptr &lightsource)
    {
        fs::path p{meta_path};
        return load_envi_hdr(meta_path, p.replace_extension("raw").string(), lightsource);
    }

    SigPolySpectrum load_sigpoly(const std::string &path, ISpectrum::csptr &lightsource)
    {
        std::ifstream file(path);
        if(!file) throw std::runtime_error("Cannot open file");

        auto loaded = csv::parse_line<Float, Float, Float>(file, ' ');
        if(!loaded) {
            throw std::runtime_error("Error reading data");
        }

        _d6500ptr(lightsource);

        return {{std::get<0>(*loaded), std::get<1>(*loaded), std::get<2>(*loaded)}};
    }

    SigPolySpectralImage load_sigpoly_img(const std::string &path, ISpectrum::csptr &lightsource)
    {
        std::ifstream file{path};
        if(!file) throw std::runtime_error("Cannot open file");

        const uint64_t marker = binary::read<uint64_t>(file);
        if(marker != SIGPOLY_FILE_MARKER) throw std::runtime_error("Incorrect file format");
        const uint64_t floatsize = binary::read<uint16_t>(file);
        if(floatsize != sizeof(Float)) throw std::runtime_error("Unsupported float size in image");
        const uint32_t width = binary::read<uint32_t>(file);
        const uint32_t height = binary::read<uint32_t>(file);

        SigPolySpectralImage img{int(width), int(height)};
        auto *ptr = img.raw_data();

        for(uint32_t i = 0; i < width * height; ++i) {
            ptr[i].set(binary::read_vec<Float>(file));
        }

        _d6500ptr(lightsource);

        return img;
    }

    namespace {
        enum class SpectrumFormat 
        {
            UNKNOWN = 0,
            SIGPOLY,
            BASIC_SPD
        };

        SpectrumFormat _guess_spec_format(const std::string &ext)
        {
            static const std::unordered_map<std::string, SpectrumFormat> formats{
                {".spd", SpectrumFormat::BASIC_SPD},
                {".spspec", SpectrumFormat::SIGPOLY}
            };

            auto it = formats.find(ext);
            if(it != formats.end()) {
                return it->second;
            }
            return SpectrumFormat::UNKNOWN;
        }

        enum class SpectralImgFormat 
        {   
            UNKNOWN = 0,
            SIGPOLY_SIF,
            BASIC_JSON,
            BASIC_ENVI_HDR
        };

        SpectralImgFormat _guess_specimg_format(const std::string &ext)
        {
            static const std::unordered_map<std::string, SpectralImgFormat> formats{
                {".sif", SpectralImgFormat::SIGPOLY_SIF},
                {".json", SpectralImgFormat::BASIC_JSON},
                {".hdr", SpectralImgFormat::BASIC_ENVI_HDR} 
            };

            auto it = formats.find(ext);
            if(it != formats.end()) {
                return it->second;
            }
            return SpectralImgFormat::UNKNOWN;
        }

    }

    bool load_spectrum(const std::string path, ISpectrum::ptr &s, ISpectrum::csptr &lightsource)
    {
        fs::path p{path};
        SpectrumFormat f = _guess_spec_format(p.extension());
        switch(f) {
        case SpectrumFormat::SIGPOLY:
            s.reset(new SigPolySpectrum(load_sigpoly(path, lightsource)));
            return true;
        case SpectrumFormat::BASIC_SPD:
            s.reset(new BasicSpectrum(load_spd(path, lightsource)));
            return true;
        default:
            return false;
        }
    }

    bool load_spectral_image(const std::string path, ISpectralImage::ptr &img, ISpectrum::csptr &lightsource)
    {
        fs::path p{path};

        SpectralImgFormat f = _guess_specimg_format(p.extension());
        try {
            switch(f) {
            case SpectralImgFormat::SIGPOLY_SIF:
                img.reset(new SigPolySpectralImage(load_sigpoly_img(path, lightsource)));
                return true;
            case SpectralImgFormat::BASIC_JSON:
                img.reset(new BasicSpectralImage(load_json_meta(path, lightsource)));
                return true;
            case SpectralImgFormat::BASIC_ENVI_HDR:
                img.reset(new BasicSpectralImage(load_envi_hdr(path, lightsource)));
                return true;
            default:
                return false;
            }
        }
        catch(std::runtime_error &e) {
            std::cerr << e.what() << std::endl;
            return false;
        }
    }

}