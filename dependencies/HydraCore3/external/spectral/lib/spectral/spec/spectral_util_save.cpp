#include <spec/spectral_util.h>
#include <internal/serialization/binary.h>
#include <internal/common/constants.h>
#include <internal/common/refl.h>
#include <internal/common/format.h>
#include <fstream>
#include <stdexcept>
#include <memory>
#include <limits>
#include <filesystem>
#include <stb_image_write.h>
#include <nlohmannjson/json.hpp>

namespace fs = std::filesystem;
using namespace spec;

namespace
{
    const std::string IMG_FILENAME_FORMAT = "w_" + FLOAT_FORMAT + ".png";
    const std::string SPD_OUTPUT_FORMAT = FLOAT_FORMAT + " " + FLOAT_FORMAT + "\n";

    void _to_stream(void *context, void *data, int size) 
    {

        std::ostream *stream = reinterpret_cast<std::ostream *>(context);
        stream->write(reinterpret_cast<const char *>(data), size);
        stream->flush();
    }

    int write_png_to_stream(std::ostream &stream, int width, int height, int channels, const unsigned char *buf) noexcept(true)
    {
        return stbi_write_png_to_func(_to_stream, reinterpret_cast<void *>(&stream), width, height, channels, buf, 0);
    }

    void normalize_and_convert_to_rgb(const BasicSpectralImage &img, unsigned char *dst, const std::vector<Float> &wavelenghts, int channels, Float &range_out, Float &min_val_out)
    {

        //calculate normalization data
        Float min_val = std::numeric_limits<Float>::max();
        Float max_val = std::numeric_limits<Float>::min();

        const BasicSpectrum *ptr;
        const BasicSpectrum *end = img.raw_data() + img.get_height() * img.get_width();
        for(ptr = img.raw_data(); ptr < end; ++ptr) {
            for(Float w : wavelenghts) {
                Float val = (*ptr)[w];
                if(val > max_val) max_val = val; 
                if(val < min_val) min_val = val;
            }
        }

        Float range = max_val - min_val;
        if(range < 1.0f) { //what about small values?
            range = 1.0f;
        }


        const BasicSpectrum *data = img.raw_data();
        //normalize and write to rgb buffer
        const int vector_size = wavelenghts.size();
        for(long i = 0; i < img.get_height() * img.get_width(); ++i) {
            for(int c = 0; c < vector_size; ++c) {
                Float w = data[i][wavelenghts[c]];
                Float w_norm = (w - min_val) / range;
                dst[i * channels + c] = static_cast<unsigned char>(w_norm * 255.999f);
            }
            for(int c = wavelenghts.size(); c < channels; ++c) {
                dst[i * channels + c] = 0; //zero additional channels
            }
        }

        range_out = range;
        min_val_out = min_val;
    }
}

using json = nlohmann::json;

namespace spec
{

    namespace util {

        const std::string META_FILENAME = "meta.json";

        void Metadata::save(std::ostream &stream) const
        {
            json meta;
            meta["width"] = width;
            meta["height"] = height;

            meta["format"] = format;

            json wavelenghts_array = json::array();
            for(const auto &metaentry : wavelenghts) {
                json entry;
                entry["filename"] = metaentry.filename;
                json targets = json::array();
                for(Float val : metaentry.targets)
                    targets.insert(targets.end(), val);

                entry["targets"] = targets;
                entry["norm_min_val"] = metaentry.norm_min_val;
                entry["norm_range"] = metaentry.norm_range;
                wavelenghts_array.insert(wavelenghts_array.end(), entry);
            }
            meta["wavelenghts"] = wavelenghts_array;


            stream << meta.dump(2);
            stream.flush();
        }

        Metadata Metadata::load(std::istream &stream)
        {   

            Metadata metadata;
            json meta;
            stream >> meta;

            meta.at("width").get_to(metadata.width);
            meta.at("height").get_to(metadata.height);

            meta.at("format").get_to(metadata.format);


            const auto &wlarray = meta.at("wavelenghts");
            for(const auto &e : wlarray) {
                metadata.wavelenghts.push_back({});
                MetadataEntry &entry = metadata.wavelenghts.back();

                e.at("filename").get_to(entry.filename);
                const auto &targets_arr = e.at("targets");

                for(const auto &elem : targets_arr) {
                    entry.targets.push_back(elem.get<Float>());
                }
                e.at("norm_min_val").get_to(entry.norm_min_val);
                e.at("norm_range").get_to(entry.norm_range);
            }
            metadata.wavelenghts.shrink_to_fit();
            return metadata;
        }

        void save_spd(const std::string &path, const BasicSpectrum &spectre)
        {
            std::ofstream file(path, std::ios::trunc);
            if(!file) throw std::runtime_error("Cannot open file");

            const auto &map = spectre.get_map();

            for(const Float wl : spectre.get_wavelenghts()) {
                file << format(SPD_OUTPUT_FORMAT, wl, map.at(wl));
            }

            file.flush();
        }

        void save_wavelenghts_to_png_multichannel(std::ostream &stream, const BasicSpectralImage &img, const std::vector<Float> &wavelenghts, SavingResult &res, int requested_channels)
        {
            const int vector_size = wavelenghts.size();
            if(vector_size < 1 || vector_size > 4) throw std::invalid_argument("Illegal wavelenghts size");

            int channels = vector_size > requested_channels ? vector_size : requested_channels;
            const int width = img.get_width();
            const int height = img.get_height();


            std::unique_ptr<unsigned char[]> buf{new unsigned char[width * height * channels]};

            normalize_and_convert_to_rgb(img, buf.get(), wavelenghts, channels, res.norm_range, res.norm_min);

            int code = write_png_to_stream(stream, width, height, channels, buf.get());
            if(!code) {
                res.success = false;
                throw std::runtime_error("Error saving to file");
            }
            res.channels_used = channels;
        }

        void save_wavelenght_to_png1(std::ostream &stream, const BasicSpectralImage &img, Float wavelenght, SavingResult &res)
        {
            const int width = img.get_width();
            const int height = img.get_height();
            std::unique_ptr<unsigned char[]> buf{new unsigned char[width * height]};

            normalize_and_convert_to_rgb(img, buf.get(), {wavelenght}, 1, res.norm_range, res.norm_min);

            int code = write_png_to_stream(stream, width, height, 1, buf.get());
            if(!code) {
                res.success = false;
                throw std::runtime_error("Error saving to file");
            }
            res.channels_used = 1;
        }

        bool save_as_png1(const BasicSpectralImage &image, const std::string &dir, const std::string &meta_filename, const ISpectrum &lightsource) {

            Metadata metadata;
            metadata.width = image.get_width();
            metadata.height = image.get_height();

            metadata.format = "png1";

            const fs::path dir_path{dir};
            fs::create_directories(dir_path);
            
            if(!isa<BasicSpectrum>(lightsource)) return false;


            save_spd(dir_path / "light.spd", static_cast<const BasicSpectrum &>(lightsource));

            SavingResult saving_result;
            for(Float w : image.get_wavelenghts()) {
                //save as 1-channel png
                std::string filename = format(IMG_FILENAME_FORMAT, w);
                fs::path img_path = dir_path / filename;
                std::fstream file(img_path, std::ios::out | std::ios::binary | std::ios::trunc);
                try {
                    save_wavelenght_to_png1(file, image, w, saving_result);
                } catch(...) {
                    file.close();
                    throw;
                }
                //add entry to metadata
                metadata.wavelenghts.push_back(MetadataEntry{filename, {w}, saving_result.norm_min, saving_result.norm_range});
                file.close();
            }
            //save metadata
            std::fstream meta_file(dir_path / meta_filename, std::ios::out | std::ios::trunc);
            metadata.save(meta_file);
            meta_file.close();

            return true;
        }

        bool save_sigpoly(const std::string path, const SigPolySpectrum &spectrum)
        {
            std::ofstream file(path, std::ios::trunc);
            if(!file) throw std::runtime_error("Cannot open file");

            file << format("%f %f %f", spectrum[0], spectrum[1], spectrum[2]);

            file.flush();
            return true;
        }
        
        bool save_sigpoly_img(const std::string path, const SigPolySpectralImage &img)
        {
            std::ofstream file(path, std::ios::trunc);
            if(!file) throw std::runtime_error("Cannot open file");

            const unsigned size = img.get_height() * img.get_width();

            //HEADER
            binary::write<uint64_t>(file, SIGPOLY_FILE_MARKER);
            binary::write<uint16_t>(file, sizeof(Float));
            binary::write<uint32_t>(file, img.get_width());
            binary::write<uint32_t>(file, img.get_height());

            //DATA
            const auto *ptr = img.raw_data();
            for(unsigned i = 0; i < size; ++i) {
                binary::write_vec<Float>(file, ptr[i].get());
            }

            file.flush();
            return false;
        }
        
        bool save(const std::string &directory_path, const std::string &input_filename, const ISpectrum &s) {
            fs::path p{directory_path};
            if(isa<BasicSpectrum>(s)) {
                const BasicSpectrum &spectrum = static_cast<const BasicSpectrum &>(s);
                save_spd(p / (input_filename + ".spd"), spectrum);
                return true;
            }
            if(isa<SigPolySpectrum>(s)) {
                const SigPolySpectrum &spectrum = static_cast<const SigPolySpectrum &>(s);
                return save_sigpoly(p / (input_filename + ".spspec"), spectrum);
            }
            return false;
        }

        bool save(const std::string &directory_path, const std::string &input_filename, const ISpectralImage &s) {
            if(s.get_width() == 1 && s.get_height() == 1) {
                return save(directory_path, input_filename, s.at(0, 0));
            }
            fs::path p{directory_path};
            if(isa<BasicSpectralImage>(s)) {
                const BasicSpectralImage &img = static_cast<const BasicSpectralImage &>(s);
                return save_as_png1(img, p / input_filename);
                
            }
            if(isa<SigPolySpectralImage>(s)) {
                const SigPolySpectralImage &img = static_cast<const SigPolySpectralImage &>(s);
                return save_sigpoly_img(p / (input_filename + ".sif"), img);
            }
            return false;
        }
    }
}