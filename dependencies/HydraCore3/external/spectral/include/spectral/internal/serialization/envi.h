#ifndef INCLUDE_SPECTRAL_INTERNAL_SERIALIZATION_ENVI_H
#define INCLUDE_SPECTRAL_INTERNAL_SERIALIZATION_ENVI_H
#include <spectral/internal/math/math_fwd.h>
#include <string>
#include <unordered_map>
#include <vector>

namespace spec {

    struct MetaENVI
    {

        enum class ByteOrder {LITTLE_ENDIAN_ORDER, BIG_ENDIAN_ORDER}; 
        enum class Interleave {BSQ, BIL, BIP};
        enum class UnitType {NANOMETER, UNSUPPORTED};
        enum class DataType {FLOAT32, FLOAT64, UNSUPPORTED};


        std::size_t header_offset = 0; // skip bytes from start of image
        UnitType wavelength_units = UnitType::NANOMETER;

        int samples; // aka width
        int lines; // aka height
        int bands; // aka channels

        ByteOrder byte_order;
        DataType data_type;
        Interleave interleave;

        std::vector<Float> wavelength;
        std::vector<unsigned> illuminant;

        std::unordered_map<std::string, std::string> additional;

        static MetaENVI load(const std::string &path);
    };

}

#endif