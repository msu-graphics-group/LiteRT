#include <spec/sigpoly_lut.h>
#include <internal/math/math.h>
#include <internal/serialization/binary.h>
#include <cinttypes>
#include <stdexcept>

namespace spec {

    namespace {

        inline bool validate_c(int a, int b, int c) {
            return a >= 0 && a <= 255 && b >= 0 && b <= 255 && c >= 0 && c <= 255;
        }

        inline int safe_int(int i, int size)
        {
            return i >= size ? size - 1 : i;
        }

        bool validate_header(std::istream &src)
        {
            uint64_t marker = binary::read<uint64_t>(src);
            if(!src) return false;
            uint16_t floatsize = binary::read<uint16_t>(src);
            if(!src) return false;

            return marker == SigpolyLUT::FILE_MARKER && floatsize == sizeof(Float);
        }
    }

    Float SigpolyLUT::aid_to_alpha(int alpha_id) const
    {
        return math::smoothstep2(alpha_id / static_cast<Float>(size - 1));
    }

    vec3 SigpolyLUT::eval(int a, int b, int alpha) const
    {   
        if(!validate_c(a, b, alpha)) {
            return {};
        }

        const int a1_id = a / step;
        const int a2_id = safe_int(a1_id + 1, size);
        const int b1_id = b / step;
        const int b2_id = safe_int(b1_id + 1, size);

        const int a1 = safe_int(a1_id * step, 256);
        const int a2 = a == 255 ? 256 : safe_int(a2_id * step, 256);
        const int b1 = safe_int(b1_id * step, 256);
        const int b2 = b == 255 ? 256 : safe_int(b2_id * step, 256);


        const Float alphaf = alpha / 255.0f;
        const unsigned alpha1_id = static_cast<int>((size - 1) * math::inv_smoothstep2(alphaf)); 
        const unsigned alpha2_id = alpha1_id == size - 1 ? size - 1 : alpha1_id + 1;
        const Float alphaf1 = aid_to_alpha(alpha1_id);
        const Float alphaf2 = alpha1_id == alpha2_id ? 2.0f : aid_to_alpha(alpha2_id);

        const Float daf = (a2 - a1) / 255.0f;
        const Float daf1 = (a - a1) / 255.0f;
        const Float daf2 = (a2 - a) / 255.0f;

        const Float dbf = (b2 - b1) / 255.0f;
        const Float dbf1 = (b - b1) / 255.0f;
        const Float dbf2 = (b2 - b) / 255.0f; 

        const Float dalphaf = alphaf2 - alphaf1;
        const Float dalphaf1 = alphaf - alphaf1;
        const Float dalphaf2 = alphaf2 - alphaf;

        const Float t = daf * dbf * dalphaf;
        const Float div = t > 0 ? (1.0f / t) : 1.0f;
        
        return at(a1_id, b1_id, alpha1_id) * daf2 * dbf2 * dalphaf2 * div
             + at(a1_id, b1_id, alpha2_id) * daf2 * dbf2 * dalphaf1 * div
             + at(a1_id, b2_id, alpha1_id) * daf2 * dbf1 * dalphaf2 * div
             + at(a1_id, b2_id, alpha2_id) * daf2 * dbf1 * dalphaf1 * div
             + at(a2_id, b1_id, alpha1_id) * daf1 * dbf2 * dalphaf2 * div
             + at(a2_id, b1_id, alpha2_id) * daf1 * dbf2 * dalphaf1 * div
             + at(a2_id, b2_id, alpha1_id) * daf1 * dbf1 * dalphaf2 * div
             + at(a2_id, b2_id, alpha2_id) * daf1 * dbf1 * dalphaf1 * div;
    }

    SigpolyLUT SigpolyLUT::load_from(std::istream &src)
    {
        if(!validate_header(src)) throw std::invalid_argument("Unsupported file");
        uint16_t step = binary::read<uint16_t>(src);
        SigpolyLUT lut{step};
        const unsigned size = lut.size * lut.size * lut.size;
        for(unsigned i = 0; i < size; ++i) {
            lut.data[i] = binary::read_vec<Float>(src);
        }
        return lut;
    }

}