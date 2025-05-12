#include <spec/fourier_lut.h>
#include <internal/serialization/binary.h>

#include <spec/spectral_util.h>
#include <spec/conversions.h>
#include <spec/fourier_spectrum.h>

#include <iostream>
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

            return marker == FourierLUT::FILE_MARKER && floatsize == sizeof(Float);
        }
    }


    std::vector<Float> FourierLUT::eval(int r, int g, int b, Float power) const
    {
         if(!validate_c(r, g, b)) {
            return {};
        }
        /*
        unsigned n = power_values.size() - 1;
        Float p_mul = 1.0f;
        for(unsigned i = 0; i < power_values.size(); ++i) {
            if(power_values[i] > power) {
                n = i - 1;
                break;
            }
        }
        p_mul = power / power_values[n];*/
        unsigned n = 0;
        Float p_mul = power / power_values[0];

        const int r1_id = r / step;
        const int r2_id = safe_int(r1_id + 1, size);
        const int g1_id = g / step;
        const int g2_id = safe_int(g1_id + 1, size);
        const int b1_id = b / step;
        const int b2_id = safe_int(b1_id + 1, size);

        const int r1 = safe_int(r1_id * step, 256);
        const int r2 = r == 255 ? 256 : safe_int(r2_id * step, 256);
        const int g1 = safe_int(g1_id * step, 256);
        const int g2 = g == 255 ? 256 : safe_int(g2_id * step, 256);
        const int b1 = safe_int(b1_id * step, 256);
        const int b2 = b == 255 ? 256 : safe_int(b2_id * step, 256);

        std::cout << size << " " << r1 << " " << r2 << std::endl;

        const Float drf  = Float(r2 - r1) / 255.0f;
        const Float drf1 = Float(r - r1) / 255.0f;
        const Float drf2 = Float(r2 - r) / 255.0f;

        const Float dgf  = Float(g2 - g1) / 255.0f;
        const Float dgf1 = Float(g - g1) / 255.0f;
        const Float dgf2 = Float(g2 - g) / 255.0f;

        const Float dbf  = Float(b2 - b1) / 255.0f;
        const Float dbf1 = Float(b - b1) / 255.0f;
        const Float dbf2 = Float(b2 - b) / 255.0f;

        Float t = drf * dgf * dbf;
        const Float div = t > 0.0f ? (1.0f / t) : 1.0f;

        std::vector<Float> res(m + 1, 0.0f);

        add(res, r1_id, g1_id, b1_id, n, drf2 * dgf2 * dbf2 * div * p_mul);
        add(res, r1_id, g1_id, b2_id, n, drf2 * dgf2 * dbf1 * div * p_mul);
        add(res, r1_id, g2_id, b1_id, n, drf2 * dgf1 * dbf2 * div * p_mul);
        add(res, r1_id, g2_id, b2_id, n, drf2 * dgf1 * dbf1 * div * p_mul);
        add(res, r2_id, g1_id, b1_id, n, drf1 * dgf2 * dbf2 * div * p_mul);
        add(res, r2_id, g1_id, b2_id, n, drf1 * dgf2 * dbf1 * div * p_mul);
        add(res, r2_id, g2_id, b1_id, n, drf1 * dgf1 * dbf2 * div * p_mul);
        add(res, r2_id, g2_id, b2_id, n, drf1 * dgf1 * dbf1 * div * p_mul);

//        for(unsigned i = 0; i <= m; ++i) std::cout << res[i] << ",";
//        std::cout << std::endl;


        for(Float f : res) {
            if(std::isnan(f)) std::cout << "NAN" << std::endl;
        }

        return res;
    }

    FourierLUT FourierLUT::load_from(std::istream &src)
    {
        if(!validate_header(src)) throw std::invalid_argument("Unsupported file");
        uint16_t step = binary::read<uint16_t>(src);
        uint16_t m = binary::read<uint16_t>(src);

        uint16_t p_size = binary::read<uint16_t>(src);
        std::vector<Float> power_values(p_size);
        for(unsigned i = 0; i < p_size; ++i) {
            power_values[i] = binary::read<Float>(src);
        }

        FourierLUT lut{power_values, step, m};
        const unsigned size = power_values.size() * lut.size * lut.size * lut.size * (m + 1);
        for(unsigned i = 0; i < size; ++i) {
            lut.data[i] = binary::read<Float>(src);
        }
        return lut;
    }

    void FourierLUT::add(std::vector<Float> &res, unsigned r, unsigned g, unsigned b, unsigned n, Float mul) const
    {
        unsigned offset = (((n * size + r) * size + g) * size + b) * (m + 1);
        for(unsigned i = 0; i <= m; ++i) {
            res[i] += data[offset + i] * mul;
        }
      /*  std::cout << "ADDING: " << r * step << " " << g * step << " " << b * step << " with ";
        //for(unsigned i = 0; i <= m; ++i) std::cout << res[i] << ",";
        //std::cout << std::endl;

        std::vector<Float> coef(m + 1);
        std::copy(data.begin() + offset, data.begin() + offset + m + 1, coef.data());
        FourierEmissionSpectrum spec{std::move(coef)};
        std::cout << util::get_cie_y_integral(spec) << " " << xyz2rgb_unsafe(spectre2xyz0(spec)) << std::endl;*/
    }
}