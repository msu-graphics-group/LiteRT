#include <spec/metrics.h>
#include <spec/conversions.h>
#include <internal/common/constants.h>
#include <cassert>

namespace spec::metrics {

//------COLORS-------
    Float cie_delta_e(const vec3 &y1, const vec3 &y2)
    {
        return vec3::distance(rgb2cielab(y1), rgb2cielab(y2));
    }

//------SPECTRA------
    Float mae(const ISpectrum &y1, const ISpectrum &y2, const std::vector<Float> &wavelenghts)
    {
        Float res = 0.0f;
        for(Float wl : wavelenghts) {
            res += std::fabs(y1(wl) - y2(wl));
        }
        return res / Float((WAVELENGHTS_END - WAVELENGHTS_START) / WAVELENGHTS_STEP);
    }

    Float sam(const ISpectrum &y1, const ISpectrum &y2, const std::vector<Float> &wavelenghts)
    {
        Float dist1 = 0.0f;
        Float dist2 = 0.0f;
        Float dot = 0.0f;
        for(Float wl : wavelenghts)
        {
            const Float val1 = y1(wl);
            dist1 += val1 * val1;
            const Float val2 = y2(wl);
            dist2 += val2 * val2;
            dot += val1 * val2;
        }

        if(dist1 == 0.0f || dist2 == 0.0f) {
            for(Float wl : wavelenghts)
            {
                if(y1(wl) != y2(wl)) {
                    return math::PI / 2.0f;
                }
            }
            return 0.0f;
        }

        const Float mdist = std::sqrt(dist1) * std::sqrt(dist2);
        Float cosine_sim = dot / mdist;

        return std::acos(math::clamp(cosine_sim, 0.0f, 1.0f));
    }
    
//------IMAGES------
    Float mse(const Image &y1, const Image &y2)
    {
        assert(y1.get_width() == y2.get_width() && y1.get_height() == y2.get_height());

        Float res = 0.0f;
        const Pixel *raw1 = y1.raw_data();
        const Pixel *raw2 = y2.raw_data();
        const unsigned sz = y1.get_width() * y1.get_height();
        for(unsigned i = 0; i < sz; ++i) {
            res += vec3::distance2(raw1[i].to_vec3(), raw2[i].to_vec3()); 
        }
        return res / sz;
    }

    Float psnr(const Image &y1, const Image &y2)
    {
        return 10.0f * std::log10(3.0f / mse(y1, y2));
    }

    Float ssim(const Image &y1, const Image &y2)
    {
        (void) y1; (void) y2;
        return 1.0f;//TODO
    }

}