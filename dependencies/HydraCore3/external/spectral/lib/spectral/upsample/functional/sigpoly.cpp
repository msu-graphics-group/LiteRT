#include <upsample/functional/sigpoly.h>

namespace spec::upsample {

    namespace {

        int argmax(const Pixel &p)
        {
            int m = p[0] >= p[1] ? 0 : 1;
            return p[m] >= p[2] ? m : 2;
        }

        void upsample_to(const Pixel &pixel, SigPolySpectrum &s, const SigpolyLUT lut[3])
        {
            int amax = argmax(pixel);
            int a = 0, b = 0;
            int alpha = pixel[amax];
            if(alpha != 0) {
                a = pixel[(amax + 1) % 3] / static_cast<Float>(alpha) * 255.0f;
                b = pixel[(amax + 2) % 3] / static_cast<Float>(alpha) * 255.0f;
            }
           // std::cout << "amax: " << amax << " a, b, alpha: " << a << " " << b << " " << alpha << std::endl;

            s = SigPolySpectrum(lut[amax].eval(a, b, alpha));
        }

    }

    SigPolySpectrum sigpoly_int(const Pixel &pixel, const SigpolyLUT lut[3])
    {
        SigPolySpectrum spectrum;
        upsample_to(pixel, spectrum, lut);
        return spectrum;
    }

}