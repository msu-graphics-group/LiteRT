#ifndef INCLUDE_SPECTAL_INTERNAL_MATH_LEVINSON_H
#define INCLUDE_SPECTAL_INTERNAL_MATH_LEVINSON_H
#include <vector>
#include <cassert>
namespace spec::math {

    template<typename T, typename P>
    std::vector<T> levinson(const std::vector<T> &data, const std::vector<P> &y)
    {
        const int N = (data.size() - 1) / 2;

        auto t = T(1.0) / data[N];
        std::vector<T> fn{t};
        std::vector<T> bn{t};
        std::vector<T> xn(N + 1);
        xn[0] = y[0] * t;

        for(int i = 1; i <= N; ++i) {
            T ef1{0.0};
            T eb1{0.0};
            T ex1{0.0};

            for(int j = 0; j < i; ++j) { //check bounds
                ef1 += data[N + i - j] * fn[j];
                ex1 += data[N + i - j] * xn[j];
                eb1 += data[N - j - 1] * bn[j];
            }
            std::vector<T> fn1(i + 1, T(0.0));
            std::vector<T> bn1(i + 1, T(0.0));

            T div = T(1.0) / (T(1.0) - ef1 * eb1);
            for(int j = 0; j < i; ++j) {
                fn1[j] = div * fn[j];
                bn1[j] = -eb1 * div * fn[j];
            }
            for(int j = 1; j <= i; ++j) {
                fn1[j] -= ef1 * div * bn[j - 1];
                bn1[j] += div * bn[j - 1];
            }

            T mul = y[i] - ex1;
            for(int j = 0; j < i; ++j) {
                xn[j] += mul * bn1[j];
            }
            xn[i] = mul * bn1[i];

            fn = std::move(fn1);
            bn = std::move(bn1);
        }
        return xn;
    }

}


#endif