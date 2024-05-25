#pragma once

#include <vector>
#include "LiteMath/LiteMath.h"
#include <algorithm>

namespace solver
{
    void find_interval(const float* coefs, const float& x1, const float& x2, const int& n, float interval[2]);
    //  Count polinom value
    float f(const float* coefs, const float& x, const int& n);
    //  Count polinom first derrivative value
    float df(const float* coefs, const float& x, const int& n);
    //  Count polinom second derrivative value
    float ddf(const float* coefs, const float& x, const int& n);
    void polinomMiltiplier(const float* p1, const float* p2, float* res, const int& n1, const int& n2, const float& polinom_coef);
    //  before call this func need to set Ray coords from world position to voxel space
    void coefsDecrease(const std::vector<float>& coefs, const LiteMath::float3& P, const LiteMath::float3& D, float* res);
    
    //  Newton-Raphson Save Method
    float nr_solver(const float* coefs, const float& x1, const float& x2, const float& acc);
    //  Halley’s Method
    float halley_solver(const float* coefs, const float& x1, const float& x2, const float& acc);
    //  Bisection Method
    float bisection_solver(const float* coefs, const float& x1, const float& x2, const float& acc);
    //  Brent's Method
    float brent_solver(const float* coefs, const float& x1, const float& x2, const float& acc);

    //  For test
    float calc_test_res(const std::vector<float>& coefs, const LiteMath::float3& P, const LiteMath::float3& D, const float& t);
};
