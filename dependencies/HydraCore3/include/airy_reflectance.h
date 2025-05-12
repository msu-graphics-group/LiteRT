#pragma once
#include "cglobals.h"
#include "crandom.h"
#include "cmaterial.h"

// I - income medium
// F - film layer
// T - outcome medium
static inline float FrFilmRefl(float cosThetaI, complex etaI, complex etaF, complex etaT, float thickness, float lambda)
{
  complex sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinThetaF = sinThetaI * (etaI.re * etaI.re) / (etaF * etaF);
  complex cosThetaF = complex_sqrt(1.0f - sinThetaF);
  complex sinThetaT = sinThetaI * (etaI.re * etaI.re) / (etaT * etaT);
  complex cosThetaT = complex_sqrt(1.0f - sinThetaT);
  
  complex phaseDiff = filmPhaseDiff(cosThetaF, etaF, thickness, lambda);

  float result = 0;
  uint polarization[2] = {PolarizationS, PolarizationP};
  for (int p = 0; p <= 1; ++p)
  {
    complex FrReflI = FrComplexRefl(cosThetaI, cosThetaF, etaI, etaF, polarization[p]);
    complex FrReflF = FrComplexRefl(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex FrRefl  = FrReflF * std::exp(-phaseDiff.im) * complex(std::cos(phaseDiff.re), std::sin(phaseDiff.re));
    FrRefl          = (FrReflI + FrRefl) / (1 + FrReflI * FrRefl);
    result += complex_norm(FrRefl);
  }

  return result / 2;
}

static inline float FrFilmRefr(float cosThetaI, complex etaI, complex etaF, complex etaT, float thickness, float lambda)
{
  complex sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinThetaF = sinThetaI * (etaI.re * etaI.re) / (etaF * etaF);
  complex cosThetaF = complex_sqrt(1.0f - sinThetaF);
  complex sinThetaT = sinThetaI * (etaI.re * etaI.re) / (etaT * etaT);
  complex cosThetaT = complex_sqrt(1.0f - sinThetaT);
  
  complex phaseDiff = filmPhaseDiff(cosThetaF, etaF, thickness, lambda);

  float result = 0;
  uint polarization[2] = {PolarizationS, PolarizationP};
  for (int p = 0; p <= 1; ++p)
  {
    complex FrReflI = FrComplexRefl(cosThetaI, cosThetaF, etaI, etaF, polarization[p]);
    complex FrReflF = FrComplexRefl(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex FrRefrI = FrComplexRefr(cosThetaI, cosThetaF, etaI, etaF, polarization[p]);
    complex FrRefrF = FrComplexRefr(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex nom    = FrRefrI * FrRefrF * std::exp(-phaseDiff.im / 2) * complex(std::cos(phaseDiff.re / 2), std::sin(phaseDiff.re / 2));
    complex denom = 1 + FrReflI * FrReflF * std::exp(-phaseDiff.im) * complex(std::cos(phaseDiff.re), std::sin(phaseDiff.re));
    result += complex_norm(nom / denom);
  }
  result *= getRefractionFactor(cosThetaI, cosThetaT, etaI, etaT);
  return result / 2;
}

static inline FrReflRefr FrFilm(float cosThetaI, complex etaI, complex etaF, complex etaT, float thickness, float lambda)
{
  complex sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinThetaF = sinThetaI * (etaI.re * etaI.re) / (etaF * etaF);
  complex cosThetaF = complex_sqrt(1.0f - sinThetaF);
  complex sinThetaT = sinThetaI * (etaI.re * etaI.re) / (etaT * etaT);
  complex cosThetaT = complex_sqrt(1.0f - sinThetaT);

  complex phaseDiff = filmPhaseDiff(cosThetaF, etaF, thickness, lambda);

  FrReflRefr result = {0, 0};
  uint polarization[2] = {PolarizationS, PolarizationP};
  for (int p = 0; p <= 1; ++p)
  {
    complex FrReflI = FrComplexRefl(cosThetaI, cosThetaF, etaI, etaF, polarization[p]);
    complex FrReflF = FrComplexRefl(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex FrRefrI = FrComplexRefr(cosThetaI, cosThetaF, etaI, etaF, polarization[p]);
    complex FrRefrF = FrComplexRefr(cosThetaF, cosThetaT, etaF, etaT, polarization[p]);

    complex exp_1 = std::exp(-phaseDiff.im / 2) * complex(std::cos(phaseDiff.re / 2), std::sin(phaseDiff.re / 2));
    complex exp_2 = exp_1 * exp_1;

    complex denom = 1 + FrReflI * FrReflF * exp_2;
    if (complex_norm(denom) < 1e-6f)
    {
      result.refl += 0.5f;
    }
    else
    {
      result.refl += complex_norm((FrReflI + FrReflF * exp_2) / denom) / 2;
      result.refr += complex_norm(FrRefrI * FrRefrF * exp_1 / denom) / 2;
    }

  }
  result.refr *= getRefractionFactor(cosThetaI, cosThetaT, etaI, etaT);

  return result;
}

static inline FrReflRefr calculateMultFrFilmForward(const complex *a_cosTheta, const complex *a_ior, const complex *a_phaseDiff, const uint layers, const uint p)
{
  complex FrRefl = FrComplexRefl(a_cosTheta[layers - 1], a_cosTheta[layers], a_ior[layers - 1], a_ior[layers], p);
  complex FrRefr = FrComplexRefr(a_cosTheta[layers - 1], a_cosTheta[layers], a_ior[layers - 1], a_ior[layers], p);
  for (int i = layers - 2; i >= 0; --i)
  {
    complex FrReflI = FrComplexRefl(a_cosTheta[i], a_cosTheta[i + 1], a_ior[i], a_ior[i + 1], p);
    complex FrRefrI = FrComplexRefr(a_cosTheta[i], a_cosTheta[i + 1], a_ior[i], a_ior[i + 1], p);
    complex exp_1 = std::exp(-a_phaseDiff[i].im / 2.f) * complex(std::cos(a_phaseDiff[i].re / 2.f), std::sin(a_phaseDiff[i].re / 2.f));

    FrRefr = FrRefrI * FrRefr * exp_1;
    FrRefl = FrRefl * exp_1 * exp_1;

    complex denom = 1 + FrReflI * FrRefl;
    
    if (complex_norm(denom) < 1e-6f)
    {
      FrRefr = 0.0f;
      FrRefl = 1.0f;
    }
    else
    {
      FrRefr = FrRefr / denom;
      FrRefl = (FrReflI + FrRefl) / denom;
    }
  }
  return {complex_norm(FrRefl), complex_norm(FrRefr)};
}

static inline FrReflRefr calculateMultFrFilmBackward(const complex *a_cosTheta, const complex *a_ior, const complex *a_phaseDiff, const uint layers, const uint p)
{
  complex FrRefl = FrComplexRefl(a_cosTheta[1], a_cosTheta[0], a_ior[1], a_ior[0], p);
  complex FrRefr = FrComplexRefr(a_cosTheta[1], a_cosTheta[0], a_ior[1], a_ior[0], p);
  for (int i = 1; i < layers; ++i)
  {
    complex FrReflI = FrComplexRefl(a_cosTheta[i + 1], a_cosTheta[i], a_ior[i + 1], a_ior[i], p);
    complex FrRefrI = FrComplexRefr(a_cosTheta[i + 1], a_cosTheta[i], a_ior[i + 1], a_ior[i], p);
    complex exp_1 = std::exp(-a_phaseDiff[i - 1].im / 2.f) * complex(std::cos(a_phaseDiff[i - 1].re / 2.f), std::sin(a_phaseDiff[i - 1].re / 2.f));
    FrRefr = FrRefrI * FrRefr * exp_1;
    FrRefl = FrRefl * exp_1 * exp_1;

    complex denom = 1 + FrReflI * FrRefl;
    
    if (complex_norm(denom) < 1e-6f)
    {
      FrRefr = 0.0f;
      FrRefl = 1.0f;
    }
    else
    {
      FrRefr = FrRefr / denom;
      FrRefl = (FrReflI + FrRefl) / denom;
    }
  }
  return {complex_norm(FrRefl), complex_norm(FrRefr)};
}

static inline FrReflRefr multFrFilm(float cosThetaI, const complex* a_ior, const float* thickness, uint layers, float lambda)
{
  complex a_cosTheta[layers + 1];
  complex a_phaseDiff[layers - 1];
  a_cosTheta[0] = complex(cosThetaI);

  float sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinTheta;
  complex cosTheta;
  for (int i = 1; i <= layers; ++i)
  {
    sinTheta = sinThetaI * a_ior[0].re * a_ior[0].re / (a_ior[i] * a_ior[i]);
    cosTheta = complex_sqrt(1.0f - sinTheta);
    a_cosTheta[i] = cosTheta;
    if (i < layers)
      a_phaseDiff[i - 1] = filmPhaseDiff(cosTheta, a_ior[i], thickness[i - 1], lambda);
  }

  FrReflRefr result_P = calculateMultFrFilmForward(a_cosTheta, a_ior, a_phaseDiff, layers, PolarizationP);
  FrReflRefr result_S = calculateMultFrFilmForward(a_cosTheta, a_ior, a_phaseDiff, layers, PolarizationS);
  FrReflRefr result = {(result_P.refl + result_S.refl) / 2.f, (result_P.refr + result_S.refr) / 2.f};
  result.refr *= getRefractionFactor(cosThetaI, a_cosTheta[layers], a_ior[0], a_ior[layers]);

  return result;
}

static inline FrReflRefr multFrFilm_r(float cosThetaI, const complex* a_ior, const float* thickness, uint layers, float lambda)
{
  complex a_cosTheta[layers + 1];
  complex a_phaseDiff[layers - 1];
  a_cosTheta[layers] = complex(cosThetaI);

  float sinThetaI = 1.0f - cosThetaI * cosThetaI;
  complex sinTheta = complex(1.0);
  complex cosTheta = complex(1.0);
  for (int i = layers - 1; i >= 0; --i)
  {
    sinTheta = sinThetaI * a_ior[layers].re * a_ior[layers].re / (a_ior[i] * a_ior[i]);
    cosTheta = complex_sqrt(1.0f - sinTheta);
    a_cosTheta[i] = cosTheta;
    if (i > 0)
      a_phaseDiff[i - 1] = filmPhaseDiff(cosTheta, a_ior[i], thickness[i - 1], lambda);
  }

  FrReflRefr result_P = calculateMultFrFilmBackward(a_cosTheta, a_ior, a_phaseDiff, layers, PolarizationP);
  FrReflRefr result_S = calculateMultFrFilmBackward(a_cosTheta, a_ior, a_phaseDiff, layers, PolarizationS);
  FrReflRefr result = {(result_P.refl + result_S.refl) / 2.f, (result_P.refr + result_S.refr) / 2.f};
  result.refr *= getRefractionFactor(cosThetaI, a_cosTheta[0], a_ior[layers], a_ior[0]);
  
  return result;
}
