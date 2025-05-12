#include "integrator_dr.h"

#include "include/cmaterial.h"
#include "include/cmat_gltf.h"
#include "include/cmat_conductor.h"
#include "include/cmat_glass.h"
#include "include/cmat_diffuse.h"
#include "include/cmat_plastic.h"
#include "include/cmat_dielectric.h"

#include <chrono>
#include <string>
#include <omp.h>

#include "utils.h"
#include "imageutils.h"

#include "Image2d.h"
using LiteImage::Image2D;
using LiteImage::Sampler;
using LiteImage::ICombinedImageSampler;
using namespace LiteMath;

void IntegratorDR::LoadSceneEnd()
{
  m_texAddressTable.resize(m_textures.size());
  for(auto& texInfo : m_texAddressTable)
    texInfo = {size_t(-1),0,0,0,0};

  m_gradSize = 0;
}

std::pair<size_t, size_t> IntegratorDR::PutDiffTex2D(uint32_t texId, uint32_t width, uint32_t height, uint32_t channels)
{
  if(texId >= m_texAddressTable.size())
  {
    std::cout << "[IntegratorDR::PutDiffTex2D]: bad tex id = " << texId << std::endl;
    return std::make_pair(size_t(-1), 0);
  }

  m_texAddressTable[texId].offset   = m_gradSize;
  m_texAddressTable[texId].width    = width;
  m_texAddressTable[texId].height   = height;
  m_texAddressTable[texId].channels = channels;
  m_texAddressTable[texId].fwidth   = float(width);
  m_texAddressTable[texId].fheight  = float(height);

  size_t oldOffset = m_gradSize;
  size_t currSize  = size_t(width)*size_t(height)*size_t(channels);
  
  m_gradSize += currSize;
  return std::make_pair(oldOffset, currSize);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

static inline int4 bilinearOffsets(const float ffx, const float ffy, const int w, const int h)
{
	const int sx = (ffx > 0.0f) ? 1 : -1;
	const int sy = (ffy > 0.0f) ? 1 : -1;

	const int px = (int)(ffx);
	const int py = (int)(ffy);

	int px_w0, px_w1, py_w0, py_w1;
  // wrap
	{
		px_w0 = px        % w;
		px_w1 = (px + sx) % w;

		px_w0 = (px_w0 < 0) ? px_w0 + w : px_w0;
		px_w1 = (px_w1 < 0) ? px_w1 + w : px_w1;
	}

  // wrap
	{
		py_w0 = py        % h;
		py_w1 = (py + sy) % h;

		py_w0 = (py_w0 < 0) ? py_w0 + h : py_w0;
		py_w1 = (py_w1 < 0) ? py_w1 + h : py_w1;
	}

	const int offset0 = py_w0*w + px_w0;
	const int offset1 = py_w0*w + px_w1;
	const int offset2 = py_w1*w + px_w0;
	const int offset3 = py_w1*w + px_w1;

	return int4(offset0, offset1, offset2, offset3);
}

float4 IntegratorDR::Tex2DFetchAD(uint texId, float2 a_uv, const float* tex_data)
{
  const auto info = m_texAddressTable[texId];

  if(info.offset != size_t(-1) && tex_data != nullptr && m_gradMode != 0) 
  {
    const float m_fw     = info.fwidth;
    const float m_fh     = info.fheight;
    const int tex_width  = info.width;
    const int tex_height = info.height;
    
    float ffx = a_uv.x * m_fw - 0.5f; // a_texCoord should not be very large, so that the float does not overflow later. 
    float ffy = a_uv.y * m_fh - 0.5f; // This is left to the responsibility of the top level.
    
    auto sampler = m_textures[texId]->sampler();

    if ((sampler.addressU == Sampler::AddressMode::CLAMP) != 0 && ffx < 0) ffx = 0.0f;
    if ((sampler.addressV == Sampler::AddressMode::CLAMP) != 0 && ffy < 0) ffy = 0.0f;
    
    // Calculate the weights for each pixel
    //
    const int   px = (int)(ffx);
    const int   py = (int)(ffy);
    
    const float fx  = std::abs(ffx - (float)px);
    const float fy  = std::abs(ffy - (float)py);
    const float fx1 = 1.0f - fx;
    const float fy1 = 1.0f - fy;
    
    const float w1 = fx1 * fy1;
    const float w2 = fx  * fy1;
    const float w3 = fx1 * fy;
    const float w4 = fx  * fy;
    
    const int4 offsets = bilinearOffsets(ffx, ffy, tex_width, tex_height);

    // Calculate the weighted sum of pixels (for each color channel)
    //
    if(info.channels == 4)
    {
      const float4 f1    = float4(tex_data[info.offset+offsets.x*4+0], tex_data[info.offset+offsets.x*4+1], tex_data[info.offset+offsets.x*4+2], tex_data[info.offset+offsets.x*4+3]);
      const float4 f2    = float4(tex_data[info.offset+offsets.y*4+0], tex_data[info.offset+offsets.y*4+1], tex_data[info.offset+offsets.y*4+2], tex_data[info.offset+offsets.y*4+3]);
      const float4 f3    = float4(tex_data[info.offset+offsets.z*4+0], tex_data[info.offset+offsets.z*4+1], tex_data[info.offset+offsets.z*4+2], tex_data[info.offset+offsets.z*4+3]);
      const float4 f4    = float4(tex_data[info.offset+offsets.w*4+0], tex_data[info.offset+offsets.w*4+1], tex_data[info.offset+offsets.w*4+2], tex_data[info.offset+offsets.w*4+3]);
  
      const float outr = f1.x * w1 + f2.x * w2 + f3.x * w3 + f4.x * w4;
      const float outg = f1.y * w1 + f2.y * w2 + f3.y * w3 + f4.y * w4;
      const float outb = f1.z * w1 + f2.z * w2 + f3.z * w3 + f4.z * w4;
      const float outa = f1.w * w1 + f2.w * w2 + f3.w * w3 + f4.w * w4;
      
      return float4(outr, outg, outb, outa);
    }
    else
    {
      const float f1 = tex_data[info.offset+offsets.x];
      const float f2 = tex_data[info.offset+offsets.y];
      const float f3 = tex_data[info.offset+offsets.z];
      const float f4 = tex_data[info.offset+offsets.w];
  
      const float outVal = f1 * w1 + f2 * w2 + f3 * w3 + f4 * w4;
      
      return float4(outVal, outVal, outVal, outVal);
    }
  }
  else
    return m_textures[texId]->sample(a_uv);
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void IntegratorDR::kernel_InitEyeRay(uint tid, const uint* packedXY, float4* rayPosAndNear, float4* rayDirAndFar, const float* a_data) // (tid,tidX,tidY,tidZ) are SPECIAL PREDEFINED NAMES!!!
{
  if(tid >= m_maxThreadId)
    return;
  const uint XY = packedXY[tid];

  const uint x = (XY & 0x0000FFFF);
  const uint y = (XY & 0xFFFF0000) >> 16;

  const float xCoordNormalized = (float(x + m_winStartX) + 0.5f)/float(m_fbWidth);
  const float yCoordNormalized = (float(y + m_winStartY) + 0.5f)/float(m_fbHeight);

  float3 rayDir = EyeRayDirNormalized(xCoordNormalized, yCoordNormalized, m_projInv);
  float3 rayPos = float3(0,0,0);

  transform_ray3f(m_worldViewInv, 
                  &rayPos, &rayDir);
  
  *rayPosAndNear = to_float4(rayPos, 0.0f);
  *rayDirAndFar  = to_float4(rayDir, FLT_MAX);
}

bool IntegratorDR::kernel_RayTrace(uint tid, const float4* rayPosAndNear, float4* rayDirAndFar,
                                   Lite_Hit* out_hit, float2* out_bars, const float* a_data)
{
  if(tid >= m_maxThreadId)
    return false;
  const float4 rayPos = *rayPosAndNear;
  const float4 rayDir = *rayDirAndFar ;

  CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHit(rayPos, rayDir);
  
  Lite_Hit res;
  res.primId = hit.primId;
  res.instId = hit.instId;
  res.geomId = hit.geomId;
  res.t      = hit.t;

  float2 baricentrics = float2(hit.coords[0], hit.coords[1]);
 
  *out_hit  = res;
  *out_bars = baricentrics;
  return (res.primId != -1);
}


void IntegratorDR::kernel_CalcRayColor(uint tid, const Lite_Hit* in_hit, const float2* bars, float4* finalColor, const uint* in_pakedXY, float* out_color, const float* a_data)
{ 
  if(tid >= m_maxThreadId)
    return;

  const Lite_Hit hit = *in_hit;
  if(hit.geomId == -1)
  {
    out_color[tid] = 0;
    return;
  }

  const uint32_t matId  = m_matIdByPrimId[m_matIdOffsets[hit.geomId] + hit.primId];
  const float4 mdata    = m_materials[matId].colors[GLTF_COLOR_BASE];
  const float2 uv       = *bars;

  const uint triOffset  = m_matIdOffsets[hit.geomId];
  const uint vertOffset = m_vertOffset  [hit.geomId];

  const uint A = m_triIndices[(triOffset + hit.primId)*3 + 0];
  const uint B = m_triIndices[(triOffset + hit.primId)*3 + 1];
  const uint C = m_triIndices[(triOffset + hit.primId)*3 + 2];
  const float4 data1 = (1.0f - uv.x - uv.y)*m_vNorm4f[A + vertOffset] + uv.y*m_vNorm4f[B + vertOffset] + uv.x*m_vNorm4f[C + vertOffset];
  const float4 data2 = (1.0f - uv.x - uv.y)*m_vTang4f[A + vertOffset] + uv.y*m_vTang4f[B + vertOffset] + uv.x*m_vTang4f[C + vertOffset];
  float3 hitNorm     = to_float3(data1);
  float3 hitTang     = to_float3(data2);
  float2 hitTexCoord = float2(data1.w, data2.w);

  const uint   texId     = m_materials[matId].texid[0];
  const float2 texCoordT = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], hitTexCoord);
  const float4 texColor  = Tex2DFetchAD(texId, texCoordT, a_data); 
  const float3 color     = mdata.w > 0.0f ? clamp(float3(mdata.w,mdata.w,mdata.w), 0.0f, 1.0f) : to_float3(mdata*texColor);

  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  (*finalColor) = to_float4(color, 0);

  out_color[(y*m_winWidth+x)*4 + 0] = color.x;
  out_color[(y*m_winWidth+x)*4 + 1] = color.y;
  out_color[(y*m_winWidth+x)*4 + 2] = color.z;
  out_color[(y*m_winWidth+x)*4 + 3] = 0.0f;
}


float4 IntegratorDR::CastRayDR(uint tid, uint channels, float* out_color, const float* a_data)
{
  float4 rayPosAndNear, rayDirAndFar;
  kernel_InitEyeRay(tid, m_packedXY.data(), &rayPosAndNear, &rayDirAndFar, a_data);

  Lite_Hit hit; 
  float2   baricentrics; 
  if(!kernel_RayTrace(tid, &rayPosAndNear, &rayDirAndFar, &hit, &baricentrics, a_data))
    return float4(0,0,0,0);
  
  float4 finalColor;
  kernel_CalcRayColor(tid, &hit, &baricentrics, &finalColor, m_packedXY.data(), out_color, a_data);
  return finalColor;
}

extern float __enzyme_autodiff(void*, ...);
int enzyme_const, enzyme_dup, enzyme_out;


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

double RegLossImage1D(size_t a_size, const float* data)
{
  double summ = 0.0f;

  for(size_t i=1;i<a_size-1;i++) {
    float diffLeft  = data[i] - data[i-1];
    float diffRight = data[i] - data[i+1];
    summ += double(diffLeft*diffLeft + diffRight*diffRight);
  }

  return summ/double(a_size);
}

double RegLossImage2D(int w, int h, const float* data)
{
  double summ = 0.0f;
  std::vector<double> lines(h);

  for(int y=1;y<h-1;y++) {
    lines[y] = 0.0;
    for(int x=1;x<w-1;x++) {
      float diffTop    = data[y*w+x] - data[(y+1)*w+x];
      float diffBottom = data[y*w+x] - data[(y-1)*w+x];
      float diffLeft   = data[y*w+x] - data[y*w+x-1];
      float diffRight  = data[y*w+x] - data[y*w+x+1];
      lines[y] += std::sqrt(double(diffLeft*diffLeft + diffRight*diffRight + diffTop*diffTop + diffBottom*diffBottom));
    }
    summ += lines[y];
  }
 
  return summ;
}

using LiteMath::dot3;

double RegLossImage2D4f(int w, int h, const float* data)
{
  double summ = 0.0f;
  std::vector<double> lines(h);

  for(int y=1;y<h-1;y++) {
    lines[y] = 0.0;
    for(int x=1;x<w-1;x++) {
      float4 p0 = float4(data[(y*w+x)*4+0],     data[(y*w+x)*4+1],     data[(y*w+x)*4+2],     data[(y*w+x)*4+3]);
      float4 p1 = float4(data[((y+1)*w+x)*4+0], data[((y+1)*w+x)*4+1], data[((y+1)*w+x)*4+2], data[((y+1)*w+x)*4+3]);
      float4 p2 = float4(data[((y-1)*w+x)*4+0], data[((y-1)*w+x)*4+1], data[((y-1)*w+x)*4+2], data[((y-1)*w+x)*4+3]); 
      float4 p3 = float4(data[(y*w+x-1)*4+0], data[(y*w+x-1)*4+1], data[(y*w+x-1)*4+2], data[(y*w+x-1)*4+3]);
      float4 p4 = float4(data[(y*w+x+1)*4+0], data[(y*w+x+1)*4+1], data[(y*w+x+1)*4+2], data[(y*w+x+1)*4+3]);

      float4 diffTop    = p0 - p1;
      float4 diffBottom = p0 - p2;
      float4 diffLeft   = p0 - p3;
      float4 diffRight  = p0 - p4;

      lines[y] += std::sqrt(double(dot3(diffLeft,diffLeft) + dot3(diffRight,diffRight) + dot3(diffTop,diffTop) + dot3(diffBottom,diffBottom)));
    }
  }
  
  summ = 0.0;
  for(int sy=0; sy < h/2; sy++) {
    int index1 = h/2 + sy;
    int index2 = h/2 - sy;
    if(index1 < h-1)
      summ += lines[index1];
    if(sy!=0 && index2 >= 1)
      summ += lines[index2];
  }
 
  return summ; //double(w*h);
}

void Image1DRegularizer(size_t a_size, const float* data, float* grad)
{
  __enzyme_autodiff((void*)RegLossImage1D, 
                           enzyme_const, a_size,
                           enzyme_dup,   data, grad);

}

void Image2D4fRegularizer(int w, int h, const float* data, float* grad)
{
  __enzyme_autodiff((void*)RegLossImage2D4f, 
                           enzyme_const, w,
                           enzyme_const, h,
                           enzyme_dup,   data, grad);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

float PixelLossRT(IntegratorDR* __restrict__ pIntegrator,
                  const float*  __restrict__ a_refImg,
                        float*  __restrict__ out_color,
                  const float*  __restrict__ a_data, 
                  const uint*   __restrict__ in_pakedXY, 
                  uint tid, uint channels, uint pitch,
                  float*  __restrict__       outLoss)
{
  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  const uint yRef  = pIntegrator->m_winHeight - y - 1; // in input images and when load data from HDD y has different direction
  float4 colorRend = pIntegrator->CastRayDR(tid, channels, out_color, a_data);
  float4 colorRef  = float4(a_refImg[(yRef*pitch+x)*channels + 0], 
                            a_refImg[(yRef*pitch+x)*channels + 1], 
                            a_refImg[(yRef*pitch+x)*channels + 2], 0.0f);

  float4 diff = colorRend - colorRef;
  float loss = LiteMath::dot3(diff, diff);
  (*outLoss) = loss;
  return loss;
}                      

float IntegratorDR::RayTraceDR(uint tid, uint channels, float* out_color, uint a_passNum,
                                const float* a_refImg, const float* a_data, float* a_dataGrad, size_t a_gradSize)
{
  memset(a_dataGrad, 0, sizeof(float)*a_gradSize);

  // init separate gradient for each thread
  //
  //std::vector<float> grads[MAXTHREADS_CPU];
  //for(int i=0;i<MAXTHREADS_CPU;i++)
  //   std::fill(grads[i].begin(), grads[i].end(), 0.0f);

  //double avgLoss = 0.0;
  auto start = std::chrono::high_resolution_clock::now();
  //#ifndef _DEBUG
  //#pragma omp parallel for default(shared) // num_threads(MAXTHREADS_CPU)
  //#endif

  float avgLoss = 0.0f;
  
  if(m_gradMode != 0)
  {
    for (int i = 0; i < int(tid); ++i) {
      float lossVal = 0.0f;
      __enzyme_autodiff((void*)PixelLossRT, 
                         enzyme_const, this,
                         enzyme_const, a_refImg,
                         enzyme_const, out_color,
                         enzyme_dup,   a_data, a_dataGrad,
                         enzyme_const, m_packedXY.data(),
                         enzyme_const, uint(i),
                         enzyme_const, channels,
                         enzyme_const, m_winWidth,
                         enzyme_const, &lossVal);
      avgLoss += float(lossVal)/float(a_passNum);
    }
  }
  else
  {
    for (int i = 0; i < int(tid); ++i) {
      float lossVal = PixelLossRT(this, a_refImg, out_color, a_data, m_packedXY.data(),
                                uint(i), channels, m_winWidth, &lossVal);
      avgLoss += float(lossVal)/float(a_passNum);
    }
  }

  shadowPtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;

  // accumulate gradient from different threads (parallel reduction/hist)
  //

  //for(int i=0;i<MAXTHREADS_CPU;i++) 
  //  for(size_t j=0;j<a_gradSize; j++)
  //    a_dataGrad[j] += grads[i][j];

  //avgLoss /= float(m_winWidth*m_winHeight);
  //std::cout << "avgLoss = " << avgLoss << std::endl;
  
  //std::ofstream fout("z_grad.txt");
  //for(size_t i=0; i<a_gradSize; i++)
  //  fout << a_dataGrad[i]/float(a_passNum) << std::endl;
  //fout.close();

  return avgLoss;
}

BsdfSample IntegratorDR::MaterialSampleAndEval(uint a_materialId, uint tid, uint bounce, float4 wavelengths, float3 v, float3 n, float3 tan, float2 tc, 
                                               MisData* a_misPrev, const uint a_currRayFlags, const float* drands, const float* dparams)
{
  BsdfSample res;
  {
    res.val   = float4(0, 0, 0, 0);
    res.pdf   = 1.0f;
    res.dir   = float3(0,1,0);
    res.ior   = 1.0f;
    res.flags = a_currRayFlags;
    res.ior   = 1.0f;
  }

  const float* matRands  = drands + LENS_RANDS + bounce*RND_PER_BOUNCE + RND_MTL_ID;
  const float4 rands     = float4(matRands[0], matRands[1], matRands[2], matRands[3]); // GetRandomNumbersMats(tid, a_gen, int(bounce));

  uint32_t currMatId = a_materialId;
  uint     mtype     = m_materials[currMatId].mtype;
  uint     layer     = 0;
  
  // BSDF is multiplied (outside) by cosThetaOut1.
  // When normal map is enables this becames wrong because normal is changed;
  // First : return cosThetaOut in sam;
  // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
  //
  const uint normalMapId   = m_materials[currMatId].texid[1];
  const float3 geomNormal  = n;
        float3 shadeNormal = n;

  if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF)
    shadeNormal = BumpMapping(normalMapId, currMatId, geomNormal, tan, tc);

  const float2 texCoordT = mulRows2x4(m_materials[currMatId].row0[0], m_materials[currMatId].row1[0], tc);
  const uint   texId     = m_materials[currMatId].texid[0];
  const float4 texColor  = Tex2DFetchAD(texId, texCoordT, dparams); //m_textures[texId]->sample(texCoordT);
  const uint cflags      = m_materials[currMatId].cflags;

  float4 fourScalarMatParams = float4(1,1,1,1);

  switch(mtype)
  {
    case MAT_TYPE_GLTF:
    if(KSPEC_MAT_TYPE_GLTF != 0)
    {
      const float4 color = m_materials[currMatId].colors[GLTF_COLOR_BASE]*texColor;
      gltfSampleAndEval(m_materials.data() + currMatId, rands, v, shadeNormal, tc, color, fourScalarMatParams, &res);
    }
    break;
   
    default:
    break;
  }
  
  // BSDF is multiplied (outside) by cosThetaOut1.
  // When normal map is enables this becames wrong because normal is changed;
  // First : return cosThetaOut in sam;
  // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
  //
  if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF)
  {
    const float cosThetaOut1 = std::abs(dot(res.dir, geomNormal));
    const float cosThetaOut2 = std::abs(dot(res.dir, shadeNormal));
    res.val *= cosThetaOut2 / std::max(cosThetaOut1, 1e-10f);
  }

  return res;
}

BsdfEval IntegratorDR::MaterialEval(uint a_materialId, float4 wavelengths, float3 l, float3 v, float3 n, float3 tan, float2 tc, 
                                    const float* drands, const float* dparams)
{
  BsdfEval res;
  {
    res.val = float4(0,0,0,0);
    res.pdf   = 0.0f;
  }

  MatIdWeight currMat = make_id_weight(a_materialId, 1.0f);
  MatIdWeight material_stack[KSPEC_BLEND_STACK_SIZE];
  if(KSPEC_MAT_TYPE_BLEND != 0)
    material_stack[0] = currMat;
  int top = 0;
  bool needPop = false;

  //do
  //{
    //if(KSPEC_MAT_TYPE_BLEND != 0)
    //{
    //  if(needPop)
    //  {
    //    top--;
    //    currMat = material_stack[std::max(top, 0)];
    //  }
    //  else
    //    needPop = true; // if not blend, pop on next iter
    //} 
    
    // BSDF is multiplied (outside) by old cosThetaOut.
    // When normal map is enables this becames wrong because normal is changed;
    // First : return cosThetaOut in sam;
    // Second: apply cos(theta2)/cos(theta1) to cos(theta1) to get cos(theta2)
    //
    const float3 geomNormal = n;
          float3 shadeNormal = n;
    float bumpCosMult = 1.0f; 
    const uint normalMapId = m_materials[currMat.id].texid[1];
    if(KSPEC_BUMP_MAPPING != 0 && normalMapId != 0xFFFFFFFF) 
    {
      shadeNormal = BumpMapping(normalMapId, currMat.id, geomNormal, tan, tc);
      const float3 lDir     = l;     
      const float  clampVal = 1e-6f;  
      const float cosThetaOut1 = std::max(dot(lDir, geomNormal),  0.0f);
      const float cosThetaOut2 = std::max(dot(lDir, shadeNormal), 0.0f);
      bumpCosMult              = cosThetaOut2 / std::max(cosThetaOut1, clampVal);
      if (cosThetaOut1 <= 0.0f)
        bumpCosMult = 0.0f;
    }

    const float2 texCoordT = mulRows2x4(m_materials[currMat.id].row0[0], m_materials[currMat.id].row1[0], tc);
    const uint   texId     = m_materials[currMat.id].texid[0];
    const float4 texColor  = Tex2DFetchAD(texId, texCoordT, dparams); // m_textures[texId]->sample(texCoordT);
    const uint   mtype     = m_materials[currMat.id].mtype;
    const uint   cflags    = m_materials[currMat.id].cflags;

    float4 fourScalarMatParams = float4(1,1,1,1);

    BsdfEval currVal;
    {
      currVal.val = float4(0,0,0,0);
      currVal.pdf   = 0.0f;
    }
  
    switch(mtype)
    {
      case MAT_TYPE_GLTF:
      if(KSPEC_MAT_TYPE_GLTF != 0)
      {
        const float4 color     = (m_materials[currMat.id].colors[GLTF_COLOR_BASE]) * texColor;
        gltfEval(m_materials.data() + currMat.id, l, v, shadeNormal, tc, color, fourScalarMatParams, &currVal);
        res.val += currVal.val * currMat.weight * bumpCosMult;
        res.pdf += currVal.pdf * currMat.weight;
      }
      break;
      
      default:
        break;
    }

  //} while(KSPEC_MAT_TYPE_BLEND != 0 && top > 0);

  return res;
}

LightSample IntegratorDR::LightSampleRev(int a_lightId, float3 rands, float3 illiminationPoint, const float* drands, const float* dparams)
{
  const uint   gtype  = m_lights[a_lightId].geomType;
  const float2 rands2 = float2(rands.x, rands.y);
  switch(gtype)
  {
    case LIGHT_GEOM_DIRECT: return directLightSampleRev(m_lights.data() + a_lightId, rands2, illiminationPoint);
    case LIGHT_GEOM_SPHERE: return sphereLightSampleRev(m_lights.data() + a_lightId, rands2);
    case LIGHT_GEOM_POINT:  return pointLightSampleRev (m_lights.data() + a_lightId);
    case LIGHT_GEOM_ENV: 
    if(KSPEC_LIGHT_ENV != 0)
    {
      const uint32_t offset = m_lights[a_lightId].pdfTableOffset;
      const uint32_t sizeX  = m_lights[a_lightId].pdfTableSizeX;
      const uint32_t sizeY  = m_lights[a_lightId].pdfTableSizeY;
      
      const Map2DPiecewiseSample sam = SampleMap2D(rands, offset, int(sizeX), int(sizeY));

      // apply inverse texcoord transform to get phi and theta (SKY_DOME_INV_MATRIX0 in HydraCore2)
      //
      const float2 texCoordT = mulRows2x4(m_lights[a_lightId].samplerRow0Inv, m_lights[a_lightId].samplerRow1Inv, sam.texCoord);

      float sintheta = 0.0f;
      const float3 sampleDir = texCoord2DToSphereMap(texCoordT, &sintheta);
      const float3 samplePos = illiminationPoint + sampleDir*1000.0f; // TODO: add sceen bounding sphere radius here
      const float  samplePdf = (sam.mapPdf * 1.0f) / (2.f * M_PI * M_PI * std::max(std::abs(sintheta), 1e-20f)); // TODO: pass computed pdf to 'LightEvalPDF'
      
      LightSample res;
      res.hasIES = false;
      res.isOmni = true;
      res.norm   = sampleDir; 
      res.pos    = samplePos;
      res.pdf    = samplePdf; // evaluated here for environment lights 
      return res;
    }
    default:                return areaLightSampleRev  (m_lights.data() + a_lightId, rands2);
  };
}


float IntegratorDR::LightEvalPDF(int a_lightId, float3 illuminationPoint, float3 ray_dir, const float3 lpos, const float3 lnorm, float a_envPdf, const float* drands, const float* dparams)
{
  const uint gtype = m_lights[a_lightId].geomType;
  if(gtype == LIGHT_GEOM_ENV)
    return a_envPdf;

  const float hitDist   = length(illuminationPoint - lpos);
  const float cosValTmp = dot(ray_dir, -1.0f*lnorm);
  float cosVal = 1.0f;
  switch(gtype)
  {
    case LIGHT_GEOM_SPHERE:
    {
      // const float  lradius = m_lights[a_lightId].size.x;
      // const float3 lcenter = to_float3(m_lights[a_lightId].pos);
      //if (DistanceSquared(illuminationPoint, lcenter) - lradius*lradius <= 0.0f)
      //  return 1.0f;
      const float3 dirToV = normalize(lpos - illuminationPoint);
      cosVal = std::abs(dot(dirToV, lnorm));
    }
    break;

    case LIGHT_GEOM_POINT:
    {
      if(m_lights[a_lightId].distType == LIGHT_DIST_LAMBERT)
        cosVal = std::max(cosValTmp, 0.0f);
    };
    break;

    default: // any type of area light
    //cosVal = std::max(cosValTmp, 0.0f);                                                               ///< Note(!): actual correct way for area lights
    cosVal = (m_lights[a_lightId].iesId == uint(-1)) ? std::max(cosValTmp, 0.0f) : std::abs(cosValTmp); ///< Note(!): this is not physically correct for area lights, see test_206;
    break;                                                                                              ///< Note(!): dark line on top of image for pink light appears because area light don't shine to the side. 
  };
  
  return PdfAtoW(m_lights[a_lightId].pdfA, hitDist, cosVal);
}

float4 IntegratorDR::LightIntensity(uint a_lightId, float4 a_wavelengths, float3 a_rayPos, float3 a_rayDir, const float* drands, const float* dparams)
{
  float4 lightColor = m_lights[a_lightId].intensity;  
  
  // get spectral data for light source
  //
  const uint specId = m_lights[a_lightId].specId;
  if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0 && specId < 0xFFFFFFFF)
  {
    const uint2 data  = m_spec_offset_sz[specId];
    const uint offset = data.x;
    const uint size   = data.y;
    lightColor = SampleUniformSpectrum(m_spec_values.data() + offset, a_wavelengths, size);
  }
  lightColor *= m_lights[a_lightId].mult;
  
  // get ies data for light source
  //
  const uint iesId = m_lights[a_lightId].iesId;
  if(KSPEC_LIGHT_IES != 0 && iesId != uint(-1))
  {
    if((m_lights[a_lightId].flags & LIGHT_FLAG_POINT_AREA) != 0)
      a_rayDir = normalize(to_float3(m_lights[a_lightId].pos) - a_rayPos);
    const float3 dirTrans = to_float3(m_lights[a_lightId].iesMatrix*to_float4(a_rayDir, 0.0f));
    float sintheta        = 0.0f;
    const float2 texCoord = sphereMapTo2DTexCoord((-1.0f)*dirTrans, &sintheta);
    const float4 texColor = m_textures[iesId]->sample(texCoord);
    lightColor *= texColor;
  }

  // get environment color
  //
  const uint texId = m_lights[a_lightId].texId;

  if(m_lights[a_lightId].distType == LIGHT_DIST_SPOT) // areaSpotLightAttenuation
  {
    float cos1      = m_lights[a_lightId].lightCos1;
    float cos2      = m_lights[a_lightId].lightCos2;
    float3 norm     = to_float3(m_lights[a_lightId].norm);
    float cos_theta = std::max(-dot(a_rayDir, norm), 0.0f);
    lightColor *= mylocalsmoothstep(cos2, cos1, cos_theta);

    if(KSPEC_LIGHT_PROJECTIVE != 0 && (m_lights[a_lightId].flags & LIGHT_FLAG_PROJECTIVE) != 0 && texId != uint(-1))
    {
      const float4x4 mat             = m_lights[a_lightId].iesMatrix;
      const float4 posLightClipSpace = mat*to_float4(a_rayPos, 1.0f); // 
      const float3 posLightSpaceNDC  = to_float3(posLightClipSpace)/posLightClipSpace.w;                         // perspective division
      const float2 shadowTexCoord    = float2(posLightSpaceNDC.x, posLightSpaceNDC.y)*0.5f + float2(0.5f, 0.5f); // just shift coords from [-1,1] to [0,1]  
      const float4 texColor          = m_textures[texId]->sample(shadowTexCoord);
      lightColor *= texColor;
    }
  }
  else if(KSPEC_LIGHT_ENV != 0 && texId != uint(-1))
  {
    float sintheta = 0.0f;
    const float2 texCoord  = sphereMapTo2DTexCoord(a_rayDir, &sintheta);
    const float2 texCoordT = mulRows2x4(m_lights[a_lightId].samplerRow0, m_lights[a_lightId].samplerRow1, texCoord);
    const float4 texColor  = m_textures[texId]->sample(texCoordT);
    lightColor *= texColor;
  }

  return lightColor;
}


float4 IntegratorDR::EnvironmentColor(float3 a_dir, float& outPdf, const float* drands, const float* dparams)
{
  float4 color = m_envColor;
  
  // apply tex color
  //
  const uint envTexId = m_envTexId;
  if(KSPEC_LIGHT_ENV != 0 && envTexId != uint(-1))
  {
    float sinTheta  = 1.0f;
    const float2 tc = sphereMapTo2DTexCoord(a_dir, &sinTheta);
    const float2 texCoordT = mulRows2x4(m_envSamRow0, m_envSamRow1, tc);
    
    if (sinTheta != 0.f && m_envEnableSam != 0 && m_intergatorType == INTEGRATOR_MIS_PT && m_envLightId != uint(-1))
    {
      const uint32_t offset = m_lights[m_envLightId].pdfTableOffset;
      const uint32_t sizeX  = m_lights[m_envLightId].pdfTableSizeX;
      const uint32_t sizeY  = m_lights[m_envLightId].pdfTableSizeY;

      // apply inverse texcoord transform to get phi and theta and than get correct pdf from table 
      //
      const float mapPdf = evalMap2DPdf(texCoordT, m_pdfLightData.data() + offset, int(sizeX), int(sizeY));
      outPdf = (mapPdf * 1.0f) / (2.f * M_PI * M_PI * std::max(std::abs(sinTheta), 1e-20f));  
    }

    const float4 texColor = m_textures[envTexId]->sample(texCoordT); 
    color *= texColor; 
  }

  return color;
}


float4 IntegratorDR::PathTraceReplay(uint tid, uint channels, uint cpuThreadId, float* out_color, 
                                     const float* drands, const float* dparams)
{
  float4  accumColor      = float4(0.0f);
  float4  accumThroughput = float4(1.0f);
  MisData mis             = makeInitialMisData();
  uint    rayFlags        = 0;

  float4 rayPosAndNear, rayDirAndFar;
  float4 wavelengths;
  float  time;
  //kernel_InitEyeRay2(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &rayFlags, &mis, &time);
  {
    const uint XY = m_packedXY[tid];
    const uint x  = (XY & 0x0000FFFF);
    const uint y  = (XY & 0xFFFF0000) >> 16;
  
    //if(x == 256 && y == 256)
    //{
    //  int a = 2;
    //  std::cout << "x = " << x << " , y = " << y << std::endl;
    //}

    const float4 pixelOffsets = float4(drands[0], drands[1], drands[2], drands[3]);
    const float2 wt           = float2(drands[4], drands[5]);
  
    const float xCoordNormalized = (float(x) + pixelOffsets.x)/float(m_winWidth);
    const float yCoordNormalized = (float(y) + pixelOffsets.y)/float(m_winHeight);
  
    float3 rayDir = EyeRayDirNormalized(xCoordNormalized, yCoordNormalized, m_projInv);
    float3 rayPos = float3(0,0,0);
    
    if(KSPEC_SPECTRAL_RENDERING !=0 && m_spectral_mode != 0)
      wavelengths = SampleWavelengths(wt.x, LAMBDA_MIN, LAMBDA_MAX);
    else
      wavelengths = float4(0.0f);
  
    time = wt.y;
    transform_ray3f(m_worldViewInv, &rayPos, &rayDir);
  
    rayPosAndNear = to_float4(rayPos, 0.0f);
    rayDirAndFar  = to_float4(rayDir, FLT_MAX);
  }

  for(uint bounce = 0; bounce < m_traceDepth; bounce++) 
  {
    float4 hitPart1, hitPart2, hitPart3;
    uint   instId;

    //kernel_RayTrace2(tid, bounce, &rayPosAndNear, &rayDirAndFar, &time, 
    //                 &hitPart1, &hitPart2, &hitPart3, &instId, &rayFlags);
    if(!isDeadRay(rayFlags))
    {
      const CRT_Hit hit = m_recorded[cpuThreadId].perBounce[bounce].hit;
      //const CRT_Hit hit = m_pAccelStruct->RayQuery_NearestHit(rayPosAndNear, rayDirAndFar, time);

      if(hit.geomId != uint32_t(-1))
      {
        const float2 uv     = float2(hit.coords[0], hit.coords[1]);
        
        // slightly undershoot the intersection to prevent self-intersection and other bugs
        const float3 hitPos = to_float3(rayPosAndNear) + hit.t * (1.f - 1e-6f) * to_float3(rayDirAndFar);
    
        const uint triOffset  = m_matIdOffsets[hit.geomId];
        const uint vertOffset = m_vertOffset  [hit.geomId];
      
        const uint A = m_triIndices[(triOffset + hit.primId)*3 + 0];
        const uint B = m_triIndices[(triOffset + hit.primId)*3 + 1];
        const uint C = m_triIndices[(triOffset + hit.primId)*3 + 2];
    
        const float4 data1 = (1.0f - uv.x - uv.y)*m_vNorm4f[A + vertOffset] + uv.y*m_vNorm4f[B + vertOffset] + uv.x*m_vNorm4f[C + vertOffset];
        const float4 data2 = (1.0f - uv.x - uv.y)*m_vTang4f[A + vertOffset] + uv.y*m_vTang4f[B + vertOffset] + uv.x*m_vTang4f[C + vertOffset];
    
        float3 hitNorm     = to_float3(data1);
        float3 hitTang     = to_float3(data2);
        float2 hitTexCoord = float2(data1.w, data2.w);
    
        // transform surface point with matrix and flip normal if needed
        //
        hitNorm = mul3x3(m_normMatrices[hit.instId], hitNorm);
        hitTang = mul3x3(m_normMatrices[hit.instId], hitTang);
    
        hitNorm = normalize(hitNorm);
        hitTang = normalize(hitTang);
        
        const float flipNorm = dot(to_float3(rayDirAndFar), hitNorm) > 0.001f ? -1.0f : 1.0f; // beware of transparent materials which use normal sign to identity "inside/outside" glass for example
        hitNorm              = flipNorm * hitNorm;
        hitTang              = flipNorm * hitTang; // do we need this ??
    
        if (flipNorm < 0.0f) rayFlags |=  RAY_FLAG_HAS_INV_NORMAL;
        else                 rayFlags &= ~RAY_FLAG_HAS_INV_NORMAL;
        
        const uint midOriginal = m_matIdByPrimId[m_matIdOffsets[hit.geomId] + hit.primId];
        const uint midRemaped  = RemapMaterialId(midOriginal, hit.instId);
    
        rayFlags = packMatId(rayFlags, midRemaped);
        hitPart1 = to_float4(hitPos,  hitTexCoord.x); 
        hitPart2 = to_float4(hitNorm, hitTexCoord.y);
        hitPart3 = to_float4(hitTang, hit.t);
        instId   = hit.instId;
      }
      else
      {
        const uint flagsToAdd = (bounce == 0) ? (RAY_FLAG_PRIME_RAY_MISS | RAY_FLAG_IS_DEAD | RAY_FLAG_OUT_OF_SCENE) : (RAY_FLAG_IS_DEAD | RAY_FLAG_OUT_OF_SCENE);
        rayFlags              = rayFlags | flagsToAdd;
      }
    }

    //kernel_SampleLightSource(tid, &rayPosAndNear, &rayDirAndFar, &wavelengths, &hitPart1, &hitPart2, &hitPart3, &rayFlags, &time,
    //                         bounce, &gen, &shadeColor);
    float4 shadeColor = float4(0.0f, 0.0f, 0.0f, 0.0f);
    if(!isDeadRay(rayFlags))
    {
      const uint32_t matId = extractMatId(rayFlags);
      const float3 ray_dir = to_float3(rayDirAndFar);
      const float4 lambda  = wavelengths;
    
      SurfaceHit hit;
      hit.pos  = to_float3(hitPart1);
      hit.norm = to_float3(hitPart2);
      hit.tang = to_float3(hitPart3);
      hit.uv   = float2(hitPart1.w, hitPart2.w);
      
      const int bounceTmp = int(bounce); 
      //const float4 rands = GetRandomNumbersLgts(tid, &gen, bounceTmp);
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      const float* lgtRands = drands + LENS_RANDS + bounce*RND_PER_BOUNCE + RND_LTG_ID;
      const float4 rands = float4(lgtRands[0], lgtRands[1], lgtRands[2], lgtRands[3]);
      ///////////////////////////////////////////////////////////////////////////////////////////////////////////
      const int lightId  = std::min(int(std::floor(rands.w * float(m_lights.size()))), int(m_lights.size() - 1u));

      if(lightId >= 0) // no lights or invalid light id
      {
        const LightSample lSam = LightSampleRev(lightId, to_float3(rands), hit.pos, drands, dparams);
        const float  hitDist   = std::sqrt(dot(hit.pos - lSam.pos, hit.pos - lSam.pos));
      
        const float3 shadowRayDir = normalize(lSam.pos - hit.pos); // explicitSam.direction;
        const float3 shadowRayPos = hit.pos + hit.norm * std::max(maxcomp(hit.pos), 1.0f)*5e-6f; // TODO: see Ray Tracing Gems, also use flatNormal for offset
      
        const bool   inIllumArea  = (dot(shadowRayDir, lSam.norm) < 0.0f) || lSam.isOmni || lSam.hasIES;
        const bool needShade      = inIllumArea && (m_recorded[cpuThreadId].perBounce[bounce].inShadow == 0);

        if(needShade) /// (!!!) expression-way to compute 'needShade', RT pipeline bug work around, if change check test_213
        {
          const BsdfEval bsdfV    = MaterialEval(matId, lambda, shadowRayDir, (-1.0f)*ray_dir, hit.norm, hit.tang, hit.uv, drands, dparams);
          float cosThetaOut       = std::max(dot(shadowRayDir, hit.norm), 0.0f);
          
          float      lgtPdfW      = LightPdfSelectRev(lightId) * LightEvalPDF(lightId, shadowRayPos, shadowRayDir, lSam.pos, lSam.norm, lSam.pdf, drands, dparams);
          float      misWeight    = (m_intergatorType == INTEGRATOR_MIS_PT) ? misWeightHeuristic(lgtPdfW, bsdfV.pdf) : 1.0f;
          const bool isDirect     = (m_lights[lightId].geomType == LIGHT_GEOM_DIRECT); 
          const bool isPoint      = (m_lights[lightId].geomType == LIGHT_GEOM_POINT); 
          
          if(isDirect)
          {
            misWeight = 1.0f;
            lgtPdfW   = 1.0f;
          }
          else if(isPoint)
            misWeight = 1.0f;
      
          const bool isDirectLight = !hasNonSpecular(rayFlags);
          if((m_renderLayer == FB_DIRECT   && !isDirectLight) || 
             (m_renderLayer == FB_INDIRECT && isDirectLight)) // skip some number of bounces if this is set
            misWeight = 0.0f;
            
          
          const float4 lightColor = LightIntensity(lightId, lambda, shadowRayPos, shadowRayDir, drands, dparams);
          shadeColor = (lightColor * bsdfV.val / lgtPdfW) * cosThetaOut * misWeight;
        }
      }
    }

    //kernel_NextBounce(tid, bounce, &hitPart1, &hitPart2, &hitPart3, &instId, &shadeColor,
    //                  &rayPosAndNear, &rayDirAndFar, &wavelengths, &accumColor, &accumThroughput, &gen, &mis, &rayFlags);
    
    if(!isDeadRay(rayFlags))
    {
      const uint32_t matId = extractMatId(rayFlags);

      // process surface hit case
      //
      const float3 ray_dir = to_float3(rayDirAndFar);
      const float3 ray_pos = to_float3(rayPosAndNear);
      const float4 lambda  = wavelengths;
      
      SurfaceHit hit;
      hit.pos  = to_float3(hitPart1);
      hit.norm = to_float3(hitPart2);
      hit.tang = to_float3(hitPart3);
      hit.uv   = float2(hitPart1.w, hitPart2.w);
    
      const float hitDist      = hitPart3.w;
      const MisData prevBounce = mis;
      const float   prevPdfW   = prevBounce.matSamplePdf;

      // process light hit case
      //
      if(m_materials[matId].mtype == MAT_TYPE_LIGHT_SOURCE)
      {
        const uint   texId     = m_materials[matId].texid[0];
        const float2 texCoordT = mulRows2x4(m_materials[matId].row0[0], m_materials[matId].row1[0], hit.uv);
        const float4 texColor  = m_textures[texId]->sample(texCoordT);
        const uint   lightId   = m_instIdToLightInstId[instId]; 
        
        const float4 emissColor = m_materials[matId].colors[EMISSION_COLOR];
        float4 lightIntensity   = emissColor * texColor;
    
        if(lightId != 0xFFFFFFFF)
        {
          const float lightCos = dot(to_float3(rayDirAndFar), to_float3(m_lights[lightId].norm));
          const float lightDirectionAtten = (lightCos < 0.0f || m_lights[lightId].geomType == LIGHT_GEOM_SPHERE) ? 1.0f : 0.0f;
          lightIntensity = LightIntensity(lightId, lambda, ray_pos, to_float3(rayDirAndFar), drands, dparams)*lightDirectionAtten;
        }
    
        float misWeight = 1.0f;
        if(m_intergatorType == INTEGRATOR_MIS_PT) 
        {
          if(bounce > 0 && lightId != 0xFFFFFFFF)
          {
            const float lgtPdf  = LightPdfSelectRev(lightId) * LightEvalPDF(lightId, ray_pos, ray_dir, hit.pos, hit.norm, 1.0f, drands, dparams);
            misWeight           = misWeightHeuristic(prevPdfW, lgtPdf);
            if (prevPdfW <= 0.0f) // specular bounce
              misWeight = 1.0f;
          }
        }
        else if(m_intergatorType == INTEGRATOR_SHADOW_PT && hasNonSpecular(rayFlags))
          misWeight = 0.0f;
        
        const bool isDirectLight  = !hasNonSpecular(rayFlags);
        const bool isFirstNonSpec = (rayFlags & RAY_FLAG_FIRST_NON_SPEC) != 0;
        if(m_renderLayer == FB_INDIRECT && (isDirectLight || isFirstNonSpec))
          misWeight = 0.0f;
    
        float4 currAccumColor      = accumColor;
        float4 currAccumThroughput = accumThroughput;
        
        currAccumColor += currAccumThroughput * lightIntensity * misWeight;
       
        accumColor = currAccumColor;
        rayFlags   = rayFlags | (RAY_FLAG_IS_DEAD | RAY_FLAG_HIT_LIGHT);
      }
      else
      {
        const uint bounceTmp    = bounce;
        const BsdfSample matSam = MaterialSampleAndEval(matId, tid, bounceTmp, lambda, (-1.0f)*ray_dir, hit.norm, hit.tang, hit.uv, &mis, rayFlags, drands, dparams);
        const float4 bxdfVal    = matSam.val * (1.0f / std::max(matSam.pdf, 1e-20f));
        const float  cosTheta   = std::abs(dot(matSam.dir, hit.norm)); 
  
        MisData nextBounceData      = mis;        // remember current pdfW for next bounce
        nextBounceData.matSamplePdf = (matSam.flags & RAY_EVENT_S) != 0 ? -1.0f : matSam.pdf; 
        nextBounceData.cosTheta     = cosTheta;   
        mis                         = nextBounceData;
  
        if(m_intergatorType == INTEGRATOR_STUPID_PT)
        {
          accumThroughput *= cosTheta * bxdfVal; 
        }
        else if(m_intergatorType == INTEGRATOR_SHADOW_PT || m_intergatorType == INTEGRATOR_MIS_PT)
        {
          const float4 currThoroughput = accumThroughput;
          float4 currAccumColor        = accumColor;
      
          currAccumColor += currThoroughput * shadeColor;
          accumColor      = currAccumColor;
          accumThroughput = currThoroughput*cosTheta*bxdfVal; 
        }
      
        // compute point on the other side of the surface in case of transmission
        if((matSam.flags & RAY_EVENT_T) != 0)
        {
          hit.pos = hit.pos + hitDist * ray_dir * 2 * 1e-6f;
        }  
      
        rayPosAndNear = to_float4(OffsRayPos(hit.pos, hit.norm, matSam.dir), 0.0f); // todo: use flatNormal for offset
        rayDirAndFar  = to_float4(matSam.dir, FLT_MAX);
        
        uint nextFlags = ((rayFlags & ~RAY_FLAG_FIRST_NON_SPEC) | matSam.flags); // always force reset RAY_FLAG_FIRST_NON_SPEC;
        if(m_renderLayer == FB_DIRECT && hasNonSpecular(rayFlags))               // NOTE: use currRayFlags for check, not nextFlags because of MIS: a ray may hit light source in next bounce
          nextFlags |= RAY_FLAG_IS_DEAD;                                         //       but if we already have non specular bounce previously, definitely can stop  
        else if(!hasNonSpecular(rayFlags) && hasNonSpecular(nextFlags))
          nextFlags |= RAY_FLAG_FIRST_NON_SPEC;
        rayFlags = nextFlags;    
      }              
    }
  }

  //kernel_HitEnvironment(tid, &rayFlags, &rayDirAndFar, &mis, &accumThroughput, &accumColor);
  {
    float envPdf = 1.0f;
    float4 envColor = EnvironmentColor(to_float3(rayDirAndFar), envPdf, drands, dparams);
  
    const auto misPrev  = mis;
    const bool isSpec   = isSpecular(&misPrev);
    const bool exitZero = (rayFlags & RAY_FLAG_PRIME_RAY_MISS) != 0;
  
    if(m_intergatorType == INTEGRATOR_MIS_PT && m_envEnableSam != 0 && !isSpec && !exitZero)
    {
      float lgtPdf    = LightPdfSelectRev(m_envLightId)*envPdf;
      float bsdfPdf   = misPrev.matSamplePdf;
      float misWeight = misWeightHeuristic(bsdfPdf, lgtPdf); // (bsdfPdf*bsdfPdf) / (lgtPdf*lgtPdf + bsdfPdf*bsdfPdf);
      envColor *= misWeight;    
    }
    else if(m_intergatorType == INTEGRATOR_SHADOW_PT && m_envEnableSam != 0)
    {
      envColor = float4(0.0f);
    }

    accumColor += (accumThroughput) * envColor;
  }

  return accumColor;
}

float PixelLossPT(IntegratorDR* __restrict__ pIntegrator,
                  uint tid, uint channels, uint pitch, uint cpuThreadId,
                  const float*  __restrict__ a_refImg,
                        float*  __restrict__ out_color,
                  const uint*   __restrict__ in_pakedXY, 
                  float*        __restrict__ outLoss,
                  const float*  __restrict__ a_drands,
                  const float*  __restrict__ a_dparams)
{
  const uint XY = in_pakedXY[tid];
  const uint x  = (XY & 0x0000FFFF);
  const uint y  = (XY & 0xFFFF0000) >> 16;

  float4 colorRend = pIntegrator->PathTraceReplay(tid, channels, cpuThreadId, out_color, 
                                                  a_drands, a_dparams);

  const uint yRef  = pIntegrator->m_winHeight - y - 1; // in input images and when load data from HDD y has different direction
  float4 colorRef  = float4(a_refImg[(yRef*pitch+x)*channels + 0], 
                            a_refImg[(yRef*pitch+x)*channels + 1], 
                            a_refImg[(yRef*pitch+x)*channels + 2], 0.0f);
  
  out_color[(y*pitch+x)*channels + 0] += colorRend.x;
  out_color[(y*pitch+x)*channels + 1] += colorRend.y;
  out_color[(y*pitch+x)*channels + 2] += colorRend.z;

  float4 diff = colorRend - colorRef;
  float loss = LiteMath::dot3(diff, diff);
  (*outLoss) = loss;
  return loss;
}   


float IntegratorDR::PathTraceDR(uint size, uint channels, float* out_color, uint a_passNum,
                                const float* a_refImg, const float* a_data, float* a_dataGrad, size_t a_gradSize)
{
  m_disableImageContrib = 1;
  memset(a_dataGrad, 0, sizeof(float)*a_gradSize);

  // init separate gradient for each thread
  //
  std::vector<float> grads[MAXTHREADS_CPU];
  for(int i=0;i<MAXTHREADS_CPU;i++) {
    grads[i].resize(a_gradSize);
    std::fill(grads[i].begin(), grads[i].end(), 0.0f);
  }

  //double avgLoss = 0.0;
  auto start = std::chrono::high_resolution_clock::now();
  float avgLoss = 0.0f;
  
  if(m_gradMode != 0)
  {
    #ifndef _DEBUG
    #pragma omp parallel for default(shared) num_threads(MAXTHREADS_CPU)
    #endif
    for (int i = 0; i < int(size); ++i) 
    {
      float lossVal = 0.0f;
      for(int passId = 0; passId < int(a_passNum); passId++) 
      {
        // (1) record non differentiable data during common PT 
        //
        auto cpuThreadId = omp_get_thread_num();
        this->m_recorded[cpuThreadId].recordEnabled = true;
        this->PathTrace(i, channels, out_color);
        this->m_recorded[cpuThreadId].recordEnabled = false;
        
        // (2) perform path trace replay and differentiate actual function
        //
        __enzyme_autodiff((void*)PixelLossPT, 
                           enzyme_const, this,
                           enzyme_const, uint(i),
                           enzyme_const, channels,
                           enzyme_const, m_winWidth,
                           enzyme_const, cpuThreadId,
                           enzyme_const, a_refImg,
                           enzyme_const, out_color,
                           enzyme_const, m_packedXY.data(),
                           enzyme_const, &lossVal,
                           enzyme_const, this->m_recorded[cpuThreadId].perBounceRands.data(),
                           enzyme_dup,   a_data, grads[cpuThreadId].data());

        avgLoss += float(lossVal)/float(a_passNum);
      }
    }
  }
  else
  {
    //for (int i = 0; i < int(tid); ++i) {
    //  float lossVal = PixelLossPT(this, a_refImg, out_color, a_data, m_packedXY.data(),
    //                              uint(i), channels, m_winWidth, &lossVal);
    //  avgLoss += float(lossVal)/float(a_passNum);
    //}
  }

  diffPtTime = float(std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - start).count())/1000.f;

  // accumulate gradient from different threads (parallel reduction/hist)
  //
  for(int i=0;i<MAXTHREADS_CPU;i++) 
    for(size_t j=0;j<a_gradSize; j++)
      a_dataGrad[j] += grads[i][j];

  avgLoss /= float(m_winWidth*m_winHeight);

  //std::cout << "avgLoss = " << avgLoss << std::endl;
  //std::cout.flush();
  
  //std::ofstream fout("z_grad.txt");
  //for(size_t i=0; i<a_gradSize; i++)
  //  fout << a_dataGrad[i]/float(a_passNum) << std::endl;
  //fout.close();

  m_disableImageContrib = 0;
  return avgLoss;
}

