#include "NeuralRT.h"
#include "../render_common.h"

template<uint32_t bsize> 
void NeuralRT::kernelBE1D_SphereTracing(uint32_t blockNum)
{
  for(uint32_t blockId = 0; blockId < blockNum; blockId++) 
  {
    for(uint32_t localId = 0; localId < bsize; localId++) [[parallel]]  // full parallel
    {
      const uint32_t x  = NEURALRT_BSIZE * (blockId % (m_width / NEURALRT_BSIZE)) + localId % NEURALRT_BSIZE;
      const uint32_t y  = NEURALRT_BSIZE * (blockId / (m_width / NEURALRT_BSIZE)) + localId / NEURALRT_BSIZE;
      float tmp_mem[2 * NEURAL_SDF_MAX_LAYER_SIZE];

      float3 rayDir = normalize(EyeRayDirNormalized((float(x)+0.5f)/float(m_width), (float(y)+0.5f)/float(m_height), m_projInv));
      float3 rayPos = float3(0,0,0);

      transform_ray3f(m_worldViewInv, 
                      &rayPos, &rayDir);

      float2 tNearFar = RayBoxIntersection2(rayPos, SafeInverse(rayDir), float3(-1,-1,-1), float3(1,1,1));

      constexpr float EPS = 1e-5f;
      constexpr uint32_t max_iters = 1000;
      float t = tNearFar.x;
      float d = 1e6f;
      uint32_t iter = 0;
      while (t < tNearFar.y && iter < max_iters && d > EPS)
      {
        float3 p = rayPos + t*rayDir;

        //calculate SIREN SDF
        NeuralProperties prop = m_SdfNeuralProperties[0];
        uint32_t t_ofs1 = 0;
        uint32_t t_ofs2 = NEURAL_SDF_MAX_LAYER_SIZE;

        tmp_mem[t_ofs1 + 0] = p.x;
        tmp_mem[t_ofs1 + 1] = p.y;
        tmp_mem[t_ofs1 + 2] = p.z;

        for (int l = 0; l < prop.layer_count; l++)
        {
          uint32_t m_ofs = prop.layers[l].offset;
          uint32_t b_ofs = prop.layers[l].offset + prop.layers[l].in_size * prop.layers[l].out_size;
          for (int i = 0; i < prop.layers[l].out_size; i++)
          {
            tmp_mem[t_ofs2 + i] = m_SdfNeuralData[b_ofs + i];
            for (int j = 0; j < prop.layers[l].in_size; j++)
              tmp_mem[t_ofs2 + i] += tmp_mem[t_ofs1 + j] * m_SdfNeuralData[m_ofs + i * prop.layers[l].in_size + j];
            if (l < prop.layer_count - 1)
              tmp_mem[t_ofs2 + i] = std::sin(SIREN_W0 * tmp_mem[t_ofs2 + i]);
          }

          t_ofs2 = t_ofs1;
          t_ofs1 = (t_ofs1 + NEURAL_SDF_MAX_LAYER_SIZE) % (2 * NEURAL_SDF_MAX_LAYER_SIZE);
        }

        d = tmp_mem[t_ofs1];
        t += d + EPS;
      }

      if (d <= EPS)
      {
        float z = t;
        float z_near = 0.1;
        float z_far = 10;
        float depth = ((z - z_near) / (z_far - z_near));
        uint32_t col= uint32_t(255*depth);
        m_ImageData[y * m_width + x] = 0xFF000000 | (col<<16) | (col<<8) | col; 
      }
      else
        m_ImageData[y * m_width + x] = 0xFF000000;
    }
  }
}

#ifndef KERNEL_SLICER  
NeuralRT::NeuralRT()
{
  m_SdfNeuralProperties.reserve(1);
  m_SdfNeuralData.reserve(10000);
}

uint32_t NeuralRT::AddGeom_NeuralSdf(NeuralProperties neural_properties, float *data, BuildOptions a_qualityLevel)
{
  assert(m_SdfNeuralProperties.size() == 0); //only one neural SDF is available

  unsigned np_offset = m_SdfNeuralData.size();

  unsigned total_size = 0;
  for (int i=0; i<neural_properties.layer_count; i++)
    total_size += neural_properties.layers[i].out_size*(neural_properties.layers[i].in_size + 1);

  assert(total_size > 0);

  m_SdfNeuralProperties.push_back(neural_properties);
  for (int i=0; i<neural_properties.layer_count; i++)
    m_SdfNeuralProperties.back().layers[i].offset += np_offset;

  m_SdfNeuralData.insert(m_SdfNeuralData.end(), data, data+total_size);

  return m_SdfNeuralProperties.size()-1;
}

void NeuralRT::Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, 
                      const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj, int a_passNum)
{
  m_width = a_width;
  m_height = a_height;
  m_worldView = a_worldView;
  m_proj = a_proj;
  m_projInv = inverse4x4(a_proj);
  m_worldViewInv = inverse4x4(a_worldView);
  m_ImageData.resize(a_width*a_height, 0u);

  for (int i=0;i<a_passNum;i++)
    kernelBE1D_SphereTracing<NEURALRT_BSIZE*NEURALRT_BSIZE>(a_width*a_height/(NEURALRT_BSIZE*NEURALRT_BSIZE));

  memcpy(imageData, m_ImageData.data(), sizeof(uint32_t)*a_width*a_height);
}
#endif