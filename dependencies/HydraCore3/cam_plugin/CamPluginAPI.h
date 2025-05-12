#pragma once
#include <cstdint>      // uint32_t
#include <cstddef>      // for size_t
#include <string>

#ifndef KERNEL_SLICER
struct CamParameters    ///<! add any parameter you like to this structure
{
  float fov;
  float aspect;
  float nearPlane;
  float farPlane;
  int   spectralMode;
  std::string opticFile;
};
#endif

struct RayPosAndW 
{
  float origin[3]; ///<! ray origin, x,y,z
  float wave;      ///<! wavelength
};

struct RayDirAndT 
{
  float direction[3]; ///<! normalized ray direction, x,y,z
  float time;         ///<! time in ... 
};

struct ICamRaysAPI2
{
  virtual ~ICamRaysAPI2() {}

  /**
   \brief Set camera parameters
   \param a_width         - image width
   \param a_height        - image height
   \param a_camNodeText   - all other camera parameters which you can directly by reading this node
  */
  virtual void SetParameters(int a_width, int a_height, const CamParameters& a_params) {}

  virtual void SetBatchSize(int a_tileSize) = 0;

  /**
   \brief Put portion of rays in execution queue
   \param out_rayPosAndNear - packed ray origin    (x,y,z) and tNear (w)
   \param out_rayDirAndFar  - packed ray direction (x,y,z) and tFar  (w)
   \param out_auxData       - packed ray wavelengs and other aux data
   \param in_blockSize      - ray portion size     (may depend on GPU/device, usually == 1024*512)  
    Please note that it is assumed that rays are uniformly distributed over image plane (and all other integrated dimentions like position on lens)
    for the whole period of time (all passes), the example will be provided.
  */
  virtual void MakeRaysBlock(RayPosAndW* out_rayPosAndNear4f, RayDirAndT* out_rayDirAndFar4f, uint32_t in_blockSize, int subPassId) = 0;

  /**
  \brief Add contribution
  \param out_color  - out float4 image of size a_width*a_height
  \param colors     - in float4 array of size a_size
  \param a_size     - array size
  \param a_width    - image width
  \param a_height   - image height
  */
  virtual void AddSamplesContributionBlock(float* out_color4f, const float* colors4f, uint32_t in_blockSize, 
                                           uint32_t a_width, uint32_t a_height, int subPassId) = 0;

  virtual void CommitDeviceData(){}
  virtual void GetExecutionTime(const char* a_funcName, float a_out[4]){}                                          
};
