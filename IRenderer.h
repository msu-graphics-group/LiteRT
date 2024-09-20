#pragma once
#include <cstdint>
#include <memory>

#include "LiteMath.h"
#include "dependencies/HydraCore3/external/CrossRT/CrossRT.h"

struct IRenderer
{
  IRenderer(){ }
  virtual ~IRenderer(){}

  virtual const char* Name() const { return ""; }
  virtual bool LoadScene(const char* a_scenePath) = 0;

  virtual void Clear (uint32_t a_width, uint32_t a_height, const char* a_what) = 0;
  virtual void Render(uint32_t* imageData, uint32_t a_width, uint32_t a_height, const char* a_what, int a_passNum = 1) = 0;

  virtual void SetViewport(int a_xStart, int a_yStart, int a_width, int a_height){}
  virtual void SetAccelStruct(std::shared_ptr<ISceneObject> a_customAccelStruct) {}
  virtual std::shared_ptr<ISceneObject> GetAccelStruct() { return nullptr; }
  
  virtual void GetExecutionTime(const char* a_funcName, float a_out[4]){}; // will be overriden in generated class

  // for future GPU impl
  //
  virtual void CommitDeviceData() {}                                     // will be overriden in generated class

  virtual void UpdateMembersPlainData() {}                               // will be overriden in generated class, optional function
  virtual void UpdateMembersVectorData() {}                              // will be overriden in generated class, optional function
  virtual void UpdateMembersTexureData() {}                              // will be overriden in generated class, optional function
  
  virtual void InitAnimation(){}                                         ///<! mostly for demo case
  virtual void UpdateAnimation(float delta_t){}                          ///<! mostly for demo case
  virtual void UpdateCamera(const LiteMath::float4x4& a_worldView, const LiteMath::float4x4& a_proj) {}

protected:

  IRenderer(const IRenderer& rhs) {}
  IRenderer& operator=(const IRenderer& rhs) { return *this;}
  
  virtual uint32_t GetGeomNum() const  { return 0; };
  virtual uint32_t GetInstNum() const  { return 0; };
  virtual const LiteMath::float4* GetGeomBoxes() const { return nullptr; };
};

