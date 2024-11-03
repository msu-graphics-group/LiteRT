#pragma once

//enum GeomType
static constexpr unsigned TYPE_MESH_TRIANGLE              =  0;
static constexpr unsigned TYPE_SDF_GRID                   =  1;

static constexpr unsigned TYPE_SDF_FRAME_OCTREE           =  3;
static constexpr unsigned TYPE_RF_GRID                    =  4;
static constexpr unsigned TYPE_SDF_SVS                    =  5;
static constexpr unsigned TYPE_SDF_SBS                    =  6;
static constexpr unsigned TYPE_GS_PRIMITIVE               =  7;
static constexpr unsigned TYPE_SDF_FRAME_OCTREE_TEX       =  8;

static constexpr unsigned TYPE_SDF_SBS_TEX                = 10;
static constexpr unsigned TYPE_SDF_SBS_COL                = 11;
static constexpr unsigned TYPE_SDF_SBS_ADAPT              = 12;

static constexpr unsigned TYPE_SDF_SBS_ADAPT_TEX          = 14;
static constexpr unsigned TYPE_SDF_SBS_ADAPT_COL          = 15;
static constexpr unsigned TYPE_NURBS                      = 16;
static constexpr unsigned TYPE_GRAPHICS_PRIM              = 17;
static constexpr unsigned TYPE_COCTREE_V1                 = 18;
static constexpr unsigned TYPE_COCTREE_V2                 = 19;

static constexpr unsigned SH_TYPE = 27; //5 bits for type

//enum RenderDevice
static constexpr unsigned DEVICE_CPU     = 0; //render on CPU
static constexpr unsigned DEVICE_GPU     = 1; //render on GPU using compute shaders
static constexpr unsigned DEVICE_GPU_RTX = 2; //render on GPU using RTX pipeline
//Currently the choice between compute shaders and RTX should be made in CMake files (USE_RTX option) and
//there is no difference in behavior between DEVICE_GPU and DEVICE_GPU_RTX, what was chosen in Cmake is always used

//enum SdfNodeIntersect
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_ST       = 0;// Sphere tracing inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_ANALYTIC = 1;// Explicitly finding ray/sdf intersection inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_NEWTON   = 2;// Using Newton method to find ray/sdf intersection inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_BBOX     = 3;// Intersect with node bbox for debug purposes
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_IT       = 4;// Interval tracing inside node

//enum SdfOctreeIntersect
static constexpr unsigned OCTREE_INTERSECT_BVH      = 0;//use BVH
static constexpr unsigned OCTREE_INTERSECT_TRAVERSE = 1;//no BVH, traverse octree directly

//enum MultiRenderMode
static constexpr unsigned MULTI_RENDER_MODE_MASK                 =  0; //white object, black background
static constexpr unsigned MULTI_RENDER_MODE_LAMBERT_NO_TEX       =  1; //Lambert shading, no texture, no shadows
static constexpr unsigned MULTI_RENDER_MODE_DEPTH                =  2; //visualize depth
static constexpr unsigned MULTI_RENDER_MODE_LINEAR_DEPTH         =  3; //visualize depth, color changes linearly in predefined range
static constexpr unsigned MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH =  4; //visualize depth, color changes linearly in predefined range (inverted)
static constexpr unsigned MULTI_RENDER_MODE_PRIMITIVE            =  5; //each primitive has distinct color from palette
static constexpr unsigned MULTI_RENDER_MODE_TYPE                 =  6; //each type has distinct color from palette
static constexpr unsigned MULTI_RENDER_MODE_GEOM                 =  7; //each geodId has distinct color from palette
static constexpr unsigned MULTI_RENDER_MODE_NORMAL               =  8; //visualize normals (abs for each coordinate)
static constexpr unsigned MULTI_RENDER_MODE_BARYCENTRIC          =  9; //visualize barycentric coordinates for triangle mesh
static constexpr unsigned MULTI_RENDER_MODE_ST_ITERATIONS        = 10; //for SDFs replace primId with number of iterations for sphere tracing
static constexpr unsigned MULTI_RENDER_MODE_RF                   = 11; //default mode for rendering radiance fields
static constexpr unsigned MULTI_RENDER_MODE_PHONG_NO_TEX         = 12; //Phong shading, no texture, shadows
static constexpr unsigned MULTI_RENDER_MODE_GS                   = 13; //rendering Gaussian splats
static constexpr unsigned MULTI_RENDER_MODE_RF_DENSITY           = 14; //rendering density from radiance field
static constexpr unsigned MULTI_RENDER_MODE_TEX_COORDS           = 15; //rendering texture coordinates in RG channels
static constexpr unsigned MULTI_RENDER_MODE_DIFFUSE              = 16; //rendering diffuse color from material
static constexpr unsigned MULTI_RENDER_MODE_LAMBERT              = 17; //Lambert shading, no shadows
static constexpr unsigned MULTI_RENDER_MODE_PHONG                = 18; //Phong shading
static constexpr unsigned MULTI_RENDER_MODE_HSV_DEPTH            = 19; //visualize depth, uses HSV color model

//enum NormalMode
static constexpr unsigned NORMAL_MODE_GEOMETRY     = 0; //geometry normal, default for all types of geometry
static constexpr unsigned NORMAL_MODE_VERTEX       = 1; //vertex normal, available only for meshes with vertex normals gives
static constexpr unsigned NORMAL_MODE_SDF_SMOOTHED = 2; //smoothed SDF normal, available only for SBS with layouts containing neighbors

//enum RayGenMode
static constexpr unsigned RAY_GEN_MODE_REGULAR = 0;
static constexpr unsigned RAY_GEN_MODE_RANDOM  = 1;

//enum interpolationMode
static constexpr unsigned TRILINEAR_INTERPOLATION_MODE = 0;
static constexpr unsigned TRICUBIC_INTERPOLATION_MODE = 1;

struct MultiRenderPreset
{
  unsigned render_mode;        //enum MultiRenderMode
  unsigned octree_intersect;   //enum SdfOctreeIntersect
  unsigned sdf_node_intersect; //enum SdfNodeIntersect
  unsigned normal_mode;        //enum NormalMode
  unsigned ray_gen_mode;       //enum RayGenMode
  unsigned spp;                //samples per pixel
  unsigned interpolation_type; //0 - trilinear, 1 - tricubic
};

static MultiRenderPreset getDefaultPreset()
{
  MultiRenderPreset p;
  p.render_mode = MULTI_RENDER_MODE_LAMBERT_NO_TEX;
  p.octree_intersect = OCTREE_INTERSECT_BVH;
  p.sdf_node_intersect = SDF_OCTREE_NODE_INTERSECT_ST;
  p.normal_mode = NORMAL_MODE_GEOMETRY;
  p.ray_gen_mode = RAY_GEN_MODE_REGULAR;
  p.spp = 1;
  p.interpolation_type = TRILINEAR_INTERPOLATION_MODE;

  return p;
}