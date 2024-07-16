#pragma once

//enum GeomType
static constexpr unsigned TYPE_MESH_TRIANGLE        =  0;
static constexpr unsigned TYPE_SDF_PRIMITIVE        =  1;
static constexpr unsigned TYPE_SDF_GRID             =  2;
static constexpr unsigned TYPE_SDF_OCTREE           =  3;
static constexpr unsigned TYPE_SDF_FRAME_OCTREE     =  4;
static constexpr unsigned TYPE_RF_GRID              =  5;
static constexpr unsigned TYPE_SDF_SVS              =  6;
static constexpr unsigned TYPE_SDF_SBS              =  7;
static constexpr unsigned TYPE_GS_PRIMITIVE         =  8;
static constexpr unsigned TYPE_SDF_FRAME_OCTREE_TEX =  9;
static constexpr unsigned TYPE_SDF_SBS_SINGLE_NODE  = 10;
static constexpr unsigned TYPE_SDF_SBS_TEX          = 11;
static constexpr unsigned TYPE_SDF_SBS_COL          = 12;

//enum SdfOctreeSampler
static constexpr unsigned SDF_OCTREE_SAMPLER_MIPSKIP_3X3     = 0; //go to the deepest level possible, resampling larger nodes
static constexpr unsigned SDF_OCTREE_SAMPLER_MIPSKIP_CLOSEST = 1; //go deeper while resampling is not needed, then sample
static constexpr unsigned SDF_OCTREE_SAMPLER_CLOSEST         = 2;

//enum SdfNodeIntersect
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_ST       = 0;// Sphere tracing inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_ANALYTIC = 1;// Explicitly finding ray/sdf intersection inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_NEWTON   = 2;// Using Newton method to find ray/sdf intersection inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_BBOX     = 3;// Intersect with node bbox for debug purposes
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_IT       = 4;// Interval tracing inside node

//enum MultiRenderMode
static constexpr unsigned MULTI_RENDER_MODE_MASK                 =  0; //white object, black background
static constexpr unsigned MULTI_RENDER_MODE_LAMBERT_NO_TEX       =  1; //Lambert shading, no texture, no shadows
static constexpr unsigned MULTI_RENDER_MODE_DEPTH                =  2; //visualize depth
static constexpr unsigned MULTI_RENDER_MODE_LINEAR_DEPTH         =  3; //visualize depth, color changes linearly in predefined range
static constexpr unsigned MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH =  4; //visualize depth, color changes linearly in predefined range (inverted)
static constexpr unsigned MULTI_RENDER_MODE_PRIMIVIVE            =  5; //each primitive has distinct color from palette
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

//enum MeshNormalMode
static constexpr unsigned MESH_NORMAL_MODE_GEOMETRY = 0; //geometry normal, faceted look but it do not rely on vertex normals
static constexpr unsigned MESH_NORMAL_MODE_VERTEX   = 1; //vertex normal, smooth look but vertex normals should be available

struct MultiRenderPreset
{
  unsigned render_mode;        //enum MultiRenderMode
  unsigned sdf_octree_sampler; //enum SdfOctreeSampler
  unsigned sdf_node_intersect; //enum SdfNodeIntersect
  unsigned mesh_normal_mode;   //enum MeshNormalMode
};