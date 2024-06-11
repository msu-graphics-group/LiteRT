#pragma once

//enum GeomType
static constexpr unsigned TYPE_MESH_TRIANGLE       = 0;
static constexpr unsigned TYPE_SDF_PRIMITIVE       = 1;
static constexpr unsigned TYPE_SDF_GRID            = 2;
static constexpr unsigned TYPE_SDF_OCTREE          = 3;
static constexpr unsigned TYPE_SDF_FRAME_OCTREE    = 4;
static constexpr unsigned TYPE_RF_GRID             = 5;
static constexpr unsigned TYPE_SDF_SVS             = 6;
static constexpr unsigned TYPE_SDF_SBS             = 7;
static constexpr unsigned TYPE_GS_PRIMITIVE        = 8;
static constexpr unsigned TYPE_SDF_HP              = 9;

//enum SdfOctreeSampler
static constexpr unsigned SDF_OCTREE_SAMPLER_MIPSKIP_3X3 = 0; //go to the deepest level possible, resampling larger nodes
static constexpr unsigned SDF_OCTREE_SAMPLER_MIPSKIP_CLOSEST = 1; //go deeper while resampling is not needed, then sample
static constexpr unsigned SDF_OCTREE_SAMPLER_CLOSEST = 2;

//enum SdfFrameOctreeBLAS
static constexpr unsigned SDF_OCTREE_BLAS_NO = 0; //use trivial BLAS with 2 bboxes and full sphere tracing later on
static constexpr unsigned SDF_OCTREE_BLAS_DEFAULT = 1; //use BVH with one leaf for every non-empty leaf node of octree

//enum SdfFrameOctreeIntersect
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_DEFAULT = 0; //sphere tracing + octree traversal
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_ST = 1;// only with SDF_OCTREE_BLAS_DEFAULT! Sphere tracing inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_ANALYTIC = 2;// only with SDF_OCTREE_BLAS_DEFAULT! Explicitly finding ray/sdf intersection inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_NEWTON = 3;// only with SDF_OCTREE_BLAS_DEFAULT! Using Newton method to find ray/sdf intersection inside node
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_BBOX = 4;// only with SDF_OCTREE_BLAS_DEFAULT! Intersect with node bbox for debug purposes
static constexpr unsigned SDF_OCTREE_NODE_INTERSECT_IT = 5;// only with SDF_OCTREE_BLAS_DEFAULT! Interval tracing inside node

//enum MultiRenderMode
static constexpr unsigned MULTI_RENDER_MODE_MASK = 0; //white object, black background
static constexpr unsigned MULTI_RENDER_MODE_LAMBERT = 1;
static constexpr unsigned MULTI_RENDER_MODE_DEPTH = 2;
static constexpr unsigned MULTI_RENDER_MODE_LINEAR_DEPTH = 3;
static constexpr unsigned MULTI_RENDER_MODE_INVERSE_LINEAR_DEPTH = 4;
static constexpr unsigned MULTI_RENDER_MODE_PRIMIVIVE = 5; //each primitive has distinct color from palette
static constexpr unsigned MULTI_RENDER_MODE_TYPE = 6; //each type has distinct color from palette
static constexpr unsigned MULTI_RENDER_MODE_GEOM = 7; //each geodId has distinct color from palette
static constexpr unsigned MULTI_RENDER_MODE_NORMAL = 8; //visualize normals (abs for each coordinate)
static constexpr unsigned MULTI_RENDER_MODE_BARYCENTRIC = 9; //visualize barycentric coordinates for triangle mesh
static constexpr unsigned MULTI_RENDER_MODE_SPHERE_TRACE_ITERATIONS = 10; //for SDFs replace primId with number of iterations for sphere tracing
static constexpr unsigned MULTI_RENDER_MODE_RF = 11; //default mode for rendering radiance fields
static constexpr unsigned MULTI_RENDER_MODE_PHONG = 12; //rendering meshes/SDFs woth Phong shading
static constexpr unsigned MULTI_RENDER_MODE_GS = 13; //rendering Gaussian splats
static constexpr unsigned MULTI_RENDER_MODE_RF_DENSITY = 14; //rendering density from radiance field

//enum MeshNormalMode
static constexpr unsigned MESH_NORMAL_MODE_GEOMETRY = 0; //geometry normal, faceted look but it do not rely on vertex normals
static constexpr unsigned MESH_NORMAL_MODE_VERTEX = 1; //vertex normal, smooth look but vertex normals should be available

struct MultiRenderPreset
{
  unsigned mode; //enum MultiRenderMode
  unsigned sdf_octree_sampler; //enum SdfOctreeSampler
  unsigned spp; //samples per pixel, should be a square (1, 4, 9, 16 etc.)
  unsigned sdf_frame_octree_blas; //enum SdfFrameOctreeBLAS
  unsigned sdf_frame_octree_intersect; //enum SdfFrameOctreeIntersect
  unsigned mesh_normal_mode; //enum MeshNormalMode
};