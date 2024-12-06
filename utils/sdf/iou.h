#include "sdf_converter.h"

namespace IoU
{
  float IoU_frame_octree(const std::vector<SdfFrameOctreeNode> &frame_nodes, sdf_converter::MultithreadedDistanceFunction sdf, unsigned points);
}