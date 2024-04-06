#include <iostream>
#include <fstream>
#include <queue>
#include <stack>
#include <limits>
#include <cassert>
#include <cmath>
#include <cfloat>
#include <algorithm>
#include <unordered_set>

#include "cbvh.h"
#include "embree3/rtcore.h"

extern double g_buildTime;
extern uint64_t g_buildTris;

namespace embree
{
  void build(RTCBuildQuality quality, std::vector<RTCBuildPrimitive> &prims_i, size_t extraSpace, BVHTree &a_res);
  extern uint32_t g_recommendedPrimsInLeaf;
}

// NOINTERVALS format
//
static constexpr uint32_t START_MASK = 0x00FFFFFF;
static constexpr uint32_t END_MASK = 0xFF000000;
static constexpr uint32_t SIZE_MASK = 0x7F000000;
static constexpr uint32_t LEAF_BIT = 0x80000000;
static constexpr uint32_t EMPTY_NODE = 0x7fffffff;

static inline uint32_t PackOffsetAndSize(uint32_t start, uint32_t size)
{
  return LEAF_BIT | ((size << 24) & SIZE_MASK) | (start & START_MASK);
}

static std::ostream &operator<<(std::ostream &os, const BVHNode &dt)
{
  os << dt.boxMin.x << "\t" << dt.boxMin.y << "\t" << dt.boxMin.z << "\t|\t" << dt.leftOffset << std::endl;
  os << dt.boxMax.x << "\t" << dt.boxMax.y << "\t" << dt.boxMax.z << "\t|\t" << dt.escapeIndex << std::endl;
  return os;
}

static inline bool IsLeaf(const BVHNode &a_node) { return (a_node.leftOffset == LEAF_NORMAL); }
static inline bool IsEmpty(const BVHNode &a_node) { return (a_node.leftOffset == LEAF_EMPTY); }
static inline bool IsValid(const BVHNode &a_node) { return (a_node.leftOffset < LEAF_EMPTY); }

static inline BVHNode DummyNode()
{
  BVHNode dummyNode;
  dummyNode.boxMin = float3(0, 0, 0);
  dummyNode.boxMax = float3(0, 0, 0);
  dummyNode.leftOffset = 0x80000000;  // or 0xFFFFFFFD according to old cbvh convention
  dummyNode.escapeIndex = 0xFFFFFFFF; // or 0xFFFFFFFD according to old cbvh convention
  return dummyNode;
}

BuilderPresets BuilderPresetsFromString(const char *a_str)
{
  BuilderPresets presets = BuilderPresets();

  const std::string a_buildName(a_str);
  presets.quality = BVHQuality::HIGH;

  char symb = a_buildName[a_buildName.size() - 1];
  if (std::isdigit(symb))
    presets.primsInLeaf = int(symb) - int('0');

  return presets;
}

void CheckNodesAlign(const std::vector<BVHNode> &a_nodes, int a_alignCoeff)
{
  bool isOk = true;
  for (size_t i = 0; i < a_nodes.size(); i++)
  {
    const auto &node = a_nodes[i];
    if ((node.leftOffset & LEAF_BIT) == 0)
    {
      if (node.leftOffset % a_alignCoeff != 0)
        isOk = false;
    }
  }

  const char *timeFileName = "z_align_check.csv";
  std::ofstream fout;
  std::ifstream fin(timeFileName);
  if (!fin.is_open())
  {
    fout.open(timeFileName);
    fout << "Align;" << std::endl;
  }
  else
  {
    fin.close();
    fout.open(timeFileName, std::ios::app);
  }

  if (isOk)
    fout << "OK;" << std::endl;
  else
    fout << "FAILED!;" << std::endl;
}

struct TraverseDFS
{
  TraverseDFS(const std::vector<BVHNode> &a_nodesOld, std::vector<BVHNode> &a_nodesNew) : oldNodes(a_nodesOld), newNodes(a_nodesNew) {}

  uint32_t VisitNode(uint32_t a_nodeOffsetLeft, uint32_t a_parentEscapeIndex)
  {
    if ((a_nodeOffsetLeft & LEAF_BIT) != 0)
      return a_nodeOffsetLeft;

    const auto &nodeLeft = oldNodes[a_nodeOffsetLeft + 0];
    const auto &nodeRight = oldNodes[a_nodeOffsetLeft + 1];

    if (newNodes.size() % 2 != 0)
    {
      newNodes.push_back(DummyNode()); // never happends?
      alignCounter++;
    }

    newNodes.push_back(nodeLeft);
    newNodes.push_back(nodeRight);

    const uint32_t leftOffsetNew = uint32_t(newNodes.size() - 2);

    newNodes[leftOffsetNew + 0].escapeIndex = leftOffsetNew + 1;
    newNodes[leftOffsetNew + 1].escapeIndex = a_parentEscapeIndex;

    newNodes[leftOffsetNew + 0].leftOffset = VisitNode(nodeLeft.leftOffset, leftOffsetNew + 1);
    newNodes[leftOffsetNew + 1].leftOffset = VisitNode(nodeRight.leftOffset, a_parentEscapeIndex);

    return leftOffsetNew;
  }

  uint32_t alignCounter = 0;

protected:
  const std::vector<BVHNode> &oldNodes;
  std::vector<BVHNode> &newNodes;
};

std::vector<BVHNode> AlignNodes2(const std::vector<BVHNode> &a_nodes)
{
  if (a_nodes.size() == 0)
    return a_nodes;

  std::vector<BVHNode> newNodes;
  newNodes.reserve(a_nodes.size() * 2);

  if (a_nodes.size() == 1)
  {
    newNodes.resize(2);
    newNodes[0] = a_nodes[0];
    newNodes[1] = DummyNode();
    return newNodes;
  }

  TraverseDFS trav(a_nodes, newNodes);
  trav.VisitNode(a_nodes[0].leftOffset, ESCAPE_ROOT);

  if (trav.alignCounter != 0)
    std::cout << "[AlignNodes2]: " << trav.alignCounter << "nodes was aligned!" << std::endl;

  if (newNodes.size() % 2 != 0)
    newNodes.push_back(DummyNode());

  return newNodes;
}

struct TraverseDFS4
{
  TraverseDFS4(const std::vector<BVHNode> &a_nodesOld, std::vector<BVHNode> &a_nodesNew) : oldNodes(a_nodesOld), newNodes(a_nodesNew) {}

  uint32_t VisitNode(uint32_t a_nodeOffsetLeft, uint32_t a_parentEscapeIndex)
  {
    if ((a_nodeOffsetLeft & LEAF_BIT) != 0)
      return a_nodeOffsetLeft;

    const auto &node0 = oldNodes[a_nodeOffsetLeft + 0];
    const auto &node1 = oldNodes[a_nodeOffsetLeft + 1];
    const auto &node2 = oldNodes[a_nodeOffsetLeft + 2];
    const auto &node3 = oldNodes[a_nodeOffsetLeft + 3];

    while (newNodes.size() % 4 != 0)
    {
      newNodes.push_back(DummyNode()); // never happends?
      alignCounter++;
    }

    newNodes.push_back(node0);
    newNodes.push_back(node1);
    newNodes.push_back(node2);
    newNodes.push_back(node3);

    const uint32_t leftOffsetNew = uint32_t(newNodes.size() - 4);

    newNodes[leftOffsetNew + 0].escapeIndex = leftOffsetNew + 1;
    newNodes[leftOffsetNew + 1].escapeIndex = leftOffsetNew + 2;
    newNodes[leftOffsetNew + 2].escapeIndex = leftOffsetNew + 3;
    newNodes[leftOffsetNew + 3].escapeIndex = a_parentEscapeIndex;

    newNodes[leftOffsetNew + 0].leftOffset = VisitNode(node0.leftOffset, leftOffsetNew + 1);
    newNodes[leftOffsetNew + 1].leftOffset = VisitNode(node1.leftOffset, leftOffsetNew + 2);
    newNodes[leftOffsetNew + 2].leftOffset = VisitNode(node2.leftOffset, leftOffsetNew + 3);
    newNodes[leftOffsetNew + 3].leftOffset = VisitNode(node3.leftOffset, a_parentEscapeIndex);

    return leftOffsetNew;
  }

  uint32_t alignCounter = 0;

protected:
  const std::vector<BVHNode> &oldNodes;
  std::vector<BVHNode> &newNodes;
};

std::vector<BVHNode> AlignNodes4(const std::vector<BVHNode> &a_nodes)
{
  if (a_nodes.size() == 0)
    return a_nodes;

  std::vector<BVHNode> newNodes;
  newNodes.reserve(a_nodes.size() * 2);

  TraverseDFS4 trav(a_nodes, newNodes);
  trav.VisitNode(a_nodes[0].leftOffset, ESCAPE_ROOT);

  if (trav.alignCounter != 0)
    std::cout << "[AlignNodes4]: " << trav.alignCounter << "nodes was aligned!" << std::endl;

  while (newNodes.size() % 4 != 0)
    newNodes.push_back(DummyNode());

  return newNodes;
}

struct Converter2to4
{
  Converter2to4(const std::vector<BVHNode> &in_nodes, std::vector<BVHNode> &out_nodes) : m_inNodes(in_nodes), m_outNodes(out_nodes)
  {
    m_outNodes.reserve(m_inNodes.size());
    m_outNodes.resize(0);
  }

  static inline uint32_t EXTRACT_START(uint32_t a_leftOffset) { return a_leftOffset & START_MASK; }
  static inline uint32_t EXTRACT_COUNT(uint32_t a_leftOffset) { return (a_leftOffset & SIZE_MASK) >> 24; }

  uint32_t Visit(uint32_t a_offset) //
  {
    if ((a_offset & LEAF_BIT) != 0)
      return a_offset;

    const BVHNode leftNode = m_inNodes[a_offset + 0];
    const BVHNode rightNode = m_inNodes[a_offset + 1];

    if ((leftNode.leftOffset & LEAF_BIT) != 0 && (rightNode.leftOffset & LEAF_BIT) != 0)
    {
      const uint32_t start1 = EXTRACT_START(leftNode.leftOffset);
      const uint32_t count1 = EXTRACT_COUNT(leftNode.leftOffset);

      const uint32_t start2 = EXTRACT_START(rightNode.leftOffset);
      const uint32_t count2 = EXTRACT_COUNT(rightNode.leftOffset);

      const uint32_t start = std::min(start1, start2);
      const uint32_t count = count1 + count2;

      return PackOffsetAndSize(start, count);
    }

    const uint32_t currSize = m_outNodes.size();
    m_outNodes.resize(currSize + 4);

    if ((leftNode.leftOffset & LEAF_BIT) == 0 && (rightNode.leftOffset & LEAF_BIT) == 0) // directly get 4 nodes
    {
      m_outNodes[currSize + 0] = m_inNodes[leftNode.leftOffset + 0];
      m_outNodes[currSize + 1] = m_inNodes[leftNode.leftOffset + 1];
      m_outNodes[currSize + 2] = m_inNodes[rightNode.leftOffset + 0];
      m_outNodes[currSize + 3] = m_inNodes[rightNode.leftOffset + 1];
    }
    else if ((leftNode.leftOffset & LEAF_BIT) == 0)
    {
      m_outNodes[currSize + 0] = m_inNodes[leftNode.leftOffset + 0];
      m_outNodes[currSize + 1] = m_inNodes[leftNode.leftOffset + 1];
      m_outNodes[currSize + 2] = rightNode;
      m_outNodes[currSize + 3] = DummyNode();
    }
    else if ((rightNode.leftOffset & LEAF_BIT) == 0)
    {
      m_outNodes[currSize + 0] = leftNode;
      m_outNodes[currSize + 1] = m_inNodes[rightNode.leftOffset + 0];
      m_outNodes[currSize + 2] = m_inNodes[rightNode.leftOffset + 1];
      m_outNodes[currSize + 3] = DummyNode();
    }

    m_outNodes[currSize + 0].leftOffset = Visit(m_outNodes[currSize + 0].leftOffset);
    m_outNodes[currSize + 1].leftOffset = Visit(m_outNodes[currSize + 1].leftOffset);
    m_outNodes[currSize + 2].leftOffset = Visit(m_outNodes[currSize + 2].leftOffset);
    m_outNodes[currSize + 3].leftOffset = Visit(m_outNodes[currSize + 3].leftOffset);

    return currSize;
  }

  const std::vector<BVHNode> &m_inNodes;
  std::vector<BVHNode> &m_outNodes;
};

std::vector<BVHNode> BVH2ToBVH4(const std::vector<BVHNode> &a_nodes)
{
  std::vector<BVHNode> result;
  if (a_nodes.size() == 2)
  {
    result = a_nodes;
    result.push_back(DummyNode());
    result.push_back(DummyNode());
  }
  else
  {
    Converter2to4 converter(a_nodes, result);
    converter.Visit(0);
  }
  return result;
}

BVHTree BuildBVHSmall(const float4* a_vertices, size_t a_vertNum, const uint* a_indices, size_t a_indexNum, BVHPresets a_presets)
{
  LiteMath::Box4f box;
  for(size_t i=0;i<a_vertNum;i++)
    box.include(a_vertices[i]); 
    
  BVHTree treeData;
  //treeData.indicesReordered = std::vector<uint32_t>(a_indices, a_indices+a_indexNum);
  treeData.indicesReordered = {0,1};
    
  treeData.nodes.resize(a_presets.childrenNum+1);
  treeData.intervals.resize(a_presets.childrenNum+1);
  treeData.nodes[0].boxMin = to_float3(box.boxMin); // - float3(0.1f, 0.1f, 0.1f);
  treeData.nodes[0].boxMax = to_float3(box.boxMax); // + float3(0.1f, 0.1f, 0.1f);;
  treeData.nodes[0].leftOffset  = 1;
  treeData.nodes[0].escapeIndex = uint32_t(-2);
  treeData.intervals[0] = Interval(0, uint32_t(a_indexNum/3));
    
  treeData.nodes[1].boxMin = to_float3(box.boxMin);
  treeData.nodes[1].boxMax = to_float3(box.boxMax);
  treeData.nodes[1].leftOffset  = uint32_t(-1);
  treeData.nodes[1].escapeIndex = (a_presets.desiredFormat == FMT_BVH2Node32_Interval32_Dynamic) ? uint32_t(-1) : 2;
  treeData.intervals[1]         = Interval(0, uint32_t(a_indexNum/3));
  for(int i=2;i<a_presets.childrenNum+1;i++)
  {
    treeData.nodes[i].boxMin      = float3(0.0f);
    treeData.nodes[i].boxMax      = float3(0.0f);
    treeData.nodes[i].leftOffset  = uint32_t(-3); // invalid node
    treeData.nodes[i].escapeIndex = (a_presets.desiredFormat == FMT_BVH2Node32_Interval32_Dynamic) ? uint32_t(-1) : i+1;
    treeData.intervals[i]         = Interval(0, 0);
  }
  if(a_presets.desiredFormat == FMT_BVH4Node32_Interval32_Static || a_presets.desiredFormat == FMT_BVH2Node32_Interval32_Static)
    treeData.nodes[treeData.nodes.size()-1].escapeIndex = uint32_t(-2);
  treeData.format = a_presets.desiredFormat;
  return treeData;
}

static void PrintNode(uint32_t currNodeOffset, uint32_t currDeep, const BVHTree& a_tree, std::ostream& out, std::unordered_set<uint32_t>& a_visited)
{
  auto p = a_visited.find(currNodeOffset);
  if(p == a_visited.end())
    a_visited.insert(currNodeOffset);
  else
  {
    std::cout << "node at " << currNodeOffset << " visited twice!" << std::endl;
    out << "node at " << currNodeOffset << " visited twice!" << std::endl;
    std::cout.flush();
    out.flush();
    return;
  }

  const auto currNode = a_tree.nodes[currNodeOffset];
  const bool isLeaf   = (currNode.leftOffset == LEAF_NORMAL || currNode.leftOffset == LEAF_EMPTY);
  for(uint32_t i=0;i<currDeep;i++)
    out << " ";
  out << " offset = " << currNodeOffset << "; ";
  if(isLeaf)
    out << " leaf : [" << a_tree.intervals[currNodeOffset].start << ":" << a_tree.intervals[currNodeOffset].count << "]";
  else
    out << " node : [" << a_tree.intervals[currNodeOffset].start << ":" << a_tree.intervals[currNodeOffset].count << "]";
  out << std::endl;

  for(uint32_t i=0;i<currDeep;i++)
    out << " ";
  out << " bbox   = {(" << currNode.boxMin.x << ", " << currNode.boxMin.y << ", " << currNode.boxMin.z << ") --";
  out << " (" << currNode.boxMax.x << ", " << currNode.boxMax.y << ", " << currNode.boxMax.z << ")}" << std::endl;

  if(isLeaf)
    return;

  if(a_tree.format == CBVH_FORMATS::FMT_BVH2Node32_Interval32_Static)
  {
    PrintNode(currNode.leftOffset + 0, currDeep + 1, a_tree, out, a_visited);
    PrintNode(currNode.leftOffset + 1, currDeep + 1, a_tree, out, a_visited);
  }
  else if(a_tree.format == CBVH_FORMATS::FMT_BVH2Node32_Interval32_Dynamic)
  {
    PrintNode(currNode.leftOffset , currDeep + 1, a_tree, out, a_visited);
    PrintNode(currNode.escapeIndex, currDeep + 1, a_tree, out, a_visited);
  }
  else if(a_tree.format == CBVH_FORMATS::FMT_BVH4Node32_Interval32_Static)
  {
    PrintNode(currNode.leftOffset + 0, currDeep + 1, a_tree, out, a_visited);
    PrintNode(currNode.leftOffset + 1, currDeep + 1, a_tree, out, a_visited);
    PrintNode(currNode.leftOffset + 2, currDeep + 1, a_tree, out, a_visited);
    PrintNode(currNode.leftOffset + 3, currDeep + 1, a_tree, out, a_visited);
  }
  else // unsupported format
  {

  }
  
}

void BVHTree::Print(std::ostream& out)
{
  std::unordered_set<uint32_t> visitedNodes;
  visitedNodes.reserve(nodes.size());
  visitedNodes.clear();

  auto oldPrec = out.precision(3);
  PrintNode(0,0,*this,out,visitedNodes);
  out.precision(oldPrec);
}

BVHTreeCommon BuildBVH(const float *a_vpos3f, size_t a_vertNum, size_t a_vByteStride,
                                     const uint32_t *a_indices, size_t a_indexNum, BuilderPresets a_presets)
{
  const float4 *inVertices = (const float4 *)a_vpos3f;
  std::vector<float4> vertDataTemp;

  g_buildTris += a_indexNum / 3;

  if (a_vByteStride != 16)
  {
    vertDataTemp.resize(a_vertNum);
    const size_t stride = a_vByteStride / 4;
    for (size_t i = 0; i < a_vertNum; i++)
    {
      vertDataTemp[i].x = a_vpos3f[i * stride + 0];
      vertDataTemp[i].y = a_vpos3f[i * stride + 1];
      vertDataTemp[i].z = a_vpos3f[i * stride + 2];
      vertDataTemp[i].w = 1.0f;
    }
    inVertices = vertDataTemp.data();
  }

  BVHPresets presets1;
  presets1.primsInLeaf = a_presets.primsInLeaf;
  presets1.quality = a_presets.quality;

  bool convert2to4 = false;
  if (a_presets.format == BVH4_LEFT_OFFSET)
  {
    a_presets.format = BVH2_LEFT_OFFSET;
    convert2to4 = true;
  }

  switch (a_presets.format)
  {
  case BVH2_LEFT_OFFSET:
    presets1.childrenNum = 2;
    presets1.desiredFormat = FMT_BVH2Node32_Interval32_Static;
    break;

  case BVH2_LEFT_RIGHT:
    presets1.childrenNum = 2;
    presets1.desiredFormat = FMT_BVH2Node32_Interval32_Dynamic;
    break;

  case BVH4_LEFT_OFFSET:
    presets1.childrenNum = 4;
    presets1.desiredFormat = FMT_BVH4Node32_Interval32_Static;
    break;

  default:
  {
    std::cout << "[BuildBVH] ERROR: wrong input format = " << int(a_presets.format) << std::endl;
    return BVHTreeCommon();
  }
  break;
  }

  BVHTree bvhData;
  printf("params %d %d (%d < %d)\n",(int)presets1.quality, (int)presets1.childrenNum, (int)a_indexNum/3, (int)size_t(presets1.primsInLeaf));
  if (a_indexNum/3 <= size_t(a_presets.primsInLeaf))
    bvhData = BuildBVHSmall(inVertices, a_vertNum, a_indices, a_indexNum, presets1);
  else 
    bvhData = BuildBVHEmbree(inVertices, a_vertNum, a_indices, a_indexNum, presets1);

  for (size_t i = 0; i < bvhData.nodes.size(); i++)
  {
    if (int(bvhData.nodes[i].leftOffset) < 0)
      bvhData.nodes[i].leftOffset = PackOffsetAndSize(bvhData.intervals[i].start, bvhData.intervals[i].count);
  }

  if (a_presets.format == BVH4_LEFT_OFFSET) //
  {
    auto nodesAligned = AlignNodes4(bvhData.nodes);
    // CheckNodesAlign(nodesAligned, 4);
    return BVHTreeCommon(nodesAligned, bvhData.indicesReordered);
  }
  else if (convert2to4)
  {
    auto nodesAligned = BVH2ToBVH4(AlignNodes2(bvhData.nodes));
    return BVHTreeCommon(nodesAligned, bvhData.indicesReordered);
  }
  else
  {
    if (a_presets.format == BVH2_LEFT_RIGHT) // assume we don't have to aligh "BVH2_LEFT_RIGHT"
    {
      for (auto &node : bvhData.nodes) // other builders save true escapeIndex in 'escapeIndex' field
        node.escapeIndex = node.leftOffset + 1;
      return BVHTreeCommon(bvhData.nodes, bvhData.indicesReordered);
    }
    else
    {
      auto nodesAligned = AlignNodes2(bvhData.nodes);
      // CheckNodesAlign(nodesAligned, 2);
      return BVHTreeCommon(nodesAligned, bvhData.indicesReordered);
    }
  }
}

struct TraverseDFS_ESCI2
{
  TraverseDFS_ESCI2(const std::vector<BVHNode> &a_nodesOld,
                    std::vector<uint32_t> &a_escape) : oldNodes(a_nodesOld), escape(a_escape) {}

  void VisitNode(uint32_t a_nodeOffset, uint32_t a_parentESCI)
  {
    if ((a_nodeOffset & LEAF_BIT) != 0)
      return;
    escape[a_nodeOffset] = a_parentESCI;
    const auto &currNode = oldNodes[a_nodeOffset];
    VisitNode(currNode.leftOffset, currNode.escapeIndex);
    VisitNode(currNode.escapeIndex, a_parentESCI);
  }

protected:
  const std::vector<BVHNode> &oldNodes;
  std::vector<uint32_t> &escape;
};

std::vector<BVHNode> RightOffsToEscapeIndex(const std::vector<BVHNode> &a_tree) // todo: make this algorithm non recursive
{
  std::vector<BVHNode> res(a_tree.size());
  std::vector<uint32_t> escape(a_tree.size());

  // for(uint32_t i=0;i<uint32_t(parent.size());i++)
  //{
  //   parent[i] = uint32_t(-1);
  //   escape[i] = ESCAPE_ROOT;
  // }
  //
  // for(uint32_t i=0;i<uint32_t(a_tree.size());i++)
  //{
  //   const auto& currNode  = a_tree[i];
  //   const bool isLeaf = ((currNode.leftOffset & LEAF_BIT) != 0);
  //   if(!isLeaf)
  //   {
  //     parent[currNode.leftOffset]  = i;
  //     parent[currNode.escapeIndex] = i;
  //     escape[currNode.leftOffset]  = currNode.escapeIndex; // escape(left) = right
  //   }
  // }

  TraverseDFS_ESCI2 trav(a_tree, escape);
  trav.VisitNode(0, ESCAPE_ROOT); // todo: implement tree top-down and bottom-up traversal in a different way in loops

  for (uint32_t i = 0; i < uint32_t(a_tree.size()); i++)
  {
    auto currNode = a_tree[i];
    currNode.escapeIndex = escape[i];
    res[i] = currNode;
  }

  return res;
}

BVHTree BuildBVH(const BVHNode *a_nodes, size_t a_objNum, BuilderPresets a_presets)
{
  const BVHNode *input = a_nodes;
  std::vector<BVHNode> copy;

  // set indices inside 'box-nodes' for internal builder
  copy = std::vector<BVHNode>(a_nodes, a_nodes + a_objNum);
  for (size_t i = 0; i < copy.size(); i++)
  {
    copy[i].leftOffset = uint32_t(i);
    copy[i].escapeIndex = 1;
  }
  input = copy.data();

  /*
  prepare primitives
  embree::build(quality, primBoxData, primTriData, (a_indexNum/6), res);
  */
  std::vector<RTCBuildPrimitive> primBoxData(a_objNum);
  for (int i = 0; i < a_objNum; i++)
  {
    RTCBuildPrimitive prim;
    prim.lower_x = input[i].boxMin.x;
    prim.lower_y = input[i].boxMin.y;
    prim.lower_z = input[i].boxMin.z;
    prim.geomID = 0;
    prim.upper_x = input[i].boxMax.x;
    prim.upper_y = input[i].boxMax.y;
    prim.upper_z = input[i].boxMax.z;
    prim.primID = (unsigned)i;
    primBoxData[i] = prim;
  }
  RTCBuildQuality quality = RTC_BUILD_QUALITY_HIGH;
  if (a_presets.quality == BVHQuality::LOW)
    quality = RTC_BUILD_QUALITY_LOW;
  else if (a_presets.quality == BVHQuality::MEDIUM)
    quality = RTC_BUILD_QUALITY_MEDIUM;
  else if (a_presets.quality == BVHQuality::HIGH)
    quality = RTC_BUILD_QUALITY_HIGH;
  printf("Build !!!\n");
  embree::g_recommendedPrimsInLeaf = a_presets.primsInLeaf;
  BVHTree lbvh;
  embree::build(quality, primBoxData, (a_objNum / 2), lbvh);
  if (a_presets.format == BVH2_LEFT_ROPES)
    lbvh.nodes = RightOffsToEscapeIndex(lbvh.nodes);

  // restricted way for TLAS, single BLAS per leaf
  bool isNotOne = false;
  for (size_t i = 0; i < lbvh.nodes.size(); i++)
  {
    if (int(lbvh.nodes[i].leftOffset) < 0 && lbvh.nodes[i].leftOffset != 0x80000000)
    {
      const uint32_t objectId = lbvh.indicesReordered[lbvh.intervals[i].start];
      //printf("%d obj interval %d %d\n", (int)i, (int)objectId, (int)lbvh.intervals[i].count);
      lbvh.nodes[i].leftOffset = PackOffsetAndSize(objectId, lbvh.intervals[i].count);
      if (lbvh.intervals[i].count > 1)
        isNotOne = true;
    }
  }

  if (isNotOne)
    std::cout << "[BuildBVH] warning: leaf contains more than one primitive!" << std::endl;

  if (a_presets.format == BVH2_LEFT_OFFSET)
    lbvh.nodes = AlignNodes2(lbvh.nodes);
  
  return lbvh;
}