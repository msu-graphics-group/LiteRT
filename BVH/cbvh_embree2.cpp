#include <iostream>
#include <queue>
#include <stack>
#include <limits>
#include <tuple>
#include <cmath>
#include <chrono>

#include "cbvh.h"

using LiteMath::Box4f;

using LiteMath::float3;
using LiteMath::float4;
using LiteMath::float4x4;
using LiteMath::uint;
using LiteMath::min;
using LiteMath::max;

#include "embree3/rtcore.h"

extern double g_buildTime;

namespace embree
{
  uint32_t g_recommendedPrimsInLeaf = 4;
  float    g_overSplitThreshold     = 0.25f;
  
  /* This function is called by the builder to signal progress and to
   * report memory consumption. */
  bool memoryMonitor(void* userPtr, ssize_t bytes, bool post) {
    return true;
  }

  bool buildProgress (void* userPtr, double f) {
    return true;
  }

  float RelativeBoxOverlap(const Box4f& box1, const Box4f& box2) //#TODO: opt this with transpose4 funct
  {
    const Box4f ovlp = BoxBoxOverlap(box1, box2);
  
    const float sa1 = box1.surfaceArea();
    const float sa2 = box2.surfaceArea();
    const float sa3 = ovlp.surfaceArea();
  
    return std::min(sa3/std::min(sa1,sa2), 1.0f);
  }

  struct Node
  {
    Node(){}
    virtual ~Node(){}
    virtual float sah() const = 0;
    virtual bool  isLeaf() const = 0;
    uint32_t primsInNode;
  };

  static inline Box4f merge(const Box4f& a, const Box4f& b) { return Box4f(min(a.boxMin, b.boxMin), max(a.boxMax, b.boxMax)); }

  struct InnerNode : public Node
  {
    Box4f bounds[2];
    Node* children[2];
  
    InnerNode() {
      bounds  [0] = bounds[1]   = Box4f();
      children[0] = children[1] = nullptr;
      primsInNode = 0;
    }

    float sah() const override { 
      return 1.0f + (bounds[0].surfaceArea()*children[0]->sah() + bounds[1].surfaceArea()*children[1]->sah())/merge(bounds[0],bounds[1]).surfaceArea();
    }

    bool  isLeaf() const override { return false; }

    static void* create (RTCThreadLocalAllocator alloc, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(InnerNode),16);
      return (void*) new (ptr) InnerNode;
    }

    static void  setChildren (void* nodePtr, void** childPtr, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->children[i] = (Node*) childPtr[i];
    }

    static void  setBounds (void* nodePtr, const RTCBounds** bounds, unsigned int numChildren, void* userPtr)
    {
      assert(numChildren == 2);
      for (size_t i=0; i<2; i++)
        ((InnerNode*)nodePtr)->bounds[i] = *(const Box4f*) bounds[i];
    }
  };

  struct LeafNode : public Node
  {
    unsigned id;
    Box4f bounds;

    LeafNode (unsigned a_id, const Box4f& a_bounds) : id(a_id), bounds(a_bounds) {}

    float sah() const override {
      return 1.0f;
    }

    bool  isLeaf() const override { return true; }

    static void* create (RTCThreadLocalAllocator alloc, const RTCBuildPrimitive* prims, size_t numPrims, void* userPtr)
    {
      assert(numPrims == 1);
      void* ptr = rtcThreadLocalAlloc(alloc,sizeof(LeafNode),16);
      return (void*) new (ptr) LeafNode(prims->primID,*(Box4f*)prims);
    }
  };

  uint32_t EvaluatePrimsInNode(Node* node)
  {
    if(node == nullptr)
      return 0;

    if(node->isLeaf())
    {
      node->primsInNode = 1;
      return 1;
    }
    
    InnerNode* pInnderNode = (InnerNode*)node;
    node->primsInNode      = EvaluatePrimsInNode(pInnderNode->children[0]) + EvaluatePrimsInNode(pInnderNode->children[1]); 
    return node->primsInNode;
  }

  std::vector<uint32_t> g_prims;

  void GetPimListToGPrims(Node* node)
  {
    if(node == nullptr)
      return;
    
    if(node->isLeaf())
    {
      LeafNode* leaf = (LeafNode*)node;
      g_prims.push_back(leaf->id);
    }  
    else
    {
      InnerNode* pInnderNode = (InnerNode*)node;
      GetPimListToGPrims(pInnderNode->children[0]);
      GetPimListToGPrims(pInnderNode->children[1]);
    }
  }  

  struct NextNode
  {
    uint32_t       leftOffset = uint32_t(-1);
    Interval interval   = Interval(0,0);
  };

  NextNode ConvertTreeRec(Node* node, BVHTree& a_res, uint32_t a_parentEscapeIndex)
  {
    if(node == nullptr)
      return NextNode();
    
    NextNode nodeData;
    if(node->isLeaf())
    {
      LeafNode* leaf = (LeafNode*)node;
      const uint32_t oldTris = a_res.indicesReordered.size();
      a_res.indicesReordered.push_back(leaf->id);
      
      BVHNode currNode;
      currNode.boxMin = to_float3(leaf->bounds.boxMin);
      currNode.boxMax = to_float3(leaf->bounds.boxMax);
      currNode.leftOffset  = LEAF_NORMAL;
      currNode.escapeIndex = LEAF_NORMAL;
      nodeData.interval    = Interval(oldTris, 1);
      a_res.intervals.push_back(nodeData.interval);
      a_res.nodes.push_back(currNode);
      return nodeData;      
    }

    if(node->primsInNode <= g_recommendedPrimsInLeaf) // #TODO: estimate relative bounding box overlap, probably we can do better than 4 triangle in leaf
    {
      InnerNode* pInnderNode = (InnerNode*)node;
      const float childrenOverlap = RelativeBoxOverlap(pInnderNode->bounds[0], pInnderNode->bounds[1]);
      //std::cout << "childrenOverlap = " << childrenOverlap << std::endl;
      if(childrenOverlap > g_overSplitThreshold)  // 0.25--0.5f
      {
        g_prims.resize(0);
        GetPimListToGPrims(node);
        
        // put triangles in side reordered buffer
        //
        const uint32_t oldTris = a_res.indicesReordered.size();
        a_res.indicesReordered.resize(a_res.indicesReordered.size() + g_prims.size());
        for(size_t triId = 0; triId < g_prims.size(); triId++)
          a_res.indicesReordered[oldTris + triId] = g_prims[triId];
        
        BVHNode currNode;
        {
          currNode.boxMin = to_float3(min(pInnderNode->bounds[0].boxMin, pInnderNode->bounds[1].boxMin));
          currNode.boxMax = to_float3(max(pInnderNode->bounds[0].boxMax, pInnderNode->bounds[1].boxMax));
          currNode.leftOffset  = LEAF_NORMAL;
          currNode.escapeIndex = LEAF_NORMAL;
        }
        nodeData.interval = Interval(oldTris, uint32_t(g_prims.size()));
        a_res.intervals.push_back(nodeData.interval);
        a_res.nodes.push_back(currNode);
        return nodeData;
      }
    }
    
    
    InnerNode* pInnderNode = (InnerNode*)node;
    BVHNode leftNode, rightNode;
    {
      leftNode.boxMin  = to_float3(pInnderNode->bounds[0].boxMin);
      leftNode.boxMax  = to_float3(pInnderNode->bounds[0].boxMax);
      rightNode.boxMin = to_float3(pInnderNode->bounds[1].boxMin);
      rightNode.boxMax = to_float3(pInnderNode->bounds[1].boxMax);
      
      leftNode.leftOffset   = LEAF_NORMAL;
      leftNode.escapeIndex  = LEAF_NORMAL;
      rightNode.leftOffset  = LEAF_NORMAL;
      rightNode.escapeIndex = LEAF_NORMAL;
    }
      
    nodeData.leftOffset = uint32_t(a_res.nodes.size());
    a_res.intervals.push_back(Interval(0,0));
    a_res.intervals.push_back(Interval(0,0));
    a_res.nodes.push_back(leftNode);
    a_res.nodes.push_back(rightNode);
    
    a_res.nodes[nodeData.leftOffset + 0].escapeIndex = nodeData.leftOffset + 1;
    a_res.nodes[nodeData.leftOffset + 1].escapeIndex = a_parentEscapeIndex;

    NextNode dataLeft  = ConvertTreeRec(pInnderNode->children[0], a_res, a_res.nodes[nodeData.leftOffset + 0].escapeIndex);
    NextNode dataRight = ConvertTreeRec(pInnderNode->children[1], a_res, a_res.nodes[nodeData.leftOffset + 1].escapeIndex);

    a_res.nodes[nodeData.leftOffset + 0].leftOffset = dataLeft.leftOffset;  // fix leftOffset after we done with children
    a_res.nodes[nodeData.leftOffset + 1].leftOffset = dataRight.leftOffset; // fix leftOffset after we done with children
    a_res.intervals[nodeData.leftOffset + 0] = dataLeft.interval;           // same for intervals
    a_res.intervals[nodeData.leftOffset + 1] = dataRight.interval;          // same for intervals

    assert(dataLeft.interval.start + dataLeft.interval.count == dataRight.interval.start);
    nodeData.interval = Interval(dataLeft.interval.start, dataLeft.interval.count + dataRight.interval.count);
    
    return nodeData;
  }

  void ConvertTree(Node* node, BVHTree& a_res)
  {
    uint32_t totalPrims = EvaluatePrimsInNode(node);
    
    a_res.nodes.resize(0);            a_res.nodes.reserve(totalPrims/2);
    a_res.intervals.resize(0);        a_res.intervals.reserve(totalPrims/2);
    a_res.indicesReordered.resize(0); a_res.indicesReordered.reserve(totalPrims);
    
    g_prims.reserve(32);
    
    // (1) put root node first
    //
    assert(!node->isLeaf());
    InnerNode* pInnderNode = (InnerNode*)node;
    BVHNode root;
    {
      root.boxMin = to_float3(min(pInnderNode->bounds[0].boxMin, pInnderNode->bounds[1].boxMin));
      root.boxMax = to_float3(max(pInnderNode->bounds[0].boxMax, pInnderNode->bounds[1].boxMax));  
      root.leftOffset  = LEAF_NORMAL;
      root.escapeIndex = ESCAPE_ROOT;
    }

    a_res.intervals.push_back(Interval(0,pInnderNode->primsInNode));
    a_res.nodes.push_back(root);

    // (2) then convert tree format to out BVH2_Static format
    //
    NextNode rootData         = ConvertTreeRec(node, a_res, 0xFFFFFFFE);
    a_res.nodes[0].leftOffset = rootData.leftOffset; 
    a_res.intervals[0]        = rootData.interval;
  }

  void build(RTCBuildQuality quality, std::vector<RTCBuildPrimitive>& prims_i, size_t extraSpace, BVHTree& a_res)
  {
    printf("ITS ALIVE\n");
    auto a_device = rtcNewDevice("");
    rtcSetDeviceMemoryMonitorFunction(a_device,memoryMonitor,nullptr);

    RTCBVH bvh = rtcNewBVH(a_device);

    auto& prims = prims_i;
    prims.reserve(prims_i.size()+extraSpace);
    prims.resize(prims_i.size());

    /* settings for BVH build */
    RTCBuildArguments arguments = rtcDefaultBuildArguments();
    arguments.byteSize = sizeof(arguments);
    arguments.buildFlags = RTC_BUILD_FLAG_NONE; //RTC_BUILD_FLAG_DYNAMIC; // ????????????????????
    arguments.buildQuality = quality;
    arguments.maxBranchingFactor = 2;
    arguments.maxDepth = 1024;
    arguments.sahBlockSize = 1;
    arguments.minLeafSize = 1;
    arguments.maxLeafSize = 1;
    arguments.traversalCost = 1.0f;
    arguments.intersectionCost = 1.0f;
    arguments.bvh = bvh;
    arguments.primitives     = prims.data();
    arguments.primitiveCount = prims.size();
    arguments.primitiveArrayCapacity = prims.capacity();
    arguments.createNode = InnerNode::create;
    arguments.setNodeChildren = InnerNode::setChildren;
    arguments.setNodeBounds = InnerNode::setBounds;
    arguments.createLeaf = LeafNode::create;
    arguments.splitPrimitive = nullptr;
    arguments.buildProgress = buildProgress;
    arguments.userPtr = nullptr;
    
    auto before = std::chrono::high_resolution_clock::now();
    Node* root = (Node*) rtcBuildBVH(&arguments);
    float time = std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - before).count()/1000.f;
    g_buildTime += double(time);

    ConvertTree(root, a_res);

    rtcReleaseBVH(bvh);
    rtcReleaseDevice(a_device); 
  }
}

BVHTree BuildBVHEmbree(const float4 *a_vertices, size_t a_vertNum, const uint *a_indices, size_t a_indexNum, BVHPresets a_presets)
{
  RTCBuildQuality quality = RTC_BUILD_QUALITY_HIGH;
  if (a_presets.quality == BVHQuality::LOW)
    quality = RTC_BUILD_QUALITY_LOW;
  else if (a_presets.quality == BVHQuality::MEDIUM)
    quality = RTC_BUILD_QUALITY_MEDIUM;
  else if (a_presets.quality == BVHQuality::HIGH)
    quality = RTC_BUILD_QUALITY_HIGH;

  embree::g_recommendedPrimsInLeaf = a_presets.primsInLeaf;
  embree::g_overSplitThreshold = 0.5f;
  if (a_presets.primsInLeaf >= 4)
    embree::g_overSplitThreshold = 0.25f;

  std::vector<RTCBuildPrimitive> primBoxData(a_indexNum / 3);

  for (size_t i = 0; i < a_indexNum / 3; i++)
  {
    const uint iA = a_indices[i * 3 + 0];
    const uint iB = a_indices[i * 3 + 1];
    const uint iC = a_indices[i * 3 + 2];

    Box4f box;
    box.boxMin = min(a_vertices[iA], min(a_vertices[iB], a_vertices[iC]));
    box.boxMax = max(a_vertices[iA], max(a_vertices[iB], a_vertices[iC]));

    RTCBuildPrimitive prim;
    prim.lower_x = box.boxMin.x;
    prim.lower_y = box.boxMin.y;
    prim.lower_z = box.boxMin.z;
    prim.geomID = 0;
    prim.upper_x = box.boxMax.x;
    prim.upper_y = box.boxMax.y;
    prim.upper_z = box.boxMax.z;
    prim.primID = (unsigned)i;
    primBoxData[i] = prim;
  }

  BVHTree res;
  embree::build(quality, primBoxData, (a_indexNum / 6), res);
  res.format = CBVH_FORMATS::FMT_BVH2Node32_Interval32_Static;
  return res;
}