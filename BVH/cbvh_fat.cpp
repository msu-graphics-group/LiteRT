#include <fstream>
#include <chrono>
#include <unordered_set>

#include "cbvh.h"

using LiteMath::float4;

#include "../raytrace_common.h"

LayoutPresets LayoutPresetsFromString(const char* a_str)
{
  LayoutPresets layout;
  std::string m_layoutName(a_str);

  if(m_layoutName == "DepthFirst" || m_layoutName == "DFS" || m_layoutName == "DFL")
  {
    layout.layout = LAYOUT_DFS;
    layout.grSzId = 0;
    layout.grSzXX = 0;
  }
  else if(m_layoutName == "OrderedDepthFirst" || m_layoutName == "ODFS" || m_layoutName == "ODFL")
  {
    layout.layout = LAYOUT_ODFS;
    layout.grSzId = 0;
    layout.grSzXX = 0;
  }
  else if(m_layoutName == "BreadthFirst" || m_layoutName == "BFS" || m_layoutName == "BFL")
  {
    layout.layout = LAYOUT_BFS;
    layout.grSzId = 0;
    layout.grSzXX = 0;
  }
  else if (m_layoutName.find("Treelet") != std::string::npos || m_layoutName.find("TRB") != std::string::npos)
  {
    size_t leavesNumberInTreelet = 4;
    std::string str = m_layoutName.substr(m_layoutName.size() - 2);

    if (str.find_first_not_of("0123456789") == std::string::npos) 
      leavesNumberInTreelet = stoi(str);
    else
    {
      str = m_layoutName.substr(m_layoutName.size() - 1);
      if (str.find_first_not_of("0123456789") == std::string::npos)
        leavesNumberInTreelet = stoi(str);
    }

    layout.grSzId = std::log2(int(leavesNumberInTreelet))-1;
    layout.grSzXX = int(leavesNumberInTreelet); // actual size is 2*int(leavesNumberInTreelet)-1;

    if (m_layoutName.find("SuperSuperTreelet") != std::string::npos)
      layout.layout = LAYOUT_COLBVH_TRB3;
    else if (m_layoutName.find("SuperTreelet") != std::string::npos)
      layout.layout = LAYOUT_COLBVH_TRB2;
    else
      layout.layout = LAYOUT_COLBVH_TRB1;
   
    const bool a_aligned = m_layoutName.find("Aligned") != std::string::npos;
    const bool a_merge   = m_layoutName.find("Merged") != std::string::npos;

    if (m_layoutName.find("SuperTreelet") != std::string::npos && a_aligned && a_merge)
      layout.layout = LAYOUT_CALBVH;
  }
  else if (m_layoutName.find("Clusterized") != std::string::npos || m_layoutName.find("TRB") != std::string::npos || m_layoutName == "opt")
  {
    size_t cluster_size = 31; // default of MAX size of the lowest-level (smallest) cluster
    std::string str = m_layoutName.substr(m_layoutName.size() - 2);
    if (str.find_first_not_of("0123456789") == std::string::npos)
      cluster_size = stoi(str);
    else
    {
      str = m_layoutName.substr(m_layoutName.size() - 1);
      if (str.find_first_not_of("0123456789") == std::string::npos)
        cluster_size = stoi(str);
    }
    
    layout.layout = LAYOUT_COLBVH_YM06;
    layout.grSzId = std::log2(int(cluster_size+1))-3;
    layout.grSzXX = int(cluster_size);          // actual size is cluster_size+1;
  }

  return layout;
}

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

struct TreeConverter2
{
  TreeConverter2(const std::vector<BVHNode>&  a_input,
                 std::vector<BVHNodePair>&     a_out) : m_input(a_input),
                                                       m_out(a_out) {}

    
  const std::vector<BVHNode>&    m_input;
  std::vector<BVHNodePair>&       m_out;
  uint32_t ProcessBVHNode(uint32_t currNodeId);
};

uint32_t TreeConverter2::ProcessBVHNode(uint32_t leftOffset)
{
  assert(leftOffset + 1 < m_input.size());
  assert(m_out.capacity() > m_out.size()+1);

  const BVHNode leftNode  = m_input[leftOffset + 0];
  const BVHNode rightNode = m_input[leftOffset + 1];

  m_out.push_back(BVHNodePair());
  uint32_t currNodeIndex  = uint32_t(m_out.size()-1);
  BVHNodePair& fatNode     = m_out[currNodeIndex];
  fatNode.left = leftNode;
  fatNode.right = rightNode;

  if(leftNode.leftOffset & LEAF_BIT)
    fatNode.left.leftOffset = leftNode.leftOffset;
  else
    fatNode.left.leftOffset = ProcessBVHNode(leftNode.leftOffset);

  if(rightNode.leftOffset & LEAF_BIT)
    fatNode.right.leftOffset = rightNode.leftOffset;
  else
    fatNode.right.leftOffset = ProcessBVHNode(rightNode.leftOffset);
  
  return currNodeIndex;
}


std::vector<BVHNodePair> CreateFatTreeArray(const std::vector<BVHNode>& a_input)
{
  std::vector<BVHNodePair> result;
  result.reserve(a_input.size()); // this is important, array should not be reallocated!
  result.resize(0);

  //for(size_t i=0;i<a_input.size();i++)
  //  std::cout << i << ": (" << a_input[i].leftOffset << "," << a_input[i].escapeIndex << ")" << std::endl;

  TreeConverter2 tc(a_input, result);
  tc.ProcessBVHNode(0);
  
  result.shrink_to_fit();         
  return result;
}


///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
double g_buildTime;
uint64_t g_buildTris;

BVHTreeFat BuildBVHFat(const float* a_vpos3f,     size_t a_vertNum, size_t a_vByteStride, 
                       const uint32_t* a_indices, size_t a_indexNum, std::vector<uint32_t> &startCount,
                       BuilderPresets a_presets, LayoutPresets a_layout)
{
  std::vector<BVHNodePair> bvhFat;
  std::vector<uint32_t>   objIndicesReordered;
  
  // (1) build
  {
    a_presets.format    = BVH2_LEFT_OFFSET;
    auto bvhData        = BuildBVH(a_vpos3f, a_vertNum, a_vByteStride, a_indices, a_indexNum, startCount, a_presets);
    bvhFat              = CreateFatTreeArray(bvhData.nodes);
    objIndicesReordered = bvhData.indices;
  }

  return BVHTreeFat(bvhFat, objIndicesReordered);
}

BVHTreeFat BuildBVHFatCustom(const BVHNode* a_nodes, size_t a_objNum, BuilderPresets a_presets, LayoutPresets a_layout)
{
  std::vector<BVHNodePair> bvhFat;
  std::vector<uint32_t>   objIndicesReordered;
  
  // (1) build
  {
    a_presets.format    = BVH2_LEFT_OFFSET;
    a_presets.primsInLeaf = 1;
    auto bvhData        = BuildBVH(a_nodes, a_objNum, a_presets);
    bvhFat              = CreateFatTreeArray(bvhData.nodes);
    objIndicesReordered = bvhData.indicesReordered;
  }

  return BVHTreeFat(bvhFat, objIndicesReordered);
}

struct PrintDFS
{
  PrintDFS(const std::vector<BVHNodePair>& a_nodes, std::ofstream& a_out) : nodes(a_nodes), out(a_out) {}
  
  const std::vector<BVHNodePair>& nodes; 
  std::ofstream& out;

  void VisitNode(uint32_t a_nodeOffset)
  {
    const auto& currNode = nodes[a_nodeOffset];

    // print node here
    out << a_nodeOffset << std::endl;

    if(!(currNode.left.leftOffset & LEAF_BIT)) {
      VisitNode(currNode.left.leftOffset);
      out << a_nodeOffset << " -> " << currNode.left.leftOffset << std::endl;
    }


    if(!(currNode.right.leftOffset & LEAF_BIT)) {
      VisitNode(currNode.right.leftOffset);
      out << a_nodeOffset << " -> " << currNode.right.leftOffset << std::endl;
    }
  }

};

void PrintForGraphViz(const std::vector<BVHNodePair>& a_nodes, const std::vector<int>& a_treeletRoots, const std::vector<int>& a_treeletRootsSuper, const char* a_fileName)
{
  std::ofstream fout(a_fileName);
  PrintDFS trav(a_nodes, fout);
  fout << "digraph D {" << std::endl;
  fout << "rankdir=\"LR\"" << std::endl;
  trav.VisitNode(0);
  
  if(a_treeletRoots.size() == 0) {
    fout << "}" << std::endl;
    return;
  }

  if(a_treeletRootsSuper.size() == 0) {
    for(size_t trId=0;trId<a_treeletRoots.size()-1;trId++) {
      int start = a_treeletRoots[trId];
      int end   = a_treeletRoots[trId+1];
      fout << "subgraph cluster_p" << trId <<  "{";
      for(int i=start; i<end;i++) 
        fout << i << ";";
      fout << "}" << std::endl;
    }
  }
  else {
    std::vector< std::vector<int> > superTreelsts(a_treeletRootsSuper.size());
    std::unordered_set<int> processedTreelets;
    for(size_t sId=0;sId<a_treeletRootsSuper.size()-1;sId++) {
      int start_s = a_treeletRootsSuper[sId+0];
      int end_s   = a_treeletRootsSuper[sId+1];
      for(size_t trId=0;trId<a_treeletRoots.size()-1;trId++) {
        int start = a_treeletRoots[trId+0];
        int end   = a_treeletRoots[trId+1];
        if(start >= start_s && end <= end_s) {
          superTreelsts[sId].push_back(trId);
          processedTreelets.insert(trId);
        }
      }
    }
    
    size_t clustId = 0;
    for(size_t j=0;j<superTreelsts.size();j++) {
      fout << "subgraph cluster_g" << j <<  "{" << std::endl;
      for(size_t trId=0;trId<superTreelsts[j].size();trId++,clustId++) {
        int trId2 = superTreelsts[j][trId];
        int start = a_treeletRoots[trId2+0];
        int end   = a_treeletRoots[trId2+1];
        fout << "subgraph cluster_p" << clustId <<  "{";
        for(int i=start; i<end;i++) 
          fout << i << ";";
        fout << "}" << std::endl;
      }
      fout << "}" << std::endl;
    }
    
    for(size_t trId=0;trId<a_treeletRoots.size()-1;trId++,clustId++) { // print treeelets that were not in 'superTreelsts' due to the last element process bug
      int start = a_treeletRoots[trId];
      int end   = a_treeletRoots[trId+1];
      if(processedTreelets.find(trId) == processedTreelets.end()) {
        fout << "subgraph cluster_p" << clustId <<  "{";
        for(int i=start; i<end;i++) 
          fout << i << ";";
        fout << "}" << std::endl;
      }
    }
      
  }

  fout << "}" << std::endl;
}
