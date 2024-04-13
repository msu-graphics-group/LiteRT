#include <vector>
#include <memory>
#include <limits>
#include <cassert>
#include <chrono>
#include <array>

#include "vk_copy.h"
#include "vk_context.h"
#include "vk_images.h"

#include "eye_ray_gpu.h"
#include "include/MultiRenderer_gpu_ubo.h"

static uint32_t ComputeReductionSteps(uint32_t whole_size, uint32_t wg_size)
{
  uint32_t steps = 0;
  while (whole_size > 1)
  {
    steps++;
    whole_size = (whole_size + wg_size - 1) / wg_size;
  }
  return steps;
}

constexpr uint32_t KGEN_FLAG_RETURN            = 1;
constexpr uint32_t KGEN_FLAG_BREAK             = 2;
constexpr uint32_t KGEN_FLAG_DONT_SET_EXIT     = 4;
constexpr uint32_t KGEN_FLAG_SET_EXIT_NEGATIVE = 8;
constexpr uint32_t KGEN_REDUCTION_LAST_STEP    = 16;


void MultiRenderer_GPU::UpdatePlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  const size_t maxAllowedSize = std::numeric_limits<uint32_t>::max();
  auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct.get());
  m_uboData.m_projInv = m_projInv;
  m_uboData.m_worldViewInv = m_worldViewInv;
  m_uboData.m_pAccelStruct_m_preset = pUnderlyingImpl->m_preset;
  m_uboData.m_preset = m_preset;
  m_uboData.m_height = m_height;
  m_uboData.m_width = m_width;
  m_uboData.m_pAccelStruct_m_ConjIndices_size     = uint32_t( m_pAccelStruct_m_ConjIndices->size() );     assert( m_pAccelStruct_m_ConjIndices->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_ConjIndices_capacity = uint32_t( m_pAccelStruct_m_ConjIndices->capacity() ); assert( m_pAccelStruct_m_ConjIndices->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_RFGridData_size     = uint32_t( m_pAccelStruct_m_RFGridData->size() );     assert( m_pAccelStruct_m_RFGridData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_RFGridData_capacity = uint32_t( m_pAccelStruct_m_RFGridData->capacity() ); assert( m_pAccelStruct_m_RFGridData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_RFGridSizes_size     = uint32_t( m_pAccelStruct_m_RFGridSizes->size() );     assert( m_pAccelStruct_m_RFGridSizes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_RFGridSizes_capacity = uint32_t( m_pAccelStruct_m_RFGridSizes->capacity() ); assert( m_pAccelStruct_m_RFGridSizes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfConjunctions_size     = uint32_t( m_pAccelStruct_m_SdfConjunctions->size() );     assert( m_pAccelStruct_m_SdfConjunctions->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfConjunctions_capacity = uint32_t( m_pAccelStruct_m_SdfConjunctions->capacity() ); assert( m_pAccelStruct_m_SdfConjunctions->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_size     = uint32_t( m_pAccelStruct_m_SdfFrameOctreeNodes->size() );     assert( m_pAccelStruct_m_SdfFrameOctreeNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_capacity = uint32_t( m_pAccelStruct_m_SdfFrameOctreeNodes->capacity() ); assert( m_pAccelStruct_m_SdfFrameOctreeNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_size     = uint32_t( m_pAccelStruct_m_SdfFrameOctreeRoots->size() );     assert( m_pAccelStruct_m_SdfFrameOctreeRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_capacity = uint32_t( m_pAccelStruct_m_SdfFrameOctreeRoots->capacity() ); assert( m_pAccelStruct_m_SdfFrameOctreeRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridData_size     = uint32_t( m_pAccelStruct_m_SdfGridData->size() );     assert( m_pAccelStruct_m_SdfGridData->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridData_capacity = uint32_t( m_pAccelStruct_m_SdfGridData->capacity() ); assert( m_pAccelStruct_m_SdfGridData->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridOffsets_size     = uint32_t( m_pAccelStruct_m_SdfGridOffsets->size() );     assert( m_pAccelStruct_m_SdfGridOffsets->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridOffsets_capacity = uint32_t( m_pAccelStruct_m_SdfGridOffsets->capacity() ); assert( m_pAccelStruct_m_SdfGridOffsets->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridSizes_size     = uint32_t( m_pAccelStruct_m_SdfGridSizes->size() );     assert( m_pAccelStruct_m_SdfGridSizes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfGridSizes_capacity = uint32_t( m_pAccelStruct_m_SdfGridSizes->capacity() ); assert( m_pAccelStruct_m_SdfGridSizes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfNeuralProperties_size     = uint32_t( m_pAccelStruct_m_SdfNeuralProperties->size() );     assert( m_pAccelStruct_m_SdfNeuralProperties->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfNeuralProperties_capacity = uint32_t( m_pAccelStruct_m_SdfNeuralProperties->capacity() ); assert( m_pAccelStruct_m_SdfNeuralProperties->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfObjects_size     = uint32_t( m_pAccelStruct_m_SdfObjects->size() );     assert( m_pAccelStruct_m_SdfObjects->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfObjects_capacity = uint32_t( m_pAccelStruct_m_SdfObjects->capacity() ); assert( m_pAccelStruct_m_SdfObjects->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfOctreeNodes_size     = uint32_t( m_pAccelStruct_m_SdfOctreeNodes->size() );     assert( m_pAccelStruct_m_SdfOctreeNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfOctreeNodes_capacity = uint32_t( m_pAccelStruct_m_SdfOctreeNodes->capacity() ); assert( m_pAccelStruct_m_SdfOctreeNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfOctreeRoots_size     = uint32_t( m_pAccelStruct_m_SdfOctreeRoots->size() );     assert( m_pAccelStruct_m_SdfOctreeRoots->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfOctreeRoots_capacity = uint32_t( m_pAccelStruct_m_SdfOctreeRoots->capacity() ); assert( m_pAccelStruct_m_SdfOctreeRoots->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfParameters_size     = uint32_t( m_pAccelStruct_m_SdfParameters->size() );     assert( m_pAccelStruct_m_SdfParameters->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_SdfParameters_capacity = uint32_t( m_pAccelStruct_m_SdfParameters->capacity() ); assert( m_pAccelStruct_m_SdfParameters->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_allNodePairs_size     = uint32_t( m_pAccelStruct_m_allNodePairs->size() );     assert( m_pAccelStruct_m_allNodePairs->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_allNodePairs_capacity = uint32_t( m_pAccelStruct_m_allNodePairs->capacity() ); assert( m_pAccelStruct_m_allNodePairs->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_bvhOffsets_size     = uint32_t( m_pAccelStruct_m_bvhOffsets->size() );     assert( m_pAccelStruct_m_bvhOffsets->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_bvhOffsets_capacity = uint32_t( m_pAccelStruct_m_bvhOffsets->capacity() ); assert( m_pAccelStruct_m_bvhOffsets->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomIdByInstId_size     = uint32_t( m_pAccelStruct_m_geomIdByInstId->size() );     assert( m_pAccelStruct_m_geomIdByInstId->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomIdByInstId_capacity = uint32_t( m_pAccelStruct_m_geomIdByInstId->capacity() ); assert( m_pAccelStruct_m_geomIdByInstId->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomOffsets_size     = uint32_t( m_pAccelStruct_m_geomOffsets->size() );     assert( m_pAccelStruct_m_geomOffsets->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomOffsets_capacity = uint32_t( m_pAccelStruct_m_geomOffsets->capacity() ); assert( m_pAccelStruct_m_geomOffsets->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomTypeByGeomId_size     = uint32_t( m_pAccelStruct_m_geomTypeByGeomId->size() );     assert( m_pAccelStruct_m_geomTypeByGeomId->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_geomTypeByGeomId_capacity = uint32_t( m_pAccelStruct_m_geomTypeByGeomId->capacity() ); assert( m_pAccelStruct_m_geomTypeByGeomId->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_indices_size     = uint32_t( m_pAccelStruct_m_indices->size() );     assert( m_pAccelStruct_m_indices->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_indices_capacity = uint32_t( m_pAccelStruct_m_indices->capacity() ); assert( m_pAccelStruct_m_indices->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_instMatricesInv_size     = uint32_t( m_pAccelStruct_m_instMatricesInv->size() );     assert( m_pAccelStruct_m_instMatricesInv->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_instMatricesInv_capacity = uint32_t( m_pAccelStruct_m_instMatricesInv->capacity() ); assert( m_pAccelStruct_m_instMatricesInv->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_nodesTLAS_size     = uint32_t( m_pAccelStruct_m_nodesTLAS->size() );     assert( m_pAccelStruct_m_nodesTLAS->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_nodesTLAS_capacity = uint32_t( m_pAccelStruct_m_nodesTLAS->capacity() ); assert( m_pAccelStruct_m_nodesTLAS->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_origNodes_size     = uint32_t( m_pAccelStruct_m_origNodes->size() );     assert( m_pAccelStruct_m_origNodes->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_origNodes_capacity = uint32_t( m_pAccelStruct_m_origNodes->capacity() ); assert( m_pAccelStruct_m_origNodes->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_primIndices_size     = uint32_t( m_pAccelStruct_m_primIndices->size() );     assert( m_pAccelStruct_m_primIndices->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_primIndices_capacity = uint32_t( m_pAccelStruct_m_primIndices->capacity() ); assert( m_pAccelStruct_m_primIndices->capacity() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_vertPos_size     = uint32_t( m_pAccelStruct_m_vertPos->size() );     assert( m_pAccelStruct_m_vertPos->size() < maxAllowedSize );
  m_uboData.m_pAccelStruct_m_vertPos_capacity = uint32_t( m_pAccelStruct_m_vertPos->capacity() ); assert( m_pAccelStruct_m_vertPos->capacity() < maxAllowedSize );
  m_uboData.m_packedXY_size     = uint32_t( m_packedXY.size() );     assert( m_packedXY.size() < maxAllowedSize );
  m_uboData.m_packedXY_capacity = uint32_t( m_packedXY.capacity() ); assert( m_packedXY.capacity() < maxAllowedSize );
  a_pCopyEngine->UpdateBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
}

void MultiRenderer_GPU::ReadPlainMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  a_pCopyEngine->ReadBuffer(m_classDataBuffer, 0, &m_uboData, sizeof(m_uboData));
  auto pUnderlyingImpl = dynamic_cast<BVHRT*>(m_pAccelStruct.get());
  m_projInv = m_uboData.m_projInv;
  m_worldViewInv = m_uboData.m_worldViewInv;
  pUnderlyingImpl->m_preset = m_uboData.m_pAccelStruct_m_preset;
  m_preset = m_uboData.m_preset;
  m_height = m_uboData.m_height;
  m_width = m_uboData.m_width;
  m_pAccelStruct_m_ConjIndices->resize(m_uboData.m_pAccelStruct_m_ConjIndices_size);
  m_pAccelStruct_m_RFGridData->resize(m_uboData.m_pAccelStruct_m_RFGridData_size);
  m_pAccelStruct_m_RFGridSizes->resize(m_uboData.m_pAccelStruct_m_RFGridSizes_size);
  m_pAccelStruct_m_SdfConjunctions->resize(m_uboData.m_pAccelStruct_m_SdfConjunctions_size);
  m_pAccelStruct_m_SdfFrameOctreeNodes->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeNodes_size);
  m_pAccelStruct_m_SdfFrameOctreeRoots->resize(m_uboData.m_pAccelStruct_m_SdfFrameOctreeRoots_size);
  m_pAccelStruct_m_SdfGridData->resize(m_uboData.m_pAccelStruct_m_SdfGridData_size);
  m_pAccelStruct_m_SdfGridOffsets->resize(m_uboData.m_pAccelStruct_m_SdfGridOffsets_size);
  m_pAccelStruct_m_SdfGridSizes->resize(m_uboData.m_pAccelStruct_m_SdfGridSizes_size);
  m_pAccelStruct_m_SdfNeuralProperties->resize(m_uboData.m_pAccelStruct_m_SdfNeuralProperties_size);
  m_pAccelStruct_m_SdfObjects->resize(m_uboData.m_pAccelStruct_m_SdfObjects_size);
  m_pAccelStruct_m_SdfOctreeNodes->resize(m_uboData.m_pAccelStruct_m_SdfOctreeNodes_size);
  m_pAccelStruct_m_SdfOctreeRoots->resize(m_uboData.m_pAccelStruct_m_SdfOctreeRoots_size);
  m_pAccelStruct_m_SdfParameters->resize(m_uboData.m_pAccelStruct_m_SdfParameters_size);
  m_pAccelStruct_m_allNodePairs->resize(m_uboData.m_pAccelStruct_m_allNodePairs_size);
  m_pAccelStruct_m_bvhOffsets->resize(m_uboData.m_pAccelStruct_m_bvhOffsets_size);
  m_pAccelStruct_m_geomIdByInstId->resize(m_uboData.m_pAccelStruct_m_geomIdByInstId_size);
  m_pAccelStruct_m_geomOffsets->resize(m_uboData.m_pAccelStruct_m_geomOffsets_size);
  m_pAccelStruct_m_geomTypeByGeomId->resize(m_uboData.m_pAccelStruct_m_geomTypeByGeomId_size);
  m_pAccelStruct_m_indices->resize(m_uboData.m_pAccelStruct_m_indices_size);
  m_pAccelStruct_m_instMatricesInv->resize(m_uboData.m_pAccelStruct_m_instMatricesInv_size);
  m_pAccelStruct_m_nodesTLAS->resize(m_uboData.m_pAccelStruct_m_nodesTLAS_size);
  m_pAccelStruct_m_origNodes->resize(m_uboData.m_pAccelStruct_m_origNodes_size);
  m_pAccelStruct_m_primIndices->resize(m_uboData.m_pAccelStruct_m_primIndices_size);
  m_pAccelStruct_m_vertPos->resize(m_uboData.m_pAccelStruct_m_vertPos_size);
  m_packedXY.resize(m_uboData.m_packedXY_size);
}

void MultiRenderer_GPU::UpdateVectorMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{
  if(m_pAccelStruct_m_ConjIndices->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_ConjIndicesBuffer, 0, m_pAccelStruct_m_ConjIndices->data(), m_pAccelStruct_m_ConjIndices->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_RFGridData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_RFGridDataBuffer, 0, m_pAccelStruct_m_RFGridData->data(), m_pAccelStruct_m_RFGridData->size()*sizeof(float) );
  if(m_pAccelStruct_m_RFGridSizes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_RFGridSizesBuffer, 0, m_pAccelStruct_m_RFGridSizes->data(), m_pAccelStruct_m_RFGridSizes->size()*sizeof(unsigned long) );
  if(m_pAccelStruct_m_SdfConjunctions->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfConjunctionsBuffer, 0, m_pAccelStruct_m_SdfConjunctions->data(), m_pAccelStruct_m_SdfConjunctions->size()*sizeof(struct SdfConjunction) );
  if(m_pAccelStruct_m_SdfFrameOctreeNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfFrameOctreeNodesBuffer, 0, m_pAccelStruct_m_SdfFrameOctreeNodes->data(), m_pAccelStruct_m_SdfFrameOctreeNodes->size()*sizeof(struct SdfFrameOctreeNode) );
  if(m_pAccelStruct_m_SdfFrameOctreeRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfFrameOctreeRootsBuffer, 0, m_pAccelStruct_m_SdfFrameOctreeRoots->data(), m_pAccelStruct_m_SdfFrameOctreeRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfGridData->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfGridDataBuffer, 0, m_pAccelStruct_m_SdfGridData->data(), m_pAccelStruct_m_SdfGridData->size()*sizeof(float) );
  if(m_pAccelStruct_m_SdfGridOffsets->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfGridOffsetsBuffer, 0, m_pAccelStruct_m_SdfGridOffsets->data(), m_pAccelStruct_m_SdfGridOffsets->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfGridSizes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfGridSizesBuffer, 0, m_pAccelStruct_m_SdfGridSizes->data(), m_pAccelStruct_m_SdfGridSizes->size()*sizeof(struct LiteMath::uint3) );
  if(m_pAccelStruct_m_SdfNeuralProperties->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfNeuralPropertiesBuffer, 0, m_pAccelStruct_m_SdfNeuralProperties->data(), m_pAccelStruct_m_SdfNeuralProperties->size()*sizeof(struct NeuralProperties) );
  if(m_pAccelStruct_m_SdfObjects->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfObjectsBuffer, 0, m_pAccelStruct_m_SdfObjects->data(), m_pAccelStruct_m_SdfObjects->size()*sizeof(struct SdfObject) );
  if(m_pAccelStruct_m_SdfOctreeNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfOctreeNodesBuffer, 0, m_pAccelStruct_m_SdfOctreeNodes->data(), m_pAccelStruct_m_SdfOctreeNodes->size()*sizeof(struct SdfOctreeNode) );
  if(m_pAccelStruct_m_SdfOctreeRoots->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfOctreeRootsBuffer, 0, m_pAccelStruct_m_SdfOctreeRoots->data(), m_pAccelStruct_m_SdfOctreeRoots->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_SdfParameters->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_SdfParametersBuffer, 0, m_pAccelStruct_m_SdfParameters->data(), m_pAccelStruct_m_SdfParameters->size()*sizeof(float) );
  if(m_pAccelStruct_m_allNodePairs->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_allNodePairsBuffer, 0, m_pAccelStruct_m_allNodePairs->data(), m_pAccelStruct_m_allNodePairs->size()*sizeof(struct BVHNodePair) );
  if(m_pAccelStruct_m_bvhOffsets->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_bvhOffsetsBuffer, 0, m_pAccelStruct_m_bvhOffsets->data(), m_pAccelStruct_m_bvhOffsets->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_geomIdByInstId->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_geomIdByInstIdBuffer, 0, m_pAccelStruct_m_geomIdByInstId->data(), m_pAccelStruct_m_geomIdByInstId->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_geomOffsets->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_geomOffsetsBuffer, 0, m_pAccelStruct_m_geomOffsets->data(), m_pAccelStruct_m_geomOffsets->size()*sizeof(struct LiteMath::uint2) );
  if(m_pAccelStruct_m_geomTypeByGeomId->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_geomTypeByGeomIdBuffer, 0, m_pAccelStruct_m_geomTypeByGeomId->data(), m_pAccelStruct_m_geomTypeByGeomId->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_indices->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_indicesBuffer, 0, m_pAccelStruct_m_indices->data(), m_pAccelStruct_m_indices->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_instMatricesInv->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_instMatricesInvBuffer, 0, m_pAccelStruct_m_instMatricesInv->data(), m_pAccelStruct_m_instMatricesInv->size()*sizeof(struct LiteMath::float4x4) );
  if(m_pAccelStruct_m_nodesTLAS->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_nodesTLASBuffer, 0, m_pAccelStruct_m_nodesTLAS->data(), m_pAccelStruct_m_nodesTLAS->size()*sizeof(struct BVHNode) );
  if(m_pAccelStruct_m_origNodes->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_origNodesBuffer, 0, m_pAccelStruct_m_origNodes->data(), m_pAccelStruct_m_origNodes->size()*sizeof(struct BVHNode) );
  if(m_pAccelStruct_m_primIndices->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_primIndicesBuffer, 0, m_pAccelStruct_m_primIndices->data(), m_pAccelStruct_m_primIndices->size()*sizeof(unsigned int) );
  if(m_pAccelStruct_m_vertPos->size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_pAccelStruct_m_vertPosBuffer, 0, m_pAccelStruct_m_vertPos->data(), m_pAccelStruct_m_vertPos->size()*sizeof(struct LiteMath::float4) );
  if(m_packedXY.size() > 0)
    a_pCopyEngine->UpdateBuffer(m_vdata.m_packedXYBuffer, 0, m_packedXY.data(), m_packedXY.size()*sizeof(unsigned int) );
}

void MultiRenderer_GPU::UpdateTextureMembers(std::shared_ptr<vk_utils::ICopyEngine> a_pCopyEngine)
{ 
}

void MultiRenderer_GPU::PackXYMegaCmd(uint tidX, uint tidY)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;
  
  struct KernelArgsPC
  {
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  uint32_t sizeX  = uint32_t(tidX);
  uint32_t sizeY  = uint32_t(tidY);
  uint32_t sizeZ  = uint32_t(1);
  
  pcData.m_sizeX  = tidX;
  pcData.m_sizeY  = tidY;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  vkCmdPushConstants(m_currCmdBuffer, PackXYMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}

void MultiRenderer_GPU::CastRaySingleMegaCmd(uint32_t tidX, uint32_t* out_color)
{
  uint32_t blockSizeX = 256;
  uint32_t blockSizeY = 1;
  uint32_t blockSizeZ = 1;
  
  struct KernelArgsPC
  {
    uint32_t m_sizeX;
    uint32_t m_sizeY;
    uint32_t m_sizeZ;
    uint32_t m_tFlags;
  } pcData;
  
  uint32_t sizeX  = uint32_t(tidX);
  uint32_t sizeY  = uint32_t(1);
  uint32_t sizeZ  = uint32_t(1);
  
  pcData.m_sizeX  = tidX;
  pcData.m_sizeY  = 1;
  pcData.m_sizeZ  = 1;
  pcData.m_tFlags = m_currThreadFlags;
  vkCmdPushConstants(m_currCmdBuffer, CastRaySingleMegaLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(KernelArgsPC), &pcData);
  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastRaySingleMegaPipeline);
  vkCmdDispatch    (m_currCmdBuffer, (sizeX + blockSizeX - 1) / blockSizeX, (sizeY + blockSizeY - 1) / blockSizeY, (sizeZ + blockSizeZ - 1) / blockSizeZ);
}


void MultiRenderer_GPU::copyKernelFloatCmd(uint32_t length)
{
  uint32_t blockSizeX = MEMCPY_BLOCK_SIZE;

  vkCmdBindPipeline(m_currCmdBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, copyKernelFloatPipeline);
  vkCmdPushConstants(m_currCmdBuffer, copyKernelFloatLayout, VK_SHADER_STAGE_COMPUTE_BIT, 0, sizeof(uint32_t), &length);
  vkCmdDispatch(m_currCmdBuffer, (length + blockSizeX - 1) / blockSizeX, 1, 1);
}

VkBufferMemoryBarrier MultiRenderer_GPU::BarrierForClearFlags(VkBuffer a_buffer)
{
  VkBufferMemoryBarrier bar = {};
  bar.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  bar.pNext               = NULL;
  bar.srcAccessMask       = VK_ACCESS_TRANSFER_WRITE_BIT;
  bar.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
  bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.buffer              = a_buffer;
  bar.offset              = 0;
  bar.size                = VK_WHOLE_SIZE;
  return bar;
}

VkBufferMemoryBarrier MultiRenderer_GPU::BarrierForSingleBuffer(VkBuffer a_buffer)
{
  VkBufferMemoryBarrier bar = {};
  bar.sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
  bar.pNext               = NULL;
  bar.srcAccessMask       = VK_ACCESS_SHADER_WRITE_BIT;
  bar.dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
  bar.srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
  bar.buffer              = a_buffer;
  bar.offset              = 0;
  bar.size                = VK_WHOLE_SIZE;
  return bar;
}

void MultiRenderer_GPU::BarriersForSeveralBuffers(VkBuffer* a_inBuffers, VkBufferMemoryBarrier* a_outBarriers, uint32_t a_buffersNum)
{
  for(uint32_t i=0; i<a_buffersNum;i++)
  {
    a_outBarriers[i].sType               = VK_STRUCTURE_TYPE_BUFFER_MEMORY_BARRIER;
    a_outBarriers[i].pNext               = NULL;
    a_outBarriers[i].srcAccessMask       = VK_ACCESS_SHADER_WRITE_BIT;
    a_outBarriers[i].dstAccessMask       = VK_ACCESS_SHADER_READ_BIT;
    a_outBarriers[i].srcQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    a_outBarriers[i].dstQueueFamilyIndex = VK_QUEUE_FAMILY_IGNORED;
    a_outBarriers[i].buffer              = a_inBuffers[i];
    a_outBarriers[i].offset              = 0;
    a_outBarriers[i].size                = VK_WHOLE_SIZE;
  }
}

void MultiRenderer_GPU::PackXYCmd(VkCommandBuffer a_commandBuffer, uint tidX, uint tidY)
{
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT }; 
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, PackXYMegaLayout, 0, 1, &m_allGeneratedDS[0], 0, nullptr);
  PackXYMegaCmd(tidX, tidY);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr); 
}

void MultiRenderer_GPU::CastRaySingleCmd(VkCommandBuffer a_commandBuffer, uint32_t tidX, uint32_t* out_color)
{
  m_currCmdBuffer = a_commandBuffer;
  VkMemoryBarrier memoryBarrier = { VK_STRUCTURE_TYPE_MEMORY_BARRIER, nullptr, VK_ACCESS_SHADER_WRITE_BIT, VK_ACCESS_SHADER_READ_BIT }; 
  vkCmdBindDescriptorSets(a_commandBuffer, VK_PIPELINE_BIND_POINT_COMPUTE, CastRaySingleMegaLayout, 0, 1, &m_allGeneratedDS[1], 0, nullptr);
  CastRaySingleMegaCmd(tidX, out_color);
  vkCmdPipelineBarrier(m_currCmdBuffer, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, VK_PIPELINE_STAGE_COMPUTE_SHADER_BIT, 0, 1, &memoryBarrier, 0, nullptr, 0, nullptr); 
}



void MultiRenderer_GPU::PackXYBlock(uint tidX, uint tidY, uint32_t a_numPasses)
{
  // (1) get global Vulkan context objects
  //
  VkInstance       instance       = m_ctx.instance;
  VkPhysicalDevice physicalDevice = m_ctx.physicalDevice;
  VkDevice         device         = m_ctx.device;
  VkCommandPool    commandPool    = m_ctx.commandPool; 
  VkQueue          computeQueue   = m_ctx.computeQueue; 
  VkQueue          transferQueue  = m_ctx.transferQueue;
  auto             pCopyHelper    = m_ctx.pCopyHelper;
  auto             pAllocatorSpec = m_ctx.pAllocatorSpecial;

  // (2) create GPU objects
  //
  auto outFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  if(PackXY_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  

  VkDeviceMemory buffersMem = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, buffers);
  VkDeviceMemory imagesMem  = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, std::vector<VkBuffer>(), images);
  
  vk_utils::MemAllocInfo tempMemoryAllocInfo;
  tempMemoryAllocInfo.memUsage = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // TODO, select depending on device and sample/application (???)
  if(buffers.size() != 0)
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, buffers);
  if(images.size() != 0)
  {
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, images2);
    for(auto imgMem : images)
    {
      VkImageViewCreateInfo imageView{};
      imageView.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
      imageView.image    = imgMem->image;
      imageView.format   = imgMem->format;
      imageView.subresourceRange = {};
      imageView.subresourceRange.aspectMask     = imgMem->aspectMask;
      imageView.subresourceRange.baseMipLevel   = 0;
      imageView.subresourceRange.levelCount     = imgMem->mipLvls;
      imageView.subresourceRange.baseArrayLayer = 0;
      imageView.subresourceRange.layerCount     = 1;
      VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &imgMem->view));
    }
  }
  
  auto afterCreateObjects = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;
  
  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later
  
  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;
  
  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_PackXY(); 

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //  
  m_exTimePackXY.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(PackXY_local.needToClearOutput)
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkEndCommandBuffer(commandBuffer);  
    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimePackXY.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    PackXYCmd(commandBuffer, tidX, tidY);      
    vkEndCommandBuffer(commandBuffer);  
    auto start = std::chrono::high_resolution_clock::now();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    }
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimePackXY.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }
  
  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimePackXY.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources 
  //
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);
  
  m_exTimePackXY.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void MultiRenderer_GPU::CastRaySingleBlock(uint32_t tidX, uint32_t* out_color, uint32_t a_numPasses)
{
  // (1) get global Vulkan context objects
  //
  VkInstance       instance       = m_ctx.instance;
  VkPhysicalDevice physicalDevice = m_ctx.physicalDevice;
  VkDevice         device         = m_ctx.device;
  VkCommandPool    commandPool    = m_ctx.commandPool; 
  VkQueue          computeQueue   = m_ctx.computeQueue; 
  VkQueue          transferQueue  = m_ctx.transferQueue;
  auto             pCopyHelper    = m_ctx.pCopyHelper;
  auto             pAllocatorSpec = m_ctx.pAllocatorSpecial;

  // (2) create GPU objects
  //
  auto outFlags = VK_BUFFER_USAGE_STORAGE_BUFFER_BIT | VK_BUFFER_USAGE_TRANSFER_SRC_BIT;
  if(CastRaySingle_local.needToClearOutput)
    outFlags |= VK_IMAGE_USAGE_TRANSFER_DST_BIT;
  std::vector<VkBuffer> buffers;
  std::vector<VkImage>  images2;
  std::vector<vk_utils::VulkanImageMem*> images;
  auto beforeCreateObjects = std::chrono::high_resolution_clock::now();
  VkBuffer out_colorGPU = vk_utils::createBuffer(device, tidX*sizeof(uint32_t ), outFlags);
  buffers.push_back(out_colorGPU);
  

  VkDeviceMemory buffersMem = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, buffers);
  VkDeviceMemory imagesMem  = VK_NULL_HANDLE; // vk_utils::allocateAndBindWithPadding(device, physicalDevice, std::vector<VkBuffer>(), images);
  
  vk_utils::MemAllocInfo tempMemoryAllocInfo;
  tempMemoryAllocInfo.memUsage = VK_MEMORY_PROPERTY_DEVICE_LOCAL_BIT; // TODO, select depending on device and sample/application (???)
  if(buffers.size() != 0)
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, buffers);
  if(images.size() != 0)
  {
    pAllocatorSpec->Allocate(tempMemoryAllocInfo, images2);
    for(auto imgMem : images)
    {
      VkImageViewCreateInfo imageView{};
      imageView.sType    = VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO;
      imageView.viewType = VK_IMAGE_VIEW_TYPE_2D;
      imageView.image    = imgMem->image;
      imageView.format   = imgMem->format;
      imageView.subresourceRange = {};
      imageView.subresourceRange.aspectMask     = imgMem->aspectMask;
      imageView.subresourceRange.baseMipLevel   = 0;
      imageView.subresourceRange.levelCount     = imgMem->mipLvls;
      imageView.subresourceRange.baseArrayLayer = 0;
      imageView.subresourceRange.layerCount     = 1;
      VK_CHECK_RESULT(vkCreateImageView(device, &imageView, nullptr, &imgMem->view));
    }
  }
  
  auto afterCreateObjects = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msAPIOverhead = std::chrono::duration_cast<std::chrono::microseconds>(afterCreateObjects - beforeCreateObjects).count()/1000.f;
  
  auto afterCopy2 = std::chrono::high_resolution_clock::now(); // just declare it here, replace value later
  
  auto afterInitBuffers = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(afterInitBuffers - afterCreateObjects).count()/1000.f;
  
  auto beforeSetInOut = std::chrono::high_resolution_clock::now();
  this->SetVulkanInOutFor_CastRaySingle(out_colorGPU, 0); 

  // (3) copy input data to GPU
  //
  auto beforeCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(beforeCopy - beforeSetInOut).count()/1000.f;
  auto afterCopy = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msCopyToGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy - beforeCopy).count()/1000.f;
  //  
  m_exTimeCastRaySingle.msExecuteOnGPU = 0;
  //// (3.1) clear all outputs if we are in RTV pattern
  //
  if(CastRaySingle_local.needToClearOutput)
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    vkCmdFillBuffer(commandBuffer, out_colorGPU, 0, VK_WHOLE_SIZE, 0); // zero output buffer out_colorGPU
    vkEndCommandBuffer(commandBuffer);  
    auto start = std::chrono::high_resolution_clock::now();
    vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastRaySingle.msExecuteOnGPU  += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }

  // (4) now execute algorithm on GPU
  //
  {
    VkCommandBuffer commandBuffer = vk_utils::createCommandBuffer(device, commandPool);
    VkCommandBufferBeginInfo beginCommandBufferInfo = {};
    beginCommandBufferInfo.sType = VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO;
    beginCommandBufferInfo.flags = VK_COMMAND_BUFFER_USAGE_SIMULTANEOUS_USE_BIT;
    vkBeginCommandBuffer(commandBuffer, &beginCommandBufferInfo);
    CastRaySingleCmd(commandBuffer, tidX, out_color);      
    vkEndCommandBuffer(commandBuffer);  
    auto start = std::chrono::high_resolution_clock::now();
    for(uint32_t pass = 0; pass < a_numPasses; pass++) {
      vk_utils::executeCommandBufferNow(commandBuffer, computeQueue, device);
    }
    vkFreeCommandBuffers(device, commandPool, 1, &commandBuffer);
    auto stop = std::chrono::high_resolution_clock::now();
    m_exTimeCastRaySingle.msExecuteOnGPU += std::chrono::duration_cast<std::chrono::microseconds>(stop - start).count()/1000.f;
  }
  
  // (5) copy output data to CPU
  //
  auto beforeCopy2 = std::chrono::high_resolution_clock::now();
  pCopyHelper->ReadBuffer(out_colorGPU, 0, out_color, tidX*sizeof(uint32_t ));
  this->ReadPlainMembers(pCopyHelper);
  afterCopy2 = std::chrono::high_resolution_clock::now();
  m_exTimeCastRaySingle.msCopyFromGPU = std::chrono::duration_cast<std::chrono::microseconds>(afterCopy2 - beforeCopy2).count()/1000.f;

  // (6) free resources 
  //
  vkDestroyBuffer(device, out_colorGPU, nullptr);
  if(buffersMem != VK_NULL_HANDLE)
    vkFreeMemory(device, buffersMem, nullptr);
  if(imagesMem != VK_NULL_HANDLE)
    vkFreeMemory(device, imagesMem, nullptr);
  
  m_exTimeCastRaySingle.msAPIOverhead += std::chrono::duration_cast<std::chrono::microseconds>(std::chrono::high_resolution_clock::now() - afterCopy2).count()/1000.f;
}

void MultiRenderer_GPU::GetExecutionTime(const char* a_funcName, float a_out[4])
{
  vk_utils::ExecTime res = {};
  if(std::string(a_funcName) == "PackXY" || std::string(a_funcName) == "PackXYBlock")
    res = m_exTimePackXY;
  if(std::string(a_funcName) == "CastRaySingle" || std::string(a_funcName) == "CastRaySingleBlock")
    res = m_exTimeCastRaySingle;
  a_out[0] = res.msExecuteOnGPU;
  a_out[1] = res.msCopyToGPU;
  a_out[2] = res.msCopyFromGPU;
  a_out[3] = res.msAPIOverhead;             
}

