#include "sparse_octree_builder.h"
#include "omp.h"
#include <set>
#include <chrono>
#include <unordered_map>
#include <atomic>
#include <functional>

namespace LiteMath
{
  static inline int2 operator%(const int2 a, const int2 b) { return int2{a.x % b.x, a.y % b.y}; }
  static inline int3 operator%(const int3 a, const int3 b) { return int3{a.x % b.x, a.y % b.y, a.z % b.z}; }
  static inline int4 operator%(const int4 a, const int4 b) { return int4{a.x % b.x, a.y % b.y, a.z % b.z, a.w % b.w}; }

  static inline uint2 operator%(const uint2 a, const uint2 b) { return uint2{a.x % b.x, a.y % b.y}; }
  static inline uint3 operator%(const uint3 a, const uint3 b) { return uint3{a.x % b.x, a.y % b.y, a.z % b.z}; }
  static inline uint4 operator%(const uint4 a, const uint4 b) { return uint4{a.x % b.x, a.y % b.y, a.z % b.z, a.w % b.w}; }
}

static bool is_border(float distance, int level)
{
  return level < 2  ? true : std::abs(distance) < sqrt(3)*pow(2, -level);
}

static bool is_leaf(unsigned offset)
{
  return (offset == 0) || (offset & INVALID_IDX);
}

//level_size is (1 << level)
static bool is_border_node(float min_val, float max_val, unsigned level_size)
{
  float d_max = 2*sqrt(3)/level_size;
  return (min_val <= 0) && (max_val > -d_max);
}

namespace sdf_converter
{
  struct LargeNode
  {
    float3 p;
    float d;
    unsigned level;
    unsigned group_idx;
    unsigned thread_id;
    unsigned children_idx;
    float value;
  };

  void save_p_and_d_from_tl_octree_rec(const cmesh4::TriangleListOctree &tl_octree, 
                                       std::vector<std::pair<float3, float>> &data, 
                                       unsigned idx, float3 p, float d)
  {
    unsigned ofs = tl_octree.nodes[idx].offset;
    data[idx] = std::pair(p, d);
    if (ofs != 0)
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        save_p_and_d_from_tl_octree_rec(tl_octree, data, ofs + i, ch_p, ch_d);
      }
    }
  }

  static float trilinear_interp(const float values[8], float3 dp)
  {
    return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
          (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
          (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
          (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
          (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
          (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
          (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
          (  dp.x)*(  dp.y)*(  dp.z)*values[7];
  }

  float count_distance_at_brick_on_position(const cmesh4::SimpleMesh &mesh, const cmesh4::TriangleListOctree &tl_octree, int idx, float3 pos, float2 &tex_coord, unsigned &mat)
  {
    float sign = 1, min_dist_sq = 1000000;

    float3 surface_pos = float3(0, 0, 0), min_a = float3(0, 0, 0), min_b = float3(0, 0, 0), min_c = float3(0, 0, 0);
    int min_ti = -1;
    for (int tri=0; tri<tl_octree.nodes[idx].tid_count; tri++)
    {
      int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+tri];
      float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
      float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
      float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
      float3 vt = pos - cmesh4::closest_point_triangle(pos, a, b, c);
      float dst_sq = LiteMath::dot(vt, vt);
      if (min_dist_sq > dst_sq)
      {
        min_dist_sq = dst_sq;
        min_ti = t_i;
        min_a = a;
        min_b = b;
        min_c = c;
        surface_pos = cmesh4::closest_point_triangle(pos, a, b, c);
        if (LiteMath::dot(LiteMath::cross(a - b, a - c), vt) < 0)
        {
          sign = -1;
        }
        else
        {
          sign = 1;
        }
      }
    }
    float3 bc = cmesh4::barycentric(surface_pos, min_a, min_b, min_c);
    tex_coord = bc.x*mesh.vTexCoord2f[mesh.indices[3*min_ti+0]] + bc.y*mesh.vTexCoord2f[mesh.indices[3*min_ti+1]] + bc.z*mesh.vTexCoord2f[mesh.indices[3*min_ti+2]];
    mat = mesh.matIndices[min_ti];
    return min_dist_sq * sign;
  }

  /*float count_rmse()
  {
    float rmse = 0.0f;
    for (int x = 0; x <= 2; x += 1)
    {
      for (int y = 0; y <= 2; y += 1)
      {
        for (int z = 0; z <= 2; z += 1)
        {
          float3 off_idx = float3((float)x / 2.0, (float)y / 2.0, (float)z / 2.0);
          float coeff = 1.0;
          if (x == 1) coeff *= 2.0;
          if (y == 1) coeff *= 2.0;
          if (z == 1) coeff *= 2.0;
          float sdf_val = sdf(corner + offset * off_idx, 0);//TODO save it to cash and take
          rmse += pow((sdf_val - trilinear_interpolation(node_values, off_idx)), 2);
        }
      }
    }
    return sqrt(rmse / 64.0);
  }*/

  void linear_mesh_octree_to_global_octree_with_precision(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree,
                                         GlobalOctree &out_octree,
                                         unsigned min_layer, unsigned max_layer, float precision)
  {
    assert(min_layer < max_layer);
    struct PositionHasher
    {
      std::size_t operator()(const float3& k) const
      {
        // Compute individual hash values for first,
        // second and third and combine them using XOR
        // and bit shifting:
        return (  (std::hash<float>()(k.x)
                ^ (std::hash<float>()(k.y) << 1)) >> 1)
                ^ (std::hash<float>()(k.z) << 1);
      }
    };
    struct PositionEqual
    {
      bool operator()(const float3& lhs, const float3& rhs) const
      {
        return std::abs(lhs.x - rhs.x) < 1e-12f && std::abs(lhs.y - rhs.y) < 1e-12f && std::abs(lhs.z - rhs.z) < 1e-12f;
      }
    };
    out_octree.nodes.resize(1);
    out_octree.values_f.resize(0);
    unsigned v_size = out_octree.header.brick_size + out_octree.header.brick_pad * 2 + 1;
    unsigned mat;
    float2 tex_c;
    struct NodeData
    {
      unsigned tl_idx;
      unsigned global_idx;
      float d;
      float3 pos;
      unsigned layer_num;
      std::unordered_map<float3, std::pair<float, float>/*real, interpolated*/, PositionHasher, PositionEqual> node_distances;
    };
    std::vector<NodeData> checks;
    NodeData elem;
    elem.d = 1;
    elem.pos = float3(0, 0, 0);
    elem.layer_num = 0;
    elem.node_distances = {};
    elem.tl_idx = 0;
    elem.global_idx = 0;
    checks.push_back(elem);
    int pad = out_octree.header.brick_pad, size = out_octree.header.brick_size;
    for (int i = 0; i < checks.size(); ++i)
    {
      float3 pos = 2.0f*(checks[i].d*checks[i].pos) - 1.0f;
      out_octree.nodes[checks[i].global_idx].val_off = out_octree.values_f.size();
      out_octree.values_f.resize(out_octree.values_f.size() + v_size * v_size * v_size);
      out_octree.nodes[checks[i].global_idx].is_not_void = tl_octree.nodes[checks[i].tl_idx].tid_count != 0;
      out_octree.nodes[checks[i].global_idx].offset = 0;
      for (int x = -pad; x <= size + pad; ++x)
      {
        for (int y = -pad; y <= size + pad; ++y)
        {
          for (int z = -pad; z <= size + pad; ++z)
          {
            float3 ch_pos = pos + 2*(checks[i].d/out_octree.header.brick_size)*float3(x, y, z);
            float dist = count_distance_at_brick_on_position(mesh, tl_octree, checks[i].tl_idx, ch_pos, tex_c, mat);
            checks[i].node_distances[ch_pos] = std::pair(dist, dist);
            out_octree.values_f[out_octree.nodes[checks[i].global_idx].offset + v_size * v_size * (x + pad) + v_size * (y + pad) + z + pad] = dist;
            if ((x == 0 || x == size) && (y == 0 || y == size) && (z == 0 || z == size))
            {
              int num = (int(x / size) << 2) + (int(y / size) << 1) + int(z / size);
              out_octree.nodes[checks[i].global_idx].tex_coords[num] = tex_c;
            }
            if (x == size / 2 && y == size / 2 && z == size / 2 && size != 1)
            {
              out_octree.nodes[checks[i].global_idx].material_id = mat;
            }
          }
        }
      }
      if (size == 1)
      {
        float3 ch_pos = pos + checks[i].d;
        count_distance_at_brick_on_position(mesh, tl_octree, checks[i].tl_idx, ch_pos, tex_c, mat);
        out_octree.nodes[checks[i].global_idx].material_id = mat;
      }
      
      bool is_div = checks[i].layer_num <= min_layer;
      if(tl_octree.nodes[checks[i].tl_idx].tid_count == 0 || checks[i].layer_num >= max_layer)
      {
        is_div = false;
      }
      else if(!is_div)
      {
        float3 pos = 2.0f*(checks[i].d*checks[i].pos) - 1.0f;
        float error = 0;
        for (int x = 0; x <= size * 2; ++x)
        {
          for (int y = 0; y <= size * 2; ++y)
          {
            for (int z = 0; z <= size * 2; ++z)
            {
              float3 ch_pos = pos + (checks[i].d/out_octree.header.brick_size)*float3(x, y, z);
              float diff = 0;
              if (checks[i].node_distances.find(ch_pos) == checks[i].node_distances.end())
              {
                float dist = count_distance_at_brick_on_position(mesh, tl_octree, checks[i].tl_idx, ch_pos, tex_c, mat);
                float values[8];
                for (int j = 0; j < 8; ++j)
                {
                  int xs = (j >> 2) * 2 - 1;
                  int ys = ((j >> 1) & 1) * 2 - 1;
                  int zs = (j & 1) * 2 - 1;
                  float3 interp_pos = ch_pos + float3(xs * (x % 2), ys * (y % 2), zs * (z % 2)) * (checks[i].d/out_octree.header.brick_size);
                  values[j] = checks[i].node_distances[interp_pos].first;
                }
                float interp = trilinear_interp(values, float3(0.5, 0.5, 0.5));
                checks[i].node_distances[ch_pos] = std::pair(dist, interp);
                float coeff = 1;
                if (x != 0 && x != size * 2)
                {
                  coeff *= 2;
                }
                if (y != 0 && y != size * 2)
                {
                  coeff *= 2;
                }
                if (z != 0 && z != size * 2)
                {
                  coeff *= 2;
                }
                error += coeff * std::pow(dist - interp, 2);

              }
            }
          }
        }
        error /= (size * size * size * 64);
        error = sqrt(error);
        if (error >= precision) is_div = true;
      }

      if (is_div)
      {
        for (int j = 0; j < 8; ++j)
        {
          elem.d = checks[i].d / 2.0;
          elem.pos = 2 * checks[i].pos + float3(j >> 2, (j >> 1) & 1, j & 1);
          elem.layer_num = checks[i].layer_num + 1;
          elem.node_distances = {};
          elem.tl_idx = tl_octree.nodes[checks[i].tl_idx].offset;
          elem.global_idx = out_octree.nodes.size() + j;
          checks.push_back(elem);
        }
        out_octree.nodes[checks[i].global_idx].offset = out_octree.nodes.size();
        out_octree.nodes.resize(out_octree.nodes.size() + 8);
      }
    }
  }

  void linear_mesh_octree_to_global_octree(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree,
                                         GlobalOctree &out_octree, unsigned max_threads)
  {
    omp_set_num_threads(max_threads);

    std::vector<std::pair<float3, float>> p_d(tl_octree.nodes.size());

    unsigned v_size = out_octree.header.brick_size + 2*out_octree.header.brick_pad + 1;

    out_octree.nodes.resize(tl_octree.nodes.size());
    out_octree.values_f.resize(out_octree.nodes.size()*v_size*v_size*v_size);

    save_p_and_d_from_tl_octree_rec(tl_octree, p_d, 0, float3(0,0,0), 1);

    #pragma omp parallel for
    for (unsigned idx = 0; idx < tl_octree.nodes.size(); ++idx)
    {
      unsigned ofs = tl_octree.nodes[idx].offset;
      out_octree.nodes[idx].offset = ofs;
      out_octree.nodes[idx].val_off = idx*v_size*v_size*v_size;
      out_octree.nodes[idx].is_not_void = true;

      if (ofs == 0)
      {

        float min_val =  1000;
        float max_val = -1000;

        float3 p = p_d[idx].first;
        float d = p_d[idx].second;

        float3 pos = 2.0f*(d*p) - 1.0f;
        //out_octree.nodes[idx].val_off = idx*v_size*v_size*v_size;

        for (int i = -(int)out_octree.header.brick_pad; i <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++i)
        {
          for (int j = -(int)out_octree.header.brick_pad; j <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++j)
          {
            for (int k = -(int)out_octree.header.brick_pad; k <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++k)
            {
              float3 ch_pos = pos + 2*(d/out_octree.header.brick_size)*float3(i,j,k);
              float sign = 1, min_dist_sq = 1000000;

              float3 surface_pos = float3(0, 0, 0), min_a = float3(0, 0, 0), min_b = float3(0, 0, 0), min_c = float3(0, 0, 0);
              int min_ti = -1;

              //float val = 1000.0f;

              //if (distances.find(ch_pos) == distances.end())
              //{
              for (int tri=0; tri<tl_octree.nodes[idx].tid_count; tri++)
              {
                int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+tri];
                float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
                float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
                float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
                float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
                float dst_sq = LiteMath::dot(vt, vt);
                if (min_dist_sq > dst_sq)
                {
                  min_dist_sq = dst_sq;
                  min_ti = t_i;
                  min_a = a;
                  min_b = b;
                  min_c = c;
                  surface_pos = cmesh4::closest_point_triangle(ch_pos, a, b, c);
                  if (LiteMath::dot(LiteMath::cross(a - b, a - c), vt) < 0)
                  {
                    sign = -1;
                  }
                  else
                  {
                    sign = 1;
                  }
                }
              }
              /*if (tl_octree.nodes[idx].tid_count != 0)
              {
                distances[ch_pos] = sign * sqrtf(min_dist_sq);
                val = distances[ch_pos];
              }*/
              //val = sign * sqrtf(min_dist_sq);
              //}
              /*if (tl_octree.nodes[idx].tid_count != 0)
              {
                val = distances[ch_pos];
              }*/

              if ((i == 0 || i == out_octree.header.brick_size) &&
                  (j == 0 || j == out_octree.header.brick_size) &&
                  (k == 0 || k == out_octree.header.brick_size) &&
                  min_ti != -1)
              {
                int num = (int(i / out_octree.header.brick_size) << 2) + (int(j / out_octree.header.brick_size) << 1) + int(k / out_octree.header.brick_size);
                float3 bc = cmesh4::barycentric(surface_pos, min_a, min_b, min_c);
                float2 tc = bc.x*mesh.vTexCoord2f[mesh.indices[3*min_ti+0]] + bc.y*mesh.vTexCoord2f[mesh.indices[3*min_ti+1]] + bc.z*mesh.vTexCoord2f[mesh.indices[3*min_ti+2]];
                out_octree.nodes[idx].tex_coords[num] = tc;
              }

              out_octree.values_f[out_octree.nodes[idx].val_off + (i+out_octree.header.brick_pad)*v_size*v_size + 
                                                                    (j+out_octree.header.brick_pad)*v_size + 
                                                                    (k+out_octree.header.brick_pad)] = sign * sqrtf(min_dist_sq);//val;
              
              if (i >= 0 && j >= 0 && k >= 0 && i <= out_octree.header.brick_size && 
                  j <= out_octree.header.brick_size && k <= out_octree.header.brick_size)
              {
                min_val = std::min(min_val, sign * sqrtf(min_dist_sq));
                max_val = std::max(max_val, sign * sqrtf(min_dist_sq));
              }
            }
          }
        }

        if (mesh.matIndices.size() == mesh.TrianglesNum())
        {
          float3 center = pos + d*float3(1,1,1);
          float sign = 1, min_dist_sq = 1000000;
          int min_ti = -1;
          for (int tri=0; tri<tl_octree.nodes[idx].tid_count; tri++)
          {
            int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+tri];
            float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
            float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
            float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
            float3 vt = center - cmesh4::closest_point_triangle(center, a, b, c);
            float dst_sq = LiteMath::dot(vt, vt);
            if (min_dist_sq > dst_sq)
            {
              min_dist_sq = dst_sq;
              min_ti = t_i;
              if (LiteMath::dot(LiteMath::cross(a - b, a - c), vt) < 0)
              {
                sign = -1;
              }
              else
              {
                sign = 1;
              }
            }
          }
          if (min_ti >= 0)
            out_octree.nodes[idx].material_id = mesh.matIndices[min_ti];
          else
            out_octree.nodes[idx].material_id = 0;
          //printf("material id %u\n", frame[idx].material_id);
        }
        else
        {
          out_octree.nodes[idx].material_id = 0;
        }
        //tex coords
        /*for (int i = 0; i < 8; ++i)
        {
          //float3 ch_pos = pos + 2*d*float3((i >> 2) & 1, (i >> 1) & 1, i & 1);
          out_octree.nodes[idx].tex_coords[i] = float2(0, 0);//temp stubs
        }*/
        if (ofs == 0) out_octree.nodes[idx].is_not_void = (min_val <= 0 && max_val >= 0);
        //else out_octree.nodes[idx].is_not_void = true;
      }
    }
  }

  void add_global_node_rec(std::vector<GlobalOctreeNode> &nodes, std::vector<float> &values_f, SparseOctreeSettings settings, MultithreadedDistanceFunction sdf,
                           unsigned thread_id, unsigned node_idx, unsigned depth, unsigned max_depth, float3 p, float d, int brick_size, int brick_pad)
  {
    int val_size = brick_size + brick_pad * 2 + 1;
    float value_center = sdf(2.0f * ((p + float3(0.5, 0.5, 0.5)) * d) - float3(1, 1, 1), thread_id);
    float min_val = 1000;
    float max_val = -1000;
    nodes[node_idx].val_off = values_f.size();
    nodes[node_idx].is_not_void = true;
    nodes[node_idx].offset = 0;
    for (int i = 0; i < 8; ++i)
    {
      nodes[node_idx].tex_coords[i] = float2(0, 0);
    }
    for (int i = -brick_pad; i <= brick_size + brick_pad; ++i)
    {
      for (int j = -brick_pad; j <= brick_size + brick_pad; ++j)
      {
        for (int k = -brick_pad; k <= brick_size + brick_pad; ++k)
        {
          float val = sdf(2.0f * ((p + float3((float)i / (float)brick_size, (float)j / (float)brick_size, (float)k / (float)brick_size)) * d) - float3(1, 1, 1), thread_id);
          values_f.push_back(val);
          min_val = std::min(min_val, val);
          max_val = std::max(max_val, val);
        }
      }
    }
    /*for (int cid = 0; cid < 8; cid++)
    {
      nodes[node_idx].values[cid] = sdf(2.0f * ((p + float3((float)i / (float)brick_size, (float)j / (float)brick_size, (float)k / (float)brick_size)) * d) - float3(1, 1, 1), thread_id);
      min_val = std::min(min_val, nodes[node_idx].values[cid]);
      max_val = std::max(max_val, nodes[node_idx].values[cid]);
    }*/
    if (depth < max_depth && (std::abs(value_center) < sqrtf(3) * d || min_val*max_val <= 0))
    {
      nodes[node_idx].offset = nodes.size();
      
      nodes.resize(nodes.size() + 8);
      unsigned idx = nodes[node_idx].offset;
      for (unsigned cid = 0; cid < 8; cid++)
      {
        add_global_node_rec(nodes, values_f, settings, sdf, thread_id, 
                     idx + cid, depth + 1, max_depth, 2 * p + float3((cid & 4) >> 2, (cid & 2) >> 1, cid & 1), d / 2, brick_size, brick_pad);
      }
    }
    else if (min_val*max_val <= 0)
    {
      nodes[node_idx].is_not_void = true;
    }
    else
    {
      nodes[node_idx].is_not_void = false;
    }
  }

  void sdf_to_global_octree(SparseOctreeSettings settings, MultithreadedDistanceFunction sdf, 
                                      unsigned max_threads, GlobalOctree &octree)
  {
std::chrono::steady_clock::time_point t1 = std::chrono::steady_clock::now();

    omp_set_num_threads(max_threads);

    unsigned min_remove_level = std::min(settings.depth, 4u);
    std::vector<LargeNode> large_nodes;
    int lg_size = pow(2, min_remove_level);
    large_nodes.push_back({float3(0,0,0), 1.0f, 0u, 0u, 0u, 0u, 1000.0f});

    unsigned i = 0;
    while (i < large_nodes.size())
    {
      if (large_nodes[i].level < min_remove_level)
      {
        large_nodes[i].children_idx = large_nodes.size();
        for (int j=0;j<8;j++)
        {
          float ch_d = large_nodes[i].d / 2;
          float3 ch_p = 2 * large_nodes[i].p + float3((j & 4) >> 2, (j & 2) >> 1, j & 1);
          large_nodes.push_back({ch_p, ch_d, large_nodes[i].level+1, 0u, 0u, 0u, 1000.0f});
        }
      }
      i++;
    }

    std::vector<unsigned> border_large_nodes;

    for (int i=0;i<large_nodes.size();i++)
    {
      float3 pos = 2.0f * ((large_nodes[i].p + float3(0.5, 0.5, 0.5)) * large_nodes[i].d) - float3(1, 1, 1); 
      large_nodes[i].value = sdf(pos, 0);
      if (large_nodes[i].children_idx == 0 && is_border(large_nodes[i].value, large_nodes[i].level))
        border_large_nodes.push_back(i);
    }

    unsigned step = (border_large_nodes.size() + max_threads - 1) / max_threads;
    std::vector<std::vector<GlobalOctreeNode>> all_nodes(max_threads);
    std::vector<std::vector<float>> all_distances(max_threads);
    std::vector<std::vector<uint2>> all_groups(max_threads);

    for (int i=0;i<max_threads;i++)
    {
      all_nodes[i].reserve(std::min(1ul << 20, 1ul << 3*settings.depth));
    }

std::chrono::steady_clock::time_point t2 = std::chrono::steady_clock::now();

    #pragma omp parallel for
    for (int thread_id=0;thread_id<max_threads;thread_id++)
    {
      unsigned start = thread_id * step;
      unsigned end = std::min(start + step, (unsigned)border_large_nodes.size());
      for (unsigned i=start;i<end;i++)
      {
        unsigned idx = border_large_nodes[i];
        unsigned local_root_idx = all_nodes[thread_id].size();
        large_nodes[idx].group_idx = all_groups[thread_id].size();
        large_nodes[idx].thread_id = thread_id;
        all_nodes[thread_id].emplace_back();
        add_global_node_rec(all_nodes[thread_id], all_distances[thread_id], settings, sdf, thread_id, 
                            local_root_idx, large_nodes[idx].level, settings.depth, large_nodes[idx].p, large_nodes[idx].d, 
                            octree.header.brick_size, octree.header.brick_pad);
      
        all_groups[thread_id].push_back(uint2(local_root_idx, all_nodes[thread_id].size()));
      }
    }

std::chrono::steady_clock::time_point t3 = std::chrono::steady_clock::now();

    std::vector<GlobalOctreeNode> res_nodes(large_nodes.size());
    std::vector<float> values_f;

    int v_size = octree.header.brick_size + 2 * octree.header.brick_pad + 1;

    for (int i=0;i<large_nodes.size();i++)
    {
      res_nodes[i].val_off = values_f.size();
      res_nodes[i].is_not_void = true;
      for (int j = 0; j < 8; ++j)
      {
        res_nodes[i].tex_coords[j] = float2(0, 0);
      }
      for (int i0 = -(int)octree.header.brick_pad; i0 <= (int)octree.header.brick_size + (int)octree.header.brick_pad; ++i0)
      {
        for (int j0 = -(int)octree.header.brick_pad; j0 <= (int)octree.header.brick_size + (int)octree.header.brick_pad; ++j0)
        {
          for (int k0 = -(int)octree.header.brick_pad; k0 <= (int)octree.header.brick_size + (int)octree.header.brick_pad; ++k0)
          {
            float val = sdf(2.0f * ((large_nodes[i].p + float3((float)i0 / (float)octree.header.brick_size, (float)j0 / (float)octree.header.brick_size, (float)k0 / (float)octree.header.brick_size)) * large_nodes[i].d) - float3(1, 1, 1), 0);
            values_f.push_back(val);
          }
        }
      }
      //for (int cid = 0; cid < 8; cid++)
      //  res_nodes[i].values[cid] = sdf(2.0f * ((large_nodes[i].p + float3((cid & 4) >> 2, (cid & 2) >> 1, cid & 1)) * large_nodes[i].d) - float3(1, 1, 1), 0);
      
    
      res_nodes[i].offset = large_nodes[i].children_idx;
      if (large_nodes[i].children_idx == 0 && is_border(large_nodes[i].value, large_nodes[i].level))
      {
        unsigned start = all_groups[large_nodes[i].thread_id][large_nodes[i].group_idx].x + 1;
        unsigned end = all_groups[large_nodes[i].thread_id][large_nodes[i].group_idx].y;
        
        if (start == end) //this region is empty
          continue;
        
        unsigned prev_size = res_nodes.size();
        int shift = int(prev_size) - int(start);
        //int val_shift = int(values_f.size()) - int(start);
        res_nodes[i].offset = all_nodes[large_nodes[i].thread_id][start-1].offset + shift;
        //printf("%d offset %u start %u shift %d\n", i, res_nodes[i].offset, start, shift);
        res_nodes.insert(res_nodes.end(), all_nodes[large_nodes[i].thread_id].begin() + start, all_nodes[large_nodes[i].thread_id].begin() + end);

        unsigned val_start = res_nodes[prev_size].val_off;
        int val_shift = values_f.size() - int(val_start);
        int cnt = end - start;
        for (int j=prev_size;j<res_nodes.size();j++)
        {
          if (res_nodes[j].offset != 0)
            res_nodes[j].offset += shift;
          res_nodes[j].val_off += val_shift;
        }

        values_f.insert(values_f.end(), all_distances[large_nodes[i].thread_id].begin() + val_start, all_distances[large_nodes[i].thread_id].begin() + val_start + cnt * v_size * v_size * v_size);
      }
    }

std::chrono::steady_clock::time_point t4 = std::chrono::steady_clock::now();
  
    // assert(!is_leaf(res_nodes[0].offset));

    // std::vector<SdfFrameOctreeNode> frame_3;
    // frame_3.reserve(res_nodes.size());
    // frame_3.push_back(res_nodes[0]);
    // frame_3.shrink_to_fit();

    //printf("%u/%u nodes are active\n", nn, (unsigned)res_nodes.size());
    //printf("%u/%u nodes are left after elimination\n", (unsigned)frame_3.size(), (unsigned)res_nodes.size());
  
std::chrono::steady_clock::time_point t5 = std::chrono::steady_clock::now();
  
    float time_1 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t2 - t1).count();
    float time_2 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t3 - t2).count();
    float time_3 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t4 - t3).count();
    float time_4 = 1e-3f*std::chrono::duration_cast<std::chrono::microseconds>(t5 - t4).count();

    //printf("total nodes = %d, time = %6.2f ms (%.1f+%.1f+%.1f+%.1f)\n", 
    //       (int)res_nodes.size(), time_1 + time_2 + time_3 + time_4, time_1, time_2, time_3, time_4);

    omp_set_num_threads(omp_get_max_threads());

    octree.nodes = res_nodes;
    octree.values_f = values_f;

    //return res_nodes;
  }

  bool mesh_octree_to_global_octree_rec(const cmesh4::SimpleMesh &mesh,
                                         const cmesh4::TriangleListOctree &tl_octree,
                                         GlobalOctree &out_octree,
                                         unsigned idx, float3 p, float d,
                                         unsigned real_idx, unsigned &usefull_nodes)
  {
    //printf("%u\n", usefull_nodes);
    const unsigned ofs = tl_octree.nodes[idx].offset;
    unsigned num = real_idx;
    out_octree.nodes[num].offset = (ofs == 0) ? 0 : usefull_nodes;
    bool norm_broken = false;

    unsigned v_size = out_octree.header.brick_size + 2*out_octree.header.brick_pad + 1;
    float min_val =  1000;
    float max_val = -1000;
    if (ofs == 0)
    {
      float3 pos = 2.0f*(d*p) - 1.0f;
      /*groups.push_back(std::vector<float3>());
      for (int j=0; j<tl_octree.nodes[idx].tid_count; j++)
      {
        int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+j];
        //printf("%u %u %u -- %u %u\n", 3*t_i+0, tl_octree.nodes[idx].tid_offset, j, tl_octree.triangle_ids.size(), tl_octree.nodes[idx].tid_count);
        //printf("%u, %u, %u\n", mesh.indices.size(), mesh.indices[3*t_i+0], mesh.vPos4f.size());
        float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
        float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
        float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
        groups[0].push_back(a);
        groups[0].push_back(b);
        groups[0].push_back(c);
      }*/
      // for (int j=0; j<tl_octree.nodes[idx].tid_count; j++)
      // {
      //   int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+j];
      //   float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
      //   float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
      //   float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
      //   float3 t[3] = {a, b, c};
      //   bool is_find_group = false;
      //   int real_group = -1;
      //   bool first_check = false;
      //   std::vector<int> del_groups = {};
      //   std::vector<bool> del_checks = {};
      //   for (int i = 0; i < groups.size(); ++i)
      //   {
      //     //if (norm_broken) break;
          
      //     bool is_correct_norm = true;
      //     bool is_first = true;
      //     for (int u = 0; u < groups[i].size(); u += 3)
      //     {
      //       //if (norm_broken) break;

      //       bool check = false;
      //       float3 x[3] = {groups[i][u], groups[i][u+1], groups[i][u+2]};
      //       if (is_find_group)
      //       {
      //         if (is_tri_have_same_edge(t, x, check))
      //         {
      //           if (is_first)
      //           {
      //             is_first = false;
      //             is_correct_norm = check;
      //             del_groups.push_back(i);
      //             del_checks.push_back(check);
      //           }
      //           else if ((is_correct_norm && !check) || (!is_correct_norm && check))
      //           {
      //             norm_broken = true;
      //           }
      //         }
      //       }
      //       else
      //       {
      //         if (is_tri_have_same_edge(t, x, check))
      //         {
      //           is_find_group = true;
      //           is_correct_norm = check;
      //           first_check = check;
      //           is_first = false;
      //           real_group = i;
      //         }
      //       }
      //     }
      //   }
      //   //if (norm_broken) break;

      //   if (!is_find_group)
      //   {
      //     groups.push_back({a, b, c});
      //   }
      //   else
      //   {
      //     if (first_check)
      //     {
      //       groups[real_group].push_back(a);
      //       groups[real_group].push_back(b);
      //       groups[real_group].push_back(c);
      //     }
      //     else
      //     {
      //       groups[real_group].push_back(b);
      //       groups[real_group].push_back(a);
      //       groups[real_group].push_back(c);
      //     }

      //     for (int i = del_checks.size() - 1; i >= 0; --i)
      //     {
      //       for (int k = 0; k < groups[del_groups[i]].size(); k += 3)
      //       {
      //         if ((del_checks[i] && first_check) || (!del_checks[i] && !first_check))
      //         {
      //           groups[real_group].push_back(groups[del_groups[i]][k+0]);
      //           groups[real_group].push_back(groups[del_groups[i]][k+1]);
      //           groups[real_group].push_back(groups[del_groups[i]][k+2]);
      //         }
      //         else
      //         {
      //           groups[real_group].push_back(groups[del_groups[i]][k+1]);
      //           groups[real_group].push_back(groups[del_groups[i]][k+0]);
      //           groups[real_group].push_back(groups[del_groups[i]][k+2]);
      //         }
      //       }
      //       groups.erase(groups.begin() + del_groups[i]);
      //     }
      //   }
      // }

      //fill global octree leaf
      unsigned cur_values_off = out_octree.values_f.size();
      out_octree.nodes[num].val_off = cur_values_off;
      out_octree.values_f.resize(cur_values_off + v_size*v_size*v_size);
      //printf("%f - %f %d\n", 2*(d/out_octree.header.brick_size), d, out_octree.header.brick_size);
      for (int i = -(int)out_octree.header.brick_pad; i <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++i)
      {
        for (int j = -(int)out_octree.header.brick_pad; j <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++j)
        {
          for (int k = -(int)out_octree.header.brick_pad; k <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++k)
          {
            float3 ch_pos = pos + 2*(d/out_octree.header.brick_size)*float3(i,j,k);
            float sign = 1, min_dist_sq = 1000000;
            //float val = 1000.0f;

            //if (distances.find(ch_pos) == distances.end())
            //{
              for (int tri=0; tri<tl_octree.nodes[idx].tid_count; tri++)
              {
                int t_i = tl_octree.triangle_ids[tl_octree.nodes[idx].tid_offset+tri];
                float3 a = to_float3(mesh.vPos4f[mesh.indices[3*t_i+0]]);
                float3 b = to_float3(mesh.vPos4f[mesh.indices[3*t_i+1]]);
                float3 c = to_float3(mesh.vPos4f[mesh.indices[3*t_i+2]]);
                float3 vt = ch_pos - cmesh4::closest_point_triangle(ch_pos, a, b, c);
                float dst_sq = LiteMath::dot(vt, vt);
                if (min_dist_sq > dst_sq)
                {
                  min_dist_sq = dst_sq;
                  if (LiteMath::dot(LiteMath::cross(a - b, a - c), vt) < 0)
                  {
                    sign = -1;
                  }
                  else
                  {
                    sign = 1;
                  }
                }
              }
              /*if (tl_octree.nodes[idx].tid_count != 0) 
              {
                distances[ch_pos] = sign * sqrtf(min_dist_sq);
                val = distances[ch_pos];
              }*/

              
            //}
            /*else
            {
              val = distances[ch_pos];
            }*/
            
            out_octree.values_f[cur_values_off + (i+out_octree.header.brick_pad)*v_size*v_size + 
                                                 (j+out_octree.header.brick_pad)*v_size + 
                                                 (k+out_octree.header.brick_pad)] = sign * sqrtf(min_dist_sq);//val;
          
            if (i >= 0 && j >= 0 && k >= 0 && i <= out_octree.header.brick_size && 
                j <= out_octree.header.brick_size && k <= out_octree.header.brick_size)
            {
              min_val = std::min(min_val, sign * sqrtf(min_dist_sq));
              max_val = std::max(max_val, sign * sqrtf(min_dist_sq));
            }
          }
        }
      }
      //tex coords
      for (int i = 0; i < 8; ++i)
      {
        //float3 ch_pos = pos + 2*d*float3((i >> 2) & 1, (i >> 1) & 1, i & 1);
        out_octree.nodes[num].tex_coords[i] = float2(0, 0);//temp stubs
      }
    }
    /*else
    {
      unsigned cur_values_off = out_octree.values_f.size();
      out_octree.nodes[num].val_off = cur_values_off;
      out_octree.values_f.resize(cur_values_off + v_size*v_size*v_size);
      for (int i = -(int)out_octree.header.brick_pad; i <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++i)
      {
        for (int j = -(int)out_octree.header.brick_pad; j <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++j)
        {
          for (int k = -(int)out_octree.header.brick_pad; k <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++k)
          {
            out_octree.values_f[cur_values_off + (i+out_octree.header.brick_pad)*v_size*v_size + 
                                                 (j+out_octree.header.brick_pad)*v_size + 
                                                 (k+out_octree.header.brick_pad)] = 0;
          }
        }
      }
    }*/
    
    if (ofs == 0) 
    {
      out_octree.nodes[num].offset = 0;
      out_octree.nodes[num].is_not_void = (min_val <= 0 && max_val >= 0);
      //printf("leaf - %u\n", tl_octree.nodes[idx].tid_count);
      return out_octree.nodes[num].is_not_void;//tl_octree.nodes[idx].tid_count != 0;
    }
    else
    {
      bool chk = false, is_not_void = false;
      //if (tl_octree.nodes[idx].tid_count == 0) {printf("AAA\n"); chk = true;}
      //printf("{");
      unsigned real_child = usefull_nodes;
      usefull_nodes += 8;
      for (int i = 0; i < 8; i++)
      {
        float ch_d = d / 2;
        float3 ch_p = 2 * p + float3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        bool x = mesh_octree_to_global_octree_rec(mesh, tl_octree, out_octree, /*distances, texs,*/ ofs + i, ch_p, ch_d, real_child + i, usefull_nodes);
        //if (chk && x) {printf("BBB\n"); chk = false;}
        is_not_void |= x;
      }
      //printf("}");
      out_octree.nodes[num].is_not_void = is_not_void;
      if (!is_not_void)
      {
        //printf("CCC %u\n", tl_octree.nodes[idx].tid_count);
        usefull_nodes -= 8;
        out_octree.nodes[num].offset = 0;
        for (int i = -(int)out_octree.header.brick_pad; i <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++i)
        {
          for (int j = -(int)out_octree.header.brick_pad; j <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++j)
          {
            for (int k = -(int)out_octree.header.brick_pad; k <= (int)out_octree.header.brick_size + (int)out_octree.header.brick_pad; ++k)
            {
              out_octree.values_f[out_octree.nodes[num].val_off + (i+out_octree.header.brick_pad)*v_size*v_size + 
                                                  (j+out_octree.header.brick_pad)*v_size + 
                                                  (k+out_octree.header.brick_pad)] = sqrt(1000000.0f);
            }
          }
        }
      }
      return is_not_void;
    }
    return false;
  }

  unsigned global_octree_count_and_mark_active_nodes_rec(GlobalOctree &octree, unsigned nodeId)
  {
    unsigned ofs = octree.nodes[nodeId].offset;
    if (is_leaf(ofs))
    {
      return (unsigned)octree.nodes[nodeId].is_not_void;
    }   
    else
    {
      unsigned sum = 0;
      for (int i = 0; i < 8; i++)
        sum += global_octree_count_and_mark_active_nodes_rec(octree, ofs + i);
      if (sum == 0) //parent node has no active children
      {
        octree.nodes[nodeId].offset = INVALID_IDX;
        octree.nodes[nodeId].is_not_void = false;
      }
      else  //parent node has active children so it is active too
      {
        sum++;
        octree.nodes[nodeId].is_not_void = true;
      }

      return sum;
    }
  }

  void global_octree_eliminate_invalid_rec(const GlobalOctree &octree_old, unsigned oldNodeId, 
                                                 GlobalOctree &octree_new, unsigned newNodeId)
  {
    unsigned ofs = octree_old.nodes[oldNodeId].offset;
    if (!is_leaf(ofs))
    {
      unsigned new_ofs = octree_new.nodes.size();
      octree_new.nodes[newNodeId].offset = new_ofs;
      for (int i = 0; i < 8; i++)
      {
        octree_new.nodes.push_back(octree_old.nodes[ofs + i]);
        if (octree_new.nodes.back().offset & INVALID_IDX)
          octree_new.nodes.back().offset = 0;
      }
      for (int i = 0; i < 8; i++)
        global_octree_eliminate_invalid_rec(octree_old, ofs + i, octree_new, new_ofs + i);
    }
  }

  void mesh_octree_to_global_octree(const cmesh4::SimpleMesh &mesh,
                                    const cmesh4::TriangleListOctree &tl_octree, 
                                    GlobalOctree &out_octree)
  {
    GlobalOctree tmp_octree;
    tmp_octree.header = out_octree.header;
    tmp_octree.nodes.resize(tl_octree.nodes.size());

    linear_mesh_octree_to_global_octree(mesh, tl_octree, tmp_octree, omp_get_max_threads());
    
    unsigned nn = global_octree_count_and_mark_active_nodes_rec(tmp_octree, 0);
    assert(!is_leaf(tmp_octree.nodes[0].offset));

    out_octree.nodes.clear();
    out_octree.values_f = tmp_octree.values_f; //some data here is not used (no node points to it), but let it be for now.
    out_octree.nodes.reserve(tmp_octree.nodes.size());
    out_octree.nodes.push_back(tmp_octree.nodes[0]);
    global_octree_eliminate_invalid_rec(tmp_octree, 0, out_octree, 0);
    out_octree.nodes.shrink_to_fit();

    //printf("%u/%u nodes are active\n", nn, (unsigned)tmp_octree.nodes.size());
    //printf("%u/%u nodes are left after elimination\n", (unsigned)out_octree.nodes.size(), (unsigned)tmp_octree.nodes.size());
  }

  void global_octree_to_frame_octree(const GlobalOctree &octree, std::vector<SdfFrameOctreeNode> &out_frame)
  {
    assert(octree.header.brick_size == 1);
    assert(octree.header.brick_pad == 0);

    out_frame.resize(octree.nodes.size());
    for (int i = 0; i < octree.nodes.size(); ++i)
    {
      out_frame[i].offset = octree.nodes[i].offset;
      for (int j=0;j<8;j++)
        out_frame[i].values[j] = octree.values_f[octree.nodes[i].val_off + j];
    }
  }

  void global_octree_to_frame_octree_tex(const GlobalOctree &octree, std::vector<SdfFrameOctreeTexNode> &out_frame)
  {
    assert(octree.header.brick_size == 1);
    assert(octree.header.brick_pad == 0);

    out_frame.resize(octree.nodes.size());
    for (int i = 0; i < octree.nodes.size(); ++i)
    {
      out_frame[i].offset = octree.nodes[i].offset;
      out_frame[i].material_id = octree.nodes[i].material_id;
      for (int j=0;j<8;j++)
      {
        out_frame[i].values[j] = octree.values_f[octree.nodes[i].val_off + j];
        out_frame[i].tex_coords[j * 2 + 0] = octree.nodes[i].tex_coords[j].x;
        out_frame[i].tex_coords[j * 2 + 1] = octree.nodes[i].tex_coords[j].y;
      }
    }
  }

  void global_octree_to_SVS_rec(const GlobalOctree &octree, std::vector<SdfSVSNode> &svs,
                                unsigned node_idx, unsigned lod_size, uint3 p)
  {
    if (octree.nodes[node_idx].offset == 0)
    {
      //unsigned v_size = sbs.header.brick_size + 2 * sbs.header.brick_pad + 1;
      unsigned v_off = octree.nodes[node_idx].val_off;

      if (octree.nodes[node_idx].is_not_void)
      {
        unsigned n_off = svs.size();
        float d_max = 2 * sqrt(3) / lod_size;
        unsigned bits = 8;
        unsigned max_val = ((1 << bits) - 1);
        unsigned vals_per_int = 4;

        svs.emplace_back();

        svs[n_off].pos_xy = (p.x << 16) | p.y;
        svs[n_off].pos_z_lod_size = (p.z << 16) | lod_size;

        svs[n_off].values[0] = 0u;
        svs[n_off].values[1] = 0u;

        for (int i = 0; i < 8; i++)
        {
          unsigned d_compressed = std::max(0.0f, max_val * ((octree.values_f[v_off + i] + d_max) / (2 * d_max)) + 0.5f);
          d_compressed = std::min(d_compressed, max_val);
          svs[n_off].values[i / vals_per_int] |= d_compressed << (bits * (i % vals_per_int));
        }
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = lod_size / 2;
        uint3 ch_p = 2 * p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        global_octree_to_SVS_rec(octree, svs, octree.nodes[node_idx].offset + i, 2*lod_size, ch_p);
      }
    }
  }

  void global_octree_to_SVS(const GlobalOctree &octree, std::vector<SdfSVSNode> &svs)
  {
    assert(1 == octree.header.brick_size);
    assert(0 == octree.header.brick_pad);

    global_octree_to_SVS_rec(octree, svs, 0, 1, uint3(0,0,0));
  }

  void global_octree_to_SBS_rec(const GlobalOctree &octree, SdfSBS &sbs,
                                unsigned node_idx, unsigned lod_size, uint3 p)
  {
    if (octree.nodes[node_idx].offset == 0)
    {
      unsigned v_size = sbs.header.brick_size + 2 * sbs.header.brick_pad + 1;
      unsigned v_off = octree.nodes[node_idx].val_off;

      if (octree.nodes[node_idx].is_not_void)
      {
        unsigned off = sbs.values.size();
        unsigned n_off = sbs.nodes.size();
        float d_max = 2 * sqrt(3) / lod_size;
        unsigned bits = 8 * sbs.header.bytes_per_value;
        unsigned max_val = sbs.header.bytes_per_value == 4 ? 0xFFFFFFFF : ((1 << bits) - 1);
        unsigned vals_per_int = 4 / sbs.header.bytes_per_value;
        unsigned texs_size = (sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_DX_UV16 ? 32 : 0;
        unsigned dist_size = (v_size * v_size * v_size + vals_per_int - 1) / vals_per_int;

        sbs.nodes.emplace_back();
        sbs.values.resize(sbs.values.size() + dist_size + texs_size);

        sbs.nodes[n_off].data_offset = off;
        sbs.nodes[n_off].pos_xy = (p.x << 16) | p.y;
        sbs.nodes[n_off].pos_z_lod_size = (p.z << 16) | lod_size;

        for (int i = 0; i < v_size * v_size * v_size; i++)
        {
          unsigned d_compressed = std::max(0.0f, max_val * ((octree.values_f[v_off + i] + d_max) / (2 * d_max)));
          d_compressed = std::min(d_compressed, max_val);
          sbs.values[off + i / vals_per_int] |= d_compressed << (bits * (i % vals_per_int));
        }
        if ((sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_DX_UV16)
        {
          for (int i = 0; i < 8; ++i)
          {
            unsigned packed_u = ((1<<16) - 1)*LiteMath::clamp(octree.nodes[node_idx].tex_coords[i].x, 0.0f, 1.0f);
            unsigned packed_v = ((1<<16) - 1)*LiteMath::clamp(octree.nodes[node_idx].tex_coords[i].y, 0.0f, 1.0f);
            sbs.values[off + dist_size + i] = (packed_u << 16) | (packed_v & 0x0000FFFF);
          }
        }
      }
    }
    else
    {
      for (int i = 0; i < 8; i++)
      {
        float ch_d = lod_size / 2;
        uint3 ch_p = 2 * p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
        global_octree_to_SBS_rec(octree, sbs, octree.nodes[node_idx].offset + i, 2*lod_size, ch_p);
      }
    }
  }

  void global_octree_to_SBS(const GlobalOctree &octree, SdfSBS &sbs)
  {
    assert(sbs.header.brick_size == octree.header.brick_size);
    assert(sbs.header.brick_pad == octree.header.brick_pad);
    assert((sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_DX ||
           (sbs.header.aux_data & SDF_SBS_NODE_LAYOUT_MASK) == SDF_SBS_NODE_LAYOUT_DX_UV16);

    global_octree_to_SBS_rec(octree, sbs, 0, 1, uint3(0,0,0));
  }

  bool is_empty_node(const float *values)
  {
    float min_val = 1000;
    float max_val = -1000;
    for (int i = 0; i < 8; i++)
    {
      min_val = std::min(min_val, values[i]);
      max_val = std::max(max_val, values[i]);
    }

    return min_val > 0 || max_val < 0;
  }

  void global_octree_to_compact_octree_v2_rec(const GlobalOctree &octree, std::vector<uint32_t> &compact,
                                             unsigned nodeId, unsigned cNodeId, unsigned lod_size, uint3 p)
  {
    assert(cNodeId % 2 == 0);
    unsigned ofs = octree.nodes[nodeId].offset;
    //printf("node p = (%u, %u, %u), lod_size = %u\n", p.x, p.y, p.z, lod_size);
    unsigned v_off = octree.nodes[nodeId].val_off;
    assert(!(is_leaf(ofs) && !octree.nodes[nodeId].is_not_void));

    if (is_leaf(ofs))
    {
      float d_max = 2 * sqrt(3) / lod_size;
      unsigned v_off = octree.nodes[nodeId].val_off;
      for (int i = 0; i < 8; i++)
      {
        unsigned d_compressed = std::max(0.0f, 255 * ((octree.values_f[v_off + i] + d_max) / (2 * d_max)) + 0.5f);
        d_compressed = std::min(d_compressed, 255u);
        compact[cNodeId + i / 4] |= d_compressed << (8 * (i % 4));
      }
    }
    else
    {
      unsigned ch_info = 0u;
      unsigned ch_count = 0u;
      unsigned l_count  = 0u;
      unsigned char ch_is_active = 0u; //one bit per child
      unsigned char ch_is_leaf   = 0u;  //one bit per child
      int ch_ofs[8];

      for (int i = 0; i < 8; i++)
      {
        unsigned child_info = ch_count;
        if (is_leaf(octree.nodes[ofs + i].offset))
        {
          unsigned v_off = octree.nodes[ofs + i].val_off;
          if (!octree.nodes[ofs + i].is_not_void)
          {
            ch_ofs[i] = -1;
          }    
          else
          {
            ch_ofs[i] = ch_count;
            ch_count++;
            l_count ++;        
            ch_is_active |= (1 << i);    
            ch_is_leaf   |= (1 << i);
          }      
        }
        else
        {
          ch_ofs[i] = ch_count;
          ch_count++;     
          ch_is_active |= (1 << i);      
        }
      }
      
      compact[cNodeId + 0] = compact.size();
      compact[cNodeId + 1] = ch_is_active | (ch_is_leaf << 8u);

      unsigned base_ch_off = compact.size();
      compact.resize(base_ch_off + 2*ch_count, 0u);

      for (int i = 0; i < 8; i++)
      {
        if (ch_ofs[i] >= 0)
        {
          uint3 ch_p = 2 * p + uint3((i & 4) >> 2, (i & 2) >> 1, i & 1);
          global_octree_to_compact_octree_v2_rec(octree, compact, ofs + i, base_ch_off + 2*ch_ofs[i], 2 * lod_size, ch_p);
        }
      }
    }
  }

  void global_octree_to_COctreeV2(const GlobalOctree &octree, COctreeV2 &compact)
  {
    assert(octree.header.brick_size == 1);
    assert(octree.header.brick_pad == 0);

    compact.data.clear();
    compact.data.reserve(2*octree.nodes.size());
    compact.data.resize(2, 0u);

    global_octree_to_compact_octree_v2_rec(octree, compact.data, 0, 0, 1, uint3(0,0,0));

    compact.data.shrink_to_fit();
  }

  struct LayerFrameNodeInfo
  {
    unsigned idx;
    bool is_leaf;
    bool is_border;
    unsigned border_children;
  };

  struct FrameNodeQualitySort
  {
    unsigned idx;
    unsigned active_children; //active children count
    float weighted_diff; //average sdf diff. on children sample points / weight
  };

  bool fill_layers_rec(const std::vector<SdfFrameOctreeNode> &frame, std::vector<std::vector<LayerFrameNodeInfo>> &layers, 
                      unsigned level, unsigned idx)
  {  
    if (level >= layers.size())
      layers.resize(level+1);

    unsigned ofs = frame[idx].offset;
    bool leaf = is_leaf(ofs);
    float min_val = frame[idx].values[0];
    float max_val = frame[idx].values[0];
    for (int i=0;i<8;i++)
    {
      min_val = std::min(min_val, frame[idx].values[i]);
      max_val = std::max(max_val, frame[idx].values[i]);
    }
    bool border = is_border_node(min_val, max_val, 1 << level);
    unsigned border_children = 0;

    if (!leaf)
    {
      for (unsigned i=0;i<8;i++)
        border_children += fill_layers_rec(frame, layers, level+1, ofs+i);
    }

    //printf("layer %u, idx %u, leaf = %d, border = %d, border_children = %u\n", level, idx, leaf, border, border_children);
    LayerFrameNodeInfo node = {idx, leaf, border, border_children};
    layers[level].push_back(node);

    return border;
  }

  static float trilinear_interpolation(const float values[8], float3 dp)
  {
    return (1-dp.x)*(1-dp.y)*(1-dp.z)*values[0] + 
          (1-dp.x)*(1-dp.y)*(  dp.z)*values[1] + 
          (1-dp.x)*(  dp.y)*(1-dp.z)*values[2] + 
          (1-dp.x)*(  dp.y)*(  dp.z)*values[3] + 
          (  dp.x)*(1-dp.y)*(1-dp.z)*values[4] + 
          (  dp.x)*(1-dp.y)*(  dp.z)*values[5] + 
          (  dp.x)*(  dp.y)*(1-dp.z)*values[6] + 
          (  dp.x)*(  dp.y)*(  dp.z)*values[7];
  }

  static void print_layers(const std::vector<std::vector<LayerFrameNodeInfo>> &layers, bool count_only_border_nodes)
  {
    for (auto &layer : layers)
    {
      unsigned l_cnt = 0;
      for (auto &node : layer)
      {
        if (node.is_leaf && (!count_only_border_nodes || node.is_border))
          l_cnt++;
      }
      printf("layer %u (%u)\n", (unsigned)layer.size(), l_cnt);
    }
  }

  void frame_octree_limit_nodes(std::vector<SdfFrameOctreeNode> &frame, unsigned nodes_limit,
                                bool count_only_border_nodes)
  {
    if (nodes_limit >= frame.size())
      return;
    
    assert(nodes_limit > 0);
    assert(frame.size() > 0);

    std::vector<std::vector<LayerFrameNodeInfo>> layers;
    fill_layers_rec(frame, layers, 0, 0);

    unsigned cnt = 0;
    for (auto &layer : layers)
      for (auto &node : layer)
        if (node.is_leaf && (!count_only_border_nodes || node.is_border))
          cnt++;

    //print_layers(layers, count_only_border_nodes);
    //printf("cnt = %u, nodes_limit = %u\n", cnt, nodes_limit);

    //layer from which we are deleting nodes, to do so, their parent nodes are evaluated
    unsigned active_layer = layers.size() - 1;

    while (cnt > nodes_limit && active_layer > 0)
    {
      std::vector<FrameNodeQualitySort> merge_candidates;
      for (auto &p_idx : layers[active_layer-1])
      {
        //printf("%u %d %d %u\n", p_idx.idx, p_idx.is_leaf, p_idx.is_border, p_idx.border_children);
        unsigned ofs = frame[p_idx.idx].offset;
        unsigned active_children = count_only_border_nodes ? p_idx.border_children : 8;
        //if (active_children < 2)
        //  printf("ERROR: %u %u\n", p_idx.idx, active_children);
        if (is_leaf(ofs))
          continue;

        float diff = 0;
        for (unsigned i=0;i<8;i++)
        {
          SdfFrameOctreeNode &child = frame[ofs+i];
          for (unsigned j=0;j<8;j++)
          {
            float child_val = child.values[j];
            float3 parent_q = 0.5f*(float3((i & 4) >> 2, (i & 2) >> 1, i & 1) + float3((j & 4) >> 2, (j & 2) >> 1, j & 1));
            float parent_val = trilinear_interpolation(frame[p_idx.idx].values, parent_q);
            diff += std::abs(child_val - parent_val);
          }
        }
        merge_candidates.push_back({p_idx.idx, active_children, active_children > 1 ? diff/(active_children - 1) : 1000});
      }

      assert(merge_candidates.size() > 0);

      unsigned delete_potential = 0;
      for (auto &mc : merge_candidates)
        delete_potential += mc.active_children-1;

      if (cnt - delete_potential < nodes_limit)
      {
        std::sort(merge_candidates.begin(), merge_candidates.end(), 
                  [](const FrameNodeQualitySort &a, const FrameNodeQualitySort &b){return a.weighted_diff < b.weighted_diff;});
      }
      //else we have to delete the whole layer

      int c_i = 0;
      while (cnt > nodes_limit && c_i < merge_candidates.size())
      {
        unsigned idx = merge_candidates[c_i].idx;
        for (unsigned i=0;i<8;i++)
          frame[frame[idx].offset+i].offset = INVALID_IDX;
        frame[idx].offset = 0;
        //printf("%f\n", merge_candidates[c_i].weighted_diff * 7.0 / 64.0);
        //printf("removing %u %u %f, left %d\n", idx, merge_candidates[c_i].active_children, 
        //merge_candidates[c_i].weighted_diff, cnt);
        cnt -= (merge_candidates[c_i].active_children - 1);
        c_i++;
      }

      //for (int i=0;i<merge_candidates.size();i++)
      //  printf("%u %u %f\n", merge_candidates[i].idx, merge_candidates[i].weight, merge_candidates[i].weighted_diff);
      layers = {};
      fill_layers_rec(frame, layers, 0, 0);
      cnt = 0;
      for (auto &layer : layers)
        for (auto &node : layer)
          if (node.is_leaf && (!count_only_border_nodes || node.is_border))
            cnt++;

      //print_layers(layers, count_only_border_nodes);
      active_layer--;
    }

    std::vector<unsigned> idx_remap(frame.size(), 0);
    std::vector<SdfFrameOctreeNode> new_frame;

    new_frame.reserve(frame.size());
    for (unsigned i=0;i<frame.size();i++)
    {
      if (frame[i].offset != INVALID_IDX)
      {
        idx_remap[i] = new_frame.size();
        new_frame.push_back(frame[i]);
      }
    }

    for (unsigned i=0;i<new_frame.size();i++)
    {
      if (new_frame[i].offset > 0)
        new_frame[i].offset = idx_remap[new_frame[i].offset];
    }

    frame = new_frame;

    //printf("cnt left %u\n", cnt);
  }
}