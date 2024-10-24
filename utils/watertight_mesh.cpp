#include "mesh.h"
#include "vector_comparators.h"

#include <iostream>
#include <map>
#include <algorithm>

#define EPS 1e-10


namespace cmesh4
{
  int find_edge_in_planes(const std::vector<LiteMath::float4> &mesh_vertices,
                          const std::vector<unsigned int> &mesh_indices,
                           std::map<std::pair<unsigned int, unsigned int>,
                                    std::vector<unsigned int>>& edge_in_planes,
                           unsigned int ind,
                           std::pair<unsigned int, unsigned int> edge,
                           int ind1, int ind2, int ind3);
  
  float same_elem(std::vector<LiteMath::float4> mesh_vertices, unsigned int ind);

  float for_inside_line(float p_x, float m1_x, float m2_x);

  bool inside_line(LiteMath::float3 p, LiteMath::float3 a, LiteMath::float3 b);

  float triangle_square(float a,float b, float c);

  bool inside_triangle(LiteMath::float3 p, LiteMath::float3 a,
                       LiteMath::float3 b, LiteMath::float3 c);
  
  bool plane_equation(LiteMath::float3 m0, LiteMath::float3 m1, LiteMath::float3 m2,
                      LiteMath::float3 p);

  LiteMath::float3 float4_to_float3(LiteMath::float4 m);



  float coarse_triangulation(std::vector<LiteMath::float4>& mesh_vertices,
                            std::map<uint2, std::vector<float>, cmpUint2>& weights_of_sides,
                            std::vector<unsigned int>& vertex_numbs,
                            unsigned int a, unsigned int c){
    if(c == a + 1){
      return 0;
    }

    auto it = weights_of_sides.find({a, c});
    if (it != weights_of_sides.end()){
      return weights_of_sides[{a, c}][0];
    }


    float w, w1, w2;
    LiteMath::float4 p1 = mesh_vertices[vertex_numbs[a]];
    LiteMath::float4 p2 = mesh_vertices[vertex_numbs[c]];
    LiteMath::float4 p3;
    float p = 0;

    for(unsigned int b = a + 1; b < c; b++){
      w1 = coarse_triangulation(mesh_vertices, weights_of_sides, vertex_numbs, std::min(a, b), std::max(a, b));
      w2 = coarse_triangulation(mesh_vertices, weights_of_sides, vertex_numbs, std::min(b, c), std::max(b, c));
      p3 = mesh_vertices[vertex_numbs[b]];

      p += sqrt(pow((p1.x - p2.x), 2) + pow((p1.y - p2.y), 2) + pow((p1.z - p2.z), 2));
      p += sqrt(pow((p1.x - p3.x), 2) + pow((p1.y - p3.y), 2) + pow((p1.z - p3.z), 2));
      p += sqrt(pow((p3.x - p2.x), 2) + pow((p3.y - p2.y), 2) + pow((p3.z - p2.z), 2));


      w = w1 + w2 + p;

      auto it = weights_of_sides.find({a, c});
      if (it == weights_of_sides.end())
        weights_of_sides[{a, c}] = {w, float(a), float(b), float(c)};
      else
        if(weights_of_sides[{a, c}][0] > w)
          weights_of_sides[{a, c}] = {w, float(a), float(b), float(c)};
    }
    return weights_of_sides[{a, c}][0];
  }


  void instead_of_hole(std::vector<unsigned int>& no_hole,
                       std::vector<unsigned int> vertex_numbs,
                       std::map<uint2, std::vector<float>, cmpUint2>& weights_of_sides,
                       unsigned int m1, unsigned int m2){
    unsigned int a, b, c;

    auto it = weights_of_sides.find({m1, m2});
    if(it != weights_of_sides.end()){
      a = vertex_numbs[weights_of_sides[{m1, m2}][1]];
      b = vertex_numbs[weights_of_sides[{m1, m2}][2]];
      c = vertex_numbs[weights_of_sides[{m1, m2}][3]];

      no_hole.push_back(a);
      no_hole.push_back(b);
      no_hole.push_back(c);

      auto p = find(vertex_numbs.begin(), vertex_numbs.end(), b);
      if(p != vertex_numbs.end()){
        b = p - vertex_numbs.begin();
      }

      p = find(vertex_numbs.begin(), vertex_numbs.end(), a);
      if(p != vertex_numbs.end()){
        a = p - vertex_numbs.begin();
        instead_of_hole(no_hole, vertex_numbs, weights_of_sides, std::min(a, b), std::max(a, b));
      }

      p = find(vertex_numbs.begin(), vertex_numbs.end(), c);
      if(p != vertex_numbs.end()){
        c = p - vertex_numbs.begin();
        instead_of_hole(no_hole, vertex_numbs, weights_of_sides, std::min(b, c), std::max(b, c));
      }
    }
  }

  cmesh4::SimpleMesh filling_holes(cmesh4::SimpleMesh& mesh,
                                   std::vector<std::vector<unsigned int>>& vect_of_holes,
                                   std::vector<LiteMath::float4>& mesh_vertices,
                                   std::vector<unsigned int>& mesh_indices){
    std::map<uint2, std::vector<float>, cmpUint2> weights_of_sides;
    std::vector<unsigned int> vertex_numbs; // the element in vertex_numbers is the vertex itself, and its position is its number

    for(unsigned int i = 0; i < vect_of_holes[0].size(); i+=2)
      vertex_numbs.push_back(vect_of_holes[0][i]);

    coarse_triangulation(mesh_vertices, weights_of_sides, vertex_numbs, 0, vertex_numbs.size() - 1);

    std::vector<unsigned int> mesh_indices_without_holes = mesh_indices;


    std::vector<unsigned int> no_hole; // 3 numbers form a triangle
    instead_of_hole(no_hole, vertex_numbs, weights_of_sides, 0, vertex_numbs.size() - 1);

    
    for(unsigned int i = 0; i < no_hole.size(); i+=3){
      mesh_indices_without_holes.push_back(no_hole[i]);
      mesh_indices_without_holes.push_back(no_hole[i + 1]);
      mesh_indices_without_holes.push_back(no_hole[i + 2]);
    }

    mesh.indices = mesh_indices_without_holes;

    return mesh;
  }


  // Looking for holes in the mesh
  std::vector<std::vector<unsigned int>> holes_search(std::vector<uint2>& vec_edge_occurs_1time){
    std::vector<unsigned int> vertex_vec;
    std::vector<unsigned int> del_vect;
    std::vector<std::vector<unsigned int>> vect_of_holes;
    uint2 start_edge; 
    uint2 checkpoint;
    int del_vect_size;
    bool fl;
    int fl2; // if fl = 1, then compares the value of checkpoint.x with the values of start_edge
             // if fl = 2, then compares the value of checkpoint.y with the values of start_edge

    for(uint2 i : vec_edge_occurs_1time){
      vertex_vec.push_back(i.x);
      vertex_vec.push_back(i.y);
    }

    unsigned int k;
    for(unsigned int i = 0; i < vertex_vec.size(); i += 2){
      start_edge.x = vertex_vec[i];
      start_edge.y = vertex_vec[i + 1];

      fl = 0;
      fl2 = 0;
      k = i;
      while (true){
        del_vect.push_back(vertex_vec[k]);
        checkpoint.x = vertex_vec[k];
        vertex_vec.erase(vertex_vec.begin() + k);
        
        del_vect.push_back(vertex_vec[k]);
        checkpoint.y = vertex_vec[k];
        vertex_vec.erase(vertex_vec.begin() + k);

        if((fl2 == 1) && ((checkpoint.x == start_edge.x) || (checkpoint.x == start_edge.y))){
          fl = 1;
          break;
        }

        if((fl2 == 2) && ((checkpoint.y == start_edge.x) || (checkpoint.y == start_edge.y))){
          fl = 1;
          break;
        }

        auto pointer = find(vertex_vec.begin(), vertex_vec.end(), checkpoint.x);
        if(pointer == vertex_vec.end()){
          auto pointer = find(vertex_vec.begin(), vertex_vec.end(), checkpoint.y);
          if(pointer == vertex_vec.end())
            break;
          else{
            fl2 = 2;
            k = pointer - vertex_vec.begin();
            if(k % 2 != 0){
              fl2 = 1;
              k = pointer - vertex_vec.begin() - 1;
            }
          }
        }
        else{
          fl2 = 2;
          k = pointer - vertex_vec.begin();
          if(k % 2 != 0){
            fl2 = 1;
            k = pointer - vertex_vec.begin() - 1;
          }
        }
      }
      if(fl){
        for(unsigned int q = 0; q < del_vect.size() - 2; q+=2){
          if(del_vect[q + 1] == del_vect[q + 2])
            continue;
          else if(del_vect[q] == del_vect[q + 2])
            std::swap(del_vect[q], del_vect[q + 1]);
          else if (del_vect[q] == del_vect[q + 3]){
            std::swap(del_vect[q], del_vect[q + 1]);
            std::swap(del_vect[q + 2], del_vect[q + 3]);
          }
          else if(del_vect[q + 1] == del_vect[q + 3])
            std::swap(del_vect[q + 2], del_vect[q + 3]);
        }
        vect_of_holes.push_back(del_vect);
        i = -2;
      }
      del_vect_size = del_vect.size();
      del_vect.clear();
    }
    // std::cout << "There are " << vect_of_holes.size() << " holes left to close\n";
    // std::cout << "There are " << del_vect_size << " vertices in the last hole\n\n";

    return vect_of_holes;
  }


  std::map<uint2, std::vector<unsigned int>, cmpUint2> presence_of_edges_in_the_planes(const cmesh4::SimpleMesh& mesh){
    std::vector<LiteMath::float4> mesh_vertices = mesh.vPos4f;
    std::vector<unsigned int> mesh_indices = mesh.indices;

    std::map<int3, unsigned, cmpInt3> vert_indices;
    float eps = 1e-5;
    float inv_eps = 1/eps;
    std::vector<unsigned> vert_remap;
    vert_remap.resize(mesh_vertices.size());

    for (int i=0;i<mesh_vertices.size();i++)
    {
      int3 v_idx = int3(inv_eps*LiteMath::to_float3(mesh_vertices[i]) + LiteMath::float3(0.5, 0.5, 0.5));
      auto it = vert_indices.find(v_idx);
      if (it == vert_indices.end())
      {
        vert_indices[v_idx] = i;
        vert_remap[i] = i;
      }
      else
        vert_remap[i] = it->second;
    }

    std::map<uint2, std::vector<unsigned int>, cmpUint2> edge_in_planes;

    for (int i=0;i<mesh_indices.size();i+=3)
    {
      unsigned i1 = vert_remap[mesh_indices[i]];
      unsigned i2 = vert_remap[mesh_indices[i+1]];
      unsigned i3 = vert_remap[mesh_indices[i+2]];

      unsigned a = std::min(i1, std::min(i2, i3));
      unsigned c = std::max(i1, std::max(i2, i3));
      unsigned b = i1;

      if (b == a || b == c) b = i2;
      if (b == a || b == c) b = i3;

      auto it = edge_in_planes.find({a, b});
      if (it == edge_in_planes.end())
        edge_in_planes[uint2(a,b)] = {};
      edge_in_planes[{a, b}].push_back(i);
      
      it = edge_in_planes.find({b, c});
      if (it == edge_in_planes.end())
        edge_in_planes[uint2(b,c)] = {};
      edge_in_planes[{b, c}].push_back(i);
      
      it = edge_in_planes.find({a, c});
      if (it == edge_in_planes.end())
        edge_in_planes[uint2(a,c)] = {};
      edge_in_planes[{a, c}].push_back(i);
    }

    return edge_in_planes;
  }

  // ind - Quantity of holes found; if x = 1, then all the holes are patched
  cmesh4::SimpleMesh removing_holes(cmesh4::SimpleMesh& mesh, int& ind, bool& fl){
    std::vector<LiteMath::float4> mesh_vertices = mesh.vPos4f;
    std::vector<unsigned int> mesh_indices = mesh.indices;

    std::map<uint2, std::vector<unsigned int>, cmpUint2> edge_in_planes;
    edge_in_planes = presence_of_edges_in_the_planes(mesh);
    
    int i=0;
    std::vector<uint2> vec_edge_occurs_1time;
    for (auto it = edge_in_planes.begin(); it != edge_in_planes.end(); it++)
    {
      if (it->second.size() < 2){
        if(it->second.size() == 1){
          if(it->first.x != it->first.y){
            vec_edge_occurs_1time.push_back(it->first);
          }
        }
      }
      i++;
    }

    cmesh4::SimpleMesh mesh_without_holes;
    if(vec_edge_occurs_1time.size() != 0){
      std::vector<std::vector<unsigned int>> vect_of_holes = holes_search(vec_edge_occurs_1time);
      if(ind == -1){
        ind = vect_of_holes.size();
      }

      if(vect_of_holes.size() != 0){
        if(vect_of_holes[0].size() != 0){
          mesh_without_holes = filling_holes(mesh, vect_of_holes, mesh_vertices, mesh_indices);
          removing_holes(mesh_without_holes, ind, fl);
        }
      }
    }
    else
      fl = 1;
    
    return mesh_without_holes;
  }

  cmesh4::SimpleMesh before_removing_holes(cmesh4::SimpleMesh mesh, int& ind, bool& fl){
      bool res = check_watertight_mesh(mesh);
      if(res){
        ind = -2;
        return mesh;
      }

      removing_holes(mesh, ind, fl);
      return mesh;
    }

  bool fast_watertight(const cmesh4::SimpleMesh& mesh, bool verbose)
  {
    std::vector<LiteMath::float4> mesh_vertices = mesh.vPos4f;
    std::vector<unsigned int> mesh_indices = mesh.indices;

    if (mesh_vertices.size() == 0 || mesh_indices.size() == 0 || mesh_indices.size()%3 != 0)
      return false;

    std::map<uint2, std::vector<unsigned int>, cmpUint2> edge_in_planes;

    edge_in_planes = presence_of_edges_in_the_planes(mesh);

    int i=0;
    int hanging_edges = 0;
    for (auto it = edge_in_planes.begin(); it != edge_in_planes.end(); it++)
    {
      if (it->second.size() != 2){
        hanging_edges++;
      }
      i++;
    }
    
    bool watertight = true;
    if (hanging_edges > 0)
    {
      if (verbose)
        printf("WARNING: mesh has %d hanging edges\n", hanging_edges);
      watertight = false;
    }
    else if (verbose)
      printf("OK: mesh has no hanging edges\n");

    return watertight;
  }

  bool check_watertight_mesh(const cmesh4::SimpleMesh& mesh, bool verbose){
    return fast_watertight(mesh, verbose);


    std::vector<LiteMath::float4> mesh_vertices = mesh.vPos4f;
    std::vector<unsigned int> mesh_indices = mesh.indices;

    // key: edge, value: planes (3 indexes - one plane)
    std::map<std::pair<unsigned int, unsigned int>, std::vector<unsigned int>> edge_in_planes;

    if(mesh_indices.size() % 3 != 0){
      std::cout << "The number of vertices isn't a multiple of 3" << std::endl;
      return 0;
    }

    std::pair<unsigned int, unsigned int> edge;
    std::vector<unsigned int> planes;
    float el1, el2, el3;
    int find_edge;

    for(unsigned int i = 0; i < mesh_indices.size() - 2; i += 3){
      if(mesh_vertices[mesh_indices[i]].x == mesh_vertices[mesh_indices[i + 1]].x &&
         mesh_vertices[mesh_indices[i]].y == mesh_vertices[mesh_indices[i + 1]].y &&
         mesh_vertices[mesh_indices[i]].z == mesh_vertices[mesh_indices[i + 1]].z){
        std::cout << "There are three equal vertices in one triangle: vertices (" << 
                     mesh_vertices[mesh_indices[i]].x << ", " << mesh_vertices[mesh_indices[i]].y <<
                     ", " << mesh_vertices[mesh_indices[i]].z << ") with indexes " <<
                     mesh_indices[i] << " and " << mesh_indices[i + 1] << std::endl;
        return 0;
      }

      if(mesh_vertices[mesh_indices[i]].x == mesh_vertices[mesh_indices[i + 2]].x &&
         mesh_vertices[mesh_indices[i]].y == mesh_vertices[mesh_indices[i + 2]].y &&
         mesh_vertices[mesh_indices[i]].z == mesh_vertices[mesh_indices[i + 2]].z){
        std::cout << "There are two equal vertices in one triangle: vertices (" << 
                     mesh_vertices[mesh_indices[i]].x << ", " << mesh_vertices[mesh_indices[i]].y <<
                     ", " << mesh_vertices[mesh_indices[i]].z << ") with indexes " <<
                     mesh_indices[i] << " and " << mesh_indices[i + 2] << std::endl;
        return 0;
      }

      if(mesh_vertices[mesh_indices[i + 1]].x == mesh_vertices[mesh_indices[i + 2]].x &&
         mesh_vertices[mesh_indices[i + 1]].y == mesh_vertices[mesh_indices[i + 2]].y &&
         mesh_vertices[mesh_indices[i + 1]].z == mesh_vertices[mesh_indices[i + 2]].z){
        std::cout << "There are two equal vertices in one triangle: vertices (" << 
                     mesh_vertices[mesh_indices[i + 1]].x << ", " << mesh_vertices[mesh_indices[i + 1]].y <<
                     ", " << mesh_vertices[mesh_indices[i + 1]].z << ") with indexes " <<
                     mesh_indices[i + 1] << " and " << mesh_indices[i + 2] << std::endl;
        return 0;
      }
      
      edge.first = mesh_indices[i];
      edge.second = mesh_indices[i + 1];
      if((find_edge = find_edge_in_planes(mesh_vertices, mesh_indices, edge_in_planes,
                                         i, edge, i, i + 1, i + 2)) == 0){
        std::cout << "An edge with vertex  (" << mesh_vertices[edge.first].x << ", " <<
                     mesh_vertices[edge.first].y << ", " << mesh_vertices[edge.first].z <<
                     ") with index " << edge.first << " and vertex (" <<
                     mesh_vertices[edge.second].x << ", " << mesh_vertices[edge.second].y
                     << ", " << mesh_vertices[edge.second].z <<") with index " << edge.second
                     << " belongs to only one plane" << std::endl;
        return 0;
      }
      else if(find_edge < 0){
        std::cout << "A triangle with vertex (" <<
                     mesh_vertices[edge.first].x << ", " << mesh_vertices[edge.first].y <<
                     ", " << mesh_vertices[edge.first].z << ") with index " << edge.first <<
                     
                     " and vertex (" << mesh_vertices[edge.second].x << ", " <<
                     mesh_vertices[edge.second].y << ", " << mesh_vertices[edge.second].z <<
                     ") with index " << edge.second <<
                     
                     " and vertex (" << mesh_vertices[mesh_indices[i + 2]].x << ", " <<
                     mesh_vertices[mesh_indices[i + 2]].y << ", " << mesh_vertices[mesh_indices[i + 2]].z
                     << ") with index " << mesh_indices[i + 2];

        std::cout << " intersects with a triangle with vertex (" <<
                     mesh_vertices[mesh_indices[-find_edge]].x << ", " <<
                     mesh_vertices[mesh_indices[-find_edge]].y << ", " <<
                     mesh_vertices[mesh_indices[-find_edge]].z << ") with index " <<
                     mesh_indices[-find_edge] << " and vertex (" <<

                     mesh_vertices[mesh_indices[-find_edge + 1]].x << ", " <<
                     mesh_vertices[mesh_indices[-find_edge + 1]].y << ", " <<
                     mesh_vertices[mesh_indices[-find_edge + 1]].z << ") with index " <<
                     mesh_indices[-find_edge + 1] << " and vertex (" <<

                     mesh_vertices[mesh_indices[-find_edge + 2]].x << ", " <<
                     mesh_vertices[mesh_indices[-find_edge + 2]].y << ", " <<
                     mesh_vertices[mesh_indices[-find_edge + 2]].z << ") with index " <<
                     mesh_indices[-find_edge + 2] << ")";

      }

      edge.first = mesh_indices[i];
      edge.second = mesh_indices[i + 2];
      if(find_edge_in_planes(mesh_vertices, mesh_indices, edge_in_planes,
                             i, edge, -1, -1, -1) == 0){
        std::cout << "An edge with vertex  (" << mesh_vertices[edge.first].x << ", " <<
                     mesh_vertices[edge.first].y << ", " << mesh_vertices[edge.first].z <<
                     ") with index " << edge.first << " and vertex (" <<
                     mesh_vertices[edge.second].x << ", " << mesh_vertices[edge.second].y
                     << ", " << mesh_vertices[edge.second].z <<") index " << edge.second
                     << " belongs to only one plane" << std::endl;
        return 0;
      }

      edge.first = mesh_indices[i + 1];
      edge.second = mesh_indices[i + 2];
      if(find_edge_in_planes(mesh_vertices, mesh_indices, edge_in_planes,
                             i, edge, -1, -1, -1) == 0){
        std::cout << "An edge with vertex  (" << mesh_vertices[edge.first].x << ", " <<
                     mesh_vertices[edge.first].y << ", " << mesh_vertices[edge.first].z <<
                     ") with index " << edge.first << " and vertex (" <<
                     mesh_vertices[edge.second].x << ", " << mesh_vertices[edge.second].y
                     << ", " << mesh_vertices[edge.second].z <<") index " << edge.second
                     << " belongs to only one plane" << std::endl;
        return 0;
      }

        
      
    }

    std::vector<unsigned int> check;
    unsigned int c;
    bool flag = 1;
    for(const auto& [vert_pair, ind_planes]: edge_in_planes){

      if(ind_planes.size() != 6){
        for(unsigned int j = 0; j < ind_planes.size() - 2; j += 3){
          flag = 1;
          c = ind_planes[j] * 100 + ind_planes[j + 1] * 10 + ind_planes[j + 2];
          
          if(check.size() == 0){
            check.push_back(c);
          }
          else{
            for(unsigned int k = 0; k < check.size(); k++){
              if(check[k] == c){
                flag = 0;
                break;
              }
            }
            if(flag){
              check.push_back(c);
            }
          }
        }

        if(check.size() != 2){
          std::cout << "An edge with vertex (" << mesh_vertices[vert_pair.first].x << 
                    ", " << mesh_vertices[vert_pair.first].y << ", " <<
                    mesh_vertices[vert_pair.first].z << ") with index " << vert_pair.first
                    << " and vertex (" << mesh_vertices[vert_pair.second].x << ", " <<
                    mesh_vertices[vert_pair.second].y << ", " << mesh_vertices[vert_pair.second].z <<
                    ") with index " << vert_pair.second << " borders on " << check.size() <<
                    " planes" << std::endl;
          return 0;
        }
        check.clear();
      }
    }

    return 1;

  }


  int find_edge_in_planes(const std::vector<LiteMath::float4> &mesh_vertices,
                          const std::vector<unsigned int> &mesh_indices,
                           std::map<std::pair<unsigned int, unsigned int>,
                                    std::vector<unsigned int>>& edge_in_planes,
                           unsigned int ind,
                           std::pair<unsigned int, unsigned int> edge,
                           int ind1, int ind2, int ind3){
    LiteMath::float3 m0, m1, m2, p1, p2, p3;
    std::vector<unsigned int> planes;
    float el1, el2;

    unsigned int count = 0;
    unsigned int min_num, max_num;
    
    for(unsigned int i = 0; i < mesh_indices.size() - 2; i += 3){
      if(i != ind){
        m0 = float4_to_float3(mesh_vertices[mesh_indices[i]]);
        m1 = float4_to_float3(mesh_vertices[mesh_indices[i + 1]]);
        m2 = float4_to_float3(mesh_vertices[mesh_indices[i + 2]]);

        p1 = float4_to_float3(mesh_vertices[edge.first]);
        p2 = float4_to_float3(mesh_vertices[edge.second]);

        

        if(((p1.x == m0.x && p1.y == m0.y && p1.z == m0.z) && (p2.x == m1.x && p2.y == m1.y && p2.z == m1.z)) ||
           ((p2.x == m0.x && p2.y == m0.y && p2.z == m0.z) && (p1.x == m1.x && p1.y == m1.y && p1.z == m1.z)) ||
           ((p1.x == m0.x && p1.y == m0.y && p1.z == m0.z) && (p2.x == m2.x && p2.y == m2.y && p2.z == m2.z)) ||
           ((p2.x == m0.x && p2.y == m0.y && p2.z == m0.z) && (p1.x == m2.x && p1.y == m2.y && p1.z == m2.z)) ||
           ((p1.x == m1.x && p1.y == m1.y && p1.z == m1.z) && (p2.x == m2.x && p2.y == m2.y && p2.z == m2.z)) ||
           ((p2.x == m1.x && p2.y == m1.y && p2.z == m1.z) && (p1.x == m2.x && p1.y == m2.y && p1.z == m2.z))){
          count++;

          el1 = same_elem(mesh_vertices, edge.first);
          el2 = same_elem(mesh_vertices, edge.second);

          edge.first = std::min(el1, el2);
          edge.second = std::max(el1, el2);

          if(edge_in_planes.count(edge)){
            planes = edge_in_planes[edge];
          }
          
          min_num = std::min(mesh_indices[i], std::min(mesh_indices[i + 1], mesh_indices[i + 2]));
          max_num = std::max(mesh_indices[i], std::max(mesh_indices[i + 1], mesh_indices[i + 2]));

          planes.push_back(same_elem(mesh_vertices, min_num));
          for(unsigned int k = 0; k < 3; k++){
            if(mesh_indices[i + k] != min_num && mesh_indices[i + k] != max_num){
              planes.push_back(same_elem(mesh_vertices, mesh_indices[i + k]));
              break;
            }
          }
          planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 2]));


          edge_in_planes[edge] = planes;
          planes.clear();
        }

        // else if((inside_triangle(p1, m0, m1, m2) && inside_triangle(p2, m0, m1, m2)) ||
        /*
        else if(
          (inside_line(p1, m0, m1) && inside_line(p2, m0, m1)) ||
          (inside_line(p1, m0, m2) && inside_line(p2, m0, m2)) ||
          (inside_line(p1, m1, m2) && inside_line(p2, m1, m2)) ||
           
          (inside_line(m0, p1, p2) && inside_line(m1, p1, p2)) ||
          (inside_line(m0, p1, p2) && inside_line(m2, p1, p2)) ||
          (inside_line(m1, p1, p2) && inside_line(m2, p1, p2))
        ){
          count++;

          el1 = same_elem(mesh_vertices, edge.first);
          el2 = same_elem(mesh_vertices, edge.second);

          edge.first = std::min(el1, el2);
          edge.first = std::max(el1, el2);

          if(edge_in_planes.count(edge)){
            planes = edge_in_planes[edge];
          }
            
          min_num = std::min(mesh_indices[i], std::min(mesh_indices[i + 1], mesh_indices[i + 2]));
          max_num = std::max(mesh_indices[i], std::max(mesh_indices[i + 1], mesh_indices[i + 2]));

          planes.push_back(same_elem(mesh_vertices, min_num));
          for(unsigned int k = 0; k < 3; k++){
            if(mesh_indices[i + k] != min_num && mesh_indices[i + k] != max_num){
              planes.push_back(same_elem(mesh_vertices, mesh_indices[i + k]));
              break;
            }
          }
          planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 2]));


          edge_in_planes[edge] = planes;
          planes.clear();
        }
        */
        else if(ind != -1){
          // It doesn't work properly, so "continue"
          continue;
          p3 = float4_to_float3(mesh_vertices[mesh_indices[ind3]]);
          bool ins_t1, ins_t2, ins_t3;
          if((ins_t1 = inside_triangle(p1, m0, m1, m2)) &&
             (ins_t2 = inside_triangle(p2, m0, m1, m2)) &&
             (ins_t3 = inside_triangle(p3, m0, m1, m2))){
            count++;

            el1 = same_elem(mesh_vertices, edge.first);
            el2 = same_elem(mesh_vertices, edge.second);

            edge.first = std::min(el1, el2);
            edge.first = std::max(el1, el2);

            if(edge_in_planes.count(edge)){
              planes = edge_in_planes[edge];
            }
            
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 1]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 2]));

            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind1]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind2]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind3]));

            edge_in_planes[edge] = planes;
            planes.clear();
            


            el1 = same_elem(mesh_vertices, edge.first);
            el2 = same_elem(mesh_vertices, mesh_indices[i + 2]);

            edge.first = std::min(el1, el2);
            edge.first = std::max(el1, el2);

            if(edge_in_planes.count(edge)){
              planes = edge_in_planes[edge];
            }
            
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 1]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 2]));

            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind1]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind2]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind3]));

            edge_in_planes[edge] = planes;
            planes.clear();



            el1 = same_elem(mesh_vertices, edge.second);
            el2 = same_elem(mesh_vertices, mesh_indices[i + 2]);

            edge.first = std::min(el1, el2);
            edge.first = std::max(el1, el2);

            if(edge_in_planes.count(edge)){
              planes = edge_in_planes[edge];
            }
            
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 1]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[i + 2]));

            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind1]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind2]));
            planes.push_back(same_elem(mesh_vertices, mesh_indices[ind3]));

            edge_in_planes[edge] = planes;
            planes.clear();

          }

          else if((ins_t1 == 1 && ins_t2 == 1 && ins_t3 == 0) ||
                  (ins_t1 == 1 && ins_t3 == 1 && ins_t2 == 0) ||
                  (ins_t2 == 1 && ins_t3 == 1 && ins_t1 == 0) ||
                  
                  (ins_t1 == 0 && ins_t2 == 0 && ins_t3 == 1) ||
                  (ins_t1 == 0 && ins_t3 == 0 && ins_t2 == 1) ||
                  (ins_t2 == 0 && ins_t3 == 0 && ins_t1 == 1)){
            return -i;
          }
        }


      }
    }
    return count;
  }

  float same_elem(std::vector<LiteMath::float4> mesh_vertices, unsigned int ind){
    for(unsigned int j = 0; j < ind; j++){
      
      if(mesh_vertices[j].x == mesh_vertices[ind].x &&
         mesh_vertices[j].y == mesh_vertices[ind].y &&
         mesh_vertices[j].z == mesh_vertices[ind].z){
        return j;
      }
    }
    return ind;
  }

  float for_inside_line(float p_x, float m1_x, float denom){
    return (p_x - m1_x) / denom;
  }

  bool inside_line(LiteMath::float3 p, LiteMath::float3 a, LiteMath::float3 b){
    bool k = 0;

    // The equation of a straight line
    float denom1, denom2, denom3;
    float equation_x, equation_y, equation_z;

    denom1 = b.x - a.x;
    denom2 = b.y - a.y;
    denom3 = b.z - a.z;

    // equation_x = for_inside_line(p.x, a.x, denom1);
    // equation_y = for_inside_line(p.y, a.y, denom2);
    // equation_z = for_inside_line(p.z, a.z, denom3);

    if(denom1 != 0 && denom2 != 0 && denom3 != 0){
      equation_x = for_inside_line(p.x, a.x, denom1);
      equation_y = for_inside_line(p.y, a.y, denom2);
      equation_z = for_inside_line(p.z, a.z, denom3);
    }
    else if(denom1 == 0 && denom2 == 0 && denom3 == 0){
      return k;
    }
    else if(denom1 == 0){
      if(p.x != 0)
        return k;
      if(denom2 == 0){
        if(p.y != 0)
          return k;
        equation_z = for_inside_line(p.z, a.z, denom3);
        equation_x = equation_y = equation_z;
      }
      else if(denom3 == 0){
        if(p.z != 0)
          return k;
        equation_y = for_inside_line(p.y, a.y, denom2);
        equation_x = equation_z = equation_y;
      }
      else{
        equation_y = for_inside_line(p.y, a.y, denom2);
        equation_z = for_inside_line(p.z, a.z, denom3);
        equation_x = equation_z;
      }
    }
    else if(denom2 == 0){
      if(p.y != 0)
        return k;
      if(denom3 == 0){
        if(p.z != 0)
          return k;
        equation_x = for_inside_line(p.x, a.x, denom1);
        equation_y = equation_z = equation_x;
      }
      else{
        equation_x = for_inside_line(p.x, a.x, denom1);
        equation_z = for_inside_line(p.z, a.z, denom3);
        equation_y = equation_x;
      }
    }
    else{
      if(p.z != 0){
        return k;
      }
      equation_x = for_inside_line(p.x, a.x, denom1);
      equation_y = for_inside_line(p.y, a.y, denom2);
      equation_z = equation_y;
    }
    

    if(equation_x == equation_y && equation_y == equation_z){
      if(((p.x <= b.x && p.x >= a.x) && (p.y <= b.y && p.y >= a.y) && (p.z <= b.z && p.z >= a.z)) ||
         ((p.x >= b.x && p.x <= a.x) && (p.y >= b.y && p.y <= a.y) && (p.z >= b.z && p.z <= a.z))){
        k = 1;
      }
    }

    // return 0;
    return k;
  }

  float triangle_square(float a,float b, float c){
    float p=(a+b+c)/2;
    return sqrt(p*(p-a)*(p-b)*(p-c));
  } 

  bool inside_triangle(LiteMath::float3 p, LiteMath::float3 a,
                       LiteMath::float3 b, LiteMath::float3 c){
    float P_x = p.x;
    float P_y = p.y;
    float P_z = p.z;

    float A_x = a.x;
    float A_y = a.y;
    float A_z = a.z;

    float B_x = b.x; 
    float B_y = b.y;
    float B_z = b.z;

    float C_x = c.x;
    float C_y = c.y;
    float C_z = c.z;
    

    int inside = 0;
    float AB = sqrt( (A_x-B_x)*(A_x-B_x) + (A_y-B_y)*(A_y-B_y) + (A_z-B_z)*(A_z-B_z) );
    float BC = sqrt( (B_x-C_x)*(B_x-C_x) + (B_y-C_y)*(B_y-C_y) + (B_z-C_z)*(B_z-C_z) );
    float CA = sqrt( (A_x-C_x)*(A_x-C_x) + (A_y-C_y)*(A_y-C_y) + (A_z-C_z)*(A_z-C_z) );

    float AP = sqrt( (P_x-A_x)*(P_x-A_x) + (P_y-A_y)*(P_y-A_y) + (P_z-A_z)*(P_z-A_z) );
    float BP = sqrt( (P_x-B_x)*(P_x-B_x) + (P_y-B_y)*(P_y-B_y) + (P_z-B_z)*(P_z-B_z) );
    float CP = sqrt( (P_x-C_x)*(P_x-C_x) + (P_y-C_y)*(P_y-C_y) + (P_z-C_z)*(P_z-C_z) );
    float diff = (triangle_square(AP,BP,AB)+triangle_square(AP,CP,CA)+triangle_square(BP,CP,BC))-triangle_square(AB,BC,CA);
    if (fabs(diff) < EPS){
      inside=1;
    }
    // return 0;
    return inside;
}


  bool plane_equation(LiteMath::float3 m0, LiteMath::float3 m1, LiteMath::float3 m2,
                      LiteMath::float3 p){
    float a11 = p.x - m0.x;
    float a21 = p.y - m0.y;
    float a31 = p.z - m0.z;

    float a12 = m1.x - m0.x;
    float a22 = m1.y - m0.y;
    float a32 = m1.z - m0.z;

    float a13 = m2.x - m0.x;
    float a23 = m2.y - m0.y;
    float a33 = m2.z - m0.z;

    if(a11 * a22 * a33 + a12 * a23 * a31 + a21 * a32 * a13 -
       a31 * a22 * a13 - a33 * a21 * a12 - a23 * a32 * a11 == 0)
      return 1;
    else
      return 0;
  }

  LiteMath::float3 float4_to_float3(LiteMath::float4 m){
    return LiteMath::float3(m.x, m.y, m.z);
  }
}