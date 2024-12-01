#include "ply_reader.h"
#include <vector>
#include <cstring>
#include <string>
#include <fstream>

namespace cmesh4
{
  struct Token
  {
    enum Type
    {
      UNKNOWN,
      VALUE,
      PLY,
      FORMAT,
      ASCII,
      BINARY,
      END_HEADER,
      COMMENT,
      ELEMENT,
      PROPERTY,
      LIST,
      NAME,

      CHAR,
      UCHAR,
      SHORT,
      USHORT,
      INT,
      UINT,
      INT8,
      UINT8,
      INT16,
      UINT16,
      INT32,
      UINT32,
      
      FLOAT,
      DOUBLE,
      FLOAT32,
      FLOAT64
    };


    Type type = UNKNOWN;
    double value = -1;
    
  };

  inline static bool is_delim(char c)
  {
    //Windows-style endline (\r\n) will also be interpreted correctly
    return (c == ' ' || c == '\t' || c == '\r' || c == '\n');
  }

  class Tokenizer
  {
  public:
    static constexpr int MAX_TOKEN_SIZE = 1024;
    Token NextToken();
    std::vector<float> ReadValues(Token::Type type, int count);
    std::vector<int> ReadIndices(Token::Type type, int count);
    bool OpenFile(const std::string& path)
    {
      fs.open(path, std::ios::binary);
      return !fs.fail();
    }
    void CloseFile()
    {
      fs.close();
    }
    void SetStateASCII()
    {
      read_binary = false;
    }
    void SetStateBinary()
    {
      read_binary = true;
    }
    const char *GetLastToken() { return buffer; }
    int cur_line = 0;
    int cur_pos  = 0;
  private:
    std::ifstream fs;
    bool read_binary = false;
    char buffer[MAX_TOKEN_SIZE+1];
    bool eof = false;
    bool big_endian_binary = false;
  };

  Token Tokenizer::NextToken()
  {
    if (eof)
    {
      printf("[LoadMeshFromPly::ERROR] Failed to read token at line %d, pos %d. End of file\n", cur_line, cur_pos);
      return Token();
    }
    //read token to buffer
    int p = 0;
    do
    {
      buffer[p] = fs.get();
      cur_pos++;
      if (buffer[p] == '\n')
      {
        cur_line++;
        cur_pos = 0;
      }
      if (is_delim(buffer[p]))
      {
        if (p > 0)
          break;
      }
      else
        p++;
    } while (p < MAX_TOKEN_SIZE && !fs.fail());
    
    buffer[p] = '\0';

    if (fs.fail())
    {
      eof = true;
    }

    Token token;
    char *eptr = nullptr;
    token.value = strtod(buffer, &eptr);
    errno = 0;
    if (eptr != buffer && errno == 0)
    {
      token.type = Token::VALUE;
    }
    else if (strncmp(buffer, "ply", p) == 0)
    {
      token.type = Token::PLY;
    }
    else if (strncmp(buffer, "format", p) == 0)
    {
      token.type = Token::FORMAT;
    }
    else if (strncmp(buffer, "ascii", p) == 0)
    {
      token.type = Token::ASCII;
    }
    else if (strncmp(buffer, "binary_little_endian", p) == 0)
    {
      big_endian_binary = false;
      token.type = Token::BINARY;
    }
    else if (strncmp(buffer, "binary_big_endian", p) == 0)
    {
      big_endian_binary = true;
      token.type = Token::BINARY;
    }
    else if (strncmp(buffer, "end_header", p) == 0)
    {
      token.type = Token::END_HEADER;
    }
    else if (strncmp(buffer, "comment", p) == 0)
    {
      token.type = Token::COMMENT;
    }
    else if (strncmp(buffer, "element", p) == 0)
    {
      token.type = Token::ELEMENT;
    }
    else if (strncmp(buffer, "property", p) == 0)
    {
      token.type = Token::PROPERTY;
    }
    else if (strncmp(buffer, "list", p) == 0)
    {
      token.type = Token::LIST;
    }
    else if (strncmp(buffer, "char", p) == 0)
    {
      token.type = Token::CHAR;
    }
    else if (strncmp(buffer, "uchar", p) == 0)
    {
      token.type = Token::UCHAR;
    }
    else if (strncmp(buffer, "short", p) == 0)
    {
      token.type = Token::SHORT;
    }
    else if (strncmp(buffer, "ushort", p) == 0)
    {
      token.type = Token::USHORT;
    }
    else if (strncmp(buffer, "int", p) == 0)
    {
      token.type = Token::INT;
    }
    else if (strncmp(buffer, "uint", p) == 0)
    {
      token.type = Token::UINT;
    }
    else if (strncmp(buffer, "float", p) == 0)
    {
      token.type = Token::FLOAT;
    }
    else if (strncmp(buffer, "double", p) == 0)
    {
      token.type = Token::DOUBLE;
    }
    else if (strncmp(buffer, "int8", p) == 0)
    {
      token.type = Token::INT8;
    }
    else if (strncmp(buffer, "uint8", p) == 0)
    {
      token.type = Token::UINT8;
    }
    else if (strncmp(buffer, "int16", p) == 0)
    {
      token.type = Token::INT16;
    }
    else if (strncmp(buffer, "uint16", p) == 0)
    {
      token.type = Token::UINT16;
    }
    else if (strncmp(buffer, "int32", p) == 0)
    {
      token.type = Token::INT32;
    }
    else if (strncmp(buffer, "uint32", p) == 0)
    {
      token.type = Token::UINT32;
    }
    else if (strncmp(buffer, "float32", p) == 0)
    {
      token.type = Token::FLOAT32;
    }
    else if (strncmp(buffer, "float64", p) == 0)
    {
      token.type = Token::FLOAT64;
    }
    else
    {
      token.type = Token::NAME;
    }
    return token;
  }

  void big_to_little_endian(char *data, int elems, int elem_size)
  {
    char buf[256];
    for (int i = 0; i < elems; ++i)
    {
      memcpy(buf, data + i * elem_size, elem_size);
      for (int j = 0; j < elem_size; ++j)
      {
        data[i * elem_size + elem_size - 1 - j] = buf[j];
      }
    }
  }

  std::vector<float> Tokenizer::ReadValues(Token::Type type, int count)
  {
    std::vector<float> res(count, 0.0f);

    if (read_binary)
    {
      if (type == Token::DOUBLE || type == Token::FLOAT64)
      {
        std::vector<double> res_double(count);
        fs.read((char*)&res_double[0], count * sizeof(double));
        if (big_endian_binary)
          big_to_little_endian((char*)&res_double[0], count, sizeof(double));
        for (int i = 0; i < count; ++i)
          res[i] = (float)res_double[i];
      }
      else if (type == Token::FLOAT || type == Token::FLOAT32)
      {
        fs.read((char*)&res[0], count * sizeof(float));
        if (big_endian_binary)
          big_to_little_endian((char*)&res[0], count, sizeof(float));
      }
      else
      {
        printf("[ply_reader::ERROR] Unknown type (%d) for ReadValues<float>\n", type);
      }
    }
    else
    {
      if (type == Token::DOUBLE || type == Token::FLOAT64 || type == Token::FLOAT || type == Token::FLOAT32)
      {
        for (int i = 0; i < count; ++i)
        {
          Token token = NextToken();
          if (token.type == Token::VALUE)
          {
            res[i] = token.value;
          }
          else
          {
            printf("[ply_reader::ERROR] Got token (%d) instead of Token::VALUE for ReadValues<float>\n", token.type);
            break;
          }
        }
      }
      else
      {
        printf("[ply_reader::ERROR] Unknown type (%d) for ReadValues<float>\n", type);
      }
    }

    return res;
  }

  std::vector<int> Tokenizer::ReadIndices(Token::Type type, int count)
  {
    std::vector<int> res(count, -1);

    if (read_binary)
    {
      if (type == Token::CHAR || type == Token::INT8)
      {
        std::vector<char> res_char(count);
        fs.read((char*)&res_char[0], count * sizeof(char));
        for (int i = 0; i < count; ++i)
          res[i] = (int)res_char[i];
      }
      else if (type == Token::UCHAR || type == Token::UINT8)
      {
        std::vector<unsigned char> res_uchar(count);
        fs.read((char*)&res_uchar[0], count * sizeof(unsigned char));
        for (int i = 0; i < count; ++i)
          res[i] = (int)res_uchar[i];
      }
      else if (type == Token::SHORT || type == Token::INT16)
      {
        std::vector<short> res_short(count);
        fs.read((char*)&res_short[0], count * sizeof(short));
        if (big_endian_binary)
          big_to_little_endian((char*)&res_short[0], count, sizeof(short));
        for (int i = 0; i < count; ++i)
          res[i] = (int)res_short[i];
      }
      else if (type == Token::USHORT || type == Token::UINT16)
      {
        std::vector<unsigned short> res_ushort(count);
        fs.read((char*)&res_ushort[0], count * sizeof(unsigned short));
        if (big_endian_binary)
          big_to_little_endian((char*)&res_ushort[0], count, sizeof(unsigned short));
        for (int i = 0; i < count; ++i)
          res[i] = (int)res_ushort[i];
      }
      else if (type == Token::INT || type == Token::INT32)
      {
        fs.read((char*)&res[0], count * sizeof(int));
        if (big_endian_binary)
          big_to_little_endian((char*)&res[0], count, sizeof(int));
      }
      else if (type == Token::UINT || type == Token::UINT32)
      {
        std::vector<unsigned int> res_uint(count);
        fs.read((char*)&res_uint[0], count * sizeof(unsigned int));
        if (big_endian_binary)
          big_to_little_endian((char*)&res_uint[0], count, sizeof(unsigned int));
        for (int i = 0; i < count; ++i)
          res[i] = (int)res_uint[i];
      }
      else
      {
        printf("[ply_reader::ERROR] Unknown type (%d) for ReadValues<int>\n", type);
      }
    }
    else
    {
      if (type == Token::CHAR  || type == Token::INT8  || type == Token::UCHAR  || type == Token::UINT8  || 
          type == Token::SHORT || type == Token::INT16 || type == Token::USHORT || type == Token::UINT16 ||
          type == Token::INT   || type == Token::UINT  || type == Token::INT32  || type == Token::UINT32)
      {
        for (int i = 0; i < count; ++i)
        {
          Token token = NextToken();
          if (token.type == Token::VALUE)
          {
            res[i] = token.value;
          }
          else
          {
            printf("[ply_reader::ERROR] Got token (%d) instead of Token::VALUE for ReadValues<int>\n", token.type);
            break;
          }
        }
      }
      else
      {
        printf("[ply_reader::ERROR] Unknown type (%d) for ReadValues<int>\n", type);
      }
    }

    return res;
  }

  struct PlyProperty
  {
    std::string name;
    Token::Type val_type;
    Token::Type index_type;
    bool is_list = false;
  };
  struct PlyElement
  {
    std::string name;
    int count;
    std::vector<PlyProperty> properties;
  };
  struct PlyHeader
  {
    std::vector<PlyElement> elements;
  };

  SimpleMesh LoadMeshFromPly(const std::string& path, bool verbose)
  {
    PlyHeader header;

    Tokenizer tok;
    if (!tok.OpenFile(path))
    {
      printf("[ply_reader::ERROR] Failed to open file %s\n", path.c_str());
      return SimpleMesh();
    }
    
    // example of header 
    //
    // ply
    // format ascii 1.0
    // comment made by anonymous
    // comment this file is a cube
    // element vertex 8
    // property float32 x
    // property float32 y
    // property float32 z
    // element face 6
    // property list uint8 int32 vertex_index
    // end_header

    Token token = tok.NextToken();
    // ply
    if (token.type != Token::PLY)
    {
      printf("[ply_reader::ERROR] %d:%d Expected \"ply\" but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
      return SimpleMesh();
    }

    // format ascii 1.0
    token = tok.NextToken();
    if (token.type != Token::FORMAT)
    {
      printf("[ply_reader::ERROR] %d:%d Expected \"format\" but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
      return SimpleMesh();
    }

    token = tok.NextToken();
    if (token.type == Token::ASCII)
    {
      tok.SetStateASCII();
    }
    else if (token.type == Token::BINARY)
    {
      tok.SetStateBinary();
    }
    else
    {
      printf("[ply_reader::ERROR] %d:%d Expected \"ascii\" or \"binary\" but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
      return SimpleMesh();
    }

    token = tok.NextToken();
    if (token.type != Token::VALUE)
    {
      printf("[ply_reader::ERROR] %d:%d Expected version number but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
      return SimpleMesh();
    }

    token = tok.NextToken();
    while (token.type == Token::ELEMENT)
    {
      //element vertex 8
      header.elements.push_back(PlyElement());
      token = tok.NextToken();
      if (token.type != Token::NAME)
      {
        printf("[ply_reader::ERROR] %d:%d Expected element name but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
        return SimpleMesh();
      }
      header.elements.back().name = tok.GetLastToken();
      if (verbose)
        printf("[ply_reader::INFO] Element %s\n", header.elements.back().name.c_str());
      
      token = tok.NextToken();
      if (token.type != Token::VALUE)
      {
        printf("[ply_reader::ERROR] %d:%d Expected element count but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
        return SimpleMesh();
      }
      header.elements.back().count = token.value;
      if (verbose)
        printf("[ply_reader::INFO] Element %s count %d\n", header.elements.back().name.c_str(), header.elements.back().count);
      
      token = tok.NextToken();
      while (token.type == Token::PROPERTY)
      {
        //property float32 x
        //property list uint8 int32 vertex_index
        header.elements.back().properties.push_back(PlyProperty());
        token = tok.NextToken();

        if (token.type == Token::LIST)
        {
          token = tok.NextToken();
          if (token.type < Token::Type::CHAR || token.type >= Token::Type::FLOAT)
          {
            printf("[ply_reader::ERROR] %d:%d Expected property index type but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
            return SimpleMesh();
          }
          header.elements.back().properties.back().is_list = true;
          header.elements.back().properties.back().index_type = token.type;
          if (verbose)
          {
            printf("[ply_reader::INFO] Property is list\n");
            printf("[ply_reader::INFO] Property index type %s\n", tok.GetLastToken());
          }
          token = tok.NextToken();
        }
        if (token.type < Token::Type::CHAR)
        {
          printf("[ply_reader::ERROR] %d:%d Expected property type but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
          return SimpleMesh();
        }
        header.elements.back().properties.back().val_type = token.type;
        if (verbose)
          printf("[ply_reader::INFO] Property type %s\n", tok.GetLastToken());
        
        token = tok.NextToken();
        if (token.type != Token::NAME)
        {
          printf("[ply_reader::ERROR] %d:%d Expected property name but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
          return SimpleMesh();
        }
        header.elements.back().properties.back().name = tok.GetLastToken();
        if (verbose)
          printf("[ply_reader::INFO] Property name %s\n", header.elements.back().properties.back().name.c_str());
        
        token = tok.NextToken();
      }
    }

    if (token.type != Token::END_HEADER)
    {
      printf("[ply_reader::ERROR] %d:%d Expected \"end_header\" but got %d:%s\n", tok.cur_line, tok.cur_pos, token.type, tok.GetLastToken());
      return SimpleMesh();
    }

    if (verbose)
      printf("[ply_reader::INFO] Ply header loaded\n");

    int vertices_element_index = -1;
    int faces_element_index = -1;
    for (int i = 0; i < header.elements.size(); ++i)
    {
      PlyElement &el = header.elements[i];
      // property float32 x
      // property float32 y
      // property float32 z
      bool vertex_format = el.properties.size() == 3;
      vertex_format &= el.properties[0].is_list == false &&
                       el.properties[1].is_list == false && 
                       el.properties[2].is_list == false;
      vertex_format &= el.properties[0].val_type >= Token::Type::FLOAT;
      vertex_format &= el.properties[0].val_type == el.properties[1].val_type && 
                       el.properties[0].val_type == el.properties[2].val_type;

      // property list uint8 int32 vertex_index
      bool face_format = el.properties.size() == 1 && el.properties[0].is_list;
      face_format &= el.properties[0].index_type >= Token::Type::CHAR && el.properties[0].index_type < Token::Type::FLOAT;
      face_format &= el.properties[0].val_type >= Token::Type::CHAR && el.properties[0].val_type < Token::Type::FLOAT;

      if (vertex_format)
      {
        if (vertices_element_index == -1)
        {
          vertices_element_index = i;

          if (verbose)
            printf("[ply_reader::INFO] Found vertices element %d\n", i);
        }
        else
        {
          printf("[ply_reader::WARNING] More than one vertices element found (%d and %d)\n", vertices_element_index, i);
        }
      }

      if (face_format)
      {
        if (faces_element_index == -1)
        {
          faces_element_index = i;

          if (verbose)
            printf("[ply_reader::INFO] Found faces element %d\n", i);
        }
        else
        {
          printf("[ply_reader::WARNING] More than one faces element found (%d and %d)\n", faces_element_index, i);
        }
      }
    }

    if (vertices_element_index == -1)
    {
      printf("[ply_reader::ERROR] No vertices element found\n");
      return SimpleMesh();
    }

    if (faces_element_index == -1)
    {
      printf("[ply_reader::ERROR] No faces element found\n");
      return SimpleMesh();
    }

    std::vector<unsigned> real_indices;
    std::vector<float> vertices = tok.ReadValues(header.elements[vertices_element_index].properties[0].val_type, 
                                                 3*header.elements[vertices_element_index].count);

    Token::Type index_count_type = header.elements[faces_element_index].properties[0].index_type;
    Token::Type index_type = header.elements[faces_element_index].properties[0].val_type;
    for (int i = 0; i < header.elements[faces_element_index].count; ++i)
    {
      auto index_count = tok.ReadIndices(index_count_type, 1)[0];

      if (verbose && (i == 0 || i%1000000 == 0 || i == header.elements[faces_element_index].count - 1))
        printf("[ply_reader::INFO] (%d/%d) reading %d indices\n", i, header.elements[faces_element_index].count, index_count);

      if (index_count < 3)
      {
        printf("[ply_reader::ERROR] %d:%d Expected at least 3 indices but got %d\n", tok.cur_line, tok.cur_pos, index_count);
        return SimpleMesh();
      }

      auto poly_indices = tok.ReadIndices(index_type, index_count);
      for (int j = 2; j < index_count; ++j)
      {
        real_indices.push_back(poly_indices[0]);
        real_indices.push_back(poly_indices[j]);
        real_indices.push_back(poly_indices[j-1]);
      }
    }

    int num_vertices = header.elements[vertices_element_index].count;
    int num_indices = real_indices.size();

    if (verbose)
    {
      printf("[ply_reader::INFO] %d vertices and %d indices loaded\n", num_vertices, num_indices);
    }

    SimpleMesh mesh;
    mesh.vPos4f.resize(num_vertices);
    mesh.vNorm4f.resize(num_vertices, float4(0,0,0,0));
    mesh.vTang4f.resize(num_vertices, float4(0,0,0,0));
    mesh.vTexCoord2f.resize(num_vertices, float2(0,0));

    mesh.indices.resize(num_indices);
    mesh.matIndices.resize(num_indices / 3, 0u);

    for (int i = 0; i < num_vertices; ++i)
    {
      mesh.vPos4f[i] = float4(vertices[i*3+0], vertices[i*3+1], vertices[i*3+2], 1);
    }

    for (int i = 0; i < num_indices; ++i)
    {
      mesh.indices[i] = real_indices[i];
    }

    return mesh;
  }
}