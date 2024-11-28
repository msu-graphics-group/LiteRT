#include "blk.h"
#include <fstream>
#include <iostream>
#include <sstream>

using LiteMath::float2;
using LiteMath::float3;
using LiteMath::float4;
using LiteMath::uint2;
using LiteMath::uint3;
using LiteMath::uint4;
using LiteMath::int2;
using LiteMath::int3;
using LiteMath::int4;
using LiteMath::float4x4;
using LiteMath::float3x3;
using LiteMath::cross;
using LiteMath::dot;
using LiteMath::length;
using LiteMath::normalize;
using LiteMath::to_float4;
using LiteMath::to_float3;

std::string base_blk_path = "./";

int cur_line = 0;
bool in_comment_assume = false;
bool in_comment = false;
bool is_empty(const char c)
{   
    if (c == '\n')
        cur_line++;
    
    if (in_comment)
    {
        if (c == '\n')
        {
            in_comment = false;

            return true;
        }
        else
            return true;
    }
    else if (!in_comment_assume && c == '/')
    {
        in_comment_assume = true;
        return true;
    }
    else if (in_comment_assume)
    {
        if (c == '/')
        {
            in_comment_assume = false;
            in_comment = true;
            return true;
        }
        else
        {
            fprintf(stderr, "line %d hanging / found", cur_line);
            in_comment_assume = false;
        }
    }

    return (c == ' ' || c == '\n' || c == '\t');
}
bool is_div(const char c)
{
    return (c == ',' || c == ';' || c == ':' || c == '=' || c == '{' || c == '}' || c == '\'' || c == '\"');
}
std::string next_token(const char *data, int &pos)
{
    std::string res;
    if (!data || data[pos] == 0)
        res = "";
    else
    {
        while (is_empty(data[pos]))
            pos++;
        if (data[pos] == 0)
            res = "";
        else
        {
            if (is_div(data[pos]))
            {
                pos++;
                res = std::string(1, data[pos - 1]);
            }
            else
            {
                const char *start = data + pos;
                int sz = 0;
                while (!is_div(data[pos]) && !is_empty(data[pos]) && data[pos] != 0)
                {
                    pos++;
                    sz++;
                }
                res = std::string(start, sz);
            }
        }
    }
    return res;
}

bool load_block(const char *data, int &cur_pos, Block &b);
bool read_array(const char *data, int &cur_pos, Block::DataArray &a);
bool read_value(const char *data, int &cur_pos, Block::Value &v)
{
    std::string token = next_token(data, cur_pos);
    //:<type> = <description> or { <block> }
    if (token == "{")
    {
        v.bl = new Block();
        v.type = Block::ValueType::BLOCK;
        return load_block(data, cur_pos, *(v.bl));
    }
    else if (token == ":")
    { //simple value or array
        std::string type = next_token(data, cur_pos);
        if (type == "tag")
        {
            v.type = Block::ValueType::EMPTY;
            return true;
        }
        std::string eq = next_token(data, cur_pos);
        if (eq != "=")
        {
            fprintf(stderr, "line %d expected = after value type", cur_line);
            v.type = Block::ValueType::EMPTY;
            return false;
        }
        if (type == "b")
        {
            std::string val = next_token(data, cur_pos);
            v.type = Block::ValueType::BOOL;
            v.b = val == "true" || val == "True" || val == "TRUE";
        }
        else if (type == "i")
        {
            std::string val = next_token(data, cur_pos);
            v.type = Block::ValueType::INT;
            v.i = std::stol(val);
        }
        else if (type == "u" || type == "u64")
        {
            std::string val = next_token(data, cur_pos);
            v.type = Block::ValueType::UINT64;
            v.u = std::stoul(val);
        }
        else if (type == "r")
        {
            std::string val = next_token(data, cur_pos);
            v.type = Block::ValueType::DOUBLE;
            v.d = std::stod(val);
        }
        else if (type == "p2")
        {
            v.type = Block::ValueType::VEC2;
            std::string val;
            bool ok = true;
            v.v2 = float2(0, 0);

            val = next_token(data, cur_pos);
            v.v2.x = std::stod(val);

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");

            if (ok)
            {
                val = next_token(data, cur_pos);
                v.v2.y = std::stod(val);
            }
            if (!ok)
            {
                fprintf(stderr, "line %d wrong description of vector", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }
        }
        else if (type == "p3")
        {
            v.type = Block::ValueType::VEC3;
            std::string val;
            bool ok = true;
            v.v3 = float3(0, 0, 0);

            val = next_token(data, cur_pos);
            v.v3.x = std::stod(val);

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.v3.y = std::stod(val);
            }

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.v3.z = std::stod(val);
            }
            if (!ok)
            {
                fprintf(stderr, "line %d wrong description of vector", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }
        }
        else if (type == "p4")
        {
            v.type = Block::ValueType::VEC4;
            std::string val;
            bool ok = true;
            v.v4 = float4(0, 0, 0, 0);

            val = next_token(data, cur_pos);
            v.v4.x = std::stod(val);

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.v4.y = std::stod(val);
            }

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.v4.z = std::stod(val);
            }

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.v4.w = std::stod(val);
            }
            if (!ok)
            {
                fprintf(stderr, "line %d wrong description of vector", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }
        }
        else if (type == "i2")
        {
            v.type = Block::ValueType::IVEC2;
            std::string val;
            bool ok = true;
            v.iv2 = int2(0, 0);

            val = next_token(data, cur_pos);
            v.iv2.x = std::stoi(val);

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");

            if (ok)
            {
                val = next_token(data, cur_pos);
                v.iv2.y = std::stoi(val);
            }
            if (!ok)
            {
                fprintf(stderr, "line %d wrong description of integer vector", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }
        }
        else if (type == "i3")
        {
            v.type = Block::ValueType::IVEC3;
            std::string val;
            bool ok = true;
            v.iv3 = int3(0, 0, 0);

            val = next_token(data, cur_pos);
            v.iv3.x = std::stoi(val);

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.iv3.y = std::stoi(val);
            }

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.iv3.z = std::stoi(val);
            }
            if (!ok)
            {
                fprintf(stderr, "line %d wrong description of integer vector", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }
        }
        else if (type == "i4")
        {
            v.type = Block::ValueType::IVEC4;
            std::string val;
            bool ok = true;
            v.iv4 = int4(0, 0, 0, 0);

            val = next_token(data, cur_pos);
            v.iv4.x = std::stoi(val);

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.iv4.y = std::stoi(val);
            }

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.iv4.z = std::stoi(val);
            }

            val = next_token(data, cur_pos);
            ok = ok && (val == ",");
            if (ok)
            {
                val = next_token(data, cur_pos);
                v.iv4.w = std::stoi(val);
            }
            if (!ok)
            {
                fprintf(stderr, "line %d wrong description of integer vector", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }
        }
        else if (type == "m4")
        {
            v.type = Block::ValueType::MAT4;
            v.m4 = float4x4();
            std::string val;
            float mat[16];
            bool ok = true;
            int cnt = 0;


            while (cnt < 16 && ok)
            {
                val = next_token(data, cur_pos);

                mat[cnt] = std::stof(val);
                if (cnt < 15)
                {
                    val = next_token(data, cur_pos);
                    ok = ok && (val == ",");
                }
                cnt++;
            }
            if (ok)
            {
                v.m4 = float4x4(mat[0], mat[4], mat[8], mat[12],
                                 mat[1], mat[5], mat[9], mat[13],
                                 mat[2], mat[6], mat[10], mat[14],
                                 mat[3], mat[7], mat[11], mat[15]);
            }
            else
            {
                fprintf(stderr, "line %d wrong description of matrix", cur_line);
                v.type = Block::ValueType::EMPTY;
                return false;
            }

        }
        else if (type == "s")
        {
            std::string par = next_token(data, cur_pos);
            if (par == "\"")
            {
                int len = 0;
                const char *start = data + cur_pos;
                while (data[cur_pos] != 0 && data[cur_pos] != '\"')
                {
                    if (data[cur_pos] == '\n')
                        cur_line++;
                    len++;
                    cur_pos++;
                }
                if (data[cur_pos] == 0)
                {
                    v.type = Block::ValueType::EMPTY;
                    fprintf(stderr, "line %d expected \" at the end of a string", cur_line);
                    return false;
                }
                else if (data[cur_pos] == '\"')
                {
                    cur_pos++;
                    v.type = Block::ValueType::STRING;
                    v.s = new std::string(start, len);
                }
            }
        }
        else if (type == "arr")
        {
            v.type = Block::ValueType::ARRAY;
            v.a = new Block::DataArray();
            return read_array(data, cur_pos, *(v.a));
        }

        return true;
    }
    else
    {
        fprintf(stderr, "line %d expected : or { after value/block name, but %s got", cur_line, token.c_str());
        v.type = Block::ValueType::EMPTY;
        return false;
    }
}
bool read_array(const char *data, int &cur_pos, Block::DataArray &a)
{
    std::string token = next_token(data, cur_pos);
    //{ <value>, <value>, ... <value>}
    // <value> := "string" or <double>
    //all values should have the same type
    if (token == "{")
    {
        bool ok = true;
        Block::ValueType array_type = Block::ValueType::DOUBLE;
        while (ok)
        {
            Block::Value val;
            std::string tok = next_token(data, cur_pos);
            if (tok == "}")
            {
                a.type = Block::ValueType::DOUBLE;
                return true;
            }
            if (tok.empty())
              fprintf(stderr, "line %d empty token in array", cur_line);
            else if (tok == "\"")
            {
              tok = next_token(data, cur_pos);

              val.type = Block::ValueType::STRING;
              val.s = new std::string(tok);

              tok = next_token(data, cur_pos);
              if (tok == "\"'")
                fprintf(stderr, "line %d unexpected end of string in array", cur_line);
            }
            else
            {
              val.type = Block::ValueType::DOUBLE;
              val.d = std::stod(tok);
            }
            if (a.values.empty())
              array_type = val.type;
            else if (array_type != val.type)
              fprintf(stderr, "line %d array has values of diffrent types", cur_line);
            a.values.push_back(val);

            tok = next_token(data, cur_pos);
            ok = ok && (tok == ",");
            if (tok == "}")
            {
                a.type = array_type;
                return true;
            }
        }
        fprintf(stderr, "line %d expected } at the end of array", cur_line);
        return false;
    }
    else
    {
        fprintf(stderr, "line %d expected { at the start of array", cur_line);
        return false;
    }
}
bool load_block(const char *data, int &cur_pos, Block &b)
{
    bool correct = true;
    while (correct)
    {
        std::string token = next_token(data, cur_pos);
        if (token == "}")
        {
            //block closed correctly
            return correct;
        }
        else if (token == "")
        {
            //end of file
            fprintf(stderr, "line %d block loader reached end of file, } expected", cur_line);
            return false;
        }
        else
        {
            //next value
            b.names.push_back(token);
            b.values.emplace_back();
            correct = correct && read_value(data, cur_pos, b.values.back());
        }
    }
    return true;
}

bool load_block_from_string(std::string &str, Block &b)
{
    if (str.empty())
        return false;
    cur_line = 0;
    int cur_pos = 0;
    const char *data = str.c_str();
    std::string token = next_token(data, cur_pos);
    if (token == "{")
    {
        return load_block(data, cur_pos, b);
    }
    else
    {
        return false;
    }  
    return true;  
}

bool load_block_from_file(std::string path, Block &b)
{
    std::fstream f(base_blk_path + path);
    std::stringstream iss;
    if (f.fail())
    {
        fprintf(stderr, "unable to load file %s", path.c_str());
        return false;
    }
    iss << f.rdbuf();
    std::string entireFile = iss.str();
    load_block_from_string(entireFile, b);
    return true;  
}

int Block::size() const
{
    return names.size();
}

int Block::get_id(const std::string &name) const
{
    return get_next_id(name, 0);
}
int Block::get_next_id(const std::string &name, int pos) const
{
    for (int i = pos; i < names.size(); i++)
    {
        if (names[i] == name)
            return i;
    }
    return -1;
}
Block::ValueType Block::get_type(int id) const
{
    return (id >= 0 && id < size()) ? values[id].type : Block::ValueType::EMPTY;
}
Block::ValueType Block::get_type(const std::string &name) const
{
    return get_type(get_id(name));
}

int Block::get_bool(int id, bool base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::BOOL) ? values[id].b : base_val;
}
int Block::get_int(int id, int base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::INT) ? values[id].i : base_val;
}
uint64_t Block::get_uint64(int id, uint64_t base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::UINT64) ? values[id].u : base_val;
}
double Block::get_double(int id, double base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::DOUBLE) ? values[id].d : base_val;
}
float2 Block::get_vec2(int id, float2 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::VEC2) ? values[id].v2 : base_val;
}
float3 Block::get_vec3(int id, float3 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::VEC3) ? values[id].v3 : base_val;
}
float4 Block::get_vec4(int id, float4 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::VEC4) ? values[id].v4 : base_val;
}
int2 Block::get_ivec2(int id, int2 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::IVEC2) ? values[id].iv2 : base_val;
}
int3 Block::get_ivec3(int id, int3 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::IVEC3) ? values[id].iv3 : base_val;
}
int4 Block::get_ivec4(int id, int4 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::IVEC4) ? values[id].iv4 : base_val;
}
float4x4 Block::get_mat4(int id, float4x4 base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::MAT4) ? values[id].m4 : base_val;
}
std::string Block::get_string(int id, std::string base_val) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::STRING && values[id].s) ? *(values[id].s) : base_val;
}
Block *Block::get_block(int id) const
{
    return (id >= 0 && id < size() && values[id].type == Block::ValueType::BLOCK) ? values[id].bl : nullptr;
}
bool Block::get_arr(int id, std::vector<double> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == DOUBLE))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            _values.push_back(v.d);
        }
        return true;
    }
    return false;
}
bool Block::get_arr(int id, std::vector<float> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == DOUBLE))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            _values.push_back(v.d);
        }
        return true;
    }
    return false;
}
bool Block::get_arr(int id, std::vector<int> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == DOUBLE))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            _values.push_back(v.d);
        }
        return true;
    }
    return false;
}
bool Block::get_arr(int id, std::vector<unsigned> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == DOUBLE))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            _values.push_back(v.d);
        }
        return true;
    }
    return false;
}
bool Block::get_arr(int id, std::vector<short> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == DOUBLE))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            _values.push_back(v.d);
        }
        return true;
    }
    return false;
}
bool Block::get_arr(int id, std::vector<unsigned short> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == DOUBLE))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            _values.push_back(v.d);
        }
        return true;
    }
    return false;
}
bool Block::get_arr(int id, std::vector<std::string> &_values, bool replace) const
{
    if (id >= 0 && id < size() && values[id].type == Block::ValueType::ARRAY && values[id].a &&
        (values[id].a->type == STRING))
    {
        if (replace)
            _values.clear();
        for (Block::Value &v : values[id].a->values)
        {
            if (v.s)
                _values.push_back(*(v.s));
            else
                _values.push_back("");
        }
        return true;
    }
    return false;
}

int Block::get_bool(const std::string name, bool base_val) const
{
    return get_bool(get_id(name), base_val);
}
int Block::get_int(const std::string name, int base_val) const
{
    return get_int(get_id(name), base_val);
}
uint64_t Block::get_uint64(const std::string name, uint64_t base_val) const
{
    return get_uint64(get_id(name), base_val);
}
double Block::get_double(const std::string name, double base_val) const
{
    return get_double(get_id(name), base_val);
}
float2 Block::get_vec2(const std::string name, float2 base_val) const
{
    return get_vec2(get_id(name), base_val);
}
float3 Block::get_vec3(const std::string name, float3 base_val) const
{
    return get_vec3(get_id(name), base_val);
}
float4 Block::get_vec4(const std::string name, float4 base_val) const
{
    return get_vec4(get_id(name), base_val);
}
int2 Block::get_ivec2(const std::string name, int2 base_val) const
{
    return get_ivec2(get_id(name), base_val);
}
int3 Block::get_ivec3(const std::string name, int3 base_val) const
{
    return get_ivec3(get_id(name), base_val);
}
int4 Block::get_ivec4(const std::string name, int4 base_val) const
{
    return get_ivec4(get_id(name), base_val);
}
float4x4 Block::get_mat4(const std::string name, float4x4 base_val) const
{
    return get_mat4(get_id(name), base_val);
}
std::string Block::get_string(const std::string name, std::string base_val) const
{
    return get_string(get_id(name), base_val);
}
Block *Block::get_block(std::string name) const
{
    return get_block(get_id(name));
}
bool Block::get_arr(const std::string name, std::vector<double> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace);
}
bool Block::get_arr(const std::string name, std::vector<float> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace);
}
bool Block::get_arr(const std::string name, std::vector<int> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace);
}
bool Block::get_arr(const std::string name, std::vector<unsigned> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace);
}
bool Block::get_arr(const std::string name, std::vector<short> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace);
}
bool Block::get_arr(const std::string name, std::vector<unsigned short> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace);
}
bool Block::get_arr(const std::string name, std::vector<std::string> &_values, bool replace) const
{
    return get_arr(get_id(name), _values, replace); 
}

void save_value(std::string &str, Block::Value &v);
void save_block(std::string &str, Block &b)
{
    str += "{\n";
    for (int i = 0; i < b.size(); i++)
    {
        str += b.names[i];
        save_value(str, b.values[i]);
        str += "\n";
    }
    str += "}";
}
void save_arr(std::string &str, Block::DataArray &a)
{
    str += "{ ";
    if (a.type == Block::ValueType::DOUBLE)
    {
        for (int i = 0; i < a.values.size(); i++)
        {
            str += std::to_string(a.values[i].d);
            if (i < a.values.size() - 1)
                str += ", ";
        }
    }
    else if (a.type == Block::ValueType::STRING)
    {
        for (int i = 0; i < a.values.size(); i++)
        {
            if (a.values[i].s)
              str += "\""+*(a.values[i].s)+"\"";
            else 
              str += "\"\"";
            if (i < a.values.size() - 1)
                str += ", ";
        }
    }
    str += " }";
}
void save_value(std::string &str, Block::Value &v)
{
    if (v.type == Block::ValueType::EMPTY)
    {
        str += ":tag";
    }
    else if (v.type == Block::ValueType::BOOL)
    {
        str += ":b = ";
        str += v.b ? "true" : "false";
    }
    else if (v.type == Block::ValueType::INT)
    {
        str += ":i = ";
        str += std::to_string(v.i);
    }
    else if (v.type == Block::ValueType::UINT64)
    {
        str += ":u64 = ";
        str += std::to_string(v.u);
    }
    else if (v.type == Block::ValueType::DOUBLE)
    {
        str += ":r = ";
        str += std::to_string(v.d);
    }
    else if (v.type == Block::ValueType::VEC2)
    {
        str += ":p2 = ";
        str += std::to_string(v.v2.x) + ", " + std::to_string(v.v2.y);
    }
    else if (v.type == Block::ValueType::VEC3)
    {
        str += ":p3 = ";
        str += std::to_string(v.v3.x) + ", " + std::to_string(v.v3.y) + ", " + std::to_string(v.v3.z);
    }
    else if (v.type == Block::ValueType::VEC4)
    {
        str += ":p4 = ";
        str += std::to_string(v.v4.x) + ", " + std::to_string(v.v4.y) + ", " + std::to_string(v.v4.z) +
               ", " + std::to_string(v.v4.w);
    }
    else if (v.type == Block::ValueType::IVEC2)
    {
        str += ":i2 = ";
        str += std::to_string(v.iv2.x) + ", " + std::to_string(v.iv2.y);
    }
    else if (v.type == Block::ValueType::IVEC3)
    {
        str += ":i3 = ";
        str += std::to_string(v.iv3.x) + ", " + std::to_string(v.iv3.y) + ", " + std::to_string(v.iv3.z);
    }
    else if (v.type == Block::ValueType::IVEC4)
    {
        str += ":i4 = ";
        str += std::to_string(v.iv4.x) + ", " + std::to_string(v.iv4.y) + ", " + std::to_string(v.iv4.z) +
               ", " + std::to_string(v.iv4.w);
    }
    else if (v.type == Block::ValueType::MAT4)
    {
        str += ":m4 = ";
        for (int i=0;i<4;i++)
        {
            for (int j=0;j<4;j++)
            {
                str += std::to_string(v.m4(i,j));
                if (i < 3 || j < 3)
                    str += ", ";
                if (j == 3)
                    str += "  ";
            }
        }
    }
    else if (v.type == Block::ValueType::STRING && v.s)
    {
        str += ":s = \"" + *(v.s) + "\"";
    }
    else if (v.type == Block::ValueType::ARRAY && v.a)
    {
        str += ":arr = ";
        save_arr(str, *(v.a));
    }
    else if (v.type == Block::ValueType::BLOCK && v.bl)
    {
        str += " ";
        save_block(str, *(v.bl));
    }
}

void save_block_to_string(std::string &str, Block &b)
{
    save_block(str, b);   
}

void save_block_to_file(std::string path, Block &b)
{
    std::string input;

    save_block(input, b);

    std::ofstream out(base_blk_path + path);
    out << input;
    out.close();
}

void Block::Value::clear()
{
    if (type == Block::ValueType::BLOCK && bl)
    {
        bl->clear();
        delete bl;
    }
    else if (type == Block::ValueType::ARRAY && a)
        delete a;
    else if (type == Block::ValueType::STRING && s)
        delete s;

    type = Block::ValueType::EMPTY;
}
void Block::clear()
{
    for (int i = 0; i < size(); i++)
    {
        values[i].clear();
    }
    values.clear();
    names.clear();
}
bool Block::has_tag(const std::string &name) const
{
    int id = get_id(name);
    return id >= 0 && (get_type(id) == Block::ValueType::EMPTY);
}

void Block::add_bool(const std::string name, bool base_val)
{
    Block::Value val;
    val.type = Block::ValueType::BOOL;
    val.b = base_val;
    add_value(name, val);
}
void Block::add_int(const std::string name, int base_val)
{
    Block::Value val;
    val.type = Block::ValueType::INT;
    val.i = base_val;
    add_value(name, val);
}
void Block::add_uint64(const std::string name, uint64_t base_val)
{
    Block::Value val;
    val.type = Block::ValueType::UINT64;
    val.u = base_val;
    add_value(name, val);
}
void Block::add_double(const std::string name, double base_val)
{
    Block::Value val;
    val.type = Block::ValueType::DOUBLE;
    val.d = base_val;
    add_value(name, val);
}
void Block::add_vec2(const std::string name, float2 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::VEC2;
    val.v2 = base_val;
    add_value(name, val);
}
void Block::add_vec3(const std::string name, float3 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::VEC3;
    val.v3 = base_val;
    add_value(name, val);
}
void Block::add_vec4(const std::string name, float4 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::VEC4;
    val.v4 = base_val;
    add_value(name, val);
}
void Block::add_ivec2(const std::string name, int2 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::IVEC2;
    val.iv2 = base_val;
    add_value(name, val);
}
void Block::add_ivec3(const std::string name, int3 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::IVEC3;
    val.iv3 = base_val;
    add_value(name, val);
}
void Block::add_ivec4(const std::string name, int4 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::IVEC4;
    val.iv4 = base_val;
    add_value(name, val);
}
void Block::add_mat4(const std::string name, float4x4 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::MAT4;
    val.m4 = base_val;
    add_value(name, val);
}
void Block::add_string(const std::string name, std::string base_val)
{
    Block::Value val;
    val.type = Block::ValueType::STRING;
    val.s = new std::string(base_val);
    add_value(name, val);
}
void Block::add_block(const std::string name, Block *bl)
{
    Block::Value val;
    val.type = Block::ValueType::BLOCK;
    
    val.bl = new Block();
    val.bl->copy(bl);
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<double> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (double &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<float> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (float &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<int> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (int &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<unsigned> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (unsigned &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<short> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (short &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<unsigned short> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (unsigned short &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    add_value(name, val);
}
void Block::add_arr(const std::string name, std::vector<std::string> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::STRING;
    for (std::string &str : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::STRING;
        av.s = new std::string(str);
        val.a->values.push_back(av);
    }
    add_value(name, val);
}

void Block::set_bool(const std::string name, bool base_val)
{
    Block::Value val;
    val.type = Block::ValueType::BOOL;
    val.b = base_val;
    set_value(name, val);
}
void Block::set_int(const std::string name, int base_val)
{
    Block::Value val;
    val.type = Block::ValueType::INT;
    val.i = base_val;
    set_value(name, val);
}
void Block::set_uint64(const std::string name, uint64_t base_val)
{
    Block::Value val;
    val.type = Block::ValueType::UINT64;
    val.u = base_val;
    set_value(name, val);
}
void Block::set_double(const std::string name, double base_val)
{
    Block::Value val;
    val.type = Block::ValueType::DOUBLE;
    val.d = base_val;
    set_value(name, val);
}
void Block::set_vec2(const std::string name, float2 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::VEC2;
    val.v2 = base_val;
    set_value(name, val);
}
void Block::set_vec3(const std::string name, float3 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::VEC3;
    val.v3 = base_val;
    set_value(name, val);
}
void Block::set_vec4(const std::string name, float4 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::VEC4;
    val.v4 = base_val;
    set_value(name, val);
}
void Block::set_ivec2(const std::string name, int2 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::IVEC2;
    val.iv2 = base_val;
    set_value(name, val);
}
void Block::set_ivec3(const std::string name, int3 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::IVEC3;
    val.iv3 = base_val;
    set_value(name, val);
}
void Block::set_ivec4(const std::string name, int4 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::IVEC4;
    val.iv4 = base_val;
    set_value(name, val);
}
void Block::set_mat4(const std::string name, float4x4 base_val)
{
    Block::Value val;
    val.type = Block::ValueType::MAT4;
    val.m4 = base_val;
    set_value(name, val);
}
void Block::set_string(const std::string name, std::string base_val)
{
    Block::Value val;
    val.type = Block::ValueType::STRING;
    val.s = new std::string(base_val);
    set_value(name, val);
}
void Block::set_block(const std::string name, Block *bl)
{
    Block::Value val;
    val.type = Block::ValueType::BLOCK;
    val.bl = new Block();
    val.bl->copy(bl);
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<double> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (const double &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<float> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (const float &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<int> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (const int &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<unsigned> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (const unsigned &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<short> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (const short &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<unsigned short> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::DOUBLE;
    for (const unsigned short &d : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::DOUBLE;
        av.d = d;
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
void Block::set_arr(const std::string name, const std::vector<std::string> &_values)
{
    Block::Value val;
    val.type = Block::ValueType::ARRAY;
    val.a = new Block::DataArray();
    val.a->type = Block::ValueType::STRING;
    for (const std::string &str : _values)
    {
        Block::Value av;
        av.type = Block::ValueType::STRING;
        av.s = new std::string(str);
        val.a->values.push_back(av);
    }
    set_value(name, val);
}
std::string Block::get_name(int id) const
{
    return (id >= 0 && id < names.size()) ? names[id] : "";
}
void Block::add_value(const std::string &name, const Block::Value &value)
{
    values.push_back(value);
    names.push_back(name);
}
void Block::set_value(const std::string &name, const Block::Value &value)
{
    int id = get_id(name);
    if (id >= 0)
    {
        values[id].copy(value);
    }
    else 
        add_value(name,value);
}

void Block::add_detalization(Block &det)
{
    for (int i=0;i<det.size();i++)
    {
        int id = get_id(det.get_name(i));
        if (id >= 0 && values[id].type == det.get_type(i))
        {
            if (values[id].type == ValueType::BLOCK)
            {
                if (values[id].bl && det.values[i].bl)
                    values[id].bl->add_detalization(*(det.values[i].bl));
            }
            else
                values[id].copy(det.values[i]);
            //fprintf(stderr, "detalization added %s %d %d",det.get_name(i).c_str(),values[id].bl, det.values[i].bl);
        }
    }
}

void Block::copy(const Block *b)
{
    names = b->names;
    values = b->values;
    for (int i = 0;i<b->names.size();i++)
    {
        if (values[i].type == ValueType::ARRAY && values[i].a)
        {
            values[i].a = new DataArray();
            values[i].a->type = b->values[i].a->type;
            values[i].a->values = b->values[i].a->values;
        }
        if (values[i].type == ValueType::BLOCK && values[i].bl)
        {
            values[i].bl = new Block();
            values[i].bl->copy(b->values[i].bl);
        }
        if (values[i].type == ValueType::STRING && values[i].s)
        {
            values[i].s = new std::string();
            *(values[i].s) = *(b->values[i].s);
        }
    }
}

Block::~Block()
{
  clear();
}
Block &Block::operator=(Block &b)
{
  clear();
  copy(&b);
  return *this;
}
Block &Block::operator=(Block &&b)
{
  clear();
  names = std::move(b.names);
  values = std::move(b.values);
  return *this;
}